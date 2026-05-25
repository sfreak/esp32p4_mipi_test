#include <stdio.h>
#include <unistd.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_types.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_sensor.h"
#include "driver/i2c_master.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor_detect.h"
#include "driver/isp.h"
#include "esp_intr_types.h"
#include "esp_intr_alloc.h"

#include "csi_lowlevel.h"


#define I2C_SDA_IO_NUM              7
#define I2C_SCL_IO_NUM              8
#define I2C_PORT_NUM                0


#define SENSOR_SCCB_FREQ            100000

// FIXME: dynamically adjust frame sizes to match detected image sensor
// this will only work for OV9281:
#define SENSOR_DEFAULT_FORMAT_NAME  "MIPI_2lane_24Minput_RAW8_640x400_100fps"


static const char *TAG = "main";

typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;
    esp_sccb_io_handle_t sccb_handle;
    esp_cam_sensor_format_t *cam_cur_fmt;
} sensor_config_t;

esp_err_t sensor_init(sensor_config_t *out_cfg)
{
    esp_err_t ret = ESP_FAIL;

    //---------------I2C Init------------------//
    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = I2C_SDA_IO_NUM,
        .scl_io_num = I2C_SCL_IO_NUM,
        .i2c_port = I2C_PORT_NUM,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle));

    esp_cam_sensor_config_t cam_config = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
    };

    //esp_cam_sensor_device_t *cam = NULL;
    //cam = sensor_detect_function(&cam_config);

    // loop through all enabled image sensor's detect functions
    esp_cam_sensor_device_t *cam = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start; 
        p < &__esp_cam_sensor_detect_fn_array_end; 
        ++p) {
        sccb_i2c_config_t i2c_config = {
            .scl_speed_hz = SENSOR_SCCB_FREQ,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus_handle, &i2c_config, &cam_config.sccb_handle));

        cam_config.sensor_port = p->port;

        ESP_LOGI(TAG, "sccp addr 0x%02x, sensor_detect fct 0x%08x", p->sccb_addr, p->detect);

        cam = (*(p->detect))(&cam_config);
        if (cam) break;
        ESP_ERROR_CHECK(esp_sccb_del_i2c_io(cam_config.sccb_handle));
    }

    if (!cam) {
        ESP_LOGE(TAG, "failed to detect camera sensor");
        return ESP_FAIL;
    }

    esp_cam_sensor_format_array_t cam_fmt_array = {0};
    esp_cam_sensor_query_format(cam, &cam_fmt_array);
    const esp_cam_sensor_format_t *parray = cam_fmt_array.format_array;
    for (int i = 0; i < cam_fmt_array.count; i++) {
        ESP_LOGI(TAG, "fmt[%d].name:%s", i, parray[i].name);
    }

    esp_cam_sensor_format_t *cam_cur_fmt = NULL;
    for (int i = 0; i < cam_fmt_array.count; i++) {
        if (!strcmp(parray[i].name, SENSOR_DEFAULT_FORMAT_NAME)) {
            cam_cur_fmt = (esp_cam_sensor_format_t *) & (parray[i]);
        }
    }
    if (!cam_cur_fmt) {
        ESP_LOGE(TAG, "Unsupported format");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }

    ret = esp_cam_sensor_set_format(cam, (const esp_cam_sensor_format_t *) cam_cur_fmt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Format set fail");
    } else {
        ESP_LOGI(TAG, "Format in use:%s", cam_cur_fmt->name);
    }

    int enable_flag = 1;
    // Set sensor output stream
    ret = esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &enable_flag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start stream fail");
    }
    ESP_ERROR_CHECK(ret);

    out_cfg->i2c_bus_handle = i2c_bus_handle;
    out_cfg->sccb_handle = cam_config.sccb_handle;
    out_cfg->cam_cur_fmt = cam_cur_fmt;

    return ret;
}


bool mipi_on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    ESP_EARLY_LOGI(TAG, "mipi_on_get_new_trans");
    
    esp_cam_ctlr_trans_t new_trans = *(esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = new_trans.buffer;
    trans->buflen = new_trans.buflen;
    
    return false;
}

bool mipi_on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    ESP_EARLY_LOGI(TAG, "mipi_on_trans_finished: %u bytes received", trans->received_size);
    return false;
}

void isr_isp(void *arg)
{}

void isr_csi_error(void *arg)
{}

void app_main(void)
{

    //mipi ldo
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    sensor_config_t sensor_cfg = {0};
    ESP_ERROR_CHECK(sensor_init(&sensor_cfg));

    cam_ctlr_color_t csi_color = 0;
    isp_color_t isp_color = 0;
    float bytes_per_pixel = 0;
    switch(sensor_cfg.cam_cur_fmt->format) {
    case ESP_CAM_SENSOR_PIXFORMAT_RAW8:
        csi_color = CAM_CTLR_COLOR_RAW8;
        isp_color = ISP_COLOR_RAW8;
        bytes_per_pixel = 1.0;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RAW10:
        csi_color = CAM_CTLR_COLOR_RAW10;
        isp_color = ISP_COLOR_RAW10;
        bytes_per_pixel = 5/4;
        break;
    default:
        ESP_LOGE(TAG, "unsupported pixel format");
        assert(0);
        break;
    }
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = sensor_cfg.cam_cur_fmt->width,
        .v_res = sensor_cfg.cam_cur_fmt->height,
        .lane_bit_rate_mbps = sensor_cfg.cam_cur_fmt->mipi_info.mipi_clk / 1000 / 1000 * 2,
        .input_data_color_type = csi_color,
        .output_data_color_type = csi_color,
        .data_lane_num = sensor_cfg.cam_cur_fmt->mipi_info.lane_num,
        .byte_swap_en = false,
        .queue_items = 2,
    };
    esp_cam_ctlr_handle_t handle = NULL;
    
    void *frame_buffer = NULL;
    size_t frame_buffer_size = 0;
    frame_buffer_size = sensor_cfg.cam_cur_fmt->width * sensor_cfg.cam_cur_fmt->height * bytes_per_pixel;
    frame_buffer = heap_caps_calloc(1, frame_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if(!frame_buffer)
    {
        ESP_LOGE(TAG, "allocating frame buffer failed");
        return;
    }

    ESP_LOGD(TAG, "HSIZE: %d, VSIZE: %d, bits per pixel: %d", 
        sensor_cfg.cam_cur_fmt->width, sensor_cfg.cam_cur_fmt->height, bytes_per_pixel);
    ESP_LOGD(TAG, "frame_buffer_size: %zu", frame_buffer_size);
    ESP_LOGD(TAG, "frame_buffer: %p", frame_buffer);

    esp_cam_ctlr_trans_t new_trans = {
        .buffer = frame_buffer,
        .buflen = frame_buffer_size,
    };

    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_config, &handle));

    esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = mipi_on_get_new_trans,
        .on_trans_finished = mipi_on_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(handle, &callbacks, &new_trans));

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(handle));

    //---------------ISP Init------------------//
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = isp_color,
        .output_data_color_type = isp_color,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = sensor_cfg.cam_cur_fmt->width,
        .v_res = sensor_cfg.cam_cur_fmt->height,
        .flags.bypass_isp = false,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    ESP_ERROR_CHECK(esp_cam_ctlr_start(handle));
    ESP_LOGI("CAM", "Camera controller started successfully");

    usleep(1000*100);
    csi_readerrors();
    usleep(1000*100);
    csi_readerrors();
    usleep(1000*100);

    ESP_LOGI(TAG, "Main loop");
    while(1)
    {
        ESP_LOGI(TAG, "calling esp_cam_ctlr_receive...");
        ESP_ERROR_CHECK(esp_cam_ctlr_receive(handle, &new_trans, ESP_CAM_CTLR_MAX_DELAY));
        ESP_LOGI(TAG, "esp_cam_ctlr_receive returned");
        csi_readerrors();
        usleep(1000);
    }

}
