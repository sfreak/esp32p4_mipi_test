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

#include "driver/isp.h"

//#include "ov9281.h"
#include "vd56g3.h"


#define I2C_SDA_IO_NUM              7
#define I2C_SCL_IO_NUM              8
#define I2C_PORT_NUM                0


#define SENSOR_SCCB_FREQ            100000

#ifdef OV9281_SCCB_ADDR
#define SENSOR_DEFAULT_FORMAT_NAME  "MIPI_2lane_24Minput_RAW8_640x400_100fps"
#define SENSOR_SCCB_ADDR            OV9281_SCCB_ADDR
#define SENSOR_BYTES_PER_PIXEL      1
#define MIPI_CSI_DISP_HSIZE         640
#define MIPI_CSI_DISP_VSIZE         400
#define MIPI_CSI_LANE_BITRATE_MBPS  400
#define sensor_detect_function(x)   ov9281_detect(x)
#elif defined(VD56G3_SCCB_ADDR)
// VD56G3
#define SENSOR_DEFAULT_FORMAT_NAME  "MIPI_2lane_12Minput_RAW8_480x640_88fps"
#define SENSOR_SCCB_ADDR            VD56G3_SCCB_ADDR
#define SENSOR_BYTES_PER_PIXEL      1
#define MIPI_CSI_DISP_HSIZE         480
#define MIPI_CSI_DISP_VSIZE         640
#define MIPI_CSI_LANE_BITRATE_MBPS  400
#define sensor_detect_function(x)   vd56g3_detect(x)
#else
#error "No image sensor driver included"
#endif

static const char *TAG = "main";


esp_err_t sensor_init(void)
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

    //---------------SCCB Init------------------//
    sccb_i2c_config_t i2c_config = {
        .scl_speed_hz = SENSOR_SCCB_FREQ,
        .device_address = SENSOR_SCCB_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    };
    ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus_handle, &i2c_config, &cam_config.sccb_handle));

    esp_cam_sensor_device_t *cam = NULL;
    cam = sensor_detect_function(&cam_config);

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

void app_main(void)
{

    //mipi ldo
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    sensor_init();

    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = MIPI_CSI_DISP_HSIZE,
        .v_res = MIPI_CSI_DISP_VSIZE,
        .lane_bit_rate_mbps = MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RAW8,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    esp_cam_ctlr_handle_t handle = NULL;
    
    void *frame_buffer = NULL;
    size_t frame_buffer_size = 0;
    frame_buffer_size = MIPI_CSI_DISP_HSIZE * MIPI_CSI_DISP_VSIZE * SENSOR_BYTES_PER_PIXEL;
    frame_buffer = heap_caps_calloc(1, frame_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if(!frame_buffer)
    {
        ESP_LOGE(TAG, "allocating frame buffer failed");
        return;
    }

    ESP_LOGD(TAG, "HSIZE: %d, VSIZE: %d, bits per pixel: %d", MIPI_CSI_DISP_HSIZE, MIPI_CSI_DISP_VSIZE, SENSOR_BYTES_PER_PIXEL);
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
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RAW8,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = MIPI_CSI_DISP_HSIZE,
        .v_res = MIPI_CSI_DISP_VSIZE,
        .flags.bypass_isp = false,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    ESP_ERROR_CHECK(esp_cam_ctlr_start(handle));
    ESP_LOGI("CAM", "Camera controller started successfully");

    ESP_LOGI(TAG, "Main loop");
    while(1)
    {
        ESP_LOGI(TAG, "calling esp_cam_ctlr_receive...");
        ESP_ERROR_CHECK(esp_cam_ctlr_receive(handle, &new_trans, ESP_CAM_CTLR_MAX_DELAY));
        ESP_LOGI(TAG, "esp_cam_ctlr_receive returned");
        usleep(1000);
    }

}
