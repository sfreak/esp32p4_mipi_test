#include "esp_log.h"
#include "hal/mipi_csi_host_ll.h"
#include "csi_lowlevel.h"

static const char *TAG = "csi_lowlevel";

esp_err_t csi_readerrors()
{
    ESP_LOGI(TAG, "csi_readerrors");

    csi_host_dev_t *csi_host = MIPI_CSI_HOST_LL_GET_HW(0);

    // force an error just to see if it maks it into the status register or if it is masked out
    //csi_host->int_force_crc_frame_fatal.force_err_frame_data_vc0 = 1;

    ESP_LOGI(TAG, "CSI_HOST_VERSION_REG       = 0x%08x", csi_host->version.val);
    ESP_LOGI(TAG, "CSI_HOST_N_LANES_REG       = 0x%08x", csi_host->n_lanes.val);
    ESP_LOGI(TAG, "CSI_HOST_PHY_RX_REG          clk act: %u, data0 ULP: %u, data1 ULP: %u", 
        csi_host->phy_rx.phy_rxclkactivehs,csi_host->phy_rx.phy_rxulpsesc_0, csi_host->phy_rx.phy_rxulpsesc_1);
    ESP_LOGI(TAG, "CSI_HOST_INT_ST_MAIN_REG   = 0x%08x", csi_host->int_st_main.val);
    ESP_LOGI(TAG, "CSI_HOST_PHY_STOPSTATE_REG = 0x%08x", csi_host->phy_stopstate.val);

    return ESP_OK;
}