#include "esp_log.h"
#include "hal/mipi_csi_host_ll.h"
#include "csi_lowlevel.h"

static const char *TAG = "csi_lowlevel";

void csi_ll_en_int()
{
    csi_host_dev_t *csi_host = MIPI_CSI_HOST_LL_GET_HW(0);
    
    // enable all error interrupts
    csi_host->int_msk_phy_fatal.val = 3;
    csi_host->int_msk_pkt_fatal.val = 3;
    csi_host->int_msk_phy.val = 3 | (3<<16);
    csi_host->int_msk_bndry_frame_fatal.val = 0xffff;
    csi_host->int_msk_seq_frame_fatal.val = 0xffff;
    csi_host->int_msk_crc_frame_fatal.val = 0xffff;
    csi_host->int_msk_pld_crc_fatal.val = 0xffff;
    csi_host->int_msk_data_id.val = 0xffff;
    csi_host->int_msk_ecc_corrected.val = 0xffff;
}

void csi_ll_force_error()
{
    csi_host_dev_t *csi_host = MIPI_CSI_HOST_LL_GET_HW(0);

    // force an error just to see if it maks it into the status register or if it is masked out
    csi_host->int_force_crc_frame_fatal.force_err_frame_data_vc3 = 1;
}

void csi_ll_logstatus()
{
    //ESP_LOGI(TAG, "csi_readerrors");

    csi_host_dev_t *csi_host = MIPI_CSI_HOST_LL_GET_HW(0);


    ESP_LOGI(TAG, "CSI_HOST_VERSION_REG       = 0x%08x", csi_host->version.val);
    ESP_LOGI(TAG, "CSI_HOST_N_LANES_REG       = 0x%08x", csi_host->n_lanes.val);
    ESP_LOGI(TAG, "CSI_HOST_PHY_RX_REG          clk act: %u, data0 ULP: %u, data1 ULP: %u", 
        csi_host->phy_rx.phy_rxclkactivehs,csi_host->phy_rx.phy_rxulpsesc_0, csi_host->phy_rx.phy_rxulpsesc_1);
    ESP_LOGI(TAG, "CSI_HOST_INT_ST_MAIN_REG   = 0x%08x", csi_host->int_st_main.val);
    ESP_LOGI(TAG, "CSI_HOST_PHY_STOPSTATE_REG = 0x%08x", csi_host->phy_stopstate.val);

}

void csi_ll_logstatus_from_isr()
{
    csi_host_dev_t *csi_host = MIPI_CSI_HOST_LL_GET_HW(0);
    uint32_t st_main = csi_host->int_st_main.val;
    ESP_EARLY_LOGI(TAG,                        "MAIN_REG          = 0x%08x", st_main);
    if (st_main & (1<< 0)) ESP_EARLY_LOGI(TAG, "PHY_FATAL         = 0x%08x", csi_host->int_st_phy_fatal.val);
    if (st_main & (1<< 1)) ESP_EARLY_LOGI(TAG, "PKT_FATAL         = 0x%08x", csi_host->int_st_pkt_fatal.val);
    if (st_main & (1<< 2)) ESP_EARLY_LOGI(TAG, "BNDRY_FRAME_FATAL = 0x%08x", csi_host->int_st_bndry_frame_fatal.val);
    if (st_main & (1<< 3)) ESP_EARLY_LOGI(TAG, "SEQ_FRAME_FATAL   = 0x%08x", csi_host->int_st_seq_frame_fatal.val);
    if (st_main & (1<< 4)) ESP_EARLY_LOGI(TAG, "CRC_FRAME_FATAL   = 0x%08x", csi_host->int_st_crc_frame_fatal.val);
    if (st_main & (1<< 5)) ESP_EARLY_LOGI(TAG, "PLD_CRC_FATAL     = 0x%08x", csi_host->int_st_pld_crc_fatal.val);
    if (st_main & (1<< 6)) ESP_EARLY_LOGI(TAG, "DATA_ID           = 0x%08x", csi_host->int_st_data_id.val);
    if (st_main & (1<< 7)) ESP_EARLY_LOGI(TAG, "ECC_CORRECTED     = 0x%08x", csi_host->int_st_ecc_corrected.val);
    if (st_main & (1<<16)) ESP_EARLY_LOGI(TAG, "PHY               = 0x%08x", csi_host->int_st_phy.val);
}