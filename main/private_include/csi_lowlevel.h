#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void csi_ll_en_int(void);
void csi_ll_force_error(void);
void csi_ll_logstatus(void);
void csi_ll_logstatus_from_isr(void);

#ifdef __cplusplus
}
#endif
