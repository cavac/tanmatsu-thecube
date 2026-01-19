#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

// Initialize SD card power LDO
sd_pwr_ctrl_handle_t sdcard_initialize_ldo(void);

// Mount SD card filesystem at /sd (uses SDMMC 4-bit mode)
esp_err_t sdcard_mount(sd_pwr_ctrl_handle_t pwr_ctrl_handle);

// Check if SD card is mounted
bool sdcard_is_mounted(void);
