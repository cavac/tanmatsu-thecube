#include "sdcard.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"
#include "bsp/input.h"

static const char* TAG = "sdcard";
static bool mounted = false;

esp_err_t sdcard_init(void) {
    esp_err_t res;

    // Check if SD card is inserted
    bool sdcard_inserted = false;
    bsp_input_read_action(BSP_INPUT_ACTION_TYPE_SD_CARD, &sdcard_inserted);
    if (!sdcard_inserted) {
        ESP_LOGW(TAG, "No SD card inserted");
        return ESP_ERR_NOT_FOUND;
    }

    // Initialize SD card power LDO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
    res = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SD LDO power control: %s", esp_err_to_name(res));
        return res;
    }
    sd_pwr_ctrl_set_io_voltage(pwr_ctrl_handle, 3300);

    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    const char mount_point[] = "/sd";

    ESP_LOGI(TAG, "Initializing SD card (SPI mode)");

    // Use SPI mode like the launcher
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = GPIO_NUM_44,
        .miso_io_num     = GPIO_NUM_39,
        .sclk_io_num     = GPIO_NUM_43,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4000,
    };

    res = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(res));
        return res;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_42;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    res = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (res != ESP_OK) {
        if (res == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount SD card filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(res));
        }
        return res;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    sdmmc_card_print_info(stdout, card);
    mounted = true;
    return ESP_OK;
}

bool sdcard_is_mounted(void) {
    return mounted;
}
