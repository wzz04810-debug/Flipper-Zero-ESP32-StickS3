#include "furi_hal.h"
#include "boards/board.h"
#include <furi_hal_gpio.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/i2c.h>

static const char* TAG = "FuriHal";

#ifdef BOARD_HAS_M5PM1
/* PM1 can NACK the first transaction while waking from I2C idle sleep.
 * Only report success after a read-back; never clobber other power GPIOs. */
static esp_err_t pm1_read(uint8_t reg, uint8_t* data, size_t len) {
    esp_err_t err = ESP_FAIL;
    for(unsigned i = 0; i < 10; ++i) {
        err = i2c_master_write_read_device(
            BOARD_PM1_I2C_PORT, BOARD_PM1_I2C_ADDR, &reg, 1, data, len,
            pdMS_TO_TICKS(100));
        if(err == ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return err;
}

static esp_err_t pm1_update(uint8_t reg, uint8_t mask, uint8_t bits) {
    uint8_t value;
    esp_err_t err = pm1_read(reg, &value, 1);
    if(err != ESP_OK) return err;
    const uint8_t tx[] = {reg, (value & ~mask) | (bits & mask)};
    for(unsigned i = 0; i < 10; ++i) {
        err = i2c_master_write_to_device(
            BOARD_PM1_I2C_PORT, BOARD_PM1_I2C_ADDR, tx, sizeof(tx), pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
        if(err != ESP_OK) continue;
        err = pm1_read(reg, &value, 1);
        if(err == ESP_OK && (value & mask) == (bits & mask)) {
            ESP_LOGI(TAG, "M5PM1 reg %02X verified=%02X", reg, value);
            return ESP_OK;
        }
        if(err == ESP_OK) err = ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}
#endif

void furi_hal_init_early(void) {
    furi_hal_cortex_init_early();

#ifdef BOARD_PIN_PWR_EN
    /* Power-enable must be set early — powers CC1101, BQ27220 fuel gauge, WS2812 */
    static const GpioPin pwr_en = {.port = NULL, .pin = BOARD_PIN_PWR_EN};
    furi_hal_gpio_init_simple(&pwr_en, GpioModeOutputPushPull);
    furi_hal_gpio_write(&pwr_en, true);
    ESP_LOGI(TAG, "PWR_EN GPIO%d set HIGH", BOARD_PIN_PWR_EN);
#endif

#ifdef BOARD_PIN_NRF24_CSN
    /* T-Embed Plus shares SPI2 between CC1101 and NRF24. Drive NRF24 CSN HIGH
     * (deselected) and CE LOW (standby) at boot, before any CC1101 SPI traffic.
     * Without this, NRF24 sees CC1101 traffic and corrupts the bus, manifesting
     * as a stuck ~312 MHz reading in the Frequency Analyzer and total RX failure. */
    static const GpioPin nrf24_csn = {.port = NULL, .pin = BOARD_PIN_NRF24_CSN};
    furi_hal_gpio_init_simple(&nrf24_csn, GpioModeOutputPushPull);
    furi_hal_gpio_write(&nrf24_csn, true);
    ESP_LOGI(TAG, "NRF24_CSN GPIO%d set HIGH (deselect)", BOARD_PIN_NRF24_CSN);
#endif

#if defined(BOARD_PIN_NRF24_CE) && BOARD_PIN_NRF24_CE >= 0
    static const GpioPin nrf24_ce = {.port = NULL, .pin = BOARD_PIN_NRF24_CE};
    furi_hal_gpio_init_simple(&nrf24_ce, GpioModeOutputPushPull);
    furi_hal_gpio_write(&nrf24_ce, false);
    ESP_LOGI(TAG, "NRF24_CE GPIO%d set LOW (standby)", BOARD_PIN_NRF24_CE);
#endif

    /* StickS3 LCD power is gated by M5PM1 GPIO2 (L3B enable). The LCD
     * backlight GPIO can be driven correctly while the panel remains
     * completely unpowered, which results in a black screen. Configure PM1
     * before LCD init. PM1 GPIO3 is the IRQ pin on StickS3; external output
     * is controlled by PWR_CFG BOOST_EN, matching M5Unified. */
#ifdef BOARD_HAS_M5PM1
    i2c_config_t pm1_i2c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BOARD_PIN_PM1_SDA,
        .scl_io_num = BOARD_PIN_PM1_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    esp_err_t pm1_err = i2c_param_config(BOARD_PM1_I2C_PORT, &pm1_i2c);
    if(pm1_err == ESP_OK) {
        pm1_err = i2c_driver_install(BOARD_PM1_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
        if(pm1_err == ESP_OK || pm1_err == ESP_ERR_INVALID_STATE) {
            uint8_t pm1_id[2] = {0};
            pm1_err = pm1_read(0x00, pm1_id, sizeof(pm1_id));
            ESP_LOGI(TAG, "M5PM1 probe: %s id=%02X%02X",
                esp_err_to_name(pm1_err), pm1_id[1], pm1_id[0]);
            if(pm1_err == ESP_OK && (pm1_id[0] != 0x50 || pm1_id[1] != 0x20)) {
                pm1_err = ESP_ERR_INVALID_RESPONSE;
            }
            /* Register, mask, bits. GPIO2 is the LCD L3B enable; GPIO3 is
             * the separate PA/Grove control. This is the same sequence used
             * by M5GFX/M5Unified for M5StickS3. */
            const uint8_t pm1_setup[][3] = {
                {0x09, 0xFF, 0x00}, /* disable I2C idle sleep */
                {0x06, 0x08, 0x08}, /* PWR_CFG BOOST_EN: official M5pm1.setExtOutput(true) */
                {0x16, 0x30, 0x00}, /* GPIO2 function = GPIO (bits 5:4) */
                {0x10, 0x04, 0x04}, /* GPIO2 mode = output */
                {0x13, 0x04, 0x00}, /* GPIO2 push-pull */
                {0x11, 0x04, 0x04}, /* GPIO2 high: LCD power/L3B enable */
                /* GPIO3 is PM1 IRQ on StickS3, not Grove power. Leave its
                 * board-default function untouched; BOOST_EN above is the
                 * complete external-output control used by M5Unified. */
            };
            for(size_t i = 0; pm1_err == ESP_OK && i < sizeof(pm1_setup) / sizeof(pm1_setup[0]); i++) {
                pm1_err = pm1_update(pm1_setup[i][0], pm1_setup[i][1], pm1_setup[i][2]);
                if(pm1_err != ESP_OK) ESP_LOGE(TAG, "M5PM1 reg %02X failed: %s",
                    pm1_setup[i][0], esp_err_to_name(pm1_err));
            }
            if(pm1_err == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(100));
                uint8_t pwr_cfg = 0;
                pm1_read(0x06, &pwr_cfg, 1);
                uint8_t gpio_out = 0;
                pm1_read(0x11, &gpio_out, 1);
                ESP_LOGI(TAG, "M5PM1 LCD power GPIO2 verified, PWR_CFG=%02X GPIO_OUT=%02X (BOOST_EN=%u GPIO2=%u GPIO3=%u)",
                    pwr_cfg, gpio_out, (pwr_cfg >> 3) & 0x1U,
                    (gpio_out >> 2) & 0x1U, (gpio_out >> 3) & 0x1U);
            } else {
                ESP_LOGE(TAG, "LCD power setup FAILED: %s", esp_err_to_name(pm1_err));
            }
        } else {
            ESP_LOGW(TAG, "M5PM1 I2C install failed: %s", esp_err_to_name(pm1_err));
        }
    } else {
        ESP_LOGW(TAG, "M5PM1 I2C config failed: %s", esp_err_to_name(pm1_err));
    }
#endif

    ESP_LOGI(TAG, "Early init complete");
}

void furi_hal_deinit_early(void) {
}

void furi_hal_init(void) {
    /* NVS is required by WiFi and BLE — init once at boot */
    esp_err_t nvs_err = nvs_flash_init();
    if(nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    furi_hal_rtc_init();
    furi_hal_version_init();
    furi_hal_info_init();
    furi_hal_power_init();
    furi_hal_crypto_init();
    furi_hal_subghz_init();
    furi_hal_usb_init();
    furi_hal_light_init();
    furi_hal_display_init();
    furi_hal_speaker_init();
    furi_hal_nfc_init();
    ESP_LOGI(TAG, "Init complete");
}
