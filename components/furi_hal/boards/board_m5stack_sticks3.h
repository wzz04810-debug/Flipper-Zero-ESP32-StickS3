/**
 * @file board_m5stack_sticks3.h
 * M5Stack StickS3 board definition for the ESP32 Flipper port.
 *
 * StickS3 uses an ESP32-S3-PICO-1 with 8 MB flash and 8 MB OPI PSRAM. The
 * radio/NFC devices below are external modules connected through the Grove or
 * GPIO header; they are deliberately not described as onboard peripherals.
 */

#pragma once

#define BOARD_NAME        "M5Stack StickS3"
#define BOARD_ID          "m5stack_sticks3"
#define BOARD_TARGET      "esp32s3"

/* StickS3 has an ADC five-way joystick on GPIO1 and two digital buttons. */
#define BOARD_PIN_BUTTON_BOOT   11
#define BOARD_PIN_BUTTON_KEY    12
/* 蓝色侧键：短按进入深度休眠，并作为唯一的硬件唤醒键。 */
#define BOARD_PIN_DEEP_SLEEP_WAKE BOARD_PIN_BUTTON_KEY
#define BOARD_PIN_BATTERY_ADC   UINT16_MAX
#define BOARD_PIN_JOY_ADC       1
#define BOARD_JOY_ADC_DOWN      250
#define BOARD_JOY_ADC_RIGHT     850
#define BOARD_JOY_ADC_UP        1550
#define BOARD_JOY_ADC_LEFT      2200
#define BOARD_JOY_ADC_OK        2750

/* ST7789 135x240 panel. Pin mapping follows the existing StickS3 board port. */
#define BOARD_PIN_LCD_MOSI      39
#define BOARD_PIN_LCD_SCLK      40
#define BOARD_PIN_LCD_DC        45
#define BOARD_PIN_LCD_CS        41
#define BOARD_PIN_LCD_RST       21
#define BOARD_PIN_LCD_BL        38
/* Flipper UI is landscape on the StickS3 panel. The native glass is
 * 135x240; esp_lcd swaps axes to expose a 240x135 framebuffer. */
#define BOARD_LCD_H_RES         240
#define BOARD_LCD_V_RES         135
/* M5GFX uses SPI3 for the StickS3 display. */
#define BOARD_LCD_SPI_HOST      SPI3_HOST
#define BOARD_LCD_SPI_FREQ_HZ   (40 * 1000 * 1000)
#define BOARD_LCD_CMD_BITS      8
#define BOARD_LCD_PARAM_BITS   8
/* Landscape MADCTL = MX+MV, matching ST7789 135x240 rotation 1. */
#define BOARD_LCD_SWAP_XY      true
#define BOARD_LCD_MIRROR_X     true
#define BOARD_LCD_MIRROR_Y     false
#define BOARD_LCD_INVERT_COLOR true
#define BOARD_LCD_GAP_X        40
#define BOARD_LCD_GAP_Y        52
#define BOARD_LCD_SIDE_MARGIN  0
#define BOARD_LCD_COLOR_ORDER_BGR false
#define BOARD_LCD_BL_ACTIVE_LOW false
#define BOARD_LCD_FG_COLOR      0xA0FD
#define BOARD_LCD_FG_COLOR_RB   0x5F03
#define BOARD_LCD_BG_COLOR      0x0000
/* Keep startup deterministic; no diagnostic frame in normal firmware. */

/* MicroSD shares the SPI bus with the display. StickS3 has no onboard
 * microSD slot: the pins below belong to an external module on the shared
 * SPI header. Because of that the firmware keeps its own FAT volume in the
 * unused flash area after the app partition and uses it when no card answers,
 * so /ext (and /int, which lives under /ext) always work. */
#define BOARD_PIN_SD_CS         7
#define BOARD_PIN_SD_MISO       4
#define BOARD_HAS_INTERNAL_FS   1

/* No touch controller; input is provided by the ADC joystick/buttons. */
#define BOARD_PIN_TOUCH_SCL     -1
#define BOARD_PIN_TOUCH_SDA     -1
#define BOARD_PIN_TOUCH_RST     -1
#define BOARD_PIN_TOUCH_INT     -1
#define BOARD_TOUCH_I2C_ADDR    0x00
#define BOARD_TOUCH_I2C_PORT    I2C_NUM_0
#define BOARD_TOUCH_I2C_FREQ_HZ 0
#define BOARD_TOUCH_I2C_TIMEOUT 0

/* External CC1101 over the shared SPI header. */
#define BOARD_PIN_CC1101_SCK    5
#define BOARD_PIN_CC1101_CSN    2
#define BOARD_PIN_CC1101_MISO   4
#define BOARD_PIN_CC1101_MOSI   6
#define BOARD_PIN_CC1101_GDO0   3
#define BOARD_CC1101_SPI_SHARED 1

/* External NRF24L01; CE is not wired by the current StickS3 pin map. */
#define BOARD_PIN_NRF24_CSN     8
#define BOARD_PIN_NRF24_CE      -1
#define BOARD_HAS_NRF24         1

/* Grove connector: Unit NFC (ST25R3916 I2C 0x50) on G9=SDA / G10=SCL.
 * Grove has no IRQ/RST; the ST25 backend polls interrupt registers.
 * Do not alias these pins as QWIIC — StickS3 has no BQ fuel-gauge on this bus. */
#define BOARD_PIN_NFC_SCL       10
#define BOARD_PIN_NFC_SDA       9
/* Unit NFC Grove is the external I2C bus on GPIO9/10. */
#define BOARD_NFC_I2C_PORT      I2C_NUM_0
#define BOARD_NFC_I2C_FREQ_HZ   400000
/* StickS3 uses the local ST25R3916 Unit NFC; no Chameleon BLE backend. */
#define BOARD_NFC_CHAMELEON_SUPPORTED 0

/* StickS3 audio pins are available for the onboard speaker path. */
#define BOARD_PIN_SPEAKER_BCLK  17
#define BOARD_PIN_SPEAKER_WCLK  15
#define BOARD_PIN_SPEAKER_DOUT  16
#define BOARD_HAS_SPEAKER       1

/* Optional microphone / IR / RGB features. */
#define BOARD_PIN_MIC_DATA      46
#define BOARD_PIN_MIC_CLK       18
#define BOARD_PIN_IR_TX         46
#define BOARD_PIN_IR_RX         42
#define BOARD_PIN_WS2812_DATA   8
#define BOARD_WS2812_LED_COUNT  4

#define BOARD_HAS_TOUCH         0
#define BOARD_HAS_ENCODER       0
/* StickS3 joystick already emits native Up/Down/Left/Right events. */
#define BOARD_INPUT_NATIVE_DIRECTIONS 1
#define BOARD_HAS_SD_CARD       1
#define BOARD_HAS_BLE           1
#define BOARD_HAS_RGB_LED       1
#define BOARD_HAS_VIBRO         0
#define BOARD_HAS_IR            1
#define BOARD_HAS_IBUTTON       0
#define BOARD_HAS_RFID          0
#define BOARD_HAS_NFC           1
/* The ST25R3916 passive-target path is supported for on-device emulation.
 * Reader/listener attack tools remain hidden because they need stricter timing
 * and are not part of this backend's validated feature set. */
#define BOARD_NFC_LISTENER_SUPPORTED 0
#define BOARD_NFC_EMULATION_SUPPORTED 1
#define BOARD_HAS_SUBGHZ        1
#define BOARD_HAS_MIC           1

/* Internal M5PM1 supplies the LCD via PM1 GPIO2 (L3B enable).
 * Use controller 0 here; controller 1 is reserved for Grove NFC. */
#define BOARD_HAS_M5PM1         1
/* M5StickS3 routes M5PM1 on SDA47/SCL48 through I2C_NUM_1. */
#define BOARD_PM1_I2C_PORT      I2C_NUM_1
#define BOARD_PIN_PM1_SDA      47
#define BOARD_PIN_PM1_SCL      48
#define BOARD_PM1_I2C_ADDR     0x6E

/* No BQ devices are fitted on StickS3; probes use an unused bus and should
 * report absent. Keep the address macros because the generic source refers
 * to them at compile time. */
#define BQ27220_ADDR            0x55
#define BQ25896_ADDR            0x6B
#define BQ_I2C_PORT             I2C_NUM_0
#define HIGH_DRAIN_CURRENT_THRESHOLD (-200)
#define BQ25896_CHARGE_LIMIT   1280
#define FURI_HAL_POWER_VIRTUAL_CAPACITY_MAH (1300U)

/* StickS3 does not contain an LF-RFID analog front-end. */
#define BOARD_PIN_RFID_RX       UINT16_MAX
#define BOARD_PIN_RFID_TX       UINT16_MAX
#define BOARD_RFID_UART_NUM     1
