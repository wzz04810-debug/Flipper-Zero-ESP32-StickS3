/**
 * @file furi_hal_nfc_st25r3916.c
 * @brief ST25R3916/ST25R3916B I2C backend for M5Stack Unit NFC.
 *
 * The M5Stack Unit NFC only exposes Grove SDA/SCL, so interrupts are read
 * from IRQ_MAIN..IRQ_TARGET over I2C. This implementation deliberately keeps
 * every received CRC in the FIFO: the Flipper protocol stack owns CRC checks.
 */

#include "furi_hal_nfc.h"
#include <furi.h>
#include <board.h>

#include <driver/i2c.h>
#include <esp_attr.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>

#define TAG "NfcST25"
#define ST25_ADDR 0x50
#define ST25_TIMEOUT_MS 100
#define ST25_FIFO_DEPTH 512

/* ST25R3916 serial-I2C operation bytes and direct commands. */
#define ST25_SPACE_B 0x40U
#define ST25_SPACE_B_ACCESS 0xFBU
#define ST25_READ(reg) ((reg) | 0x40U)
#define ST25_FIFO_LOAD 0x80U
#define ST25_FIFO_READ 0x9FU
#define ST25_PT_A_CONFIG_LOAD 0xA0U
#define ST25_CMD_SET_DEFAULT 0xC1U
#define ST25_CMD_STOP 0xC2U
#define ST25_CMD_TX_CRC 0xC4U
#define ST25_CMD_TX_NO_CRC 0xC5U
#define ST25_CMD_TX_REQA 0xC6U
#define ST25_CMD_TX_WUPA 0xC7U
#define ST25_CMD_CLEAR_FIFO 0xDBU
#define ST25_CMD_GOTO_SENSE 0xCDU
#define ST25_CMD_GOTO_SLEEP 0xCEU
#define ST25_CMD_UNMASK_RECEIVE_DATA 0xD1U
#define ST25_CMD_ADJUST_REGULATORS 0xD6U
#define ST25_CMD_TX_WITH_CRC 0xC4U
#define ST25_CMD_TX_WITHOUT_CRC 0xC5U

/* Register map, limited to operations used by this backend. */
#define ST25_REG_IO_CONF1 0x00U
#define ST25_REG_IO_CONF2 0x01U
#define ST25_REG_OP_CONTROL 0x02U
#define ST25_REG_MODE 0x03U
#define ST25_REG_BIT_RATE 0x04U
#define ST25_REG_ISO14443A_NFC 0x05U
#define ST25_REG_PASSIVE_TARGET 0x08U
#define ST25_REG_MASK_RX_TIMER 0x0FU
#define ST25_REG_TIMER_AND_EMV 0x12U
#define ST25_REG_IRQ_MASK_MAIN 0x16U
#define ST25_REG_IRQ_MASK_TIMER_NFC 0x17U
#define ST25_REG_IRQ_MASK_ERROR_WUP 0x18U
#define ST25_REG_IRQ_MASK_TARGET 0x19U
#define ST25_REG_AUX 0x0AU
#define ST25_REG_RX_CONF1 0x0BU
#define ST25_REG_RX_CONF2 0x0CU
#define ST25_REG_RX_CONF3 0x0DU
#define ST25_REG_RX_CONF4 0x0EU
#define ST25_REG_IRQ_MAIN 0x1AU
#define ST25_REG_IRQ_TIMER_NFC 0x1BU
#define ST25_REG_IRQ_ERROR_WUP 0x1CU
#define ST25_REG_IRQ_TARGET 0x1DU
#define ST25_REG_PASSIVE_TARGET_STATUS 0x21U
#define ST25_REG_TIMER_NFC 0x1BU
#define ST25_REG_CORR_CONF1 (ST25_SPACE_B | 0x0CU)
#define ST25_REG_CORR_CONF2 (ST25_SPACE_B | 0x0DU)
#define ST25_REG_NFCIP_BIT_RATE 0x24U
#define ST25_REG_FIFO_STATUS1 0x1EU
#define ST25_REG_FIFO_STATUS2 0x1FU
#define ST25_REG_NUM_TX_BYTES1 0x22U
#define ST25_REG_NUM_TX_BYTES2 0x23U
#define ST25_REG_ANT_TUNE_A 0x26U
#define ST25_REG_ANT_TUNE_B 0x27U
#define ST25_REG_TX_DRIVER 0x28U
#define ST25_REG_PT_MOD 0x29U
#define ST25_REG_FIELD_THRESHOLD_ACTV 0x2AU
#define ST25_REG_FIELD_THRESHOLD_DEACTV 0x2BU
#define ST25_REG_OVERSHOOT_CONF1 (ST25_SPACE_B | 0x30U)
#define ST25_REG_OVERSHOOT_CONF2 (ST25_SPACE_B | 0x31U)
#define ST25_REG_UNDERSHOOT_CONF1 (ST25_SPACE_B | 0x32U)
#define ST25_REG_UNDERSHOOT_CONF2 (ST25_SPACE_B | 0x33U)
#define ST25_REG_AUX_MOD (ST25_SPACE_B | 0x28U)
#define ST25_REG_RES_AM_MOD (ST25_SPACE_B | 0x2AU)
#define ST25_REG_AUX_DISPLAY 0x31U
#define ST25_REG_IC_IDENTITY 0x3FU

#define ST25_OP_EN 0x80U
#define ST25_OP_RX_EN 0x40U
#define ST25_OP_TX_EN 0x08U
#define ST25_MODE_POLL_NFCA 0x08U
#define ST25_MODE_BITRATE_DETECT 0xC8U
#define ST25_MODE_LISTEN_NFCA 0x88U
#define ST25_MODE_MASK 0xFBU
#define ST25_AUX_NO_CRC_RX 0x80U
#define ST25_NFCA_NO_TX_PAR 0x80U
#define ST25_NFCA_NO_RX_PAR 0x40U
#define ST25_NFCA_ANTCL 0x01U
#define ST25_AUX_DISPLAY_OSC_OK 0x10U
#define ST25_AUX_DISPLAY_EFD 0x40U
#define ST25_AUX_NFC_ID_7 0x10U
#define ST25_AUX_NFC_ID_10 0x20U
#define ST25_MODE_TARGET_NFCA 0x88U
#define ST25_OP_FD_AUTO_EFD 0x03U
#define ST25_PT_FDEL_2 0x40U
#define ST25_PT_FDEL_0 0x10U
#define ST25_PT_FDEL (ST25_PT_FDEL_2 | ST25_PT_FDEL_0)
#define ST25_PT_D_AC_AP2P 0x08U
#define ST25_PT_D_212_424_1R 0x04U
#define ST25_PT_D_106_AC_A 0x01U
#define ST25_PT_CONFIG_AUTO_NFCA (ST25_PT_FDEL | ST25_PT_D_AC_AP2P | ST25_PT_D_212_424_1R)
#define ST25_PT_CONFIG_MANUAL_NFCA (ST25_PT_FDEL | ST25_PT_D_106_AC_A)
#define ST25_TIMER_MRT_STEP_512 0x08U
#define ST25_IRQ_TARGET_WU_A 0x01UL
#define ST25_IRQ_TARGET_WU_A_X 0x02UL
#define ST25_IRQ_TARGET_RXE_PTA 0x10UL
#define ST25_IRQ_TARGET_APON 0x20UL

/* IRQ value follows the official M5Stack/RFAL packing:
 *   bits 31..24 = MAIN (0x1A)
 *   bits 23..16 = TIMER/NFC (0x1B)
 *   bits 15..8  = ERROR/WUP (0x1C)
 *   bits 7..0   = PASSIVE TARGET (0x1D)
 * Reading all four status bytes clears the latched IRQs. */
#define ST25_IRQ_OSC (0x80UL << 24)
#define ST25_IRQ_FWL (0x40UL << 24)
#define ST25_IRQ_RXS (0x20UL << 24)
#define ST25_IRQ_RXE (0x10UL << 24)
#define ST25_IRQ_TXE (0x08UL << 24)
#define ST25_IRQ_COL (0x04UL << 24)
#define ST25_IRQ_NRE (0x40UL << 16)
#define ST25_IRQ_EON (0x10UL << 16)
#define ST25_IRQ_EOF (0x08UL << 16)
#define ST25_IRQ_NFCT (0x01UL << 16)
#define ST25_IRQ_CRC (0x80UL << 8)
#define ST25_IRQ_PAR (0x40UL << 8)
#define ST25_IRQ_ERR1 (0x10UL << 8)
#define ST25_IRQ_ERR2 (0x20UL << 8)
#define ST25_IRQ_TARGET_WU_MASK 0x03UL
#define ST25_IRQ_TARGET_RXE_PTA_MASK 0x10UL
#define ST25_IRQ_TARGET_APON_MASK 0x20UL

#define NFC_EVENT_ALL_BITS (FuriHalNfcEventOscOn | FuriHalNfcEventFieldOn | \
                            FuriHalNfcEventFieldOff | FuriHalNfcEventListenerActive | \
                            FuriHalNfcEventTxStart | FuriHalNfcEventTxEnd | \
                            FuriHalNfcEventRxStart | FuriHalNfcEventRxEnd | \
                            FuriHalNfcEventCollision | FuriHalNfcEventTimerFwtExpired | \
                            FuriHalNfcEventTimerBlockTxExpired | FuriHalNfcEventTimeout | \
                            FuriHalNfcEventAbortRequest)

static bool st25_ready;
static bool st25_custom_parity;
/* Human-readable reason for the last failed init, shown by the NFC app.
 * Serial console is not available while TinyUSB owns the USB PHY, so the
 * diagnostic has to reach the user through the UI. */
static char st25_diag[96] = "init not run";
static FuriHalNfcMode st25_mode = FuriHalNfcModeNum;
static FuriHalNfcTech st25_tech = FuriHalNfcTechInvalid;
static FuriMutex* st25_mutex;
static FuriEventFlag* st25_events;
static esp_timer_handle_t st25_fwt_timer;
static esp_timer_handle_t st25_block_timer;
static volatile bool st25_block_running;
static uint8_t st25_rx[ST25_FIFO_DEPTH];
static size_t st25_rx_bits;
static bool st25_listener_active;
static bool st25_listener_ready;
static bool st25_listener_manual_rx;
static bool st25_listener_rx_raw_parity;
/* Bitrate detection is reported before the passive-target engine reaches
 * Ready.  Keep it until RXE_PTA, which is the hardware hand-off point used
 * by the official M5Stack/RFAL listener state machine. */
static uint8_t st25_listener_bitrate = 0xFFU;
static uint8_t st25_listener_uid[10];
static uint8_t st25_listener_uid_len;

/* Opening the ESP32-S3 USB serial device resets the chip, so live logs cannot
 * reliably capture a hands-on NFC test. Keep the last listener observations in
 * RTC fast memory and print them after the USB-triggered reset. */
#define ST25_RTC_DIAG_MAGIC 0x4E464344UL /* "NFCD" */
typedef struct {
    uint32_t magic;
    uint32_t samples;
    uint32_t irq_or;
    uint32_t irq_count;
    uint32_t wait_calls;
    uint32_t wait_returns;
    uint32_t last_event;
    uint32_t abort_origin;
    uint8_t started;
    uint8_t efd_seen;
    uint8_t aux;
    uint8_t op;
    uint8_t mode;
    uint8_t pta;
    uint8_t bitrate;
    uint8_t threshold_actv;
    uint8_t threshold_deactv;
    uint8_t masks[4];
} St25RtcListenerDiag;

static RTC_NOINIT_ATTR St25RtcListenerDiag st25_rtc_diag;

static void st25_rtc_diag_print_and_clear(void) {
    if(st25_rtc_diag.magic == ST25_RTC_DIAG_MAGIC && st25_rtc_diag.started) {
        FURI_LOG_I(
            TAG,
            "Previous listener: stage=%u waits=%lu returns=%lu last=%08lX abort=%08lX samples=%lu irq_count=%lu irq_or=%08lX EFDseen=%u AUX=%02X OP=%02X MODE=%02X PTA=%02X BR=%02X TH=%02X/%02X MASK=%02X%02X%02X%02X",
            st25_rtc_diag.started,
            (unsigned long)st25_rtc_diag.wait_calls,
            (unsigned long)st25_rtc_diag.wait_returns,
            (unsigned long)st25_rtc_diag.last_event,
            (unsigned long)st25_rtc_diag.abort_origin,
            (unsigned long)st25_rtc_diag.samples,
            (unsigned long)st25_rtc_diag.irq_count,
            (unsigned long)st25_rtc_diag.irq_or,
            st25_rtc_diag.efd_seen,
            st25_rtc_diag.aux,
            st25_rtc_diag.op,
            st25_rtc_diag.mode,
            st25_rtc_diag.pta,
            st25_rtc_diag.bitrate,
            st25_rtc_diag.threshold_actv,
            st25_rtc_diag.threshold_deactv,
            st25_rtc_diag.masks[0],
            st25_rtc_diag.masks[1],
            st25_rtc_diag.masks[2],
            st25_rtc_diag.masks[3]);
    }
    memset(&st25_rtc_diag, 0, sizeof(st25_rtc_diag));
}
static uint8_t st25_listener_atqa[2];
static uint8_t st25_listener_sak;
static uint8_t st25_listener_pt[15];
static bool st25_is_b_variant;

static void st25_set_events(uint32_t events) {
    if(st25_events) furi_event_flag_set(st25_events, events);
}

static void st25_fwt_timer_cb(void* context) {
    UNUSED(context);
    st25_set_events(FuriHalNfcEventTimerFwtExpired);
}

static void st25_block_timer_cb(void* context) {
    UNUSED(context);
    st25_block_running = false;
    st25_set_events(FuriHalNfcEventTimerBlockTxExpired);
}

static esp_err_t st25_write_raw(const uint8_t* data, size_t len) {
    esp_err_t err = ESP_FAIL;
    /* The first transaction after power-up/idle may NACK; retry like the PM1. */
    for(unsigned i = 0; i < 5; ++i) {
        err = i2c_master_write_to_device(
            BOARD_NFC_I2C_PORT, ST25_ADDR, data, len, pdMS_TO_TICKS(ST25_TIMEOUT_MS));
        if(err == ESP_OK) return err;
        furi_delay_ms(5);
    }
    return err;
}

static esp_err_t st25_read_after(const uint8_t* command, size_t command_len, uint8_t* data, size_t len) {
    esp_err_t err = ESP_FAIL;
    for(unsigned i = 0; i < 5; ++i) {
        err = i2c_master_write_read_device(
            BOARD_NFC_I2C_PORT,
            ST25_ADDR,
            command,
            command_len,
            data,
            len,
            pdMS_TO_TICKS(ST25_TIMEOUT_MS));
        if(err == ESP_OK) return err;
        furi_delay_ms(5);
    }
    return err;
}

/* Diagnostic: log every address that ACKs, so a mis-wired or unpowered Unit
 * NFC can be told apart from a wrong I2C address. */
static void st25_scan_bus(void) {
    char found[48] = {0};
    for(uint8_t addr = 0x08; addr < 0x78; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(BOARD_NFC_I2C_PORT, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if(err == ESP_OK) {
            FURI_LOG_W(TAG, "I2C device ACK at 0x%02X", addr);
            if(strlen(found) < sizeof(found) - 6) {
                char part[8];
                snprintf(part, sizeof(part), "%02X ", addr);
                strcat(found, part);
            }
        }
    }
    snprintf(st25_diag, sizeof(st25_diag), "no ST25; bus ACK: %s",
        found[0] ? found : "none");
}

static bool st25_write_reg(uint8_t reg, uint8_t value) {
    if(reg & ST25_SPACE_B) {
        /* Space-B access is a two-byte I2C command prefix: first select
         * Space-B, then address the register without the space bit. */
        const uint8_t data[] = {ST25_SPACE_B_ACCESS, (uint8_t)(reg & 0x3FU), value};
        return st25_write_raw(data, sizeof(data)) == ESP_OK;
    }
    const uint8_t data[] = {reg, value};
    return st25_write_raw(data, sizeof(data)) == ESP_OK;
}

static bool st25_read_regs(uint8_t reg, uint8_t* values, size_t len) {
    if(reg & ST25_SPACE_B) {
        const uint8_t command[] = {ST25_SPACE_B_ACCESS, (uint8_t)((reg & 0x3FU) | 0x40U)};
        return st25_read_after(command, sizeof(command), values, len) == ESP_OK;
    }
    const uint8_t command = ST25_READ(reg);
    return st25_read_after(&command, 1, values, len) == ESP_OK;
}

static bool st25_read_reg(uint8_t reg, uint8_t* value) {
    return st25_read_regs(reg, value, 1);
}

static bool st25_command(uint8_t command) {
    /* Direct commands are encoded as operation mode 3 (0xC0). The command
     * constants already carry that mode, but keep the OR here explicit so a
     * future caller cannot accidentally issue a register operation. */
    command |= 0xC0U;
    return st25_write_raw(&command, 1) == ESP_OK;
}

static bool st25_write_fifo(const uint8_t* data, size_t len) {
    if(len > ST25_FIFO_DEPTH) return false;
    uint8_t buffer[ST25_FIFO_DEPTH + 1];
    buffer[0] = ST25_FIFO_LOAD;
    if(len) memcpy(&buffer[1], data, len);
    return st25_write_raw(buffer, len + 1) == ESP_OK;
}

static bool st25_read_fifo(uint8_t* data, size_t len) {
    const uint8_t command = ST25_FIFO_READ;
    return st25_read_after(&command, 1, data, len) == ESP_OK;
}

static bool st25_read_pt_memory(uint8_t* data, size_t len) {
    if(len > 15U) return false;
    const uint8_t command = 0xBFU;
    uint8_t response[16] = {0};
    if(st25_read_after(&command, 1, response, len + 1U) != ESP_OK) return false;
    memcpy(data, &response[1], len);
    return true;
}

static uint32_t st25_read_irq(void) {
    /* Reading MAIN clears the error/wakeup status on ST25R3916, therefore
     * ERROR must be captured first. IRQ status registers are at 0x1A..0x1D;
     * the 0x18/0x19 constants below are mask registers, not status aliases. */
    uint8_t error = 0;
    uint8_t timer_nfc = 0;
    uint8_t main = 0;
    uint8_t target = 0;
    if(!st25_read_reg(ST25_REG_IRQ_ERROR_WUP, &error) ||
       !st25_read_reg(ST25_REG_IRQ_MAIN, &main) ||
       !st25_read_reg(ST25_REG_IRQ_TIMER_NFC, &timer_nfc) ||
       !st25_read_reg(ST25_REG_IRQ_TARGET, &target))
        return 0;
    /* Match UnitST25R3916::readInterrupts(): MAIN is the high byte,
     * TIMER/NFC follows it, ERROR/WUP is next, TARGET is low byte. */
    return ((uint32_t)main << 24) | ((uint32_t)timer_nfc << 16) |
           ((uint32_t)error << 8) | (uint32_t)target;
}

static bool st25_read_rx_fifo(void) {
    uint8_t status[2];
    if(!st25_read_regs(ST25_REG_FIFO_STATUS1, status, sizeof(status))) return false;
    size_t bytes = status[0] | ((size_t)(status[1] & 0xC0U) << 2);
    uint8_t last_bits = (status[1] >> 1) & 0x07U;
    if(bytes > sizeof(st25_rx)) return false;
    if(bytes && !st25_read_fifo(st25_rx, bytes)) return false;
    st25_rx_bits = bytes ? ((last_bits == 0) ? bytes * 8 : (bytes - 1) * 8 + last_bits) : 0;

    /* Crypto1 listener frames contain one encrypted parity bit after every
     * byte. Strip those parity bits before exposing the data to the
     * byte-oriented listener protocol. Crypto1 decryption itself consumes the
     * data bytes; custom parity is required on the transmit path. */
    if(st25_listener_rx_raw_parity && st25_rx_bits >= 9 && (st25_rx_bits % 9U) == 0U) {
        uint8_t raw[ST25_FIFO_DEPTH];
        memcpy(raw, st25_rx, bytes);
        size_t raw_bit = 0;
        size_t out_byte = 0;
        size_t count = st25_rx_bits / 9U;
        for(size_t i = 0; i < count; i++) {
            uint8_t value = 0;
            for(uint8_t b = 0; b < 8U; b++, raw_bit++) {
                if(raw[raw_bit / 8U] & (1U << (raw_bit % 8U)))
                    value |= (uint8_t)(1U << b);
            }
            raw_bit++;
            st25_rx[out_byte++] = value;
        }
        st25_rx_bits = out_byte * 8U;
    }

    /* The first MIFARE Classic AUTH command is a normal 2-byte frame. Once it
     * has been observed, subsequent {NR,AR}/data frames carry encrypted
     * parity and must be delivered as the raw 9-bit-per-byte stream expected
     * by the Crypto1 listener. The generic listener API has no separate
     * "receive custom parity" switch, so arm it at this protocol boundary. */
    if(st25_mode == FuriHalNfcModeListener && st25_rx_bits == 16 &&
       (st25_rx[0] == 0x60U || st25_rx[0] == 0x61U)) {
        st25_listener_rx_raw_parity = true;
        st25_write_reg(ST25_REG_ISO14443A_NFC, ST25_NFCA_NO_RX_PAR);
    }
    FURI_LOG_D(TAG, "RX FIFO: irq-bytes=%u last=%u bits=%u", (unsigned)bytes,
        (unsigned)last_bits, (unsigned)st25_rx_bits);
    return true;
}

static FuriHalNfcError st25_wait_for_rx(uint32_t timeout_ms) {
    uint32_t start = furi_get_tick();
    bool tx_seen = false;
    while((furi_get_tick() - start) < timeout_ms) {
        uint32_t irq = st25_read_irq();
        if(irq & ST25_IRQ_TXE) {
            tx_seen = true;
            st25_set_events(FuriHalNfcEventTxEnd);
        }
        if(irq & ST25_IRQ_RXS) st25_set_events(FuriHalNfcEventRxStart);
        if(irq & ST25_IRQ_COL) {
            FURI_LOG_W(TAG, "RX collision irq=%08lX", (unsigned long)irq);
            st25_read_rx_fifo();
            st25_set_events(FuriHalNfcEventTxEnd | FuriHalNfcEventCollision);
            return FuriHalNfcErrorIncompleteFrame;
        }
        if(irq & ST25_IRQ_RXE) {
            if(!st25_read_rx_fifo()) {
                FURI_LOG_W(TAG, "RXE but FIFO read failed irq=%08lX", (unsigned long)irq);
                return FuriHalNfcErrorCommunication;
            }
            FURI_LOG_D(TAG, "RX complete irq=%08lX bits=%u", (unsigned long)irq,
                (unsigned)st25_rx_bits);
            st25_set_events(FuriHalNfcEventTxEnd | FuriHalNfcEventRxStart | FuriHalNfcEventRxEnd);
            /* Do not turn ST25 CRC/PAR status into a HAL failure here.
             * MIFARE Classic AUTH returns a 4-byte NT without a normal CRC,
             * and Crypto1 frames use software/custom parity. The FIFO data
             * must reach the ISO14443/MIFARE layer, which can distinguish
             * AUTH NT, encrypted frames, and a genuinely invalid response.
             * Reporting DataFormat here makes the generic NFC layer convert
             * the exchange into CardLost before MIFARE can inspect it. */
            return FuriHalNfcErrorNone;
        }
        if(irq & ST25_IRQ_NRE) {
            st25_set_events(FuriHalNfcEventTxEnd | FuriHalNfcEventTimerFwtExpired);
            return FuriHalNfcErrorCommunicationTimeout;
        }
        furi_delay_ms(1);
    }
    FURI_LOG_W(TAG, "RX timeout irq-last tx=%u elapsed=%lu ms", tx_seen ? 1U : 0U,
        (unsigned long)(furi_get_tick() - start));
    st25_set_events((tx_seen ? FuriHalNfcEventTxEnd : 0) | FuriHalNfcEventTimerFwtExpired);
    return FuriHalNfcErrorCommunicationTimeout;
}

static FuriHalNfcError st25_transceive(
    const uint8_t* tx_data,
    size_t tx_bits,
    bool is_sdd,
    bool custom_parity) {
    if(!st25_ready || st25_mode != FuriHalNfcModePoller ||
       st25_tech != FuriHalNfcTechIso14443a) return FuriHalNfcErrorCommunication;
    if(!tx_data || tx_bits == 0 || tx_bits > ST25_FIFO_DEPTH * 8) return FuriHalNfcErrorDataFormat;

    st25_rx_bits = 0;
    if(!st25_command(ST25_CMD_STOP) || !st25_command(ST25_CMD_CLEAR_FIFO))
        return FuriHalNfcErrorCommunication;

    /* Appended CRC belongs to the upper Flipper stack; preserve RX CRC too. */
    /* The current I2C backend returns FIFO data without a reliable per-byte
     * parity stream. Keep the previous conservative mode until the ST25
     * FIFO/parity path is implemented end-to-end. */
    uint8_t nfca = custom_parity ? (ST25_NFCA_NO_TX_PAR | ST25_NFCA_NO_RX_PAR) : 0;
    if(is_sdd) nfca |= ST25_NFCA_ANTCL;
    if(!st25_write_reg(ST25_REG_ISO14443A_NFC, nfca) ||
       !st25_write_reg(ST25_REG_AUX, ST25_AUX_NO_CRC_RX) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES1, (uint8_t)(tx_bits >> 8)) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES2, (uint8_t)tx_bits) ||
       !st25_write_fifo(tx_data, (tx_bits + 7) / 8) || !st25_command(ST25_CMD_TX_NO_CRC)) {
        return FuriHalNfcErrorCommunication;
    }

    FuriHalNfcError error = st25_wait_for_rx(100);

    /* Match RFAL's rfalCleanupTransceive(): no_tx_par, no_rx_par and antcl
     * are transaction-local settings. Leaving them set after a Crypto1 frame
     * corrupts every following REQA/WUPA/SDD frame, because those frames are
     * then transmitted and received without normal ISO14443-A parity. */
    if(!st25_write_reg(ST25_REG_ISO14443A_NFC, 0)) {
        return FuriHalNfcErrorCommunication;
    }
    st25_custom_parity = false;
    return error;
}

static FuriHalNfcError st25_configure_listener_nfca(void) {
    /* Bruce's working implementation uses the ST25 passive-target engine:
     * the chip performs NFC-A SENS/anticollision/SELECT from PT Memory, while
     * the protocol stack handles frames after activation. */
    /* Match Bruce/RFAL's listen setup.  In particular, the target IRQ
     * mask alone is not enough: EON/EOF/NRE live in the timer IRQ mask and
     * PAR/CRC/ERR1/ERR2 live in the error IRQ mask. */
    /* Passive Target NFC-A 必须是 MODE.targ | om_targ_nfca = 0x88。
     * 旧的 0xC8 会选择错误的目标调制模式，导致手机外场不会触发
     * EFD 或 Passive Target IRQ。 */
    if(!st25_write_reg(ST25_REG_MODE, ST25_MODE_LISTEN_NFCA) ||
       !st25_write_reg(ST25_REG_BIT_RATE, 0x00) ||
       /* Listener analog setup from ST25/RFAL's LISTEN_NFCA path. The
        * poller values (ANTL=0x80/0x40, RX_CONF1=0x08, RX_CONF2=0x2D)
        * are not valid for a passive target and can prevent the Unit NFC
        * from seeing the reader's field/modulation. */
       /* RFAL CHIP_LISTEN_ON values differ by silicon revision: plain
        * ST25R3916 uses 00/E0, while ST25R3916B uses 00/00. */
       !st25_write_reg(ST25_REG_ANT_TUNE_A, 0x00) ||
       !st25_write_reg(ST25_REG_ANT_TUNE_B, st25_is_b_variant ? 0x00 : 0xE0) ||
       /* Exact values after RFAL's CHIP_LISTEN_ON changes: RX_CONF1
        * selects the 12..200 kHz path; RX_CONF2 preserves the poll-common
        * AGC/squelch settings and changes only amd_sel to mixer. */
       !st25_write_reg(ST25_REG_RX_CONF1, 0x01) ||
       !st25_write_reg(ST25_REG_RX_CONF2, 0x6D) ||
       !st25_write_reg(ST25_REG_RX_CONF3, 0xD8) ||
       !st25_write_reg(ST25_REG_RX_CONF4, 0x22) ||
       !st25_write_reg(ST25_REG_CORR_CONF1, 0x47) ||
       !st25_write_reg(ST25_REG_CORR_CONF2, 0x00) ||
       /* RFAL 的 CHIP_LISTEN_ON 会关闭过冲/欠冲保护。扫描模式遗留的
        * 保护滤波会削弱边缘耦合时手机发出的 ASK 调制，监听模式必须
        * 显式清零，不能依赖上一次模式或芯片复位后的偶然默认值。 */
       !st25_write_reg(ST25_REG_OVERSHOOT_CONF1, 0x00) ||
       !st25_write_reg(ST25_REG_OVERSHOOT_CONF2, 0x00) ||
       !st25_write_reg(ST25_REG_UNDERSHOOT_CONF1, 0x00) ||
       !st25_write_reg(ST25_REG_UNDERSHOOT_CONF2, 0x00) ||
       /* Internal load modulation and card-mode PT modulation. RFAL uses
        * PT_MOD=0x51 on ST25R3916 and 0x2E on ST25R3916B. */
       !st25_write_reg(ST25_REG_AUX_MOD, 0x10) ||
       !st25_write_reg(ST25_REG_PT_MOD, st25_is_b_variant ? 0x2E : 0x51) ||
       !st25_write_reg(ST25_REG_RES_AM_MOD, 0x80) ||
       !st25_write_reg(ST25_REG_TX_DRIVER, 0x30) ||
       /* UnitST25R3916::begin() explicitly programs the external-field
        * detector. Set-default alone leaves board/silicon-dependent values
        * which can make a phone field too weak to wake passive-target mode. */
       /* 降低外部场检测门限：采用本地 RFAL 默认模拟表的 105mV/75mV
        * 组合，提高手机场强偏弱或线圈耦合偏边缘时进入被动目标监听的概率。 */
       !st25_write_reg(ST25_REG_FIELD_THRESHOLD_ACTV, 0x11) ||
       !st25_write_reg(ST25_REG_FIELD_THRESHOLD_DEACTV, 0x00) ||
       !st25_write_reg(ST25_REG_OP_CONTROL, ST25_OP_EN | ST25_OP_RX_EN | ST25_OP_FD_AUTO_EFD) ||
       /* 与已验证的 Unit NFC 流程一致：被动目标进入 Sense 前使用
        * 512/fc MRT 步进和 0x02 接收掩码，避免自定义值改变自动防冲突。 */
       !st25_write_reg(ST25_REG_TIMER_AND_EMV, ST25_TIMER_MRT_STEP_512) ||
       !st25_write_reg(ST25_REG_MASK_RX_TIMER, 0x02) ||
       /* d_106_ac_a must stay CLEAR while entering Sense: clear means the
        * ST25 passive-target engine automatically handles REQA/WUPA,
        * anticollision and SELECT at 106 kbit/s. Setting it here disables
        * automatic NFC-A anticollision, so a phone cannot even select the
        * emulated UID. It is set only after WU_A, when software takes over
        * the post-activation Type-2 commands. */
       !st25_write_reg(ST25_REG_PASSIVE_TARGET, ST25_PT_CONFIG_AUTO_NFCA) ||
       !st25_write_reg(ST25_REG_ISO14443A_NFC, 0x00) ||
       /* IRQ mask bits are active-high (1 = masked). Keep the same enabled
        * source set as M5's default_irq + mode_irq. */
       !st25_write_reg(ST25_REG_IRQ_MASK_MAIN,
           (uint8_t)~((ST25_IRQ_FWL | ST25_IRQ_RXS | ST25_IRQ_RXE | ST25_IRQ_TXE) >> 24)) ||
       !st25_write_reg(ST25_REG_IRQ_MASK_TIMER_NFC,
           (uint8_t)~((ST25_IRQ_NRE | ST25_IRQ_EON | ST25_IRQ_EOF | ST25_IRQ_NFCT) >> 16)) ||
       !st25_write_reg(ST25_REG_IRQ_MASK_ERROR_WUP,
           (uint8_t)~((ST25_IRQ_CRC | ST25_IRQ_PAR | ST25_IRQ_ERR1 | ST25_IRQ_ERR2) >> 8)) ||
       !st25_write_reg(ST25_REG_IRQ_MASK_TARGET,
           (uint8_t)~(ST25_IRQ_TARGET_WU_MASK | ST25_IRQ_TARGET_RXE_PTA_MASK |
                      ST25_IRQ_TARGET_APON_MASK)))
        return FuriHalNfcErrorCommunication;
    if(!st25_command(ST25_CMD_STOP) || !st25_command(ST25_CMD_CLEAR_FIFO) ||
       !st25_command(ST25_CMD_UNMASK_RECEIVE_DATA))
        return FuriHalNfcErrorCommunication;
    /* Reading all four status bytes clears stale IRQs before GOTO_SENSE. */
    st25_read_irq();

    uint8_t mode = 0;
    uint8_t op = 0;
    uint8_t pt = 0;
    uint8_t aux_mod = 0;
    uint8_t res_am = 0;
    uint8_t rx1 = 0;
    uint8_t rx2 = 0;
    uint8_t temv = 0;
    uint8_t mrt = 0;
    if(st25_read_reg(ST25_REG_MODE, &mode) && st25_read_reg(ST25_REG_OP_CONTROL, &op) &&
       st25_read_reg(ST25_REG_PASSIVE_TARGET, &pt) &&
       st25_read_reg(ST25_REG_AUX_MOD, &aux_mod) &&
       st25_read_reg(ST25_REG_RES_AM_MOD, &res_am) &&
       st25_read_reg(ST25_REG_RX_CONF1, &rx1) && st25_read_reg(ST25_REG_RX_CONF2, &rx2) &&
       st25_read_reg(ST25_REG_TIMER_AND_EMV, &temv) && st25_read_reg(ST25_REG_MASK_RX_TIMER, &mrt)) {
        FURI_LOG_I(TAG,
            "Listener cfg MODE=%02X OP=%02X PT=%02X AUXMOD=%02X RESAM=%02X RX=%02X/%02X TEMV=%02X MRT=%02X",
            mode, op, pt, aux_mod, res_am, rx1, rx2, temv, mrt);
    } else {
        FURI_LOG_E(TAG, "Listener cfg readback failed");
        return FuriHalNfcErrorCommunication;
    }
    return FuriHalNfcErrorNone;
}

static FuriHalNfcError st25_configure_nfca(void) {
    /* Settings are the 106kb/s NFCA subset from ST's RFAL default table.
     * Unit NFC's antenna matching is fixed in hardware; do not run AAT. */
    if(!st25_write_reg(ST25_REG_MODE, ST25_MODE_POLL_NFCA) ||
       !st25_write_reg(ST25_REG_BIT_RATE, 0x00) ||
       !st25_write_reg(ST25_REG_TX_DRIVER, 0x30) ||
       !st25_write_reg(ST25_REG_ANT_TUNE_A, 0x80) ||
       !st25_write_reg(ST25_REG_ANT_TUNE_B, 0x40) ||
       !st25_write_reg(ST25_REG_RX_CONF1, 0x08) ||
       !st25_write_reg(ST25_REG_RX_CONF2, 0x2D) ||
       !st25_write_reg(ST25_REG_RX_CONF3, 0x00) ||
       !st25_write_reg(ST25_REG_RX_CONF4, 0x00) ||
       /* Clear transaction-local anticol/custom-parity state left by an
        * interrupted exchange before enabling a fresh NFC-A session. */
       !st25_write_reg(ST25_REG_ISO14443A_NFC, 0x00) ||
       !st25_write_reg(ST25_REG_AUX, 0x00)) return FuriHalNfcErrorCommunication;
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_init(void) {
    if(st25_ready) return FuriHalNfcErrorNone;

    st25_rtc_diag_print_and_clear();

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BOARD_PIN_NFC_SDA,
        .scl_io_num = BOARD_PIN_NFC_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BOARD_NFC_I2C_FREQ_HZ,
    };
    esp_err_t error = i2c_param_config(BOARD_NFC_I2C_PORT, &conf);
    if(error != ESP_OK) return FuriHalNfcErrorCommunication;
    error = i2c_driver_install(BOARD_NFC_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if(error != ESP_OK && error != ESP_ERR_INVALID_STATE) return FuriHalNfcErrorCommunication;

    uint8_t chip_id = 0;
    if(!st25_read_reg(ST25_REG_IC_IDENTITY, &chip_id) ||
       ((chip_id & 0xF8U) != 0x28U && (chip_id & 0xF8U) != 0x30U)) {
        FURI_LOG_E(TAG, "ST25R3916 not found at 0x%02X (ID=%02X)", ST25_ADDR, chip_id);
        snprintf(st25_diag, sizeof(st25_diag), "no ST25 at 0x50 (ID=%02X)", chip_id);
        st25_scan_bus();
        return FuriHalNfcErrorCommunication;
    }

    st25_is_b_variant = (chip_id & 0xF8U) == 0x30U;

    if(!st25_command(ST25_CMD_SET_DEFAULT) || !st25_write_reg(ST25_REG_IO_CONF1, 0x07) ||
       !st25_write_reg(ST25_REG_OP_CONTROL, ST25_OP_EN)) return FuriHalNfcErrorCommunication;
    /* Match the M5Stack/ST25 reference startup: calibrate the internal
     * regulators before relying on the RF load-modulation path. */
    if(!st25_command(ST25_CMD_ADJUST_REGULATORS)) return FuriHalNfcErrorCommunication;
    furi_delay_ms(5);
    for(uint8_t i = 0; i < 20; i++) {
        uint8_t aux;
        if(st25_read_reg(ST25_REG_AUX_DISPLAY, &aux) && (aux & ST25_AUX_DISPLAY_OSC_OK)) break;
        furi_delay_ms(1);
        if(i == 19) return FuriHalNfcErrorOscillator;
    }
    if(st25_configure_nfca() != FuriHalNfcErrorNone) return FuriHalNfcErrorCommunication;

    st25_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    st25_events = furi_event_flag_alloc();
    const esp_timer_create_args_t fwt_args = {.callback = st25_fwt_timer_cb, .name = "st25_fwt"};
    const esp_timer_create_args_t block_args = {.callback = st25_block_timer_cb, .name = "st25_blk"};
    esp_timer_create(&fwt_args, &st25_fwt_timer);
    esp_timer_create(&block_args, &st25_block_timer);
    st25_ready = true;
    snprintf(st25_diag, sizeof(st25_diag), "ST25R3916%s OK (ID=%02X)",
        (chip_id & 0xF8U) == 0x30U ? "B" : "", chip_id);
    FURI_LOG_I(TAG, "ST25R3916%s detected at 0x50 on I2C%d",
        (chip_id & 0xF8U) == 0x30U ? "B" : "", BOARD_NFC_I2C_PORT);
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_is_hal_ready(void) {
    return st25_ready ? FuriHalNfcErrorNone : FuriHalNfcErrorCommunication;
}

const char* furi_hal_nfc_st25_last_diag(void) {
    return st25_diag;
}

FuriHalNfcError furi_hal_nfc_acquire(void) {
    if(!st25_ready || !st25_mutex) return FuriHalNfcErrorCommunication;
    furi_check(furi_mutex_acquire(st25_mutex, FuriWaitForever) == FuriStatusOk);
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_release(void) {
    if(st25_mutex) furi_check(furi_mutex_release(st25_mutex) == FuriStatusOk);
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_low_power_mode_start(void) {
    if(!st25_ready) return FuriHalNfcErrorNone;
    st25_write_reg(ST25_REG_OP_CONTROL, ST25_OP_EN);
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_low_power_mode_stop(void) {
    if(!st25_ready) return FuriHalNfcErrorCommunication;
    return st25_configure_nfca();
}

FuriHalNfcError furi_hal_nfc_set_mode(FuriHalNfcMode mode, FuriHalNfcTech tech) {
    if(!st25_ready) return FuriHalNfcErrorCommunication;
    st25_mode = mode;
    st25_tech = tech;
    st25_rx_bits = 0;
    st25_custom_parity = false;
    if(mode == FuriHalNfcModePoller && tech == FuriHalNfcTechIso14443a)
        return st25_configure_nfca();
    if(mode == FuriHalNfcModeListener && tech == FuriHalNfcTechIso14443a)
        return st25_configure_listener_nfca();
    FURI_LOG_W(TAG, "Mode %u / tech %u is not available in Unit NFC backend", mode, tech);
    return FuriHalNfcErrorCommunication;
}

FuriHalNfcError furi_hal_nfc_reset_mode(void) {
    if(st25_ready) furi_hal_nfc_low_power_mode_start();
    st25_mode = FuriHalNfcModeNum;
    st25_tech = FuriHalNfcTechInvalid;
    st25_rx_bits = 0;
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_field_detect_start(void) { return FuriHalNfcErrorNone; }
FuriHalNfcError furi_hal_nfc_field_detect_stop(void) { return FuriHalNfcErrorNone; }
bool furi_hal_nfc_field_is_present(void) {
    uint8_t aux;
    return st25_ready && st25_read_reg(ST25_REG_AUX_DISPLAY, &aux) && (aux & ST25_AUX_DISPLAY_EFD);
}

FuriHalNfcError furi_hal_nfc_poller_field_on(void) {
    if(!st25_ready) return FuriHalNfcErrorCommunication;
    if(!st25_write_reg(ST25_REG_OP_CONTROL, ST25_OP_EN | ST25_OP_TX_EN | ST25_OP_RX_EN))
        return FuriHalNfcErrorCommunication;
    furi_delay_ms(5);
    return FuriHalNfcErrorNone;
}

FuriHalNfcEvent furi_hal_nfc_poller_wait_event(uint32_t timeout_ms) {
    if(!st25_events) return FuriHalNfcEventTimeout;
    uint32_t flags = furi_event_flag_wait(st25_events, NFC_EVENT_ALL_BITS,
        FuriFlagWaitAny | FuriFlagNoClear, timeout_ms);
    if(flags & FuriFlagError) return FuriHalNfcEventTimeout;
    furi_event_flag_clear(st25_events, flags & NFC_EVENT_ALL_BITS);
    return (FuriHalNfcEvent)(flags & NFC_EVENT_ALL_BITS);
}

FuriHalNfcEvent furi_hal_nfc_listener_wait_event(uint32_t timeout_ms) {
    st25_rtc_diag.wait_calls++;
    if(st25_rtc_diag.magic == ST25_RTC_DIAG_MAGIC && st25_rtc_diag.started < 3)
        st25_rtc_diag.started = 3; /* HAL wait entered */
    if(!st25_events) {
        st25_rtc_diag.wait_returns++;
        st25_rtc_diag.last_event = FuriHalNfcEventTimeout;
        return FuriHalNfcEventTimeout;
    }

    /* There is no exposed IRQ wire on the Grove connector. Polling the four
     * IRQ status registers is slower than a real IRQ, but it is sufficient for
     * the PT activation and frame path used by the ST25R3916 Unit. */
    uint32_t start = furi_get_tick();
    uint32_t last_diag = start;
    while(timeout_ms == FURI_HAL_NFC_EVENT_WAIT_FOREVER ||
          (furi_get_tick() - start) < timeout_ms) {
        uint32_t pending = furi_event_flag_wait(
            st25_events, NFC_EVENT_ALL_BITS, FuriFlagWaitAny | FuriFlagNoClear, 0);
        /* 空轮询会返回 FuriFlagErrorTimeout（0xFFFFFFFE），而不是 0。
         * 该负错误码的 bit 12 恰好也是 AbortRequest；若先做按位判断，
         * 每次没有事件都会被错误当作“请求中止”。 */
        if(!(pending & FuriFlagError) && (pending & FuriHalNfcEventAbortRequest)) {
            furi_event_flag_clear(st25_events, FuriHalNfcEventAbortRequest);
            st25_rtc_diag.wait_returns++;
            st25_rtc_diag.last_event = FuriHalNfcEventAbortRequest;
            return FuriHalNfcEventAbortRequest;
        }

        uint32_t irq = st25_read_irq();
        if(irq) {
            uint8_t pta = 0;
            uint8_t br = 0;
            st25_read_reg(ST25_REG_NFCIP_BIT_RATE, &br);
            st25_read_reg(ST25_REG_PASSIVE_TARGET_STATUS, &pta);
            st25_rtc_diag.irq_or |= irq;
            st25_rtc_diag.irq_count++;
            st25_rtc_diag.pta = pta;
            st25_rtc_diag.bitrate = br;
            FURI_LOG_I(TAG, "Listener IRQ=%08lX PTA=%02X BR=%02X", (unsigned long)irq, pta, br);
        }
        uint32_t now = furi_get_tick();
        if((now - last_diag) >= 1000U) {
            uint8_t aux = 0, op = 0, mode = 0, pta = 0, th_a = 0, th_d = 0;
            uint8_t masks[4] = {0};
            st25_read_reg(ST25_REG_AUX_DISPLAY, &aux);
            st25_read_reg(ST25_REG_OP_CONTROL, &op);
            st25_read_reg(ST25_REG_MODE, &mode);
            st25_read_reg(ST25_REG_PASSIVE_TARGET_STATUS, &pta);
            st25_read_reg(ST25_REG_FIELD_THRESHOLD_ACTV, &th_a);
            st25_read_reg(ST25_REG_FIELD_THRESHOLD_DEACTV, &th_d);
            st25_read_regs(ST25_REG_IRQ_MASK_MAIN, masks, sizeof(masks));
            st25_rtc_diag.samples++;
            st25_rtc_diag.aux = aux;
            st25_rtc_diag.op = op;
            st25_rtc_diag.mode = mode;
            st25_rtc_diag.pta = pta;
            st25_rtc_diag.threshold_actv = th_a;
            st25_rtc_diag.threshold_deactv = th_d;
            memcpy(st25_rtc_diag.masks, masks, sizeof(masks));
            if(aux & ST25_AUX_DISPLAY_EFD) st25_rtc_diag.efd_seen = 1;
            FURI_LOG_I(
                TAG,
                "Listener alive AUX=%02X(EFD=%u OSC=%u RX=%u) OP=%02X MODE=%02X PTA=%02X TH=%02X/%02X MASK=%02X%02X%02X%02X",
                aux,
                (aux & ST25_AUX_DISPLAY_EFD) ? 1U : 0U,
                (aux & ST25_AUX_DISPLAY_OSC_OK) ? 1U : 0U,
                (aux & 0x08U) ? 1U : 0U,
                op,
                mode,
                pta,
                th_a,
                th_d,
                masks[0],
                masks[1],
                masks[2],
                masks[3]);
            last_diag = now;
        }
        FuriHalNfcEvent event = 0;
        /* The PT engine reports target events in the target byte, while RXE,
         * TXE and external-field events remain in their normal IRQ groups. */
        if(irq & ST25_IRQ_EON) event |= FuriHalNfcEventFieldOn;
        /* NFCT / RXE_PTA 都发生在硬件自动 NFC-A 防冲突期间。此前代码
         * 在这些中间事件中改写 MODE、BIT_RATE 和 AUX，会在 SELECT 前
         * 打断 Passive Target 状态机。此处只记录速率；真正交接必须等
         * WU_A/WU_A_X，和 Unit NFC 的已验证流程保持一致。 */
        if(irq & ST25_IRQ_NFCT) {
            uint8_t detected = 0;
            if(st25_read_reg(ST25_REG_NFCIP_BIT_RATE, &detected)) {
                st25_listener_bitrate = (detected >> 4) & 0x03U;
                FURI_LOG_D(TAG, "Listener bitrate detected=%u", st25_listener_bitrate);
            }
        }
        if(irq & (ST25_IRQ_TARGET_WU_A | ST25_IRQ_TARGET_WU_A_X)) {
            st25_listener_active = true;
            st25_listener_ready = false;
            st25_listener_manual_rx = true;
            st25_listener_bitrate = 0xFFU;
            /* WU_A/WU_A_X is the actual NFC-A activation hand-off. Stop
             * automatic anticollision responses only after this event. */
            if(!st25_write_reg(ST25_REG_PASSIVE_TARGET, ST25_PT_CONFIG_MANUAL_NFCA))
                FURI_LOG_W(TAG, "Failed to enter manual NFC-A RX");
            /* 自动 SELECT 已完成后，才交给上层保留原始 CRC 并处理
             * Type-2 命令；在此之前必须由芯片校验 SELECT 的 CRC。 */
            uint8_t aux = 0;
            if(st25_read_reg(ST25_REG_AUX, &aux)) {
                st25_write_reg(ST25_REG_AUX, aux | ST25_AUX_NO_CRC_RX);
            }
            event |= FuriHalNfcEventFieldOn | FuriHalNfcEventListenerActive;
        }
        /* APON only means that the target's field-on/anticollision-avoidance
         * phase completed.  It is not the reader's SELECT/ACTIVE indication.
         * Keep automatic NFC-A anticollision enabled here; disabling it on
         * APON prevents the reader from completing cascade/select.  The
         * actual hand-off to the software listener is WU_A/WU_A_X below. */
        if(irq & ST25_IRQ_TARGET_APON) {
            event |= FuriHalNfcEventFieldOn;
        }
        if(irq & ST25_IRQ_EOF) {
            st25_listener_active = false;
            st25_listener_ready = false;
            st25_listener_manual_rx = false;
            st25_listener_rx_raw_parity = false;
            event |= FuriHalNfcEventFieldOff;
        }
        /* Before WU_A, RXE_PTA belongs to the passive-target activation
         * state machine, not to the software listener.  Once WU_A has been
         * observed, only the normal RXE interrupt carries post-activation
         * Type-2/ISO14443 frames to the Flipper listener. */
        if((irq & ST25_IRQ_RXE) && st25_listener_active) {
            if(st25_read_rx_fifo()) event |= FuriHalNfcEventRxStart | FuriHalNfcEventRxEnd;
        }
        if(irq & ST25_IRQ_TXE) event |= FuriHalNfcEventTxEnd;
        if(irq & ST25_IRQ_COL) event |= FuriHalNfcEventCollision;
        if(event) {
            st25_rtc_diag.wait_returns++;
            st25_rtc_diag.last_event = event;
            return event;
        }

        if(timeout_ms != FURI_HAL_NFC_EVENT_WAIT_FOREVER &&
           (furi_get_tick() - start) >= timeout_ms) break;
        furi_delay_ms(1);
    }
    st25_rtc_diag.wait_returns++;
    st25_rtc_diag.last_event = FuriHalNfcEventTimeout;
    return FuriHalNfcEventTimeout;
}

FuriHalNfcError furi_hal_nfc_event_start(void) {
    if(st25_rtc_diag.magic == ST25_RTC_DIAG_MAGIC && st25_rtc_diag.started) {
        st25_rtc_diag.started = 2; /* listener worker entered */
    }
    FURI_LOG_I(TAG, "Listener worker entered event loop");
    if(st25_events) furi_event_flag_clear(st25_events, NFC_EVENT_ALL_BITS);
    return FuriHalNfcErrorNone;
}
FuriHalNfcError furi_hal_nfc_event_stop(void) { return FuriHalNfcErrorNone; }
FuriHalNfcError furi_hal_nfc_abort(void) { st25_set_events(FuriHalNfcEventAbortRequest); return FuriHalNfcErrorNone; }
FuriHalNfcError furi_hal_nfc_abort_with_origin(uintptr_t origin) {
    /* USB 打开串口会复位设备，调用点只能通过 RTC 诊断区带到下一次启动。 */
    if(st25_rtc_diag.magic == ST25_RTC_DIAG_MAGIC && st25_rtc_diag.started && origin &&
       !st25_rtc_diag.abort_origin) {
        st25_rtc_diag.abort_origin = (uint32_t)origin;
    }
    st25_set_events(FuriHalNfcEventAbortRequest);
    return FuriHalNfcErrorNone;
}

void furi_hal_nfc_timer_fwt_start(uint32_t time_fc) {
    if(!st25_fwt_timer) return;
    uint64_t us = ((uint64_t)time_fc * 1000U) / 13560U;
    esp_timer_stop(st25_fwt_timer);
    esp_timer_start_once(st25_fwt_timer, us < 10 ? 10 : us);
}
void furi_hal_nfc_timer_fwt_stop(void) { if(st25_fwt_timer) esp_timer_stop(st25_fwt_timer); }
void furi_hal_nfc_timer_block_tx_start(uint32_t time_fc) {
    uint64_t us = ((uint64_t)time_fc * 1000U) / 13560U;
    furi_hal_nfc_timer_block_tx_start_us(us < 10 ? 10 : us);
}
void furi_hal_nfc_timer_block_tx_start_us(uint32_t time_us) {
    if(!st25_block_timer) return;
    st25_block_running = true;
    esp_timer_stop(st25_block_timer);
    esp_timer_start_once(st25_block_timer, time_us < 10 ? 10 : time_us);
}
void furi_hal_nfc_timer_block_tx_stop(void) {
    if(st25_block_timer) esp_timer_stop(st25_block_timer);
    st25_block_running = false;
}
bool furi_hal_nfc_timer_block_tx_is_running(void) { return st25_block_running; }

FuriHalNfcError furi_hal_nfc_trx_reset(void) {
    st25_rx_bits = 0;
    st25_custom_parity = false;
    if(st25_events) furi_event_flag_clear(st25_events, NFC_EVENT_ALL_BITS);
    if(!st25_command(ST25_CMD_STOP) ||
       !st25_write_reg(ST25_REG_ISO14443A_NFC, 0)) {
        return FuriHalNfcErrorCommunication;
    }
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_poller_tx(const uint8_t* tx_data, size_t tx_bits) {
    return st25_transceive(tx_data, tx_bits, false, false);
}
FuriHalNfcError furi_hal_nfc_poller_rx(uint8_t* rx_data, size_t rx_data_size, size_t* rx_bits) {
    if(!rx_bits) return FuriHalNfcErrorDataFormat;
    size_t bytes = (st25_rx_bits + 7) / 8;
    if(bytes > rx_data_size) { *rx_bits = 0; return FuriHalNfcErrorBufferOverflow; }
    if(bytes && rx_data) memcpy(rx_data, st25_rx, bytes);
    *rx_bits = st25_rx_bits;
    st25_rx_bits = 0;
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_iso14443a_poller_trx_short_frame(FuriHalNfcaShortFrame frame) {
    if(!st25_ready || st25_mode != FuriHalNfcModePoller || st25_tech != FuriHalNfcTechIso14443a)
        return FuriHalNfcErrorCommunication;
    st25_rx_bits = 0;
    st25_custom_parity = false;
    if(!st25_command(ST25_CMD_STOP) || !st25_command(ST25_CMD_CLEAR_FIFO) ||
       /* Direct REQA/WUPA relies on the chip's normal ISO14443-A parity.
        * Clear state left by any aborted custom-parity transaction first. */
       !st25_write_reg(ST25_REG_ISO14443A_NFC, 0) ||
       !st25_write_reg(ST25_REG_AUX, ST25_AUX_NO_CRC_RX) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES2, 0) ||
       !st25_command(frame == FuriHalNfcaShortFrameAllReq ? ST25_CMD_TX_WUPA : ST25_CMD_TX_REQA))
        return FuriHalNfcErrorCommunication;
    FuriHalNfcError error = st25_wait_for_rx(50);
    st25_write_reg(ST25_REG_AUX, 0);
    return error;
}

FuriHalNfcError furi_hal_nfc_iso14443a_tx_sdd_frame(const uint8_t* tx_data, size_t tx_bits) {
    return st25_transceive(tx_data, tx_bits, true, false);
}
FuriHalNfcError furi_hal_nfc_iso14443a_rx_sdd_frame(uint8_t* rx_data, size_t rx_data_size, size_t* rx_bits) {
    return furi_hal_nfc_poller_rx(rx_data, rx_data_size, rx_bits);
}
FuriHalNfcError furi_hal_nfc_iso14443a_poller_tx_custom_parity(const uint8_t* tx_data, size_t tx_bits) {
    return st25_transceive(tx_data, tx_bits, false, true);
}

FuriHalNfcError furi_hal_nfc_listener_tx(const uint8_t* tx_data, size_t tx_bits) {
    if(!st25_ready || st25_mode != FuriHalNfcModeListener || !tx_data || !tx_bits)
        return FuriHalNfcErrorCommunication;
    if(tx_bits > ST25_FIFO_DEPTH * 8) return FuriHalNfcErrorBufferOverflow;
    uint8_t nfca = 0;
    if(!st25_read_reg(ST25_REG_ISO14443A_NFC, &nfca))
        return FuriHalNfcErrorCommunication;
    /* Standard byte frames let the ST25 generate ISO14443-A parity.  A
     * short 4-bit ACK/NACK has no parity bit, so automatic parity must be
     * disabled for any non-byte-aligned frame. */
    if(tx_bits % 8U) nfca |= ST25_NFCA_NO_TX_PAR;
    else nfca &= (uint8_t)~ST25_NFCA_NO_TX_PAR;
    if(!st25_command(ST25_CMD_CLEAR_FIFO) ||
       !st25_write_reg(ST25_REG_ISO14443A_NFC, nfca) ||
       !st25_write_fifo(tx_data, (tx_bits + 7) / 8) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES1, (uint8_t)(tx_bits >> 8)) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES2, (uint8_t)tx_bits) ||
       !st25_command(ST25_CMD_TX_WITHOUT_CRC)) return FuriHalNfcErrorCommunication;
    return FuriHalNfcErrorNone;
}

FuriHalNfcError furi_hal_nfc_listener_rx(uint8_t* rx_data, size_t rx_data_size, size_t* rx_bits) {
    if(!rx_bits) return FuriHalNfcErrorDataFormat;
    size_t bytes = (st25_rx_bits + 7) / 8;
    if(bytes > rx_data_size) { *rx_bits = 0; return FuriHalNfcErrorBufferOverflow; }
    if(bytes && rx_data) memcpy(rx_data, st25_rx, bytes);
    *rx_bits = st25_rx_bits;
    st25_rx_bits = 0;
    return FuriHalNfcErrorNone;
}
FuriHalNfcError furi_hal_nfc_listener_sleep(void) {
    st25_listener_active = false;
    st25_listener_manual_rx = false;
    st25_listener_rx_raw_parity = false;
    st25_write_reg(ST25_REG_ISO14443A_NFC, 0);
    return st25_command(ST25_CMD_GOTO_SLEEP) ? FuriHalNfcErrorNone : FuriHalNfcErrorCommunication;
}
FuriHalNfcError furi_hal_nfc_listener_idle(void) {
    st25_listener_active = false;
    st25_listener_ready = false;
    st25_listener_manual_rx = false;
    st25_listener_rx_raw_parity = false;
    /* 离场或 HALT 后恢复自动防冲突，否则下一次靠近会停留在手工接收
     * 路径，导致读卡器无法再次选择该标签。 */
    if(!st25_write_reg(ST25_REG_ISO14443A_NFC, 0) ||
       !st25_write_reg(ST25_REG_PASSIVE_TARGET, ST25_PT_CONFIG_AUTO_NFCA)) {
        return FuriHalNfcErrorCommunication;
    }
    return st25_command(ST25_CMD_GOTO_SENSE) ? FuriHalNfcErrorNone : FuriHalNfcErrorCommunication;
}
FuriHalNfcError furi_hal_nfc_listener_enable_rx(void) {
    uint8_t pt;
    if(!st25_read_reg(ST25_REG_PASSIVE_TARGET, &pt))
        return FuriHalNfcErrorCommunication;
    pt = (pt & 0x0FU) | ST25_PT_FDEL | ST25_PT_D_106_AC_A;
    if(!st25_write_reg(ST25_REG_PASSIVE_TARGET, pt))
        return FuriHalNfcErrorCommunication;
    st25_listener_manual_rx = true;
    return FuriHalNfcErrorNone;
}
FuriHalNfcError furi_hal_nfc_iso14443a_listener_set_col_res_data(
    uint8_t* uid, uint8_t uid_len, uint8_t* atqa, uint8_t sak) {
    if(!st25_ready || !uid || !atqa ||
       (uid_len != 4 && uid_len != 7 && uid_len != 10))
        return FuriHalNfcErrorDataFormat;
    memcpy(st25_listener_uid, uid, uid_len);
    st25_listener_uid_len = uid_len;
    memcpy(st25_listener_atqa, atqa, 2);
    st25_listener_sak = sak;
    memset(st25_listener_pt, 0, sizeof(st25_listener_pt));
    memcpy(st25_listener_pt, uid, uid_len);
    st25_listener_pt[10] = atqa[0];
    st25_listener_pt[11] = atqa[1];
    /* PT Memory contains the three cascade-level SAK bytes. For a 7-byte
     * UID, CL1 must advertise UID-not-complete (cascade bit 0x04); without
     * it a reader stops after the first anticollision level and the tag is
     * never selected. This is the same rule used by M5Unit-NFC. */
    st25_listener_pt[12] = (uid_len == 4) ? (uint8_t)(sak & ~0x04U) :
                                           (uint8_t)(sak | 0x04U);
    st25_listener_pt[13] = (uint8_t)(sak & ~0x04U);
    st25_listener_pt[14] = (uint8_t)(sak & ~0x04U);
    uint8_t pt_command[sizeof(st25_listener_pt) + 1];
    pt_command[0] = ST25_PT_A_CONFIG_LOAD;
    memcpy(&pt_command[1], st25_listener_pt, sizeof(st25_listener_pt));
    if(st25_write_raw(pt_command, sizeof(pt_command)) != ESP_OK)
        return FuriHalNfcErrorCommunication;
    uint8_t pt_readback[sizeof(st25_listener_pt)] = {0};
    if(!st25_read_pt_memory(pt_readback, sizeof(pt_readback)) ||
       memcmp(pt_readback, st25_listener_pt, sizeof(pt_readback)) != 0) {
        FURI_LOG_E(TAG, "PT memory readback mismatch");
        return FuriHalNfcErrorCommunication;
    }
    FURI_LOG_I(TAG, "Listener identity UIDlen=%u ATQA=%02X%02X SAK=%02X PTmem OK",
        uid_len, atqa[0], atqa[1], sak);

    /* PT Memory 只承载 UID/ATQA/SAK，不能在这里改写 RFAL 已为
     * LISTEN_NFCA 配好的模拟前端。尤其 ANT_TUNE_B=0xFF 会让 Unit NFC
     * 无法检测到手机外场；Bruce 的可用流程也会保留原有模拟配置。 */

    /* AUX.nfc_id selects the NFCID1 size: 00=4 bytes, 01=7 bytes,
     * 10=10 bytes on ST25R3916. Preserve no_crc_rx and unrelated AUX bits. */
    uint8_t aux = 0;
    const uint8_t nfc_id = (uid_len == 7) ? ST25_AUX_NFC_ID_7 :
                           (uid_len == 10) ? ST25_AUX_NFC_ID_10 : 0x00U;
    if(!st25_read_reg(ST25_REG_AUX, &aux)) return FuriHalNfcErrorCommunication;
    /* 自动防冲突需要芯片校验 SELECT 的 CRC。此前在这里提前设置
     * no_crc_rx，会让手机只收到 ATQA、随后无法完成 UID 选择。 */
    aux = (aux & (uint8_t)~(ST25_AUX_NFC_ID_7 | ST25_AUX_NFC_ID_10 | ST25_AUX_NO_CRC_RX)) |
          nfc_id;
    if(!st25_write_reg(ST25_REG_AUX, aux) || !st25_command(ST25_CMD_CLEAR_FIFO) ||
       !st25_command(ST25_CMD_UNMASK_RECEIVE_DATA))
        return FuriHalNfcErrorCommunication;

    /* A cold field may already be present when emulation is selected. Match
     * M5's listener state machine: keep EN+RX enabled in that case; otherwise
     * GOTO_SENSE may power the oscillator down until the phone approaches. */
    uint8_t aux_display = 0;
    if(!st25_read_reg(ST25_REG_AUX_DISPLAY, &aux_display))
        return FuriHalNfcErrorCommunication;
    if(aux_display & ST25_AUX_DISPLAY_EFD) {
        if(!st25_write_reg(
               ST25_REG_OP_CONTROL, ST25_OP_EN | ST25_OP_RX_EN | ST25_OP_FD_AUTO_EFD))
            return FuriHalNfcErrorCommunication;
    } else {
        if(!st25_command(ST25_CMD_GOTO_SENSE)) return FuriHalNfcErrorCommunication;
    }
    st25_listener_active = false;
    st25_listener_ready = false;
    st25_listener_manual_rx = false;
    st25_listener_bitrate = 0xFFU;
    memset(&st25_rtc_diag, 0, sizeof(st25_rtc_diag));
    st25_rtc_diag.magic = ST25_RTC_DIAG_MAGIC;
    st25_rtc_diag.started = 1;
    st25_rtc_diag.aux = aux_display;
    st25_rtc_diag.efd_seen = (aux_display & ST25_AUX_DISPLAY_EFD) ? 1 : 0;
    return FuriHalNfcErrorNone;
}
FuriHalNfcError furi_hal_nfc_iso14443a_listener_tx_custom_parity(
    const uint8_t* tx_data, const uint8_t* tx_parity, size_t tx_bits) {
    if(!tx_data || !tx_parity || tx_bits == 0) return FuriHalNfcErrorDataFormat;

    /* MIFARE Classic ACK/NACK is a 4-bit short frame and has no parity bit.
     * The generic Classic listener sends these through the custom-parity API,
     * so rejecting non-byte-aligned lengths makes every post-auth READ fail
     * even though the Crypto1 data path is otherwise correct. */
    size_t packed_bits = tx_bits;
    size_t packed_bytes = (packed_bits + 7U) / 8U;
    uint8_t packed[ST25_FIFO_DEPTH] = {0};
    if(tx_bits == 4U) {
        packed[0] = tx_data[0] & 0x0FU;
    } else {
        if((tx_bits % 8U) != 0U) return FuriHalNfcErrorDataFormat;
        size_t bytes = tx_bits / 8U;
        packed_bits = bytes * 9U;
        packed_bytes = (packed_bits + 7U) / 8U;
        if(packed_bytes > ST25_FIFO_DEPTH) return FuriHalNfcErrorBufferOverflow;

        size_t bit = 0;
        for(size_t i = 0; i < bytes; i++) {
            for(uint8_t b = 0; b < 8U; b++, bit++) {
                if(tx_data[i] & (1U << b))
                    packed[bit / 8U] |= (uint8_t)(1U << (bit % 8U));
            }
            if(tx_parity[i / 8U] & (1U << (i % 8U)))
                packed[bit / 8U] |= (uint8_t)(1U << (bit % 8U));
            bit++;
        }
    }
    if(packed_bytes > ST25_FIFO_DEPTH) return FuriHalNfcErrorBufferOverflow;

    if(!st25_ready || st25_mode != FuriHalNfcModeListener ||
       !st25_command(ST25_CMD_CLEAR_FIFO) || !st25_write_fifo(packed, packed_bytes) ||
       !st25_write_reg(ST25_REG_ISO14443A_NFC, ST25_NFCA_NO_TX_PAR | ST25_NFCA_NO_RX_PAR) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES1, (uint8_t)(packed_bits >> 8)) ||
       !st25_write_reg(ST25_REG_NUM_TX_BYTES2, (uint8_t)packed_bits) ||
       !st25_command(ST25_CMD_TX_WITHOUT_CRC)) return FuriHalNfcErrorCommunication;
    return FuriHalNfcErrorNone;
}
FuriHalNfcError furi_hal_nfc_iso15693_listener_tx_sof(void) { return FuriHalNfcErrorCommunication; }
FuriHalNfcError furi_hal_nfc_iso15693_detect_mode(void) { return FuriHalNfcErrorCommunication; }
FuriHalNfcError furi_hal_nfc_iso15693_force_1outof4(void) { return FuriHalNfcErrorCommunication; }
FuriHalNfcError furi_hal_nfc_iso15693_force_1outof256(void) { return FuriHalNfcErrorCommunication; }
FuriHalNfcError furi_hal_nfc_felica_listener_set_sensf_res_data(const uint8_t* i, const uint8_t il, const uint8_t* p, const uint8_t pl, const uint16_t s) { UNUSED(i); UNUSED(il); UNUSED(p); UNUSED(pl); UNUSED(s); return FuriHalNfcErrorCommunication; }
void furi_hal_nfc_emu_set_ndef(const uint8_t* msg, size_t len) { UNUSED(msg); UNUSED(len); }

/* Compatibility hooks used by the ESP32 PN532-specific MIFARE Classic poller.
 * Returning false makes it take the normal software Crypto1 + custom parity path. */
FuriHalNfcError furi_hal_nfc_pn532_mf_auth(uint8_t b, const uint8_t* k, uint8_t kt, const uint8_t* u, uint8_t ul) { UNUSED(b); UNUSED(k); UNUSED(kt); UNUSED(u); UNUSED(ul); return FuriHalNfcErrorCommunication; }
bool furi_hal_nfc_pn532_mf_is_authed(void) { return false; }
void furi_hal_nfc_pn532_mf_deauth(void) {}
