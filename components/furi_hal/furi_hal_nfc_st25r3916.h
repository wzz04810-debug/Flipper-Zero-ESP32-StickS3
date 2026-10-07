/**
 * @file furi_hal_nfc_st25r3916.h
 * @brief Native ST25R3916/ST25R3916B NFC HAL backend (I2C).
 *
 * Used by M5Stack Unit NFC on Grove (SDA/SCL only, no IRQ/RST).
 * The Arduino RFAL fork cannot be dropped into ESP-IDF, so this is a
 * native C backend that matches the existing furi_hal_nfc API.
 */
#pragma once

#include "furi_hal_nfc.h"
#include <furi.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

FuriHalNfcError furi_hal_nfc_st25_low_power_mode_start(void);
FuriHalNfcError furi_hal_nfc_st25_low_power_mode_stop(void);
FuriHalNfcError furi_hal_nfc_st25_set_mode(FuriHalNfcMode mode, FuriHalNfcTech tech);
FuriHalNfcError furi_hal_nfc_st25_reset_mode(void);
FuriHalNfcError furi_hal_nfc_st25_field_detect_start(void);
FuriHalNfcError furi_hal_nfc_st25_field_detect_stop(void);
bool furi_hal_nfc_st25_field_is_present(void);
FuriHalNfcError furi_hal_nfc_st25_poller_field_on(void);
FuriHalNfcEvent furi_hal_nfc_st25_poller_wait_event(uint32_t timeout_ms);
FuriHalNfcEvent furi_hal_nfc_st25_listener_wait_event(uint32_t timeout_ms);
FuriHalNfcError furi_hal_nfc_st25_event_start(void);
FuriHalNfcError furi_hal_nfc_st25_event_stop(void);
FuriHalNfcError furi_hal_nfc_st25_abort(void);

void furi_hal_nfc_st25_timer_fwt_start(uint32_t time_fc);
void furi_hal_nfc_st25_timer_fwt_stop(void);
void furi_hal_nfc_st25_timer_block_tx_start(uint32_t time_fc);
void furi_hal_nfc_st25_timer_block_tx_start_us(uint32_t time_us);
void furi_hal_nfc_st25_timer_block_tx_stop(void);
bool furi_hal_nfc_st25_timer_block_tx_is_running(void);

FuriHalNfcError furi_hal_nfc_st25_trx_reset(void);
FuriHalNfcError furi_hal_nfc_st25_poller_tx(const uint8_t* tx_data, size_t tx_bits);
FuriHalNfcError furi_hal_nfc_st25_poller_rx(uint8_t* rx_data, size_t rx_data_size, size_t* rx_bits);
FuriHalNfcError furi_hal_nfc_st25_listener_tx(const uint8_t* tx_data, size_t tx_bits);
FuriHalNfcError furi_hal_nfc_st25_listener_rx(uint8_t* rx_data, size_t rx_data_size, size_t* rx_bits);
FuriHalNfcError furi_hal_nfc_st25_listener_sleep(void);
FuriHalNfcError furi_hal_nfc_st25_listener_idle(void);
FuriHalNfcError furi_hal_nfc_st25_listener_enable_rx(void);

FuriHalNfcError furi_hal_nfc_st25_iso14443a_poller_trx_short_frame(FuriHalNfcaShortFrame frame);
FuriHalNfcError furi_hal_nfc_st25_iso14443a_tx_sdd_frame(const uint8_t* tx_data, size_t tx_bits);
FuriHalNfcError furi_hal_nfc_st25_iso14443a_rx_sdd_frame(
    uint8_t* rx_data,
    size_t rx_data_size,
    size_t* rx_bits);
FuriHalNfcError furi_hal_nfc_st25_iso14443a_poller_tx_custom_parity(
    const uint8_t* tx_data,
    size_t tx_bits);

FuriHalNfcError furi_hal_nfc_st25_iso14443a_listener_set_col_res_data(
    uint8_t* uid,
    uint8_t uid_len,
    uint8_t* atqa,
    uint8_t sak);
FuriHalNfcError furi_hal_nfc_st25_iso14443a_listener_tx_custom_parity(
    const uint8_t* tx_data,
    const uint8_t* tx_parity,
    size_t tx_bits);

FuriHalNfcError furi_hal_nfc_st25_iso15693_listener_tx_sof(void);
FuriHalNfcError furi_hal_nfc_st25_iso15693_detect_mode(void);
FuriHalNfcError furi_hal_nfc_st25_iso15693_force_1outof4(void);
FuriHalNfcError furi_hal_nfc_st25_iso15693_force_1outof256(void);

FuriHalNfcError furi_hal_nfc_st25_felica_listener_set_sensf_res_data(
    const uint8_t* idm,
    const uint8_t idm_len,
    const uint8_t* pmm,
    const uint8_t pmm_len,
    const uint16_t sys_code);

void furi_hal_nfc_st25_emu_set_ndef(const uint8_t* msg, size_t len);

#ifdef __cplusplus
}
#endif
