/**
 * @file target_input.c
 * StickS3 输入驱动，按原机三键逻辑映射。
 *
 * 板载只有两个实体按键（与 M5Unified / bruce-cn 一致）：
 *   GPIO11 SEL  短按 -> InputKeyOk
 *   GPIO12 DW   短按 -> 进入深度休眠；再次按下该键会完整唤醒重启
 *
 * GPIO1 上的五向摇杆是可选 HAT。未接入时 ADC 应停在空闲高电平，
 * 不能产生方向事件。
 */

#include "target_input.h"

#include <boards/board.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_err.h>
#include <furi.h>
#include <furi_hal_power.h>

#define TAG "InputStickS3"

#define INPUT_DEBOUNCE_POLLS 3U
#define INPUT_LONG_PRESS_MS 600U
#define INPUT_DOUBLE_MS 250U
#define INPUT_REPEAT_MS 700U
#define INPUT_DIRECTION_LOCK_MS 450U
#define JOY_IDLE_MIN 2800
#define JOY_READY_SAMPLES 8U

typedef enum {
    StickJoyNone = 0,
    StickJoyDown,
    StickJoyRight,
    StickJoyUp,
    StickJoyLeft,
    StickJoyOk,
} StickJoy;

typedef struct {
    gpio_num_t gpio;
    bool raw_pressed;
    bool pressed;
    uint8_t debounce;
    uint32_t started_at;
    bool long_sent;
} StickButton;

static adc_oneshot_unit_handle_t joy_adc;
static bool joy_present;
static bool joy_ready;
static uint8_t joy_idle_hits;
static int joy_idle;
static StickJoy last_joy;
static uint32_t joy_last_event_at;
static uint32_t joy_ok_started_at;
static bool joy_ok_long_sent;
static StickButton sel_btn;
static StickButton dw_btn;

static void publish_event(
    FuriPubSub* pubsub,
    InputKey key,
    InputType type,
    uint32_t* sequence_counter) {
    InputEvent event = {
        .sequence_source = INPUT_SEQUENCE_SOURCE_HARDWARE,
        .sequence_counter = ++(*sequence_counter),
        .key = key,
        .type = type,
    };
    furi_pubsub_publish(pubsub, &event);
}

static void configure_button(StickButton* button, gpio_num_t gpio) {
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    button->gpio = gpio;
    button->raw_pressed = gpio_get_level(gpio) == 0;
    button->pressed = false;
    button->debounce = INPUT_DEBOUNCE_POLLS;
    button->started_at = 0;
    button->long_sent = false;
}

static bool debounce_button(StickButton* button) {
    bool raw_pressed = gpio_get_level(button->gpio) == 0;
    if(raw_pressed == button->raw_pressed) {
        if(button->debounce < INPUT_DEBOUNCE_POLLS) button->debounce++;
    } else {
        button->raw_pressed = raw_pressed;
        button->debounce = 1;
    }
    return button->debounce >= INPUT_DEBOUNCE_POLLS;
}

static StickJoy classify_joystick(int raw) {
    /* Measured on G1: DOWN~51 RIGHT~617 UP~1255 LEFT~1975 PRESS~2567 idle>~2800.
     * Keep the original gaps so noise between bands is ignored. */
    if(raw < 0) return StickJoyNone;
    if(raw <= 250) return StickJoyDown;
    if(raw >= 400 && raw <= 850) return StickJoyRight;
    if(raw >= 1000 && raw <= 1550) return StickJoyUp;
    if(raw >= 1700 && raw <= 2200) return StickJoyLeft;
    if(raw >= 2300 && raw <= 2750) return StickJoyOk;
    return StickJoyNone;
}

static InputKey joystick_key(StickJoy joy) {
    switch(joy) {
    case StickJoyDown:
        return InputKeyDown;
    case StickJoyRight:
        return InputKeyRight;
    case StickJoyUp:
        return InputKeyUp;
    case StickJoyLeft:
        return InputKeyLeft;
    case StickJoyOk:
        return InputKeyOk;
    default:
        return InputKeyOk;
    }
}

static void poll_joystick(FuriPubSub* pubsub, uint32_t now, uint32_t* sequence_counter) {
    int raw = 0;
    if(!joy_adc) return;
    if(adc_oneshot_read(joy_adc, ADC_CHANNEL_0, &raw) != ESP_OK) return;

    if(!joy_ready) {
        if(raw > joy_idle) joy_idle = raw;
        if(++joy_idle_hits >= JOY_READY_SAMPLES) {
            joy_ready = true;
            joy_present = (joy_idle > JOY_IDLE_MIN);
            FURI_LOG_I(
                TAG,
                "Joystick idle=%d present=%d",
                joy_idle,
                joy_present ? 1 : 0);
        }
        return;
    }

    if(!joy_present) return;
    if(raw > joy_idle) joy_idle = raw;

    int dead = JOY_IDLE_MIN;
    if(joy_idle > 3300) dead = (joy_idle * 88) / 100;

    StickJoy joy = StickJoyNone;
    if(raw < dead) joy = classify_joystick(raw);

    if(joy != last_joy) {
        /* Ignore very short ADC transitions around the neutral point. This
         * prevents one physical joystick movement from selecting two items. */
        if(joy != StickJoyNone &&
           joy_last_event_at != 0 &&
           now - joy_last_event_at < furi_ms_to_ticks(INPUT_DIRECTION_LOCK_MS)) {
            return;
        }
        if(last_joy != StickJoyNone) {
            if(last_joy == StickJoyOk) {
                if(joy_ok_long_sent) {
                    publish_event(pubsub, InputKeyBack, InputTypeRelease, sequence_counter);
                } else {
                    publish_event(pubsub, InputKeyOk, InputTypeShort, sequence_counter);
                    publish_event(pubsub, InputKeyOk, InputTypeRelease, sequence_counter);
                }
                joy_ok_long_sent = false;
            } else {
                /* The Short event was already emitted when this direction
                 * became active. Do not emit another Short on release: doing
                 * so makes one joystick movement advance two keyboard/menu
                 * positions. */
                publish_event(pubsub, joystick_key(last_joy), InputTypeRelease, sequence_counter);
            }
        }
        if(joy != StickJoyNone) {
            publish_event(pubsub, joystick_key(joy), InputTypePress, sequence_counter);
            /* Treat each new joystick direction as a completed navigation
             * action. This is required by TextInput and standard menus. */
            if(joy != StickJoyOk) {
                publish_event(pubsub, joystick_key(joy), InputTypeShort, sequence_counter);
            }
            joy_last_event_at = now;
            if(joy == StickJoyOk) {
                joy_ok_started_at = now;
                joy_ok_long_sent = false;
            }
        }
        last_joy = joy;
        return;
    }

    if(joy == StickJoyOk && !joy_ok_long_sent &&
       now - joy_ok_started_at >= furi_ms_to_ticks(INPUT_LONG_PRESS_MS)) {
        /* Convert the held center press into the global Back action. Release
         * the original OK first so no view keeps an artificial OK pressed. */
        publish_event(pubsub, InputKeyOk, InputTypeRelease, sequence_counter);
        publish_event(pubsub, InputKeyBack, InputTypePress, sequence_counter);
        publish_event(pubsub, InputKeyBack, InputTypeLong, sequence_counter);
        joy_ok_long_sent = true;
        joy_last_event_at = now;
        return;
    }

    if(joy != StickJoyNone && joy != StickJoyOk &&
       now - joy_last_event_at >= furi_ms_to_ticks(INPUT_REPEAT_MS)) {
        joy_last_event_at = now;
        publish_event(pubsub, joystick_key(joy), InputTypeRepeat, sequence_counter);
    }
}

static void poll_sel_button(FuriPubSub* pubsub, uint32_t now, uint32_t* sequence_counter) {
    if(!debounce_button(&sel_btn)) return;

    bool pressed = sel_btn.raw_pressed;
    if(sel_btn.pressed == pressed) {
        if(sel_btn.pressed && !sel_btn.long_sent &&
           now - sel_btn.started_at >= furi_ms_to_ticks(INPUT_LONG_PRESS_MS)) {
            /* A long center-button hold is the universal Back action. Close
             * the pending OK press first so the dispatcher does not retain a
             * stuck confirmation key. */
            sel_btn.long_sent = true;
            publish_event(pubsub, InputKeyOk, InputTypeRelease, sequence_counter);
            publish_event(pubsub, InputKeyBack, InputTypePress, sequence_counter);
            publish_event(pubsub, InputKeyBack, InputTypeLong, sequence_counter);
        }
        return;
    }

    sel_btn.pressed = pressed;
    if(pressed) {
        sel_btn.started_at = now;
        sel_btn.long_sent = false;
        publish_event(pubsub, InputKeyOk, InputTypePress, sequence_counter);
        return;
    }

    if(sel_btn.long_sent) {
        publish_event(pubsub, InputKeyBack, InputTypeRelease, sequence_counter);
    } else {
        publish_event(pubsub, InputKeyOk, InputTypeShort, sequence_counter);
        publish_event(pubsub, InputKeyOk, InputTypeRelease, sequence_counter);
    }
}

static void poll_dw_button(FuriPubSub* pubsub, uint32_t now, uint32_t* sequence_counter) {
    UNUSED(pubsub);
    UNUSED(now);
    UNUSED(sequence_counter);

    if(!debounce_button(&dw_btn)) return;

    bool pressed = dw_btn.raw_pressed;
    if(dw_btn.pressed == pressed) return;

    dw_btn.pressed = pressed;
    if(!pressed) {
        /* GPIO12 是照片中的蓝色侧键。松开后才关机，确保 EXT0 不会因为
         * 按键仍被压低而立刻唤醒。深度休眠会停止 CPU、任务、无线和 NFC；
         * 本键不发布 InputEvent，防止它被当作普通界面操作。 */
        furi_hal_power_shutdown();
    }
}

void target_input_init(void) {
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &joy_adc));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(joy_adc, ADC_CHANNEL_0, &channel_config));

    gpio_reset_pin((gpio_num_t)BOARD_PIN_JOY_ADC);
    gpio_set_direction((gpio_num_t)BOARD_PIN_JOY_ADC, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t)BOARD_PIN_JOY_ADC);
    gpio_pulldown_dis((gpio_num_t)BOARD_PIN_JOY_ADC);

    configure_button(&sel_btn, (gpio_num_t)BOARD_PIN_BUTTON_BOOT);
    configure_button(&dw_btn, (gpio_num_t)BOARD_PIN_BUTTON_KEY);
    last_joy = StickJoyNone;
    joy_last_event_at = 0;
    joy_ok_started_at = 0;
    joy_ok_long_sent = false;
    joy_present = false;
    joy_ready = false;
    joy_idle_hits = 0;
    joy_idle = 0;
    FURI_LOG_I(TAG, "StickS3 buttons GPIO11=Ok GPIO12=deep sleep/wake");
}

void target_input_poll(FuriPubSub* pubsub, uint32_t* sequence_counter) {
    uint32_t now = furi_get_tick();
    poll_dw_button(pubsub, now, sequence_counter);
    poll_sel_button(pubsub, now, sequence_counter);
    poll_joystick(pubsub, now, sequence_counter);
}
