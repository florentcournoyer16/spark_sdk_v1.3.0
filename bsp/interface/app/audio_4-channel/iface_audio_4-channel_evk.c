/** @file  iface_audio_4_channel_evk.c
 *  @brief This file contains the application-specific function implementation
 *         that calls the EVK1.4 BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_audio_4-channel.h"
#include "evk.h"
#include "evk_usb_device.h"
#include "max98091.h"

/* CONSTANTS ******************************************************************/
#define PENDSV_IRQ_PRIORITY              13
#define AUDIO_PROCESS_TIMER_IRQ_PRIORITY 14
#define STATS_TIMER_IRQ_PRIORITY         15

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void usb_connect_callback(void);
static void usb_detect_callback_init(void);
static void usb_connection_init(void);

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_board_init(void)
{
    evk_cfg_t evk_cfg = {
        .freq = CLK_169_98MHZ,
        .vdd = VDD_3V3,
        .pendsv_prio = PENDSV_IRQ_PRIORITY,
    };
    evk_init(&evk_cfg);
    usb_detect_callback_init();
    usb_connection_init();
}

void iface_audio_coord_init(void)
{
    evk_sai_config_t sai_config = {
        .rx_sai_mono_stereo = EVK_SAI_MODE_MONO,
        .sai_bit_depth = EVK_SAI_BIT_DEPTH_16BITS,
    };

    max98091_i2c_hal_t hal;

    hal.i2c_addr = MAX98091A_I2C_ADDR;
    hal.read = evk_audio_i2c_read_byte_blocking;
    hal.write = evk_audio_i2c_write_byte_blocking;

    evk_audio_i2c_init();
    max98091_reset_codec(&hal);
    evk_timer_delay_ms(1);

    evk_audio_sai_configuration(&sai_config);

    max98091_codec_cfg_t cfg = {
        .sampling_rate = MAX98091_AUDIO_48KHZ,
        .word_size = MAX98091_AUDIO_16BITS,
        .record_enabled = true,
        .playback_enabled = false,
        .record_filter_enabled = false,
        .playback_filter_enabled = false,
    };
    max98091_init(&hal, &cfg);
}

void iface_audio_node_init(void)
{
    evk_sai_config_t sai_config = {
        .tx_sai_mono_stereo = EVK_SAI_MODE_MONO,
        .sai_bit_depth = EVK_SAI_BIT_DEPTH_16BITS,
    };

    max98091_i2c_hal_t hal;

    hal.i2c_addr = MAX98091A_I2C_ADDR;
    hal.read = evk_audio_i2c_read_byte_blocking;
    hal.write = evk_audio_i2c_write_byte_blocking;

    evk_audio_i2c_init();
    max98091_reset_codec(&hal);
    evk_timer_delay_ms(1);

    evk_audio_sai_configuration(&sai_config);

    max98091_codec_cfg_t cfg = {
        .sampling_rate = MAX98091_AUDIO_48KHZ,
        .word_size = MAX98091_AUDIO_16BITS,
        .record_enabled = false,
        .playback_enabled = true,
        .record_filter_enabled = false,
        .playback_filter_enabled = false,
    };
    max98091_init(&hal, &cfg);
}

void iface_audio_dma_complete_callbacks(void(*tx_callback)(void), void(*rx_callback)(void))
{
    evk_audio_set_sai_tx_dma_cplt_callback((irq_callback)tx_callback);
    evk_audio_set_sai_rx_dma_cplt_callback((irq_callback)rx_callback);
}

void iface_volume_handling(void (*button1_callback)(void), void (*button2_callback)(void))
{
    static bool btn1_active;
    static bool btn2_active;

    if (btn1_active) {
        if (!evk_read_btn_state(BTN1)) {
            btn1_active = false;
        }
    }
    if (btn2_active) {
        if (!evk_read_btn_state(BTN2)) {
            btn2_active = false;
        }
    }
    if (!btn1_active && !btn2_active) {
        if (evk_read_btn_state(BTN1)) {
            if (button1_callback != NULL) {
                button1_callback();
            }
            btn1_active = true;
        } else if (evk_read_btn_state(BTN2)) {
            if (button2_callback != NULL) {
                button2_callback();
            }
            btn2_active = true;
        }
    }
}

void iface_tx_conn_status(void)
{
    evk_led_toggle(LED0);
}

void iface_rx_conn_status(void)
{
    evk_led_toggle(LED1);
}

void iface_audio_process_timer_init(uint32_t period_us)
{
    evk_timer_cfg_t timer_cfg = {
        .timer_selection = EVK_TIMER_SELECTION_TIMER20,
        .time_base = EVK_TIMER_TIME_BASE_MICROSECOND,
        .time_period = period_us,
        .irq_priority = AUDIO_PROCESS_TIMER_IRQ_PRIORITY,
    };
    evk_timer_init(timer_cfg);
}

void iface_audio_process_set_timer_callback(void (*callback)(void))
{
    evk_it_set_timer20_callback(callback);
}

void iface_audio_process_timer_start(void)
{
    evk_timer_start(EVK_TIMER_SELECTION_TIMER20);
}

void iface_stats_timer_init(uint32_t period_ms)
{
    evk_timer_cfg_t timer_cfg = {
        .timer_selection = EVK_TIMER_SELECTION_TIMER3,
        .time_base = EVK_TIMER_TIME_BASE_MILLISECOND,
        .time_period = period_ms,
        .irq_priority = STATS_TIMER_IRQ_PRIORITY,
    };
    evk_timer_init(timer_cfg);
}

void iface_stats_set_timer_callback(void (*callback)(void))
{
    evk_it_set_timer3_callback(callback);
}

void iface_stats_timer_start(void)
{
    evk_timer_start(EVK_TIMER_SELECTION_TIMER3);
}

void iface_print_string(char *string)
{
    evk_usb_device_cdc_send_buf((uint8_t *)string, strlen(string));
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief USB line detection callback.
 */
static void usb_connect_callback(void)
{
    if (evk_is_usb_detected()) {
        evk_usb_device_connect();
    } else {
        evk_usb_device_disconnect();
    }
}

/** @brief Initialize the USB detect event callback function.
 */
static void usb_detect_callback_init(void)
{
    evk_set_usb_detect_callback(usb_connect_callback);
}

/** @brief Handle USB connection on initialization.
 */
static void usb_connection_init(void)
{
    if (evk_is_usb_detected()) {
        evk_usb_device_connect();
        evk_timer_delay_ms(1000);
    }
}
