/** @file  iface_pairing_basic_evk.c
 *  @brief This file contains the application-specific function implementation
 *         that calls the EVK1.4 BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
 #include "iface_pairing_basic.h"
 #include "evk.h"

/* CONSTANTS ******************************************************************/
#define PENDSV_IRQ_PRIORITY 13

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_board_init(void)
{
    evk_cfg_t evk_cfg = {
        .freq = CLK_169_98MHZ,
        .vdd = VDD_3V3,
        .pendsv_prio = PENDSV_IRQ_PRIORITY,
    };
    evk_init(&evk_cfg);
}

void iface_button_handling(void (*button1_callback)(void), void (*button2_callback)(void))
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

void iface_payload_received_status(void)
{
    evk_led_on(LED2);
}

void iface_empty_payload_received_status(void)
{
    evk_led_off(LED2);
}

void iface_led_all_on(void)
{
    evk_led_on(LED0);
    evk_led_on(LED1);
    evk_led_on(LED2);
}

void iface_led_all_off(void)
{
    evk_led_off(LED0);
    evk_led_off(LED1);
    evk_led_off(LED2);
}

void iface_led_all_toggle(void)
{
    evk_led_toggle(LED0);
    evk_led_toggle(LED1);
    evk_led_toggle(LED2);
}

void iface_notify_enter_pairing(void)
{
    uint16_t delay_ms = 250;
    uint8_t repeat = 2;

    evk_led_off(LED0);

    for (uint8_t i = 0; i < repeat; i++) {
        evk_timer_delay_ms(delay_ms);
        evk_led_on(LED0);
        evk_timer_delay_ms(delay_ms);
        evk_led_off(LED0);
    }
}

void iface_notify_not_paired(void)
{
    uint16_t delay_ms = 250;

    iface_led_all_off();

    for (uint8_t i = 0; i < 2 * 2; i++) {
        evk_timer_delay_ms(delay_ms);
        iface_led_all_toggle();
    }
}

void iface_notify_pairing_successful(void)
{
    uint16_t delay_ms = 100;

    iface_led_all_off();
    evk_timer_delay_ms(delay_ms);
    evk_led_on(LED0);
    evk_timer_delay_ms(delay_ms);
    evk_led_on(LED1);
    evk_timer_delay_ms(delay_ms);
    evk_led_on(LED2);
    evk_timer_delay_ms(delay_ms);
    evk_led_off(LED2);
    evk_timer_delay_ms(delay_ms);
    evk_led_off(LED1);
    evk_timer_delay_ms(delay_ms);
    evk_led_off(LED0);
}
