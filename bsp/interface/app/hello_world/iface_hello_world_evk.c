/** @file  iface_hello_world_evk.c
 *  @brief This file contains the application-specific function implementation
 *         that calls the EVK1.4 BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_hello_world.h"
#include <string.h>
#include "evk.h"
#include "evk_usb_device.h"

/* CONSTANTS ******************************************************************/
#define PENDSV_IRQ_PRIORITY 13

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

void iface_delay(uint32_t ms_delay)
{
    evk_timer_delay_ms(ms_delay);
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
