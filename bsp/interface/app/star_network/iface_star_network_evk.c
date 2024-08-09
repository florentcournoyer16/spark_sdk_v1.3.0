/** @file  iface_star_network_evk.c
 *  @brief This file contains the application-specific function implementation
 *         that calls the EVK1.4 BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_star_network.h"
#include "evk_usb_device.h"
#include "evk.h"

/* CONSTANTS ******************************************************************/
#define PENDSV_IRQ_PRIORITY  13
#define PRINTF_BUF_SIZE_BYTE 64

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

void iface_delay(uint32_t ms_delay)
{
    evk_timer_delay_ms(ms_delay);
}

bool iface_read_button_status(star_network_btn_t button)
{
    return evk_read_btn_state((evk_btn_t)button);
}

void iface_usb_printf(const char *fmt, ...)
{
    char std_buf[PRINTF_BUF_SIZE_BYTE];
    va_list va;

    va_start(va, fmt);

    vsprintf(std_buf, fmt, va);

    evk_usb_device_cdc_send_buf((uint8_t *)std_buf, strlen(std_buf));

    va_end(va);
}

void iface_payload_sent_status(void)
{
    evk_led_on(LED0);
}

void iface_empty_payload_sent_status(void)
{
    evk_led_off(LED0);
}

void iface_payload_received_status(void)
{
    evk_led_on(LED1);
}

void iface_empty_payload_received_status(void)
{
    evk_led_off(LED1);
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
