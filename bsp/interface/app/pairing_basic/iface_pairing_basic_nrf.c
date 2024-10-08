#include <stdint.h>
#include <dk_buttons_and_leds.h>
#include "hw_cfg.h"
#include "usens_base.h"
#include "usens_timer.h"

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_board_init(void)
{
    usens_init();
}

void iface_tx_conn_status(void)
{
    gpio_pin_toggle_dt(&led0);
}

void iface_rx_conn_status(void)
{
    gpio_pin_toggle_dt(&led1);
}

void iface_delay(uint32_t ms_delay)
{
    timer_delay_ms(ms_delay);
}

void iface_print_string(char *string)
{
    printk("%s", string);
}


void iface_payload_received_status(void)
{
    dk_set_led(REC_STATUS_LED, 1);
}

void iface_empty_payload_received_status(void)
{
    dk_set_led(REC_STATUS_LED, 0);
}

void iface_led_all_on(void)
{
    gpio_pin_set_dt(&led0, 1);
    gpio_pin_set_dt(&led1, 1);
    gpio_pin_set_dt(&led2, 1);
}

void iface_led_all_off(void)
{
    gpio_pin_set_dt(&led0, 0);
    gpio_pin_set_dt(&led1, 0);
    gpio_pin_set_dt(&led2, 0);
}

void iface_led_all_toggle(void)
{
    gpio_pin_toggle_dt(&led0);
    gpio_pin_toggle_dt(&led1);
    gpio_pin_toggle_dt(&led2);
}

void iface_notify_enter_pairing(void)
{
    uint16_t delay_ms = 250;
    uint8_t repeat = 2;

    dk_set_led(RUN_STATUS_LED, 0);

    for (uint8_t i = 0; i < repeat; i++) {
        iface_delay(delay_ms);
        dk_set_led(RUN_STATUS_LED, 1);
        iface_delay(delay_ms);
        dk_set_led(RUN_STATUS_LED, 0);
    }
}

void iface_notify_not_paired(void)
{
    uint16_t delay_ms = 250;

    iface_led_all_off();

    for (uint8_t i = 0; i < 2 * 2; i++) {
        iface_delay(delay_ms);
        iface_led_all_toggle();
    }
}

void iface_notify_pairing_successful(void)
{
    uint16_t delay_ms = 100;
    
    iface_led_all_off();
    iface_delay(delay_ms);
    dk_set_led(CON_STATUS_LED, 1);
    iface_delay(delay_ms);
    dk_set_led(RUN_STATUS_LED, 1);
    iface_delay(delay_ms);
    dk_set_led(REC_STATUS_LED, 1);
    iface_delay(delay_ms);
    dk_set_led(REC_STATUS_LED, 0);
    iface_delay(delay_ms);
    dk_set_led(RUN_STATUS_LED, 0);
    iface_delay(delay_ms);
    dk_set_led(CON_STATUS_LED, 0);
}
