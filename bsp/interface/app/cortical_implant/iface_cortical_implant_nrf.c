#include <stdint.h>
#include <dk_buttons_and_leds.h>
#include "hw_cfg.h"
#include "usens_base.h"
#include "usens_timer.h"

static bool rx_led;
static bool tx_led;
/* PUBLIC FUNCTIONS ***********************************************************/
void iface_board_init(void)
{
    rx_led = 0;
    tx_led = 0;
    usens_init();
}

void iface_tx_conn_status(void)
{
    tx_led = !tx_led;
    dk_set_led(RUN_STATUS_LED, tx_led);
}

void iface_rx_conn_status(void)
{
    
    rx_led = !rx_led;
    dk_set_led(CON_STATUS_LED, rx_led);
}

void iface_delay(uint32_t ms_delay)
{
    timer_delay_ms(ms_delay);
}

void iface_print_string(char *string)
{
    printk("%s", string);
}

