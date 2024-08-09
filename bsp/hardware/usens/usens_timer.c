#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <dk_buttons_and_leds.h>

#include "usens_timer.h"
#include "hw_cfg.h"

/* PRIVATE GLOBALS ************************************************************/
static volatile uint64_t free_running_ms_timer_tick_counter;
static volatile uint64_t free_running_quarter_ms_timer_tick_counter;
static struct k_timer radio_ms_timer;
static struct k_timer radio_quarter_ms_timer;


/* PRIVATE FUNCTION PROTOTYPES ************************************************/
extern void free_running_timer_ms_tick_callback(struct k_timer *timer_id);
extern void free_running_timer_quarter_ms_tick_callback(struct k_timer *timer_id);


/* PUBLIC FUNCTIONS ***********************************************************/
extern void timer_free_running_ms_init(uint8_t irq_priority)
{
    k_timer_init(&radio_ms_timer, free_running_timer_ms_tick_callback, NULL);
    k_timer_start(&radio_ms_timer, K_MSEC(1), K_MSEC(1));
}

extern void timer_free_running_quarter_ms_init(uint8_t irq_priority)
{
    k_timer_init(&radio_quarter_ms_timer, free_running_timer_quarter_ms_tick_callback, NULL);
    k_timer_start(&radio_quarter_ms_timer, K_USEC(250), K_USEC(250));
}

extern uint64_t timer_free_running_ms_get_tick_count(void)
{
    return free_running_ms_timer_tick_counter;
}

extern uint64_t timer_free_running_quarter_ms_get_tick_count(void)
{
    return free_running_quarter_ms_timer_tick_counter;
}

extern void timer_delay_ms(uint32_t delay_ms)
{
    k_sleep(K_MSEC(delay_ms));
}

extern uint32_t timer_get_ms_tick(void)
{
    printk("ERROR : Using timer_get_ms_tick in radio_timer.c while not implemented");
    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Callback for the free running timer tick.
 */
extern void free_running_timer_ms_tick_callback(struct k_timer *timer_id)
{
    free_running_ms_timer_tick_counter++;
}

/** @brief Callback for the free running quarter timer tick.
 */
extern void free_running_timer_quarter_ms_tick_callback(struct k_timer *timer_id)
{
    free_running_quarter_ms_timer_tick_counter++;
}
