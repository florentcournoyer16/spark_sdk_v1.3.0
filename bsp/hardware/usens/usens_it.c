#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>


#include "usens_it.h"
#include "hw_cfg.h"

#define MY_IRQ_FLAGS 0

#define PRIO_RADIO_RADIO_IRQ  	2
#define PRIO_RADIO_DMA_IRQ  	2
#define PRIO_PEND_SV_IRQ    	3

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void default_irq_callback(void);
static void init_irqs(void);
static void init_radio_irq_gpio(void);
static void init_PendSV_irq(void);
static int find_unused_irq(int start);
static void fake_PendSV_Handler(const void *args);

/* PRIVATE GLOBALS ************************************************************/
static irq_callback common_callback          = default_irq_callback;
static irq_callback radio_irq_callback       = default_irq_callback;
static irq_callback radio_dma_callback       = default_irq_callback;
static irq_callback pendsv_irq_callback      = default_irq_callback;
static struct gpio_callback uwb_irq_pin_cb_data;

/* PUBLIC FUNCTION ***********************************************************/
void usens_it_init(void)
{
    init_radio_irq_gpio();
    init_irqs();
}

void evk_it_set_common_callback(irq_callback callback)
{
    common_callback = callback;
}

void usens_set_radio_irq_callback(irq_callback callback)
{
    radio_irq_callback = callback;
}

void usens_set_radio_dma_rx_callback(irq_callback callback)
{
    radio_dma_callback = callback;
}

void usens_set_pendsv_callback(irq_callback callback)
{
    pendsv_irq_callback = callback;
}

void usens_trigger_radio_irq_callback(void)
{
    common_callback();
    radio_irq_callback();
}

void usens_trigger_pendsv_callback(void)
{
    fake_PendSV_Handler(NULL);
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Default interrupt used when initializing callbacks.
 */
static void default_irq_callback(void)
{
    return;
}

/** @brief This function handles EXTI line0 interrupt.
 */
void Radio_IRQ_IRQHandler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    common_callback();
    radio_irq_callback();
}

void fake_Radio_IRQ_IRQHandler(const void *args)
{
    common_callback();
    radio_irq_callback();
}

/** @brief This function handles EXTI line2 interrupt.
 */
void Radio_DMA_IRQHandler(const void *args)
{
    common_callback();
    radio_dma_callback();
}

/** @brief This function handles Pendable request for System Service.
 */
static void fake_PendSV_Handler(const void *args)
{
    pendsv_irq_callback();
}

/** @brief Initialize IRQ pins.
 */
static void init_radio_irq_gpio(void)
{
    uint32_t err = 0;
    err = gpio_pin_configure_dt(&uwb_irq_pin, GPIO_INPUT);
	if (err != 0) {
		printk("Error %d: failed to configure P%s.%d\n",
		       err, uwb_irq_pin.port->name, uwb_irq_pin.pin);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&uwb_irq_pin,
					      GPIO_INT_EDGE_RISING);
	if (err != 0) {
		printk("Error %d: failed to configure interrupt on P%s.%d\n",
			err, uwb_irq_pin.port->name, uwb_irq_pin.pin);
		return;
	}

	gpio_init_callback(&uwb_irq_pin_cb_data, Radio_IRQ_IRQHandler, BIT(uwb_irq_pin.pin));
    
	gpio_add_callback(uwb_irq_pin.port, &uwb_irq_pin_cb_data);
}

/** @brief Initialize PendSV IRQ.
 */
static void init_PendSV_irq(void)
{
    //PendSV_IRQn;
}

static void init_irqs(void)
{
    // fake_PendSV_IRQ = find_unused_irq(CONFIG_NUM_IRQS - 1);

    // Radio_DMA_IRQ = find_unused_irq(fake_PendSV_IRQ);
	irq_connect_dynamic(PWM3_IRQn, PRIO_PEND_SV_IRQ, fake_PendSV_Handler, NULL, MY_IRQ_FLAGS);

	NVIC_ClearPendingIRQ(PWM3_IRQn);
	NVIC_EnableIRQ(PWM3_IRQn);

    irq_connect_dynamic(CRYPTOCELL_IRQn, PRIO_RADIO_DMA_IRQ, Radio_DMA_IRQHandler, NULL, MY_IRQ_FLAGS);

	NVIC_ClearPendingIRQ(CRYPTOCELL_IRQn);
	NVIC_EnableIRQ(CRYPTOCELL_IRQn);

	irq_connect_dynamic(QSPI_IRQn, PRIO_RADIO_RADIO_IRQ, fake_Radio_IRQ_IRQHandler, NULL, MY_IRQ_FLAGS);

	NVIC_ClearPendingIRQ(QSPI_IRQn);
	NVIC_EnableIRQ(QSPI_IRQn);
}

static int find_unused_irq(int start)
{
	int i;
	for (i = start - 1; i >= 0; i--) {
		if (NVIC_GetEnableIRQ(i) == 0) {
			/*
			 * Interrupts configured statically with IRQ_CONNECT(.)
			 * are automatically enabled. NVIC_GetEnableIRQ()
			 * returning false, here, implies that the IRQ line is
			 * either not implemented or it is not enabled, thus,
			 * currently not in use by Zephyr.
			 */

			/* Set the NVIC line to pending. */
			NVIC_SetPendingIRQ(i);

			if (NVIC_GetPendingIRQ(i)) {
				/*
				 * If the NVIC line is pending, it is
				 * guaranteed that it is implemented; clear the
				 * line.
				 */
				NVIC_ClearPendingIRQ(i);

				if (!NVIC_GetPendingIRQ(i)) {
					/*
					 * If the NVIC line can be successfully
					 * un-pended, it is guaranteed that it
					 * can be used for software interrupt
					 * triggering. Return the NVIC line
					 * number.
					 */
					break;
				}
			}
		}
	}
	return i;
}