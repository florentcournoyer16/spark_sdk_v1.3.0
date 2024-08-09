#ifndef USENS_IT_H_
#define USENS_IT_H_

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
//#include <zephyr/interrupt_util.h>

/* TYPES **********************************************************************/
/** @brief Interrupt's module function callback type.
 */
typedef void (*irq_callback)(void);


static int fake_PendSV_IRQ;
static int Radio_DMA_IRQ;


/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief This function initialise interrupts handlers.
 */
void usens_it_init(void);

/** @brief This function sets the function callback for the radio pin interrupt.
 *
 *  @param[in] callback  External interrupt callback function pointer.
 */
void usens_set_radio_irq_callback(irq_callback callback);

/** @brief This function sets the function callback for the DMA_RX ISR.
 *
 *  @param[in] callback  External interrupt callback function pointer.
 */
void usens_set_radio_dma_rx_callback(irq_callback callback);

/** @brief This function sets the function callback for the pendsv.
 *
 *  @param[in] callback  External interrupt callback function pointer.
 */
void usens_set_pendsv_callback(irq_callback callback);

void usens_trigger_radio_irq_callback(void);

void usens_trigger_pendsv_callback(void);

void Radio_IRQ_IRQHandler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

#endif /* USENS_IT_H_ */