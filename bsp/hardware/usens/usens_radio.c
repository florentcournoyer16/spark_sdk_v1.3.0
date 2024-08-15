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

#include "usens_radio.h"
#include "usens_it.h"
#include "hw_cfg.h"
#include "circular_queue_critical_section.h"
/* PRIVATE DEFINES ************************************************************/
#define UWB_SHUTDOWN_ACTIVE     1
#define UWB_SHUTDOWN_INACTIVE   0

#define UWB_RESET_ACTIVE        1
#define UWB_RESET_INACTIVE      0

#define UWB_CS_CTRL_ACTIVE      1
#define UWB_CS_CTRL_INACTIVE    0

/* PIVATE VARIABLES ***********************************************************/
static struct spi_config    spi_conf;
// static struct spi_buf_set   spi_tx_set;
// static struct spi_buf_set   spi_rx_set;

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void init_radio_shutdown_gpio(void);
static void init_radio_reset_gpio(void);
static void init_radio_spi_peripheral(void);
static void init_radio_spi_buf(void);
void Radio_DMA_IRQ_trigger(const struct device *dev, int status, void *userdata);

/* PUBLIC FUNCTIONS ***********************************************************/
extern void radio_peripherals_init(void)
{
    init_radio_spi_peripheral();
    init_radio_shutdown_gpio();
    init_radio_reset_gpio();
    init_radio_spi_buf();
    radio_set_reset_pin();
    radio_reset_reset_pin();
}

extern bool radio_read_irq_pin(void)
{
    if(gpio_pin_get_dt(&uwb_irq_pin))
        return true;
    return false;
}

extern void radio_enable_irq_it(void)
{
    gpio_pin_interrupt_configure_dt(&uwb_irq_pin, GPIO_INT_EDGE_RISING);
}

extern void radio_disable_irq_it(void)
{
    gpio_pin_interrupt_configure_dt(&uwb_irq_pin, GPIO_INT_DISABLE);
}

extern void radio_enable_dma_irq_it(void)
{
    //printk("%s: Trying to enable DMA while not implemented.\n", uwb_spi_dev->name);
    NVIC_EnableIRQ(CRYPTOCELL_IRQn);
    // Not yet implemented
    return;
}

extern void radio_disable_dma_irq_it(void)
{
    //printk("%s: Trying to disable DMA while not implemented.\n", uwb_spi_dev->name);
    NVIC_DisableIRQ(CRYPTOCELL_IRQn);
    // Not yet implemented
    return;
}

extern void radio_set_shutdown_pin(void)
{
    gpio_pin_set_dt(&uwb_shutdown_pin, UWB_SHUTDOWN_ACTIVE);
}

extern void radio_reset_shutdown_pin(void)
{
    gpio_pin_set_dt(&uwb_shutdown_pin, UWB_SHUTDOWN_INACTIVE);
}

extern void radio_set_reset_pin(void)
{
    gpio_pin_set_dt(&uwb_reset_pin, UWB_RESET_ACTIVE);
}

extern void radio_reset_reset_pin(void)
{
    gpio_pin_set_dt(&uwb_reset_pin, UWB_RESET_ACTIVE);
}

extern void radio_spi_set_cs(void)
{
    gpio_pin_set_dt(&uwb_cs_ctrl.gpio, UWB_CS_CTRL_ACTIVE);
}

extern void radio_spi_reset_cs(void)
{
    gpio_pin_set_dt(&uwb_cs_ctrl.gpio, UWB_CS_CTRL_INACTIVE);
}

extern void radio_set_debug_en(void)
{

    return;
    // Pin is not connected to the MCU
}

extern void radio_reset_debug_en(void)
{
    return;
    // Pin is not connected to the MCU
}

extern void radio_spi_write_blocking(uint8_t *data, uint8_t size)
{
    if (!device_is_ready(uwb_spi_dev)) {
		printk("%s: device not ready.\n", uwb_spi_dev->name);
		return;
	}

	struct spi_buf tx_buf[1] = {
		{.buf = data, .len = 1},
	};

	struct spi_buf_set spi_tx_set = { .buffers = tx_buf, .count = 1 };

    if(spi_write(uwb_spi_dev, &spi_conf, &spi_tx_set))
    {
        printk("%s: device failed to write.\n", uwb_spi_dev->name);
    }
}

extern void radio_spi_read_blocking(uint8_t *data, uint8_t size)
{
    if (!device_is_ready(uwb_spi_dev)) {
		printk("%s: device not ready.\n", uwb_spi_dev->name);
		return;
	}

	struct spi_buf rx_buf[1] = {
		{.buf = data, .len = 1},
	};

	struct spi_buf_set spi_rx_set = { .buffers = rx_buf, .count = 1 };

    if(spi_read(uwb_spi_dev, &spi_conf, &spi_rx_set))
    {
        printk("%s: device failed to read.\n", uwb_spi_dev->name);
    }
}

extern void radio_spi_transfer_full_duplex_blocking(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    if (!device_is_ready(uwb_spi_dev)) {
		printk("%s: device not ready.\n", uwb_spi_dev->name);
		return;
	}

	struct spi_buf tx_buf = {.buf = tx_data, .len = size};
	struct spi_buf rx_buf = {.buf = rx_data, .len = size};

	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	if(spi_transceive(uwb_spi_dev, &spi_conf, &tx_set, &rx_set))
    {
        printk("%s: device failed to transfer_full_duplex\n", uwb_spi_dev->name);
    }
}

extern void radio_spi_transfer_full_duplex_non_blocking(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    if (!device_is_ready(uwb_spi_dev)) {
		printk("%s: device not ready.\n", uwb_spi_dev->name);
		return;
	}

	struct spi_buf tx_buf = {.buf = tx_data, .len = size};
	struct spi_buf rx_buf = {.buf = rx_data, .len = size};

	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	spi_transceive_cb(uwb_spi_dev, &spi_conf, &tx_set, &rx_set, Radio_DMA_IRQ_trigger, NULL);
}

extern bool radio_is_spi_busy(void)
{
    return !device_is_ready(uwb_spi_dev);
}

extern void radio_context_switch(void)
{
    NVIC_SetPendingIRQ(QSPI_IRQn);
}

extern void radio_callback_context_switch(void)
{
    NVIC_SetPendingIRQ(PWM3_IRQn);
}

extern void radio_set_spi_baudrate(spi_prescaler_t prescaler)
{
    printk("%s: Trying to set the spi baudrate while not implemented.\n", uwb_spi_dev->name);
}

/* PRIVATE FUNCTIONS **********************************************************/

/** @brief Initialize the shutdown pin.
 */
static void init_radio_shutdown_gpio(void)
{
	gpio_pin_configure_dt(&uwb_shutdown_pin, GPIO_OUTPUT_INACTIVE);
}

/** @brief Initialize the reset pin.
 */
static void init_radio_reset_gpio(void)
{
    gpio_pin_configure_dt(&uwb_reset_pin, GPIO_OUTPUT_INACTIVE);
}

/** @brief Initialize the SPI bus connected to the radio.
 */
static void init_radio_spi_peripheral(void)
{
    if (!device_is_ready(uwb_spi_dev)) {
		printk("%s: device not ready.\n", uwb_spi_dev->name);
		return;
	}

    gpio_pin_configure_dt(&uwb_cs_ctrl, GPIO_OUTPUT_INACTIVE);

    // TO VERIFY : Not sure about the  right config to set
    spi_conf.frequency = 30000000;
	spi_conf.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    //spi_conf.cs.gpio = NULL;
	spi_conf.slave = 1;
    
}

/** @brief Initialize the SPI rx and tx buffer set.
 */
static void init_radio_spi_buf(void)
{
	// const int stride = sizeof(buff[0]);

    // struct spi_buf tx_buf[1] = {
	// 	{.buf = 0, .len = (2) * stride},
	// };
	// struct spi_buf rx_buf[1] = {
	// 	{.buf = 0, .len = (2) * stride},
	// };
    // spi_tx_set.buffers = tx_buf;
    // spi_tx_set.count   = 1;
    // spi_rx_set.buffers = rx_buf;
    // spi_rx_set.count   = 1;
}

void Radio_DMA_IRQ_trigger(const struct device *dev, int status, void *userdata)
{
    //Radio_DMA_IRQHandler(NULL);
    //printk("Transfer completed");
    NVIC_SetPendingIRQ(CRYPTOCELL_IRQn);
}