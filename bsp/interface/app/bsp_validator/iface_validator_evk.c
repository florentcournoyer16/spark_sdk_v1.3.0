/** @file  iface_validator_evk.c
 *  @brief EVK BSP interface for the BSP validator application.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_validator.h"
#include "evk.h"
#include "evk_usb_device.h"

/* CONSTANTS ******************************************************************/
#define PENDSV_IRQ_PRIORITY 13

/* TYPES **********************************************************************/
typedef void (*func_ptr)(void);
typedef enum spi_freq {
    SPI_FREQ_4_MHZ = 4000000,
    SPI_FREQ_10_MHZ = 10000000,
    SPI_FREQ_20_MHZ = 20000000,
    SPI_FREQ_40_MHZ = 40000000,
    SPI_FREQ_50_MHZ = 50000000,
} spi_freq_t;

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_bsp_init(void)
{
    evk_cfg_t evk_cfg = {
        .freq = CLK_169_98MHZ,
        .vdd = VDD_3V3,
        .pendsv_prio = PENDSV_IRQ_PRIORITY,
    };
    evk_init(&evk_cfg);
}

void iface_log_io(char *string)
{
    evk_uart_swd_write_blocking((uint8_t *)string, strlen(string));
}

void iface_set_cs(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_spi_set_cs();
}

void iface_reset_cs(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_spi_reset_cs();
}

void iface_transfer_blocking(bsp_radio_t radio_index, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    (void)radio_index;
    evk_radio_spi_transfer_full_duplex_blocking(tx_data, rx_data, size);
}

void iface_transfer_dma(bsp_radio_t radio_index, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    (void)radio_index;
    evk_radio_spi_transfer_full_duplex_non_blocking(tx_data, rx_data, size);
}

void iface_set_transceiver_shutdown_pin(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_set_shutdown_pin();
}

void iface_reset_transceiver_shutdown_pin(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_reset_shutdown_pin();
}

void iface_set_transceiver_reset_pin(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_set_reset_pin();
}

void iface_reset_transceiver_reset_pin(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_reset_reset_pin();
}

bool iface_read_transceiver_irq_pin(bsp_radio_t radio_index)
{
    (void)radio_index;
    return evk_radio_read_irq_pin();
}

void iface_time_delay(uint32_t ms)
{
    evk_timer_delay_ms(ms);
}

void iface_trigger_transceiver_irq(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_context_switch();
}

void iface_disable_transceiver_irq(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_disable_irq_it();
}

void iface_enable_transceiver_irq(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_enable_irq_it();
}

void iface_disable_spi_dma_cplt_irq(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_disable_dma_irq_it();
}

void iface_enable_spi_dma_cplt_irq(bsp_radio_t radio_index)
{
    (void)radio_index;
    evk_radio_enable_dma_irq_it();
}

void iface_set_transceiver_irq_callback(bsp_radio_t radio_index, void (*callback)(void))
{
    (void)radio_index;
    evk_set_radio_irq_callback((irq_callback)callback);
}

void iface_set_spi_transfer_complete_irq_callback(bsp_radio_t radio_index, void (*callback)(void))
{
    (void)radio_index;
    evk_set_radio_dma_rx_callback((irq_callback)callback);
}

void iface_wireless_set_low_priority_irq_callback(void (*callback)(void))
{
    evk_set_pendsv_callback((irq_callback)callback);
}

void iface_wireless_trigger_low_priority_irq(void)
{
    evk_radio_callback_context_switch();
}

