/** @file  iface_validation.h
 *  @brief BSP interface for the BSP validator application.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_VALIDATOR_H_
#define IFACE_VALIDATOR_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief List available radio index.
 */
typedef enum bsp_radio {
    /*! Select the radio index 1. */
    BSP_RADIO_1 = 0,
    /*! Select the radio index 2. */
    BSP_RADIO_2 = 1,
    /*! Indicate the number of possible radio index. */
    _BSP_RADIO_COUNT = 2
} bsp_radio_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the BSP main peripherals.
 *
 *  This function initializes peripherals such as:
 *  System clock, SPI, GPIO, UART, NVIC and timers, which
 *  are required by the functions within this BSP validator
 *  application.
 */
void iface_bsp_init(void);

/** @brief Serial output for the log feature.
 *
 *  User can provide serial IO such as UART or USB.
 *
 *  @param[in] string  Message to be printed to the serial output.
 */
void iface_log_io(char *string);

/** @brief Set the SPI Chip Select Pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_set_cs(bsp_radio_t radio_index);

/** @brief Reset the SPI Chip Select Pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_reset_cs(bsp_radio_t radio_index);

/** @brief Read and write data in full duplex blocking mode.
 *
 *  @param[in] radio_index  Selected radio index.
 *  @param[in] tx_data      Pointer to the transmit data buffer.
 *  @param[in] rx_data      Pointer to the receive data buffer.
 *  @param[in] size         Number of bytes to be transmitted.
 */
void iface_transfer_blocking(bsp_radio_t radio_index, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

/** @brief Read and write data in full duplex mode using DMA.
 *
 *  @param[in] radio_index  Selected radio index.
 *  @param[in] tx_data      Pointer to the transmit data buffer.
 *  @param[in] rx_data      Pointer to the receive data buffer.
 *  @param[in] size         Number of bytes to be transmitted.
 */
void iface_transfer_dma(bsp_radio_t radio_index, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

/** @brief Set the transceiver shutdown pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_set_transceiver_shutdown_pin(bsp_radio_t radio_index);

/** @brief Reset the transceiver shutdown pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_reset_transceiver_shutdown_pin(bsp_radio_t radio_index);

/** @brief Set the transceiver reset pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_set_transceiver_reset_pin(bsp_radio_t radio_index);

/** @brief Reset the transceiver reset pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_reset_transceiver_reset_pin(bsp_radio_t radio_index);

/** @brief Return the transceiver IRQ pin status.
 *
 *  @param[in] radio_index  Selected radio index.
 */
bool iface_read_transceiver_irq_pin(bsp_radio_t radio_index);

/** @brief Delay in milliseconds.
 *
 *  @param[in] ms  Time to wait in milliseconds.
 */
void iface_time_delay(uint32_t ms);

/** @brief Manually trigger (force) the transceiver's IRQ.
 *
 *  Force an IRQ to simulate an event from the transceiver.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_trigger_transceiver_irq(bsp_radio_t radio_index);

/** @brief Disable the transceiver's ISR.
 *
 *  Disable the interrupt service routine mapped with the
 *  transceiver's IRQ pin.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_disable_transceiver_irq(bsp_radio_t radio_index);

/** @brief Enable the transceiver's ISR.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_enable_transceiver_irq(bsp_radio_t radio_index);

/** @brief Disable the SPI DMA ISR.
 *
 *  Disable the interrupt service routine mapped with the
 *  SPI DMA transfer complete IRQ.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_disable_spi_dma_cplt_irq(bsp_radio_t radio_index);

/** @brief Enable the SPI DMA ISR.
 *
 *  @param[in] radio_index  Selected radio index.
 */
void iface_enable_spi_dma_cplt_irq(bsp_radio_t radio_index);

/** @brief Set a callback function to be run within the transceiver IRQ handler.
 *
 *  Set upon application initialization, this will enable
 *  the Wireless Core to run when the transceiver generates an IRQ.
 *
 *  @param[in] radio_index  Selected radio index.
 *  @param[in] callback     Pointer to a function without parameter or return value.
 */
void iface_set_transceiver_irq_callback(bsp_radio_t radio_index, void (*callback)(void));

/** @brief Set a callback function to be run within the SPI DMA IRQ handler.
 *
 *  Set upon application initialization, this will enable
 *  the Wireless core to run when the
 *  SPI DMA transfer complete IRQ is generated.
 *
 *  @param[in] radio_index  Selected radio index.
 *  @param[in] callback     Pointer to a function without parameter or return value.
 */
void iface_set_spi_transfer_complete_irq_callback(bsp_radio_t radio_index, void (*callback)(void));

/** @brief Set a callback function to be run within the Wireless Core low priority IRQ.
 *
 *  Set upon application initialization, this will enable
 *  the Wireless core to schedule execution of this specific function on-demand
 *  through the trigger of a low priority IRQ.
 *
 *  @param[in] callback  Pointer to a function without parameter or return value.
 */
void iface_wireless_set_low_priority_irq_callback(void (*callback)(void));

/** @brief Allow the Wireless Core to trigger a low priority IRQ.
 *
 *  Context switches should be set to a low priority.
 */
void iface_wireless_trigger_low_priority_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_VALIDATOR_H_ */
