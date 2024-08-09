/** @file sr_api_hal.h
 *  @brief SR1000 hardware abstraction layer
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_API_HAL_H_
#define SR_API_HAL_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief SR1000 API Hardware Abstraction Layer.
 *
 *  This structure contains all function pointers used to interact with the
 *  microcontroller's peripherals.
 *
 *  Every functions must be implemented. If a function is not needed, the user
 *  must implement an empty function.
 *
 *  For example, if the shutdown pin is not used, simply create :
 *  void no_shutdown_pin(void) {} and pass it to both
 *  set_shutdown_pin and reset_shutdown_pin.
 */
typedef struct {
    void (*set_shutdown_pin)(void);   /*!< Set shutdown pin HIGH */
    void (*reset_shutdown_pin)(void); /*!< Set shutdown pin LOW */
    void (*set_reset_pin)(void);      /*!< Set reset pin HIGH */
    void (*reset_reset_pin)(void);    /*!< Set reset pin LOW */
    bool (*read_irq_pin)(void);       /*!< Return IRQ pin state. 0 (LOW), 1(HIGH) */
    void (*set_cs)(void);             /*!< Set CS pin HIGH */
    void (*reset_cs)(void);           /*!< Set CS pin LOW */
    void (*delay_ms)(uint32_t ms);    /*!< Blocking delay function in milliseconds */
    uint64_t (*get_tick)(void);       /*!< Get the current tick timestamp */
    uint32_t tick_frequency_hz;       /*!< Tick frequency in Hz for the get_tick function */
    void (*transfer_full_duplex_blocking)(uint8_t *tx_data, uint8_t *rx_data,
                                          uint16_t size); /*!< SPI Transfer full duplex in blocking mode.
                                                           *    SPARK Radio only support full duplex on instructions
                                                           *    actual read and write are always in half-duplex mode.
                                                           */
    void (*transfer_full_duplex_non_blocking)(uint8_t *tx_data, uint8_t *rx_data,
                                              uint16_t size); /*!< SPI Transfer full duplex in non blocking mode using DMA.
                                                               *   Spark Radio only support full duplex on instructions
                                                               *   actual read and write are always in half-duplex mode.
                                                               *   FYI : CS Pin need to be externally controlled when
                                                               *   using this mode.
                                                               */
    bool (*is_spi_busy)(void);                                /*!< Check if the status of the busy flag in the SPI Status Register */
    void (*context_switch)(void);                             /*!< Trigger the Spark radio IRQ context */
    void (*disable_radio_irq)(void);                          /*!< Disable radio IRQ interrupt source */
    void (*enable_radio_irq)(void);                           /*!< Enable radio IRQ interrupt source */
    void (*disable_radio_dma_irq)(void);                      /*!< Disable SPI DMA interrupt source */
    void (*enable_radio_dma_irq)(void);                       /*!< Enable SPI DMA interrupt source */
} radio_hal_t;

#ifdef __cplusplus
}
#endif

#endif /* SR_API_HAL_H_ */
