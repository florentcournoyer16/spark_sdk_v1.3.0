/** @file  swc_def.h
 *  @brief SPARK Wireless Core definitions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_DEF_H_
#define SWC_DEF_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>

/* TYPES **********************************************************************/
/** @brief Radio Hardware Abstraction Layer.
 */
typedef struct swc_radio_hal {
    void (*set_shutdown_pin)(void);   /*!< Set shutdown pin level high */
    void (*reset_shutdown_pin)(void); /*!< Set shutdown pin level low */
    void (*set_reset_pin)(void);      /*!< Set reset pin level high */
    void (*reset_reset_pin)(void);    /*!< Set reset pin level low */
    bool (*read_irq_pin)(void);       /*!< Return IRQ pin state */
    void (*set_cs)(void);             /*!< Set SPI chip select pin level high */
    void (*reset_cs)(void);           /*!< Set SPI chip select pin level low */
    void (*delay_ms)(uint32_t ms);    /*!< Block for a specified number of milliseconds */
    uint64_t (*get_tick)(void);       /*!< Get tick function pointer */
    uint32_t tick_frequency_hz;       /*!< Tick frequency in Hz for the get_tick function */
    void (*transfer_full_duplex_blocking)(uint8_t *tx_data, uint8_t *rx_data,
                                          uint16_t size); /*!< SPI Transfer full duplex in blocking mode */
    void (*transfer_full_duplex_non_blocking)(uint8_t *tx_data, uint8_t *rx_data,
                                              uint16_t size); /*!< SPI Transfer full duplex in non blocking mode using DMA */
    bool (*is_spi_busy)(void);           /*!< Check if the SPI controller is busy transferring bytes */
    void (*context_switch)(void);        /*!< Trigger the radio IRQ */
    void (*disable_radio_irq)(void);     /*!< Disable radio IRQ interrupt source */
    void (*enable_radio_irq)(void);      /*!< Enable radio IRQ interrupt source */
    void (*disable_radio_dma_irq)(void); /*!< Disable SPI DMA interrupt source */
    void (*enable_radio_dma_irq)(void);  /*!< Enable SPI DMA interrupt source */
} swc_radio_hal_t;

/** @brief Node's Role.
 */
typedef enum swc_role {
    SWC_ROLE_COORDINATOR, /*!< Node acts as the Coordinator */
    SWC_ROLE_NODE         /*!< Node has no special function */
} swc_role_t;

/** @brief Node's Sleep Level.
 */
typedef enum swc_sleep_level {
    SWC_SLEEP_IDLE,    /*!< Idle sleep level */
    SWC_SLEEP_SHALLOW, /*!< Shallow sleep level */
    SWC_SLEEP_DEEP     /*!< Deep sleep level */
} swc_sleep_level_t;

/** @brief Radio's Interrupt Request Polarity.
 */
typedef enum swc_irq_polarity {
    SWC_IRQ_ACTIVE_LOW, /*!< Interrupt pin active in low state */
    SWC_IRQ_ACTIVE_HIGH /*!< Interrupt pin active in high state */
} swc_irq_polarity_t;

/** @brief Radio's SPI mode.
 */
typedef enum swc_spi_mode {
    SWC_SPI_STANDARD, /*!< SPI timings are standard */
    SWC_SPI_FAST      /*!< SPI timings are optimized for high capacitive load on the bus */
} swc_spi_mode_t;

/** @brief Connection's Modulation.
 */
typedef enum swc_modulation {
    SWC_MOD_IOOK,   /*!< Inverted on-off keying */
    SWC_MOD_2BITPPM /*!< 2-bit pulse position modulation */
} swc_modulation_t;

/** @brief Connection's Forward Error Correction Level.
 */
typedef enum swc_fec_level {
    SWC_FEC_0, /*!< FEC ratio 1.00 */
    SWC_FEC_1, /*!< FEC ratio 1.33 */
    SWC_FEC_2, /*!< FEC ratio 1.66 */
    SWC_FEC_3  /*!< FEC ratio 2.00 */
} swc_fec_level_t;

/** @brief Clear Channel Assessment Fail Action.
 */
typedef enum swc_cca_fail_action {
    SWC_CCA_FORCE_TX, /*!< Force transmission */
    SWC_CCA_ABORT_TX  /*!< Abort transmission */
} swc_cca_fail_action_t;

/** @brief Wireless core events.
 */
typedef enum swc_event {
    SWC_EVENT_NONE,       /*!< No event */
    SWC_EVENT_CONNECT,    /*!< The connection is established between nodes */
    SWC_EVENT_DISCONNECT, /*!< The connection is broken between nodes */
    SWC_EVENT_ERROR       /*!< An error occured on the wireless core */
} swc_event_t;

#endif /* SWC_DEF_H_ */
