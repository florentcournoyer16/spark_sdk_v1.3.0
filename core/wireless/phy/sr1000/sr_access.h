/** @file sr_access.h
 *  @brief SR1000 Protocol external access layer file.
 *
 *  @copyright Copyright (C) 2020-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_ACCESS_H_
#define SR_ACCESS_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "sr_api_hal.h"
#include "sr_def.h"
#include "sr_phy_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define WPS_RADIO_SPI_BUFFER_SIZE 200 /*!< WPS radio SPI buffer size */

/* ERROR **********************************************************************/
#ifndef ACCESS_ADV_ERR_CHECK_EN
#define ACCESS_ADV_ERR_CHECK_EN 0
#endif

/* PUBLIC FUNCTIONS ***********************************************************/
/** @brief Access sequence instance.
 */
typedef struct access_sequence {
    uint8_t index;      /*!< Sequence index */
    uint8_t capacity;   /*!< Max sequence capacity (Only support same RX/TX size) */
    bool burst_mode;    /*!< Burst mode activated flag */
    uint8_t tx_buffer[WPS_RADIO_SPI_BUFFER_SIZE]; /*!< TX buffer available for queuing register value to write */
    uint8_t rx_buffer[WPS_RADIO_SPI_BUFFER_SIZE]; /*!< RX buffer available for receiving value from register */
} access_sequence_instance_t;

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
static inline bool is_sequence_at_max_capacity(access_sequence_instance_t *access_sequence, size_t size);
static inline bool sequence_is_empty(access_sequence_instance_t *access_sequence);
static inline bool burst_mode_is_on(access_sequence_instance_t *access_sequence);
#endif

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize an Access sequence structure
 *
 *  @param[out] access_sequence  Access sequence to initialize.
 */
static inline void sr_access_append_init(access_sequence_instance_t *access_sequence)
{
    access_sequence->index      = 0;
    access_sequence->capacity   = WPS_RADIO_SPI_BUFFER_SIZE;
    access_sequence->burst_mode = false;
}

/** @brief Append a read register command to the Access sequence
 *
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[in]  target_reg       Target register to read from.
 *  @param[in]  buffer           Application accessible read buffer.
 *  @param[out] error            Access sequence error.
 *
 *  @note The double pointer design choice here is to
 *        be able to change the location of the application buffer
 *        in run-time so that the value inside is updated right when
 *        the after the transfer is done.
 */
static inline void sr_access_append_read_8(access_sequence_instance_t *access_sequence, uint8_t target_reg, uint8_t **buffer,
                                           sr_phy_error_t *error)
{
    uint32_t index = access_sequence->index;

#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    const uint8_t append_number = 2;

    if (burst_mode_is_on(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_CANT_APPEND_IN_BURST_MODE;
        return;
    }

    if (is_sequence_at_max_capacity(access_sequence, append_number)) {
        *error = ACCESS_SEQUENCE_ERR_TOO_MUCH_QUEUED_TRANSACTION;
        return;
    }

    *error = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)error;
#endif

    access_sequence->tx_buffer[index++] = target_reg;

    /* Change application pointer to the appropriate RX buffer address for real time update */
    *buffer = &access_sequence->rx_buffer[index++];

    access_sequence->index = index;
}

/** @brief Append 2 read register commands to the Access sequence
 *
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[in]  target_reg       Target register to read from.
 *  @param[in]  buffer1          First application accessible read buffer.
 *  @param[in]  buffer2          Second application accessible read buffer.
 *  @param[out] error            Access sequence error.
 *
 *  @note The double pointer design choice here is to
 *        be able to change the location of the application buffer
 *        in run-time so that the value inside is updated right when
 *        the after the transfer is done.
 */
static inline void sr_access_append_read_16(access_sequence_instance_t *access_sequence, uint8_t target_reg, uint8_t **buffer1,
                                            uint8_t **buffer2, sr_phy_error_t *error)
{
    uint32_t index = access_sequence->index;

#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    const uint8_t append_number = 2;

    if (burst_mode_is_on(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_CANT_APPEND_IN_BURST_MODE;
        return;
    }

    if (is_sequence_at_max_capacity(access_sequence, append_number)) {
        *error = ACCESS_SEQUENCE_ERR_TOO_MUCH_QUEUED_TRANSACTION;
        return;
    }

    *error = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)error;
#endif

    access_sequence->tx_buffer[index++] = target_reg;
    /* Change application pointer to the appropriate RX buffer address for real time update */
    *buffer1 = &access_sequence->rx_buffer[index++];

    access_sequence->tx_buffer[index++] = (target_reg + 1);
    /* Change application pointer to the appropriate RX buffer address for real time update */
    *buffer2 = &access_sequence->rx_buffer[index++];

    access_sequence->index = index;
}

/** @brief Append a write register command to the Access sequence
 *
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[in]  target_reg       Target register to write to.
 *  @param[in]  value            Value to write in register.
 *  @param[out] error            Access sequence error.
 */
static inline void sr_access_append_write_8(access_sequence_instance_t *access_sequence, uint8_t target_reg, uint8_t value,
                                            sr_phy_error_t *error)
{
    uint32_t index = access_sequence->index;

#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    const uint8_t append_number = 2;

    if (burst_mode_is_on(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_CANT_APPEND_IN_BURST_MODE;
        return;
    }

    if (is_sequence_at_max_capacity(access_sequence, append_number)) {
        *error = ACCESS_SEQUENCE_ERR_TOO_MUCH_QUEUED_TRANSACTION;
        return;
    }

    *error = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)error;
#endif

    access_sequence->tx_buffer[index++] = target_reg | REG_WRITE;
    access_sequence->tx_buffer[index++] = value;

    access_sequence->index = index;
}

/** @brief Append 2 write register commands to the Access sequence
 *
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[in]  target_reg       Target register to write to.
 *  @param[in]  value            Value to write in register.
 *  @param[out] error            Access sequence error.
 */
static inline void sr_access_append_write_16(access_sequence_instance_t *access_sequence, uint8_t target_reg, reg_16_bit_t value,
                                             sr_phy_error_t *error)
{
    uint32_t index = access_sequence->index;

#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    const uint8_t append_number = 2;

    if (burst_mode_is_on(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_CANT_APPEND_IN_BURST_MODE;
        return;
    }

    if (is_sequence_at_max_capacity(access_sequence, append_number)) {
        *error = ACCESS_SEQUENCE_ERR_TOO_MUCH_QUEUED_TRANSACTION;
        return;
    }

    *error = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)error;
#endif

    access_sequence->tx_buffer[index++] = target_reg | REG_WRITE;
    access_sequence->tx_buffer[index++] = value.bytes[0];
    access_sequence->tx_buffer[index++] = (target_reg + 1) | REG_WRITE;
    access_sequence->tx_buffer[index++] = value.bytes[1];

    access_sequence->index = index;
}

/** @brief Initiate a SPI full duplex transfer in blocking mode
 *
 *  @param[in]  radio_hal        Full duplex transfer implementation.
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[out] error            Access sequence error.
 */
static inline void sr_access_transfer_blocking(radio_hal_t *radio_hal, access_sequence_instance_t *access_sequence, sr_phy_error_t *error)
{
#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    if (sequence_is_empty(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_EMPTY_SEQUENCE;
        return;
    }

    access_sequence->burst_mode = false;
    *error                      = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)(error);
#endif
    radio_hal->reset_cs();
    radio_hal->transfer_full_duplex_blocking(access_sequence->tx_buffer, access_sequence->rx_buffer, access_sequence->index);
    radio_hal->set_cs();
    access_sequence->index = 0;
}

/** @brief Initiate a SPI full duplex transfer in non blocking mode
 *
 *  @param[in]  radio_hal        Full duplex transfer implementation.
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[out] error            Access sequence error.
 */
static inline void sr_access_transfer_non_blocking(radio_hal_t *radio_hal, access_sequence_instance_t *access_sequence,
                                                   sr_phy_error_t *error)
{
#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    if (sequence_is_empty(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_EMPTY_SEQUENCE;
        return;
    }

    access_sequence->burst_mode = false;
    *error                      = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)(error);
#endif
    radio_hal->reset_cs();
    radio_hal->transfer_full_duplex_non_blocking(access_sequence->tx_buffer, access_sequence->rx_buffer, access_sequence->index);
    access_sequence->index = 0;
}

/** @brief Check if SPI is busy
 *
 *  @param[in] radio_hal  Full duplex transfer implementation.
 */
static inline bool sr_access_is_spi_busy(radio_hal_t *radio_adv)
{
    return radio_adv->is_spi_busy();
}

/** @brief Reset access advance buffer index
 *
 *  @param[in] radio_hal  Full duplex transfer implementation.
 */
static inline void sr_access_reset_index(access_sequence_instance_t *access_sequence)
{
    access_sequence->index = 0;
}

/** @brief Append a write burst command to the sequence
 *
 *  @note One should not append any of the other command after
 *        a burst command because the CS pin should be lowered
 *        at the end of a burst transaction in order to access
 *        other register that are not following.
 *
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[in]  starting_reg     Starting register to write to. Subsequent value will be written to following register.
 *  @param[in]  tx_buffer        Data to transfer in burst mode.
 *  @param[in]  buffer_size      Number of byte to transfer.
 *  @param[out] error            Output error.
 */
static inline void sr_access_append_write_burst(access_sequence_instance_t *access_sequence, uint8_t starting_reg, uint8_t *tx_buffer,
                                                uint16_t buffer_size, sr_phy_error_t *error)
{
    uint32_t index = access_sequence->index;

#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    if (burst_mode_is_on(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_CANT_APPEND_IN_BURST_MODE;
        return;
    }

    if (is_sequence_at_max_capacity(access_sequence, buffer_size)) {
        *error = ACCESS_SEQUENCE_ERR_TOO_MUCH_QUEUED_TRANSACTION;
        return;
    }

    access_sequence->burst_mode = true;
    *error                      = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)error;
#endif

    access_sequence->tx_buffer[index++] = starting_reg | REG_WRITE_BURST;

    memcpy(&access_sequence->tx_buffer[index], tx_buffer, buffer_size); /* Flawfinder: ignore */

    index += buffer_size;
    access_sequence->index = index;
}

/** @brief Append a read burst command to the sequence
 *
 *  @note One should not append any of the other command after
 *        a burst command because the CS pin should be lowered
 *        at the end of a burst transaction in order to access
 *        other register that are not following.
 *
 *  @param[in]  access_sequence  Access sequence instance.
 *  @param[in]  starting_reg     Starting register to read from.
 *  @param[in]  buffer           Buffer that will contain received data.
 *  @param[in]  buffer_size      Number of byte to receive.
 *  @param[out] error            Output error.
 *
 *  @note The double pointer design choice here is to
 *        be able to change the location of the application buffer
 *        in run-time so that the value inside is updated right when
 *        the after the transfer is done.
 */
static inline void sr_access_append_read_burst(access_sequence_instance_t *access_sequence, uint8_t starting_reg, uint8_t **buffer,
                                               uint16_t rx_size, sr_phy_error_t *error)
{
    uint32_t index = access_sequence->index;
#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
    if (burst_mode_is_on(access_sequence)) {
        *error = ACCESS_SEQUENCE_ERR_CANT_APPEND_IN_BURST_MODE;
        return;
    }

    if (is_sequence_at_max_capacity(access_sequence, rx_size)) {
        *error = ACCESS_SEQUENCE_ERR_TOO_MUCH_QUEUED_TRANSACTION;
        return;
    }

    access_sequence->burst_mode = true;
    *error                      = ACCESS_SEQUENCE_ERR_NONE;
#else
    (void)error;
#endif

    access_sequence->tx_buffer[index++] = REG_READ_BURST | starting_reg;
    /* Change application pointer to the appropriate RX buffer address for real time update */
    *buffer = &access_sequence->rx_buffer[index];
    index += rx_size;

    access_sequence->index = index;
}

/** @brief Trigger a context switch to the Radio IRQ context.
 *
 *  @param[in] radio_hak  Radio HAL instance.
 */
static inline void sr_access_context_switch(radio_hal_t *radio_hal)
{
    radio_hal->context_switch();
}

/** @brief Enable the Radio DMA interrupt.
 *
 *  @param[in] radio_adv  Radio HAL instance.
 */
static inline void sr_access_enable_dma_irq(radio_hal_t *radio_hal)
{
    radio_hal->enable_radio_dma_irq();
}

/** @brief Disable the Radio DMA interrupt.
 *
 *  @param[in] radio_adv  Radio HAL instance.
 */
static inline void sr_access_disable_dma_irq(radio_hal_t *radio_hal)
{
    radio_hal->disable_radio_dma_irq();
}

/** @brief Enable the Radio external interrupt.
 *
 *  @param[in] radio_adv  Radio HAL instance.
 */
static inline void sr_access_enable_radio_irq(radio_hal_t *radio_hal)
{
    radio_hal->enable_radio_irq();
}

/** @brief Disable the Radio external interrupt.
 *
 *  @param[in] radio_adv  Radio HAL instance.
 */
static inline void sr_access_disable_radio_irq(radio_hal_t *radio_hal)
{
    radio_hal->disable_radio_irq();
}

/** @brief Initiate an SPI transfer in non blocking mode
 *
 *  @param[in]  radio_adv  Radio HAL instance.
 *  @param[in]  tx_buffer  Pointer to the buffer to send to the radio.
 *  @param[out] rx_buffer  Buffer containing the radio response.
 *  @param[in]  size       Size of the transfer.
 */
static inline void sr_access_spi_transfer_non_blocking(radio_hal_t *radio_hal, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size)
{
    radio_hal->reset_cs();
    radio_hal->transfer_full_duplex_non_blocking(tx_buffer, rx_buffer, size);
}

/** @brief Initiate an SPI transfer in blocking mode
 *
 *  @param[in]  radio_adv  Radio HAL instance.
 *  @param[in]  tx_buffer  Pointer to the buffer to send to the radio.
 *  @param[out] rx_buffer  Buffer containing the radio response.
 *  @param[in]  size       Size of the transfer.
 */
static inline void sr_access_spi_transfer_blocking(radio_hal_t *radio_hal, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size)
{
    radio_hal->reset_cs();
    radio_hal->transfer_full_duplex_blocking(tx_buffer, rx_buffer, size);
    radio_hal->set_cs();
}

/** @brief Open the communication with the radio.
 *
 *  @param[in] radio  Radio's instance.
 */
static inline void sr_access_open(radio_hal_t *radio_hal)
{
    radio_hal->reset_cs();
}

/** @brief Close the communication with the radio.
 *
 *  @param[in] radio  Radio's instance.
 */
static inline void sr_access_close(radio_hal_t *radio_hal)
{
    radio_hal->set_cs();
}

/* PRIVATE FUNCTIONS **********************************************************/
#if (ACCESS_ADV_ERR_CHECK_EN > 0U)
/** @brief Check if sequence is at max capacity.
 *
 * This method is use to check max sequence capacity right before
 * incrementing the sequence index.
 *
 *  @param[in] access_sequence  Access sequence instance.
 *  @retval True   Sequence is at max capacity.
 *  @retval False  Sequence can still be populated.
 */
static inline bool is_sequence_at_max_capacity(access_sequence_instance_t *access_sequence, size_t size)
{
    return (access_sequence->index + size) >= access_sequence->capacity;
}

/** @brief Check if provided sequence is empty
 *
 *  @param[in] access_sequence  Access sequence instance.
 *  @retval True   Sequence is empty.
 *  @retval False  Sequence is not empty.
 */
static inline bool sequence_is_empty(access_sequence_instance_t *access_sequence)
{
    return access_sequence->index == 0;
}

/** @brief Check if burst write/read have been initiated
 *
 *  @param[in] access_sequence  Access sequence instance.
 *  @retval True   Burst mode is on.
 *  @retval False  Burst mode is off.
 *
 *  @note When burst mode is activated, user can no longer
 *        append stuff to the current sequence because
 *        the SPI CS pin need to be lowered in order to
 *        continue normal SPI operation.
 */
static inline bool burst_mode_is_on(access_sequence_instance_t *access_sequence)
{
    return access_sequence->burst_mode;
}
#endif

#ifdef __cplusplus
}
#endif
#endif /* SR_ACCESS_H_ */
