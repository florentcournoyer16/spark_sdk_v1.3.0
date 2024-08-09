/** @file  swc_error.h
 *  @brief SPARK Wireless Core error codes.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_ERROR_H_
#define SWC_ERROR_H_

/* TYPES **********************************************************************/
/** @brief Wireless API error structure.
 */
typedef enum swc_error {
    SWC_ERR_NONE = 0, /*!< No error occurred */
    SWC_ERR_NOT_INITIALIZED, /*!< The Wireless Core is not initialized */
    SWC_ERR_NOT_ENOUGH_MEMORY, /*!< Not enough memory is allocated by the application for a full Wireless Core initialization */
    SWC_ERR_NULL_PTR, /*!< A NULL pointer is passed as argument */
    SWC_ERR_FAST_SYNC_WITH_DUAL_RADIO, /*!< Fast sync and dual radio are enabled but are incompatible */
    SWC_ERR_PAN_ID, /*!< The configured PAN ID is invalid */
    SWC_ERR_NETWORK_ROLE, /*!< The configured network role is invalid */
    SWC_ERR_SLEEP_LEVEL, /*!< The configured sleep level is invalid */
    SWC_ERR_IRQ_POLARITY, /*!< The configured IRQ polarity is invalid */
    SWC_ERR_SPI_MODE, /*!< The configured SPI mode is invalid */
    SWC_ERR_MODULATION, /*!< The configured modulation is invalid */
    SWC_ERR_FEC_LEVEL, /*!< The configured FEC level is invalid */
    SWC_ERR_CCA_FAIL_ACTION, /*!< The configured CCA fail action is invalid */
    SWC_ERR_CCA_THRESHOLD, /*!< The configured CCA threshold is invalid */
    SWC_ERR_LOCAL_ADDRESS, /*!< The configured local address is invalid */
    SWC_ERR_SOURCE_ADDRESS, /*!< The configured source address is invalid */
    SWC_ERR_DESTINATION_ADDRESS, /*!< The configured destination address is invalid */
    SWC_ERR_ARQ_WITH_ACK_DISABLED, /*!< ARQ is enabled while ACK is not */
    SWC_ERR_THROTTLING_ON_RX_CONNECTION, /*!< Link throttling is enabled on an RX connection */
    SWC_ERR_THROTTLING_NOT_SUPPORTED,    /*!< Link throttling is disabled on this connection */
    SWC_ERR_NO_PAYLOAD_MEM_ALLOC_ON_RX_CONNECTION, /*!< Payload memory allocation is not enabled on a RX connection */
    SWC_ERR_TX_PULSE_COUNT, /*!< The configured TX pulse count is invalid */
    SWC_ERR_TX_PULSE_COUNT_OFFSET, /*!< The configured TX pulse count offset is invalid */
    SWC_ERR_TX_PULSE_WIDTH, /*!< The configured TX pulse width is invalid */
    SWC_ERR_TX_PULSE_WIDTH_OFFSET, /*!< The configured TX pulse width offset is invalid */
    SWC_ERR_TX_PULSE_GAIN, /*!< The configured TX pulse gain is invalid */
    SWC_ERR_TX_GAIN_OFFSET, /*!< The configured TX pulse gain offset is invalid */
    SWC_ERR_RX_PULSE_COUNT, /*!< The configured RX pulse count is invalid */
    SWC_ERR_NO_BUFFER_AVAILABLE, /*!< There is no more payload buffer available from the queue */
    SWC_ERR_ADD_CHANNEL_ON_INVALID_CONNECTION, /*! A channel is added on a connection using only auto-reply timeslots */
    SWC_ERR_INTERNAL, /*!< There is an internal Wireless Core error */
    SWC_ERR_ALREADY_CONNECTED, /*!< The Wireless Core is already connected. */
    SWC_ERR_NOT_CONNECTED,     /*!< The Wireless Core is not connected. */
    SWC_ERR_DISCONNECT_TIMEOUT, /*!< The Wireless Core failed to disconnect within the timeout
                                 *   value.
                                 */
    SWC_ERR_PAYLOAD_TOO_BIG, /*!< The configured payload size exceeds the maximum value for the current connection configuration */
    SWC_ERR_SECOND_RADIO_NOT_INIT,       /*!< Dual radio is used but swc_node_radio_add() is not called twice */
    SWC_ERR_FRAGMENTATION_NOT_SUPPORTED, /*!< The function call is not supported when the frame fragmentation is enable
                                          *   on the connection. swc_connection_receive_to_buffer() should be use instead.
                                          */
    SWC_ERR_OUTIMPED, /*!< The configured output driver impedance is invalid */
    SWC_ERR_SEND_ON_RX_CONN, /*!< User tried to send on a connection that can't send */
    SWC_ERR_ZERO_TIMESLOT_SEQ_LEN, /*!< Input parameter is out of acceptable value */
    SWC_ERR_ZERO_CHAN_SEQ_LEN, /*!< Zero value was given to channel sequence length */
    SWC_ERR_MIN_QUEUE_SIZE, /*!< Minimum queue size requirement not met */
    SWC_ERR_ZERO_TIMESLOT_COUNT, /*!< Zero was given to timeslot count */
    SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, /*!< User tried to change configuration while the SWC is running */
    SWC_ERR_SEND_QUEUE_FULL, /*!< The queue of the sender is full */
    SWC_ERR_SIZE_TOO_BIG, /*!< The payload sent is greater than the available space */
    SWC_ERR_RECEIVE_QUEUE_EMPTY, /*!< The queue of the receiver is empty */
    SWC_ERR_RX_OVERRUN, /*!< New payload received and dropped because the RX queue is full */
    SWC_ERR_TIMESLOT_CONN_LIMIT_REACHED, /*!< The maximum number of connection assigned to the time slot was already reached */
    SWC_ERR_NON_ZERO_PRIORITY_WITHOUT_CONN_ID, /*!< The connection priority is not equal to zero while the connection ID protocol is disabled */
    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, /*!< Some connection fields must be identical in order for these connections
                                                    *   to be used on the same time slot using the connection priority feature
                                                    */
    SWC_ERR_MULTI_RADIO_HAL_INVALID, /*!< The multi radio hal initialization is invalid. */
    SWC_ERR_MAX_CONN_PRIORITY, /*!< The priority of the connection is higher than the maximum priority. */
} swc_error_t;

#endif /* SWC_ERROR_H_ */
