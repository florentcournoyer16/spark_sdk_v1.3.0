/** @file  wps_error.h
 *  @brief Wireless Protocol Stack error codes.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef WPS_ERROR_H_
#define WPS_ERROR_H_

/* TYPES **********************************************************************/
/** @brief WPS error enum definition.
 */
typedef enum wps_err {
    WPS_NO_ERROR = 0,                           /*!< No error */
    WPS_NOT_INIT_ERROR,                         /*!< The WPS is not initialized.*/
    WPS_RX_OVERRUN_ERROR,                       /*!< The WPS have received a frame, but the RX queue was full. Packet is discard and
                                                 *   overrun is returned.
                                                 */
    WPS_PHY_CRITICAL_ERROR,                     /*!< L1 state machine has received a signal that is not handled. */
    WPS_QUEUE_EMPTY_ERROR,                      /*!< There is no new frame to dequeue from the WPS connection. */
    WPS_QUEUE_FULL_ERROR,                       /*!< The WPS connection queue is full, new frame have not been enqueued. */
    WPS_WRONG_TX_SIZE_ERROR,                    /*!< The size of the frame is wrong. */
    WPS_WRONG_RX_SIZE_ERROR,                    /*!< The size of the frame is wrong. */
    WPS_CONN_THROTTLE_NOT_INITIALIZED_ERROR,    /*!< Connection throttle is not initialized. */
    WPS_ALREADY_CONNECTED_ERROR,                /*!< The WPS is already connected. */
    WPS_ALREADY_DISCONNECTED_ERROR,             /*!< The WPS is already disconnected. */
    WPS_CHANNEL_SEQUENCE_NOT_INITIALIZED_ERROR, /*!< Channel sequence is not initialized. */
    WPS_RADIO_NOT_INITIALIZED_ERROR,            /*!< Radio is not initialized. Call wps_radio_init function. */
    WPS_ACK_DISABLED_ERROR,                     /*!< Acknowledge has to be enabled to use the Stop and Wait module. */
    WPS_WRITE_REQUEST_QUEUE_FULL,               /*!< WPS write request queue is full */
    WPS_READ_REQUEST_QUEUE_FULL,                /*!< WPS read request queue is full */
    WPS_REQUEST_QUEUE_FULL,                     /*!< WPS request queue is full */
    WPS_WRITE_REQUEST_XFER_TOO_LARGE,           /*!< Write request issued transfer to large */
    WPS_FRAGMENT_ERROR,                         /*!< Error during MAC fragmentation */
    WPS_CONNECT_EVENT,                          /*!< Connection event */
    WPS_DISCONNECT_EVENT,                       /*!< Disonnection event */
    WPS_DISCONNECT_TIMEOUT_ERROR,               /*!< Disconnection timeout error */
    WPS_TIMESLOT_CONN_LIMIT_REACHED_ERROR,      /*!< The timeslot connection table is full */
    WPS_SCHEDULE_RATIO_REQUEST_QUEUE_FULL,      /*!< WPS schedule ratio request queue is full */
} wps_error_t;

#endif /* WPS_ERROR_H_ */
