/** @file  wps_def.h
 *  @brief Wireless Protocol Stack definitions used by multiple modules.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef WPS_DEF_H
#define WPS_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES *******************************************************************/
#include "circular_queue.h"
#include "link_cca.h"
#include "link_channel_hopping.h"
#include "link_connect_status.h"
#include "link_fallback.h"
#include "link_gain_loop.h"
#include "link_lqi.h"
#include "link_phase.h"
#include "link_protocol.h"
#include "link_random_datarate_offset.h"
#include "link_saw_arq.h"
#include "sr_api.h"
#include "sr_def.h"
#include "sr_spectral.h"
#include "wps_error.h"
#include "xlayer_queue.h"

/* CONSTANTS ******************************************************************/
#ifndef WPS_RADIO_COUNT
#define WPS_RADIO_COUNT 1 /*!< WPS radio count */
#endif

#define WPS_RADIO_FIFO_SIZE        128 /*!< WPS radio FIFO size */
#define WPS_PAYLOAD_SIZE_BYTE_SIZE 1   /*!< Size of the payload size automatically loaded in the FIFO. */

#ifndef WPS_NB_RF_CHANNEL
#define WPS_NB_RF_CHANNEL MAX_CHANNEL_COUNT /*!< Number of RF channels */
#endif

#define WPS_PATTERN_THROTTLE_GRANULARITY 20   /*!< WPS throttle ratio granularity (100 / value) */
#define WPS_DISABLE_CCA_THRESHOLD        0xFF /*!< WPS threshold to disable CCA */
#ifndef WPS_MAX_CONN_PER_TIMESLOT
#define WPS_MAX_CONN_PER_TIMESLOT 3 /*!< Maximum number of connections per time slot */
#endif
#define WPS_MAX_CONN_PRIORITY (WPS_MAX_CONN_PER_TIMESLOT - 1) /*!< Maximum priority allowed */

/* TYPES **********************************************************************/
/** @brief WPS fragment Connection instance
 */
typedef struct frag {
    bool enabled;                            /*!< Fragmentation enable flag */
    xlayer_queue_t xlayer_queue;             /*!< Fragmentation xlayer queue */
    circular_queue_t meta_data_queue_tx;     /*!< Use to track the number of fragment that has been sent*/
    uint16_t remaining_fragment;             /*!< Remaining fragment for the frame*/
    uint16_t fragment_index;                 /*!< Current fragement index */
    uint8_t transaction_id;                  /*!< Current transaction ID */
    bool dropped_frame;                      /*!< Tell whether the current frame have been dropped */
    uint16_t enqueued_count;                 /*!< Number of payloads ready to read */
    void (*tx_success_callback)(void *parg); /*!< Function called by the wps to indicate the transmission failed */
    void *tx_success_parg_callback;          /*!< TX fail callback void pointer argument */
    void (*tx_drop_callback)(void *parg);    /*!< Function called by the wps to indicate the frame has been received */
    void *tx_drop_parg_callback;             /*!< RX fail callback void pointer argument */
    void (*tx_fail_callback)(void *parg);    /*!< Function called by the wps to indicate the transmission failed */
    void *tx_fail_parg_callback;             /*!< TX fail callback void pointer argument */
    void (*rx_success_callback)(void *parg); /*!< Function called by the wps to indicate the frame has been received */
    void *rx_success_parg_callback;          /*!< RX success callback void pointer argument */
    void (*rx_fail_callback)(void *parg);    /*!< Function called by the wps to indicate the frame has been received */
    void *rx_fail_parg_callback;             /*!< RX fail callback void pointer argument */
    void (*event_callback)(void *parg);      /*!< Function called by the wps to indicate an event */
    void *event_parg_callback;               /*!< Event callback void pointer argument */
} frag_t;

/** @brief WPS Connection.
 */
typedef struct wps_connection wps_connection_t;

/** @brief WPS statistics function.
 */
typedef struct wps_stats {
    uint32_t tx_success;           /*!< Number of payload sent */
    uint32_t tx_byte_sent;         /*!< Number of byte sent */
    uint32_t tx_drop;              /*!< Number of payload dropped */
    uint32_t tx_fail;              /*!< Number of TX payload fail */
    uint32_t rx_received;          /*!< Number of payload received */
    uint32_t rx_byte_received;     /*!< Number of byte received */
    uint32_t rx_overrun;           /*!< Number of payload dropped because of an RX buffer overrun */
    uint32_t cca_pass;             /*!< Number of times one of the CCA attempts passed */
    uint32_t cca_tx_fail;             /*!< Number of times all CCA attempts failed */
    uint32_t cca_fail;       /*!< Total number of CCA fails */
} wps_stats_t;

/** @brief WPS event enum definition.
 */
typedef enum wps_event {
    /*!< No event */
    WPS_EVENT_NONE = 0,
    /*!< There is an error on the WPS */
    WPS_EVENT_ERROR,
    /*!< Connection event */
    WPS_EVENT_CONNECT,
    /*!< Disconnection event */
    WPS_EVENT_DISCONNECT
} wps_event_t;

/** @brief Phase information.
 */
typedef struct wps_phase_info {
    phase_info_t last_local_phases_info; /*!< Last Local phase info */
    phase_info_t local_phases_info;      /*!< Local phase info */
    phase_info_t remote_phases_info;     /*!< Remote phase info */
    uint8_t local_phases_count;          /*!< Count to synchronize phase information */
    uint8_t remote_phases_count;         /*!< Count to synchronize phase information */
} wps_phase_info_t;

/** @brief WPS Connection
 */
struct wps_connection {
    uint16_t source_address;      /*!< Source address */
    uint16_t destination_address; /*!< Destination address */

    uint8_t payload_size;           /*!< Frame size (only used if fixed frame size mode is enabled) */
    wps_event_t wps_event;          /*!< WPS event */
    wps_error_t wps_error;          /*!< WPS error */

    /* Layer 2 */
    bool ack_enable;                      /*!< Ack received frame or expect ack when sending frame */
    link_phase_t link_phase;              /*!< Phase information management module */
    bool auto_sync_enable;                /*!< Auto sync mode enable flag*/
    uint8_t header_size;                  /*!< Header size in bytes */
    int32_t empty_queue_max_delay;        /*!< Max time to delay the connection timeslot when connection queue is empty */
    link_protocol_t link_protocol;        /*!< Internal connection protocol */
    saw_arq_t stop_and_wait_arq;          /*!< Stop and Wait (SaW) and Automatic Repeat Query (ARQ) */
    link_cca_t cca;                       /*!< Clear Channel Assessment */
    link_fallback_t link_fallback;        /*!< Fallback Module instance */
    lqi_t lqi;                            /*!< Link quality indicator */
    lqi_t used_frame_lqi;                 /*!< WPS frames Link quality indicator (Excludes unused or sync timeslots)*/
    lqi_t channel_lqi[WPS_NB_RF_CHANNEL]; /*!< Channel frames Link quality indicator */
    wps_stats_t wps_stats;                /*!< Wireless protocol stack statistics */
    link_connect_status_t connect_status; /*!< Connection status */
    uint32_t total_cca_fail_count;        /*!< Running total of individual CCA fails */
    frag_t frag;                          /*!< Fragmentation instance */
    uint32_t total_cca_tx_fail_count;     /*!< Running total of tx fails due to CCA */
    uint8_t priority;                     /*!< Connection priority */
    uint32_t total_pkt_dropped;           /*!< Running total of packets dropped */
    bool certification_mode_enabled;      /*!< Certification mode enable flag */

    /* Link throttle */
    uint8_t pattern_count;       /*!< Current pattern array index count */
    uint8_t active_ratio;        /*!< Active timeslot ratio, in percent */
    uint8_t pattern_total_count; /*!< Total pattern array count based on reduced ratio fraction */
    bool currently_enabled;      /*!< Connection currently enabled flag */
    bool *pattern;               /*!< Pattern array pointer, need to be allocated by application and initialized to 1 */

    /* Gain loop */
    gain_loop_t gain_loop[WPS_NB_RF_CHANNEL][WPS_RADIO_COUNT]; /*!< Gain loop */

    /* Queue */
    xlayer_queue_t *free_queue;   /*!< Xlayer free queue */
    xlayer_queue_t xlayer_queue;  /*!< Cross layer queue */
    xlayer_queue_t *rx_queue;     /*!< RX queue */
    xlayer_queue_node_t *tx_node; /*!< TX node */

    /* Layer 1 */
    frame_cfg_t frame_cfg;                                       /*!< Connection frame config */
    rf_channel_t (*channel)[WPS_NB_RF_CHANNEL][WPS_RADIO_COUNT]; /*!< RF Channel information */
    packet_cfg_t packet_cfg;                                     /*!< Packet configuration */

    /* Callback */
    void (*tx_success_callback)(void *parg);         /*!< Function called by the wps to indicate the frame has been successfully transmitted */
    void (*tx_fail_callback)(void *parg);            /*!< Function called by the wps to indicate the transmission failed */
    void (*tx_drop_callback)(void *parg);            /*!< Function called by the wps to indicate a frame is dropped */
    void (*rx_success_callback)(void *parg);         /*!< Function called by the wps to indicate the frame has been received */
    void (*ranging_data_ready_callback)(void *parg); /*!< Function called by the wps to indicate ranging data readiness */
    void (*evt_callback)(void *parg);                /*!< Function called by the wps to indicate that a WPS event appened */

    void *tx_success_parg_callback;         /*!< TX success callback void pointer argument. */
    void *tx_fail_parg_callback;            /*!< TX fail callback void pointer argument. */
    void *tx_drop_parg_callback;            /*!< TX drop callback void pointer argument. */
    void *rx_success_parg_callback;         /*!< RX success callback void pointer argument. */
    void *ranging_data_ready_parg_callback; /*!< RX success callback void pointer argument. */
    void *evt_parg_callback;                /*!< Event callback void pointer argument. */
    uint64_t (*get_tick)(void);             /*!< Get free running timer tick */
    bool tx_flush;                          /*!< Flush next packet in the wps tx queue */
};

/** @brief WPS role enumeration.
 */
typedef enum wps_role {
    NETWORK_COORDINATOR = 0, /*!< Coordinator dictate the time to the whole network */
    NETWORK_NODE             /*!< Node re-adjust its timer to stay in sync */
} wps_role_t;

/** @brief Wireless Protocol Stack radio.
 *
 *  @note This is the parameter to setup one
 *        radio instance.
 */
typedef struct wps_radio {
    radio_t radio;                                    /*!< Radio instance */
    calib_vars_t *spectral_calib_vars;                /*!< Calibration variables */
    nvm_t *nvm;                                       /*!< NVM variables */
} wps_radio_t;

/** @brief Wireless Protocol Stack node configuration.
 */
typedef struct node_config {
    wps_role_t role;                  /*!< Current node role : Coordinator or node */
    uint32_t preamble_len;            /*!< Length of the preamble, in bits */
    sleep_lvl_t sleep_lvl;            /*!< Radio sleep level */
    uint32_t crc_polynomial;          /*!< Radio CRC polynomial */
    uint16_t local_address;           /*!< Node current address */
    syncword_cfg_t syncword_cfg;      /*!< Radio(s) configuration syncword */
    isi_mitig_t isi_mitig;            /*!< ISI mitigation level */
    uint8_t rx_gain;                  /*!< Default radio RX gain */
    chip_rate_cfg_t chip_rate;        /*!< Radio PHY rate (SR1120 only) */
    bool tx_jitter_enabled;           /*!< TX jitter enabled */
    uint32_t frame_lost_max_duration; /*!< Maximum frame lost duration before link is
                                       *   considered unsynced.
                                       */
} wps_node_cfg_t;

/** @brief WPS Node definition.
 *
 *  @note This is the parameters used to
 *        setup on node instance. One node
 *        can contains multiple radio.
 */
typedef struct node {
    wps_radio_t *radio;        /*!< Wireless Protocol Stack radio */
    wps_node_cfg_t cfg;        /*!< Node configuration */
    xlayer_queue_t free_queue; /*!< Free xlayer_queue */
    uint32_t queues_size;        /*!< Total node count in all connections queues */
    uint8_t max_payload_size;    /*!< Maximum frame size */
} wps_node_t;

/** @brief Received frame.
 */
typedef struct rx_frame {
    uint8_t *payload; /*!< Pointer to payload */
    uint16_t size;    /*!< Size of payload */
} wps_rx_frame;

/** @brief Phase frame.
 */
typedef struct phase_frame {
    phase_infos_t *payload; /*!< Pointer to phase info data */
    uint16_t size;          /*!< Size of array of phase data */
} wps_phase_frame;

/** @brief WPS request type enumeration.
 */
typedef enum wps_request {
    REQUEST_NONE = 0,                  /*!< No request */
    REQUEST_MAC_CHANGE_SCHEDULE_RATIO, /*!< Request allowing application to change active timeslot ratio */
    REQUEST_PHY_WRITE_REG,             /*!< Request allowing application to write to a register */
    REQUEST_PHY_READ_REG,              /*!< Request allowing application to read a register */
    REQUEST_PHY_DISCONNECT,            /*!< Request to disconnect the wireless protocol stack */
} wps_request_t;

/** @brief WPS schedule request configuration.
 */
typedef struct wps_schedule_ratio_cfg {
    uint8_t active_ratio;          /*!< Target connection current active ratio */
    uint8_t pattern_total_count;   /*!< Target connection total pattern array size */
    uint8_t pattern_current_count; /*!< Target connection pattern index count */
    wps_connection_t *target_conn; /*!< Connection to change active timeslot ratio */
    bool pattern_cfg[WPS_PATTERN_THROTTLE_GRANULARITY]; /*!< Throttle configuration pattern */
} wps_schedule_ratio_cfg_t;

/** @brief WPS write register request info structure.
 */
typedef struct wps_write_request_info {
    uint8_t target_register; /**< Target register to write data */
    uint8_t data;            /**< Data to send to the radio register */
    bool pending_request;    /**< Flag to notify that a request is pending */
} wps_write_request_info_t;

/** @brief WPS read register request info structure.
 */
typedef struct wps_read_request_info {
    uint8_t target_register; /*!< Target register to read. */
    uint8_t *rx_buffer;      /*!< RX buffer containing register value */
    bool pending_request;    /**< Flag to notify that a request is pending */
    bool *xfer_cmplt;        /*!< Bool to notify that read register is complete */
} wps_read_request_info_t;

/** @brief WPS request arrays structure configuration.
 */
typedef struct wps_request_config_info {
    wps_schedule_ratio_cfg_t *schedule_ratio_buffer; /*!< WPS schedule request structure array */
    wps_write_request_info_t *write_request_buffer;  /*!< WPS write request structure array */
    wps_read_request_info_t *read_request_buffer;    /*!< WPS read request structure array */
    uint8_t schedule_ratio_size; /*!< WPS schedule request structure array size */
    uint8_t write_request_size;  /*!< WPS write request structure array size */
    uint8_t read_request_size;   /*!< WPS read request structure array size */
} wps_request_config_info_t;

/** @brief WPS request structure configuration.
 *
 *  @note Available choice for configuration structure are
 *      - wps_schedule_ratio_cfg_t
 *      - wps_write_request_info_t
 *      - wps_read_request_info_t
 */
typedef struct wps_request_info {
    void *config;       /*!< WPS request structure configuration. */
    wps_request_t type; /*!< WPS request type enumeration */
} wps_request_info_t;

/** @brief WPS events callback.
 */
typedef void (*wps_callback_t)(void *parg);

#ifdef __cplusplus
}
#endif

#endif /* WPS_DEF_H */
