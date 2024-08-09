/** @file wps_mac.h
 *  @brief Wireless protocol stack layer 2.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef MAC_LAYER_H_
#define MAC_LAYER_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "link_channel_hopping.h"
#include "link_ddcm.h"
#include "link_fallback.h"
#include "link_gain_loop.h"
#include "link_lqi.h"
#include "link_phase.h"
#include "link_protocol.h"
#include "link_random_datarate_offset.h"
#include "link_scheduler.h"
#include "link_tdma_sync.h"
#include "wps_callback.h"
#include "wps_def.h"
#include "wps_mac_protocols.h"
#include "wps_phy.h"
#include "xlayer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HEADER_BYTE0_SEQ_NUM_MASK      BIT(7)      /*!< MAC header byte 0, bit 7 */
#define HEADER_BYTE0_TIME_SLOT_ID_MASK BITS8(6, 0) /*!< MAC header byte 0, bit 6 downto 0 */

/* TYPES **********************************************************************/
/** @brief Wireless protocol stack MAC Layer output signal.
 */
typedef enum wps_mac_output_signal {
    MAC_SIGNAL_WPS_EMPTY = 0,        /*!< MAC layer empty output signal */
    MAC_SIGNAL_WPS_FRAME_RX_SUCCESS, /*!< MAC layer frame receive output signal */
    MAC_SIGNAL_WPS_FRAME_RX_FAIL,    /*!< MAC layer frame miss output signal */
    MAC_SIGNAL_WPS_FRAME_RX_OVERRUN, /*!< MAC layer No more space available in RX queue */
    MAC_SIGNAL_WPS_TX_SUCCESS,       /*!< MAC layer successful transmission output signal */
    MAC_SIGNAL_WPS_TX_FAIL,          /*!< MAC layer unsuccessful transmission output signal */
    MAC_SIGNAL_WPS_TX_DROP,          /*!< MAC layer dropped frame output signal */
    MAC_SIGNAL_WPS_PREPARE_DONE,     /*!< MAC layer frame prepare done signal */
    MAC_SIGNAL_SYNCING,              /*!< MAC layer Enter syncing state output signal */
} wps_mac_output_signal_t;

/** @brief Wireless protocol stack MAC Layer protocols identifiers.
 */
typedef enum wps_mac_proto_id {
    MAC_PROTO_ID_TIMESLOT_SAW = 0,      /*!< MAC layer timeslot ID and SAW protocol identifier */
    MAC_PROTO_ID_CHANNEL_INDEX,         /*!< MAC layer channel index protocol identifier */
    MAC_PROTO_ID_RDO,                   /*!< MAC layer RDO protocol identifier */
    MAC_PROTO_ID_RANGING_RESPONDER,      /*!< MAC layer ranging phase provider ID protocol identifier */
    MAC_PROTO_ID_RANGING_INITIATOR,   /*!< MAC layer ranging phase protocol identifier */
    MAC_PROTO_ID_CONNECTION_ID,         /*!< MAC layer connection ID protocol identifier */
} wps_mac_proto_id_t;

/** @brief Wireless protocol stack MAC Layer state function pointer.
 */
typedef void (*wps_mac_state_t)(void *signal_data);

/** @brief Wireless protocol stack MAC Layer output signal parameter.
 */
typedef struct wps_mac_output_signal_info {
    wps_mac_output_signal_t main_signal; /*!< Main output signal */
    wps_mac_output_signal_t auto_signal; /*!< Pending output signal */
} wps_mac_output_signal_info_t;

/** @brief Wireless protocol stack Layer 2 input signal parameters.
 */
typedef struct wps_mac_input_signal_info {
    phy_output_signal_t main_signal; /*!< MAC Layer input signal */
    phy_output_signal_t auto_signal; /*!< MAC Layer input signal */
} wps_mac_input_signal_info_t;

/** @brief Wireless protocol stack MAC Layer sync module init field.
 */
typedef struct wps_mac_sync_cfg {
    sleep_lvl_t sleep_level;  /*!< Desired sleep level for Sync */
    uint32_t preamble_len;    /*!< Frame preamble length */
    uint32_t syncword_len;    /*!< Frame syncword length */
    isi_mitig_t isi_mitig;    /*!< ISI mitigation level */
    uint8_t isi_mitig_pauses; /*!< ISI mitigation level corresponding pauses */
    bool tx_jitter_enabled;   /*!< TX jitter enable flag */
} wps_mac_sync_cfg_t;

/** @brief Wireless Protocol Stack input signals.
 */
typedef enum wps_input_signal {
    WPS_NOT_INIT,          /*!< WPS is not initialized */
    WPS_RADIO_IRQ,         /*!< WPS radio IRQ signal */
    WPS_TRANSFER_COMPLETE, /*!< WPS transfer complete signal */
    WPS_CONNECT,           /*!< WPS complete signal */
    WPS_DISCONNECT,        /*!< WPS disconnect signal */
    WPS_HALT,              /*!< WPS halt signal */
    WPS_RESUME,            /*!< WPS resume signal */
} wps_input_signal_t;

/** @brief Wireless protocol stack MAC Layer main structure.
 */
typedef struct wps_mac_struct {
    wps_mac_input_signal_info_t input_signal;   /*!< Input signal instance */
    wps_mac_output_signal_info_t output_signal; /*!< Output signal instance */

    timeslot_t *timeslot;                 /*!< Current scheduler timeslot */
    scheduler_t scheduler;                /*!< Schedule instance */
    channel_hopping_t channel_hopping;    /*!< Channel hopping instance */
    uint8_t channel_index;                /*!< Current channel hopping index */
    uint8_t network_id;                   /*!< Concurrent network ID */
    bool fast_sync_enabled;               /*!< Fast sync enable flag */
    bool delay_in_last_timeslot;          /*!< Delay was applied in last timeslot */
    uint16_t last_timeslot_delay;         /*!< Delay, in radio clock cycle, of the last timeslot. */

    uint16_t local_address;   /*!< Node address to handle RX/TX timeslot */
    uint16_t syncing_address; /*!< Syncing address */

    tdma_sync_t tdma_sync; /*!< Synchronization module instance */

    wps_role_t node_role; /*!< Current node role (Coordinator/Node) */

    xlayer_t empty_frame_tx; /*!< Xlayer instance when application TX queue is empty */
    xlayer_t empty_frame_rx; /*!< Xlayer instance when application RX queue is empty */

    xlayer_t *main_xlayer;                  /*!< MAC Layer main xlayer node */
    xlayer_t *auto_xlayer;                  /*!< MAC Layer auto xlayer node */
    xlayer_cfg_internal_t config;           /*!< Configuration */

    link_rdo_t link_rdo;              /*!< Random Datarate Offset (RDO) instance. */
    link_ddcm_t link_ddcm;            /*!< Distributed desync instance. */
    xlayer_queue_node_t *rx_node;                      /*!< RX node */
    uint8_t main_connection_id;                        /*!< Main connection ID */
    uint8_t auto_connection_id;                        /*!< Auto_reply connection ID */
    wps_connection_t *main_connection;                 /*!< Current main connection */
    wps_connection_t *auto_connection;                 /*!< Current auto reply connection */

    /* phases */
    wps_phase_info_t phase_data;      /*!< Phase data */

    void (*callback_context_switch)(void); /*!< function pointer to trigger the callback process */
    circular_queue_t callback_queue;       /*!< Circular queue instance to save the callbacks */
    circular_queue_t request_queue; /*!< Circular queue to forward application request to WPS */
    circular_queue_t *schedule_ratio_cfg_queue; /*!< WPS throttle feature configuration structure */
    circular_queue_t *write_request_queue;      /*!< WPS write register request queue */
    circular_queue_t *read_request_queue;       /*!< WPS write register request queue */

    volatile wps_input_signal_t signal; /*!< WPS current signal */

} wps_mac_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief MAC Layer initialization function.
 *
 *  @param[out] wps_mac                          MAC layer instance.
 *  @param[in]  channel_sequence                 Channel sequence.
 *  @param[in]  sync_cfg                         Synchronization module configuration.
 *  @param[in]  local_address                    Node local address.
 *  @param[in]  node_role                        Node role.
 *  @param[in]  random_channel_sequence_enabled  Random channel sequence enabled.
 *  @param[in]  network_id                       Network ID.
 *  @param[in]  frame_lost_max_duration          Maximum frame lost duration before link is
 *                                               considered unsynced.
 *  @param[in]  certification_mode_en            Certification mode enable flag.
 */
void wps_mac_init(wps_mac_t *wps_mac, channel_sequence_t *channel_sequence,
                  wps_mac_sync_cfg_t *sync_cfg, uint16_t local_address, wps_role_t node_role,
                  bool random_channel_sequence_enabled, uint8_t network_id,
                  uint32_t frame_lost_max_duration, bool certification_mode_en);

/** @brief Reset the MAC Layer object.
 *
 *  @param wps_mac  MAC Layer instance.
 */
void wps_mac_reset(wps_mac_t *wps_mac);

/** @brief Enable fast sync.
 *
 *  @param wps_mac  MAC Layer instance.
 */
void wps_mac_enable_fast_sync(wps_mac_t *wps_mac);

/** @brief Disable fast sync.
 *
 *  @param wps_mac  MAC Layer instance.
 */
void wps_mac_disable_fast_sync(wps_mac_t *wps_mac);

/** @brief WPS MAC callback from PHY.
 *
 *  @param[in] mac            MAC Layer instance.
 *  @param[in] wps_phy        PHY Layer instance.
 */
void wps_mac_phy_callback(void *mac, wps_phy_t *wps_phy);

#ifdef __cplusplus
}
#endif

#endif /* MAC_LAYER_H_ */
