/** @file wps_mac.c
 *  @brief Wireless protocol stack MAC.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps_mac.h"
#include "wps.h"
#ifdef SPARK_WPS_CFG_FILE_EXISTS
#include "spark_wps_cfg.h"
#endif

/* CONSTANTS ******************************************************************/
#define RADIO_MAX_PACKET_SIZE          255
#define SYNC_PLL_STARTUP_CYCLES        ((uint32_t)0x60)
#define SYNC_RX_SETUP_PLL_CYCLES       ((uint32_t)147)
#define MULTI_RADIO_BASE_IDX           0

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void process_main_frame_outcome(wps_mac_t *wps_mac);
static void process_auto_frame_outcome(wps_mac_t *wps_mac);
static void update_link_quality(wps_mac_t *wps_mac, wps_connection_t *connection, xlayer_t *xlayer);
static void update_sync(wps_mac_t *wps_mac);
static void process_rx_main(wps_mac_t *wps_mac);
static void process_rx_auto(wps_mac_t *wps_mac);
static void process_tx_main(wps_mac_t *wps_mac);
static void process_tx_main_empty(wps_mac_t *wps_mac);
static void process_tx_auto(wps_mac_t *wps_mac);
static void process_tx_auto_empty(wps_mac_t *wps_mac);
static void process_tx_lqi_update(wps_mac_t *wps_mac, wps_connection_t *connection,
                                  xlayer_t *xlayer);
static void prepare_frame(wps_mac_t *wps_mac, wps_phy_t *wps_phy);
static void prepare_tx_main(wps_mac_t *wps_mac);
static void prepare_rx_main(wps_mac_t *wps_mac);
static void config_tx(wps_mac_t *wps_mac, uint32_t next_channel);
static void config_rx(wps_mac_t *wps_mac, uint32_t next_channel);
static void prepare_tx_auto(wps_mac_t *wps_mac);
static void prepare_rx_auto(wps_mac_t *wps_mac);
static void process_next_timeslot(wps_mac_t *wps_mac);

static bool is_network_node(wps_mac_t *wps_mac);
static bool is_current_timeslot_tx(wps_mac_t *wps_mac);
static bool is_current_auto_reply_timeslot_tx(wps_mac_t *wps_mac);
static bool is_saw_arq_enable(wps_connection_t *connection);
static bool no_payload_received(xlayer_t *current_queue);
static void update_xlayer_sync(wps_mac_t *wps_mac, xlayer_cfg_internal_t *xlayer_cfg);
static void update_main_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer);
static void update_auto_reply_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer);
static void update_xlayer_modem_feat(wps_mac_t *wps_mac,  xlayer_cfg_internal_t *xlayer_cfg);
static void extract_header_main(wps_mac_t *wps_mac, xlayer_t *current_queue);
static void extract_header_auto(wps_mac_t *wps_mac, xlayer_t *current_queue);
static void fill_header(wps_connection_t *connection, xlayer_t *current_queue);
static void flush_timeout_frames_before_sending(wps_mac_t *wps_mac, wps_connection_t *connection,
                                                xlayer_callback_t *callback);
static void flush_tx_frame(wps_mac_t *wps_mac, wps_connection_t *connection,
                           xlayer_callback_t *callback);
static xlayer_t *get_xlayer_for_tx(wps_mac_t *wps_mac, wps_connection_t *connection);
static xlayer_t *get_xlayer_for_rx(wps_mac_t *wps_mac, wps_connection_t *connection);
static bool send_done(wps_connection_t *connection);
static void handle_link_throttle(wps_mac_t *wps_mac, uint8_t *inc_count);
static inline wps_error_t get_status_error(link_connect_status_t *link_connect_status);
static uint8_t get_highest_priority_conn_index(wps_connection_t **connections, uint8_t connection_count);
static void find_received_timeslot_and_connection_main(wps_mac_t *wps_mac);
static void find_received_timeslot_and_connection_auto(wps_mac_t *wps_mac);
static void certification_init(wps_mac_t *wps_mac);
static void certification_auto_reply_conn_config(wps_connection_t *conn_main,
                                                 wps_connection_t *conn_auto);
static void certification_send(wps_connection_t *connection);
#ifndef WPS_DISABLE_LINK_STATS
static void update_wps_stats_main(wps_mac_t *mac);
static void update_wps_stats_auto(wps_mac_t *mac);
#endif /* WPS_DISABLE_LINK_STATS */
static void update_connect_status(wps_mac_t *wps_mac, bool synced, bool ack_enabled,
                                  xlayer_t *xlayer);
static void process_pending_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy);
static void process_schedule_request(wps_mac_t *wps_mac, wps_request_info_t *request);
static void process_write_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy,
                                  wps_request_info_t *request);
static void process_read_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy,
                                 wps_request_info_t *request);
static void process_disconnect_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy);

/* PRIVATE GLOBALS ************************************************************/
uint8_t overrun_buffer[RADIO_MAX_PACKET_SIZE];

/* PUBLIC FUNCTIONS ***********************************************************/
void wps_mac_init(wps_mac_t *wps_mac, channel_sequence_t *channel_sequence,
                  wps_mac_sync_cfg_t *sync_cfg, uint16_t local_address, wps_role_t node_role,
                  bool random_channel_sequence_enabled, uint8_t network_id,
                  uint32_t frame_lost_max_duration, bool certification_mode_en)
{
    wps_mac->local_address          = local_address;
    wps_mac->node_role                   = node_role;
    wps_mac->delay_in_last_timeslot = false;
    wps_mac->last_timeslot_delay    = 0;

    wps_mac->network_id                      = network_id;

    /* Scheduler init */
    link_scheduler_init(&wps_mac->scheduler, wps_mac->local_address);
    link_scheduler_set_first_time_slot(&wps_mac->scheduler);
    link_scheduler_enable_tx(&wps_mac->scheduler);
    wps_mac->timeslot           = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
    wps_mac->main_connection_id = 0;
    wps_mac->auto_connection_id = 0;
    wps_mac->main_connection =
        link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                   wps_mac->main_connection_id);
    wps_mac->auto_connection =
        link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                   wps_mac->auto_connection_id);

    link_channel_hopping_init(&wps_mac->channel_hopping, channel_sequence,
                              random_channel_sequence_enabled, wps_mac->network_id);

    /* Sync module init */
    link_tdma_sync_init(&wps_mac->tdma_sync, sync_cfg->sleep_level,
                        SYNC_RX_SETUP_PLL_CYCLES * (sync_cfg->isi_mitig_pauses + 1),
                        frame_lost_max_duration, sync_cfg->syncword_len, sync_cfg->preamble_len,
                        SYNC_PLL_STARTUP_CYCLES, sync_cfg->isi_mitig, sync_cfg->isi_mitig_pauses,
                        local_address, wps_mac->fast_sync_enabled, sync_cfg->tx_jitter_enabled);

    if (certification_mode_en) {
        wps_mac->node_role = NETWORK_COORDINATOR;
        certification_init(wps_mac);
    }
}

void wps_mac_reset(wps_mac_t *wps_mac)
{
    /* Sync module reset */
    wps_mac->tdma_sync.frame_lost_duration = 0;
    wps_mac->tdma_sync.sync_slave_offset = 0;
    wps_mac->tdma_sync.slave_sync_state  = STATE_SYNCING;
    wps_mac->output_signal.main_signal     = MAC_SIGNAL_WPS_EMPTY;
}

void wps_mac_enable_fast_sync(wps_mac_t *wps_mac)
{
    wps_mac->fast_sync_enabled = true;
}

void wps_mac_disable_fast_sync(wps_mac_t *wps_mac)
{
    wps_mac->fast_sync_enabled = false;
}

void wps_mac_phy_callback(void *mac, wps_phy_t *wps_phy)
{
    wps_mac_t *wps_mac = (wps_mac_t *)mac;

    wps_mac->input_signal.main_signal = wps_phy_get_main_signal(wps_phy);
    wps_mac->input_signal.auto_signal = wps_phy_get_auto_signal(wps_phy);

    switch (wps_mac->input_signal.main_signal) {
    case PHY_SIGNAL_CONFIG_COMPLETE:
        process_pending_request(mac, wps_phy);
        break;
    case PHY_SIGNAL_BLOCKING_CONFIG_DONE:
        process_pending_request(mac, wps_phy);
        break;
    case PHY_SIGNAL_FRAME_SENT_ACK:
    case PHY_SIGNAL_FRAME_SENT_NACK:
    case PHY_SIGNAL_FRAME_RECEIVED:
    case PHY_SIGNAL_FRAME_MISSED:
        process_main_frame_outcome(wps_mac);
        process_auto_frame_outcome(wps_mac);
        process_next_timeslot(wps_mac);
        prepare_frame(wps_mac, wps_phy);
        wps_mac->callback_context_switch();
        break;
    case PHY_SIGNAL_CONNECT:
        process_next_timeslot(wps_mac);
        prepare_frame(wps_mac, wps_phy);
        wps_mac->callback_context_switch();
        break;
    default:
        break;
    }
}

/* PRIVATE STATE FUNCTIONS ****************************************************/
/** @brief Process main frame outcome.
 *
 *  @param[in] mac MAC layer instance.
 */
static void process_main_frame_outcome(wps_mac_t *wps_mac)
{
    switch (wps_mac->input_signal.main_signal) {
    case PHY_SIGNAL_FRAME_SENT_ACK:
    case PHY_SIGNAL_FRAME_SENT_NACK:
        if (wps_mac->main_xlayer == &wps_mac->empty_frame_tx) {
            process_tx_main_empty(wps_mac);
        } else {
            process_tx_main(wps_mac);
        }
        break;
    case PHY_SIGNAL_FRAME_RECEIVED:
    case PHY_SIGNAL_FRAME_MISSED:
        update_sync(wps_mac);
        process_rx_main(wps_mac);
        update_link_quality(wps_mac, wps_mac->main_connection, wps_mac->main_xlayer);
        break;
    default:
        break;
    }
#ifndef WPS_DISABLE_LINK_STATS
    update_wps_stats_main(wps_mac);
#endif /* WPS_DISABLE_LINK_STATS */
}

/** @brief Process auto frame outcome.
 *
 *  @param[in] mac MAC layer instance.
 */
static void process_auto_frame_outcome(wps_mac_t *wps_mac)
{
    if (wps_mac->auto_xlayer != NULL) {
        switch (wps_mac->input_signal.auto_signal) {
        case PHY_SIGNAL_FRAME_SENT_ACK:
        case PHY_SIGNAL_FRAME_SENT_NACK:
        case PHY_SIGNAL_FRAME_NOT_SENT:
            if (wps_mac->auto_xlayer == &wps_mac->empty_frame_tx) {
                process_tx_auto_empty(wps_mac);
            } else {
                process_tx_auto(wps_mac);
            }
            break;
        case PHY_SIGNAL_FRAME_RECEIVED:
        case PHY_SIGNAL_FRAME_MISSED:
            process_rx_auto(wps_mac);
            update_link_quality(wps_mac, wps_mac->auto_connection, wps_mac->auto_xlayer);
            break;
        default:
            break;
        }
#ifndef WPS_DISABLE_LINK_STATS
        update_wps_stats_auto(wps_mac);
#endif /* WPS_DISABLE_LINK_STATS */
    }
}

/** @brief Update link quality.
 *
 * This function handles LQI and gain loop module update.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void update_link_quality(wps_mac_t *wps_mac, wps_connection_t *connection, xlayer_t *xlayer)
{
    gain_loop_t *current_gain_loop = NULL;
    lqi_t *current_lqi             = NULL;
    lqi_t *current_channel_lqi     = NULL;
    uint8_t gain_index;

    current_lqi         = &connection->lqi;
    current_channel_lqi = &connection->channel_lqi[wps_mac->channel_index];
    current_gain_loop   = connection->gain_loop[wps_mac->channel_index];

    gain_index = link_gain_loop_get_gain_index(current_gain_loop);
#ifndef WPS_DISABLE_PHY_STATS
    /* Update LQI */
    link_lqi_update(current_lqi, gain_index, xlayer->frame.frame_outcome, wps_mac->config.rssi_raw,
                    wps_mac->config.rnsi_raw, wps_mac->config.phase_offset);
#ifdef WPS_ENABLE_PHY_STATS_PER_BANDS
    link_lqi_update(current_channel_lqi, gain_index, xlayer->frame.frame_outcome,
                    wps_mac->config.rssi_raw, wps_mac->config.rnsi_raw,
                    wps_mac->config.phase_offset);
#else
    (void)current_channel_lqi;
    (void)gain_index;
#endif /* WPS_ENABLE_PHY_STATS_PER_BANDS */
#else
    (void)current_channel_lqi;
    (void)current_lqi;
    (void)gain_index;
#endif /* WPS_DISABLE_PHY_STATS */
}

/** @brief Update sync.
 *
 * This function handle sync module update.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void update_sync(wps_mac_t *wps_mac)
{
    if (wps_mac->output_signal.main_signal == MAC_SIGNAL_SYNCING) {
        wps_mac->config.rx_wait_time = 0;
    }

    if (is_network_node(wps_mac)) {
        if (!link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync)) {
            link_tdma_sync_slave_find(&wps_mac->tdma_sync,
                                      wps_mac->main_xlayer->frame.frame_outcome,
                                      wps_mac->config.rx_wait_time, &wps_mac->main_connection->cca,
                                      wps_mac->config.rx_cca_retry_count);
        } else if (wps_mac->main_connection->source_address == wps_mac->syncing_address) {
            link_tdma_sync_slave_adjust(&wps_mac->tdma_sync,
                                        wps_mac->main_xlayer->frame.frame_outcome,
                                        wps_mac->config.rx_wait_time,
                                        &wps_mac->main_connection->cca,
                                        wps_mac->config.rx_cca_retry_count);
        }
    }
}

/** @brief Update the connection status for the current main connection.
 *
 *  @param[in] wps_mac      WPS MAC instance.
 *  @param[in] synced       Device is synced.
 *  @param[in] ack_enabled  Acknowledge enabled.
 */
static void update_connect_status(wps_mac_t *wps_mac, bool synced, bool ack_enabled,
                                  xlayer_t *xlayer)
{
    if (link_update_connect_status(&wps_mac->main_connection->connect_status,
                                   xlayer->frame.frame_outcome, synced, ack_enabled)) {
        wps_mac->config.callback_main.callback      = wps_mac->main_connection->evt_callback;
        wps_mac->config.callback_main.parg_callback = wps_mac->main_connection->evt_parg_callback;
        wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);

        connect_status_t status = wps_mac->main_connection->connect_status.status;

        wps_mac->main_connection->wps_event = (status == CONNECT_STATUS_CONNECTED) ?
                                                  WPS_EVENT_CONNECT :
                                                  WPS_EVENT_DISCONNECT;
    }
}

/** @brief Process reception of main frame.
 *
 * This function handles header extraction and operation after
 * the reception of valid main frame.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_rx_main(wps_mac_t *wps_mac)
{
    bool duplicate     = false;
    bool ack_enabled   = wps_mac->main_connection->ack_enable;
    bool synced = is_network_node(wps_mac) ? link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync) :
                                             true;

    link_ddcm_pll_cycles_update(&wps_mac->link_ddcm,
                                link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync));

    if (wps_mac->input_signal.main_signal != PHY_SIGNAL_FRAME_RECEIVED) {
        update_connect_status(wps_mac, synced, ack_enabled, wps_mac->main_xlayer);
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node        = NULL;
        wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_FRAME_RX_FAIL;
        return;
    }

    /* Extract Header, Current connection might be adjusted if timeslot ID don't match*/
    extract_header_main(wps_mac, wps_mac->main_xlayer);

    /* Update connection status for the current connection. */
    update_connect_status(wps_mac, synced, ack_enabled, wps_mac->main_xlayer);

    /* Copy application specific info */
    wps_mac->main_xlayer->config.rssi_raw = wps_mac->config.rssi_raw;
    wps_mac->main_xlayer->config.rnsi_raw = wps_mac->config.rnsi_raw;

    duplicate = link_saw_arq_is_rx_frame_duplicate(&wps_mac->main_connection->stop_and_wait_arq);

    /* No payload received  or duclicate */
    if (no_payload_received(wps_mac->main_xlayer) || duplicate) {
        /* Frame received is internal to MAC */
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node        = NULL;
        wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_EMPTY;
        return;
    }

#ifndef WPS_DISABLE_STATS_USED_TIMESLOTS
    lqi_t *current_wps_lqi         = &wps_mac->main_connection->used_frame_lqi;
    gain_loop_t *current_gain_loop = wps_mac->main_connection->gain_loop[wps_mac->channel_index];
    uint8_t gain_index             = link_gain_loop_get_gain_index(current_gain_loop);

    link_lqi_update(current_wps_lqi, gain_index, wps_mac->main_xlayer->frame.frame_outcome,
                    wps_mac->config.rssi_raw, wps_mac->config.rnsi_raw,
                    wps_mac->config.phase_offset);
#endif /* WPS_DISABLE_STATS_USED_TIMESLOTS */

    /* Frame is receive but there's no place for it in connection queue */
    if (!xlayer_queue_get_free_space(&wps_mac->main_connection->xlayer_queue)) {
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node        = NULL;
        wps_mac->config.callback_main.callback      = wps_mac->main_connection->evt_callback;
        wps_mac->config.callback_main.parg_callback = wps_mac->main_connection->evt_parg_callback;
        wps_mac->output_signal.main_signal          = MAC_SIGNAL_WPS_FRAME_RX_OVERRUN;
        wps_mac->main_connection->wps_error         = WPS_RX_OVERRUN_ERROR;
        wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
        return;
    }

    /* Frame successfully received */
    wps_mac->output_signal.main_signal     = MAC_SIGNAL_WPS_FRAME_RX_SUCCESS;
    wps_mac->config.callback_main.callback = wps_mac->main_connection->rx_success_callback;
    wps_mac->config.callback_main.parg_callback =
        wps_mac->main_connection->rx_success_parg_callback;
    xlayer_queue_enqueue_node(wps_mac->main_connection->rx_queue, wps_mac->rx_node);
    wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
    memcpy(&wps_mac->main_xlayer->config.phases_info, wps_mac->config.phases_info,
           sizeof(phase_info_t));
}

/** @brief Process reception of auto reply frame.
 *
 * This function handles header extraction and operation after
 * the reception of valid auto reply frame.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_rx_auto(wps_mac_t *wps_mac)
{
    link_ddcm_pll_cycles_update(&wps_mac->link_ddcm,
                                link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync));

    if (wps_mac->input_signal.auto_signal != PHY_SIGNAL_FRAME_RECEIVED) {
        update_connect_status(wps_mac, true, true, wps_mac->auto_xlayer);
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node                   = NULL;
        wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_FRAME_RX_FAIL;
        return;
    }

    /* Extract Header, Current connection might be adjusted if timeslot ID don't match*/
    extract_header_auto(wps_mac, wps_mac->auto_xlayer);

    /* Update connection status for the current connection. */
    update_connect_status(wps_mac, true, false, wps_mac->auto_xlayer);

    /* Copy application specific info */
    wps_mac->auto_xlayer->config.rssi_raw = wps_mac->config.rssi_raw;
    wps_mac->auto_xlayer->config.rnsi_raw = wps_mac->config.rnsi_raw;

    /* No payload received  or duclicate */
    if (no_payload_received(wps_mac->auto_xlayer)) {
        /* Frame received is internal to MAC */
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node                   = NULL;
        wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_EMPTY;
        return;
    }

#ifndef WPS_DISABLE_STATS_USED_TIMESLOTS
    lqi_t *current_wps_lqi         = &wps_mac->auto_connection->used_frame_lqi;
    gain_loop_t *current_gain_loop = wps_mac->auto_connection->gain_loop[wps_mac->channel_index];
    uint8_t gain_index             = link_gain_loop_get_gain_index(current_gain_loop);

    link_lqi_update(current_wps_lqi, gain_index, wps_mac->auto_xlayer->frame.frame_outcome,
                    wps_mac->config.rssi_raw, wps_mac->config.rnsi_raw,
                    wps_mac->config.phase_offset);
#endif /* WPS_DISABLE_STATS_USED_TIMESLOTS */

    /* Frame is receive but there's no place for it in connection queue */
    if (!xlayer_queue_get_free_space(&wps_mac->auto_connection->xlayer_queue)) {
        xlayer_queue_free_node(wps_mac->rx_node);
        wps_mac->rx_node                            = NULL;
        wps_mac->config.callback_auto.callback      = wps_mac->auto_connection->evt_callback;
        wps_mac->config.callback_auto.parg_callback = wps_mac->auto_connection->evt_parg_callback;
        wps_mac->output_signal.auto_signal          = MAC_SIGNAL_WPS_FRAME_RX_OVERRUN;
        wps_mac->auto_connection->wps_error         = WPS_RX_OVERRUN_ERROR;
    } else {
        /* Frame successfully received */
        wps_mac->output_signal.auto_signal     = MAC_SIGNAL_WPS_FRAME_RX_SUCCESS;
        wps_mac->config.callback_auto.callback = wps_mac->auto_connection->rx_success_callback;
        wps_mac->config.callback_auto.parg_callback =
            wps_mac->auto_connection->rx_success_parg_callback;
        xlayer_queue_enqueue_node(wps_mac->auto_connection->rx_queue, wps_mac->rx_node);
    }

    wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_auto);
}

/** @brief Process transmission of main frame.
 *
 * This function handles operation after
 * the transmission of valid main frame.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_tx_main(wps_mac_t *wps_mac)
{
    update_connect_status(wps_mac, true, wps_mac->main_connection->ack_enable,
                          wps_mac->main_xlayer);

    if (wps_mac->input_signal.main_signal == PHY_SIGNAL_FRAME_SENT_ACK) {
        wps_mac->output_signal.main_signal     = MAC_SIGNAL_WPS_TX_SUCCESS;
        wps_mac->config.callback_main.callback = wps_mac->main_connection->tx_success_callback;
        wps_mac->config.callback_main.parg_callback =
            wps_mac->main_connection->tx_success_parg_callback;
        wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
        link_saw_arq_inc_seq_num(&wps_mac->main_connection->stop_and_wait_arq);
        send_done(wps_mac->main_connection);
    } else {
        if (!wps_mac->main_connection->ack_enable) {
            wps_mac->output_signal.main_signal     = MAC_SIGNAL_WPS_TX_SUCCESS;
            wps_mac->config.callback_main.callback = wps_mac->main_connection->tx_success_callback;
            wps_mac->config.callback_main.parg_callback =
                wps_mac->main_connection->tx_success_parg_callback;
            wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
            send_done(wps_mac->main_connection);
        } else {
            wps_mac->output_signal.main_signal     = MAC_SIGNAL_WPS_TX_FAIL;
            wps_mac->config.callback_main.callback = wps_mac->main_connection->tx_fail_callback;
            wps_mac->config.callback_main.parg_callback =
                wps_mac->main_connection->tx_fail_parg_callback;
            wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
            if (!is_saw_arq_enable(wps_mac->main_connection)) {
                send_done(wps_mac->main_connection);
            }
        }
    }

#ifndef WPS_DISABLE_STATS_USED_TIMESLOTS
    link_lqi_update(&wps_mac->main_connection->used_frame_lqi,
                    link_gain_loop_get_gain_index(
                        wps_mac->main_connection->gain_loop[wps_mac->channel_index]),
                    wps_mac->main_xlayer->frame.frame_outcome, wps_mac->config.rssi_raw,
                    wps_mac->config.rnsi_raw, wps_mac->config.phase_offset);
#endif /* WPS_ENABLE_STATS_USED_TIMESLOTS */

    process_tx_lqi_update(wps_mac, wps_mac->main_connection, wps_mac->main_xlayer);
    link_ddcm_pll_cycles_update(&wps_mac->link_ddcm,
                                link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync));
    link_ddcm_post_tx_update(&wps_mac->link_ddcm, wps_mac->config.cca_try_count,
                             wps_mac->config.cca_retry_time,
                             wps_mac->output_signal.main_signal == MAC_SIGNAL_WPS_TX_SUCCESS);
}

/** @brief Process transmission of empty main frame.
 *
 * This function handles operation after
 * the transmission of empty main frame.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_tx_main_empty(wps_mac_t *wps_mac)
{
    update_connect_status(wps_mac, true, wps_mac->main_connection->ack_enable,
                          wps_mac->main_xlayer);

    wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_EMPTY;

    process_tx_lqi_update(wps_mac, wps_mac->main_connection, wps_mac->main_xlayer);
    link_ddcm_pll_cycles_update(&wps_mac->link_ddcm,
                                link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync));
}

/** @brief Process transmission of auto reply frame.
 *
 * This function handles operation after
 * the transmission of valid auto reply frame.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_tx_auto(wps_mac_t *wps_mac)
{
    update_connect_status(wps_mac, true, false, wps_mac->auto_xlayer);

    if (wps_mac->input_signal.auto_signal == PHY_SIGNAL_FRAME_NOT_SENT) {
        wps_mac->auto_xlayer->frame.frame_outcome = FRAME_WAIT;
        wps_mac->output_signal.auto_signal        = MAC_SIGNAL_WPS_TX_FAIL;
        wps_mac->config.callback_auto.callback    = wps_mac->auto_connection->tx_fail_callback;
        wps_mac->config.callback_auto.parg_callback =
            wps_mac->auto_connection->tx_fail_parg_callback;
    } else {
        wps_mac->auto_xlayer->frame.frame_outcome = FRAME_SENT_ACK_LOST;
        wps_mac->output_signal.auto_signal        = MAC_SIGNAL_WPS_TX_SUCCESS;
        wps_mac->config.callback_auto.callback    = wps_mac->auto_connection->tx_success_callback;
        wps_mac->config.callback_auto.parg_callback =
            wps_mac->auto_connection->tx_success_parg_callback;
        wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_auto);
        send_done(wps_mac->auto_connection);
    }
#ifndef WPS_DISABLE_STATS_USED_TIMESLOTS
    link_lqi_update(&wps_mac->auto_connection->used_frame_lqi,
                    link_gain_loop_get_gain_index(
                        wps_mac->auto_connection->gain_loop[wps_mac->channel_index]),
                    wps_mac->auto_xlayer->frame.frame_outcome, wps_mac->config.rssi_raw,
                    wps_mac->config.rnsi_raw, wps_mac->config.phase_offset);
#endif /* WPS_ENABLE_STATS_USED_TIMESLOTS */

    process_tx_lqi_update(wps_mac, wps_mac->auto_connection, wps_mac->auto_xlayer);
    link_ddcm_pll_cycles_update(&wps_mac->link_ddcm,
                                link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync));
    link_ddcm_post_tx_update(&wps_mac->link_ddcm, wps_mac->config.cca_try_count,
                             wps_mac->config.cca_retry_time,
                             wps_mac->output_signal.auto_signal == MAC_SIGNAL_WPS_TX_SUCCESS);
}

/** @brief Process transmission of empty auto reply frame.
 *
 * This function handles operation after
 * the transmission of empty auto reply frame.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_tx_auto_empty(wps_mac_t *wps_mac)
{
    update_connect_status(wps_mac, true, false, wps_mac->auto_xlayer);

    wps_mac->output_signal.auto_signal        = MAC_SIGNAL_WPS_EMPTY;
    wps_mac->auto_xlayer->frame.frame_outcome = FRAME_SENT_ACK_LOST;

    process_tx_lqi_update(wps_mac, wps_mac->auto_connection, wps_mac->auto_xlayer);
    link_ddcm_pll_cycles_update(&wps_mac->link_ddcm,
                                link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync));
}

static void process_tx_lqi_update(wps_mac_t *wps_mac, wps_connection_t *connection,
                                  xlayer_t *xlayer)
{
#ifndef WPS_DISABLE_PHY_STATS
    link_lqi_update(&connection->lqi,
                    link_gain_loop_get_gain_index(connection->gain_loop[wps_mac->channel_index]),
                    xlayer->frame.frame_outcome, wps_mac->config.rssi_raw, wps_mac->config.rnsi_raw,
                    wps_mac->config.phase_offset);
#endif /* WPS_ENABLE_PHY_STATS */

#ifdef WPS_ENABLE_PHY_STATS_PER_BANDS
    link_lqi_update(&connection->channel_lqi[wps_mac->channel_index],
                    link_gain_loop_get_gain_index(connection->gain_loop[wps_mac->channel_index]),
                    xlayer->frame.frame_outcome, wps_mac->config.rssi_raw, wps_mac->config.rnsi_raw,
                    wps_mac->config.phase_offset);
#endif /* WPS_ENABLE_PHY_STATS_PER_BANDS */
}

/** @brief Prepare frame.
 *
 * This function fills the mac header and send commands to the PHY to execute transfers.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void prepare_frame(wps_mac_t *wps_mac, wps_phy_t *wps_phy)
{
    if (is_current_timeslot_tx(wps_mac)) {
        /* TX timeslot */
        if (wps_mac->main_xlayer->frame.header_begin_it != NULL) {
            fill_header(wps_mac->main_connection, wps_mac->main_xlayer);
        }
    } else if (wps_mac->auto_connection != NULL) {
        /* TX timeslot auto reply */
        if (wps_mac->auto_xlayer->frame.header_begin_it != NULL) {
            fill_header(wps_mac->auto_connection, wps_mac->auto_xlayer);
        }
    }

    if (wps_mac->output_signal.main_signal == MAC_SIGNAL_SYNCING) {
        wps_phy_set_input_signal(wps_phy, PHY_SIGNAL_SYNCING);
    } else {
        wps_phy_set_input_signal(wps_phy, PHY_SIGNAL_PREPARE_RADIO);
    }
    wps_phy_set_main_xlayer(wps_phy, wps_mac->main_xlayer, &wps_mac->config);
    wps_phy_set_auto_xlayer(wps_phy, wps_mac->auto_xlayer);
    wps_phy_prepare_frame(wps_phy);
}

/** @brief Prepare main frame transmission.
 *
 * This function prepares the MAC for main frame transmission.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void prepare_tx_main(wps_mac_t *wps_mac)
{
    uint32_t next_channel  = link_channel_hopping_get_channel(&wps_mac->channel_hopping);
    uint16_t rdo_value     = link_rdo_get_offset(&wps_mac->link_rdo);
    int32_t timeslot_delay = 0;

    link_rdo_update_offset(&wps_mac->link_rdo);

    if (!is_network_node(wps_mac)) {
        timeslot_delay += link_ddcm_get_offset(&wps_mac->link_ddcm);
    }
    for (uint8_t i = 0; i < wps_mac->timeslot->main_connection_count; i++) {
        if (is_saw_arq_enable(wps_mac->timeslot->connection_main[i]) &&
            wps_mac->timeslot->connection_main[i]->stop_and_wait_arq.ttl_tick != 0) {
            flush_timeout_frames_before_sending(wps_mac, wps_mac->timeslot->connection_main[i],
                                                &wps_mac->config.callback_main);
        }
        if (wps_mac->timeslot->connection_main[i]->tx_flush) {
            flush_tx_frame(wps_mac, wps_mac->timeslot->connection_main[i],
                           &wps_mac->config.callback_main);
        }
    }
    if (wps_mac->timeslot->main_connection_count > 1) {
        wps_mac->main_connection_id =
            get_highest_priority_conn_index(wps_mac->timeslot->connection_main,
                                            wps_mac->timeslot->main_connection_count);
        wps_mac->main_connection =
            link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                       wps_mac->main_connection_id);
    }
    wps_mac->main_xlayer = get_xlayer_for_tx(wps_mac, wps_mac->main_connection);
    wps_mac->auto_xlayer = NULL;
    if (wps_mac->main_xlayer == &wps_mac->empty_frame_tx &&
        wps_mac->empty_frame_tx.frame.header_memory == NULL) {
        timeslot_delay += wps_mac->main_connection->empty_queue_max_delay;
    }
    if (wps_mac->delay_in_last_timeslot) {
        timeslot_delay -= wps_mac->last_timeslot_delay;
        wps_mac->delay_in_last_timeslot = false;
    }
    link_tdma_sync_update_tx(&wps_mac->tdma_sync,
                             timeslot_delay + link_scheduler_get_sleep_time(&wps_mac->scheduler) +
                                 rdo_value,
                             &wps_mac->main_connection->cca);
    if (wps_mac->main_xlayer == &wps_mac->empty_frame_tx &&
        wps_mac->empty_frame_tx.frame.header_memory == NULL) {
        wps_mac->last_timeslot_delay    = wps_mac->main_connection->empty_queue_max_delay;
        wps_mac->delay_in_last_timeslot = true;
    }
    wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
    wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_EMPTY;

    if (wps_mac->main_connection->connect_status.status == CONNECT_STATUS_DISCONNECTED) {
        /* Consider link broken, so maximize gain to increase chances to
         * resync at high attenuation/high range
         */
        for (uint8_t i = 0; i < MAX_CHANNEL_COUNT; i++) {
            for (uint8_t j = 0; j < WPS_RADIO_COUNT; j++) {
                link_gain_loop_reset_gain_index(&wps_mac->main_connection->gain_loop[i][j]);
            }
        }
    }

    config_tx(wps_mac, next_channel);
    update_main_xlayer_link_parameter(wps_mac, wps_mac->main_xlayer);
    update_xlayer_sync(wps_mac, &wps_mac->config);
    update_xlayer_modem_feat(wps_mac, &wps_mac->config);
}

/** @brief Prepare main frame reception.
 *
 * This function prepares the MAC for main frame reception.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void prepare_rx_main(wps_mac_t *wps_mac)
{
    uint32_t next_channel  = link_channel_hopping_get_channel(&wps_mac->channel_hopping);
    uint16_t rdo_value     = link_rdo_get_offset(&wps_mac->link_rdo);
    int32_t timeslot_delay = 0;

    link_rdo_update_offset(&wps_mac->link_rdo);

    if (wps_mac->delay_in_last_timeslot) {
        timeslot_delay -= wps_mac->last_timeslot_delay;
        wps_mac->delay_in_last_timeslot = false;
    }
    link_tdma_sync_update_rx(&wps_mac->tdma_sync,
                             timeslot_delay + link_scheduler_get_sleep_time(&wps_mac->scheduler) +
                                 rdo_value,
                             &wps_mac->main_connection->cca);
    wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
    wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_EMPTY;
    wps_mac->main_xlayer               = get_xlayer_for_rx(wps_mac, wps_mac->main_connection);
    wps_mac->auto_xlayer               = NULL;
    if ((!link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync)) &&
        (wps_mac->node_role == NETWORK_NODE) &&
        (wps_mac->main_connection->source_address == wps_mac->syncing_address)) {
        if (wps_mac->fast_sync_enabled) {
            wps_mac->output_signal.main_signal = MAC_SIGNAL_SYNCING;
            next_channel                       = (wps_mac->channel_hopping.middle_channel_idx %
                            wps_mac->channel_hopping.channel_sequence->sequence_size);
        }
    }

    if (wps_mac->main_connection->connect_status.status == CONNECT_STATUS_DISCONNECTED) {
        /* Consider link broken, so maximize gain to increase chances to
         * resync at high attenuation/high range
         */
        for (uint8_t i = 0; i < MAX_CHANNEL_COUNT; i++) {
            for (uint8_t j = 0; j < WPS_RADIO_COUNT; j++) {
                link_gain_loop_reset_gain_index(&wps_mac->main_connection->gain_loop[i][j]);
            }
        }
    }

    config_rx(wps_mac, next_channel);
    update_main_xlayer_link_parameter(wps_mac, wps_mac->main_xlayer);
    update_xlayer_sync(wps_mac, &wps_mac->config);
    update_xlayer_modem_feat(wps_mac, &wps_mac->config);
}

/** @brief Fill configuration fo TX.
 *
 * This function prepares the MAC for main frame transmission.
 *
 *  @param[in] wps_mac       MAC structure.
 *  @param[in] next_channel  Next channel.
 */
static void config_tx(wps_mac_t *wps_mac, uint32_t next_channel)
{
    uint8_t payload_size = wps_mac->main_xlayer->frame.payload_end_it -
                           wps_mac->main_xlayer->frame.payload_begin_it;
    uint8_t fallback_index = link_fallback_get_index(&wps_mac->main_connection->link_fallback,
                                                     payload_size);
    uint8_t cca_max_try_count;

    if ((wps_mac->main_connection->cca.fbk_try_count != NULL) &&
        (wps_mac->main_connection->link_fallback.threshold != NULL)) {
        cca_max_try_count = wps_mac->main_connection->cca.fbk_try_count[fallback_index];
    } else {
        cca_max_try_count = wps_mac->main_connection->cca.max_try_count;
    }
    if (cca_max_try_count == 0) {
        wps_mac->config.cca_threshold = WPS_DISABLE_CCA_THRESHOLD;
    } else {
        wps_mac->config.cca_threshold = wps_mac->main_connection->cca.threshold;
    }

    wps_mac->config.channel =
        &wps_mac->main_connection->channel[fallback_index][next_channel][MULTI_RADIO_BASE_IDX];
    wps_mac->config.packet_cfg        = wps_mac->main_connection->packet_cfg;
    wps_mac->config.cca_retry_time    = wps_mac->main_connection->cca.retry_time_pll_cycles;
    wps_mac->config.cca_max_try_count = cca_max_try_count;
    wps_mac->config.cca_try_count     = 0;
    wps_mac->config.cca_fail_action   = wps_mac->main_connection->cca.fail_action;
    wps_mac->config.sleep_level       = wps_mac->tdma_sync.sleep_mode;
    wps_mac->config.gain_loop         = wps_mac->main_connection->gain_loop[wps_mac->channel_index];
    wps_mac->config.phases_info       = &wps_mac->phase_data.local_phases_info;
    wps_mac->config.isi_mitig         = wps_mac->tdma_sync.isi_mitig;
    wps_mac->config.expect_ack        = wps_mac->main_connection->ack_enable;
    wps_mac->config.certification_header_en = wps_mac->main_connection->certification_mode_enabled;
}

/** @brief Fill configuration fo RX.
 *
 * This function prepares the MAC for main frame transmission.
 *
 *  @param[in] wps_mac       MAC structure.
 *  @param[in] next_channel  Next channel.
 */
static void config_rx(wps_mac_t *wps_mac, uint32_t next_channel)
{
    uint8_t payload_size = wps_mac->main_xlayer->frame.payload_end_it -
                           wps_mac->main_xlayer->frame.payload_begin_it;
    uint8_t fallback_index = link_fallback_get_index(&wps_mac->main_connection->link_fallback,
                                                     payload_size);
    uint8_t cca_max_try_count;

    cca_max_try_count = wps_mac->main_connection->cca.max_try_count;
    if (cca_max_try_count == 0) {
        wps_mac->config.cca_threshold = WPS_DISABLE_CCA_THRESHOLD;
    } else {
        wps_mac->config.cca_threshold = wps_mac->main_connection->cca.threshold;
    }

    wps_mac->config.channel =
        &wps_mac->main_connection->channel[fallback_index][next_channel][MULTI_RADIO_BASE_IDX];
    wps_mac->config.packet_cfg        = wps_mac->main_connection->packet_cfg;
    wps_mac->config.cca_retry_time    = wps_mac->main_connection->cca.retry_time_pll_cycles;
    wps_mac->config.cca_max_try_count = cca_max_try_count;
    wps_mac->config.cca_try_count     = 0;
    wps_mac->config.cca_fail_action   = wps_mac->main_connection->cca.fail_action;
    wps_mac->config.sleep_level       = wps_mac->tdma_sync.sleep_mode;
    wps_mac->config.gain_loop         = wps_mac->main_connection->gain_loop[wps_mac->channel_index];
    wps_mac->config.phases_info       = &wps_mac->phase_data.local_phases_info;
    wps_mac->config.isi_mitig         = wps_mac->tdma_sync.isi_mitig;
    wps_mac->config.expect_ack        = wps_mac->main_connection->ack_enable;
    wps_mac->config.certification_header_en = wps_mac->main_connection->certification_mode_enabled;
}

/** @brief Prepare auto reply frame transmission.
 *
 * This function prepares the MAC for auto reply frame transmission.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void prepare_tx_auto(wps_mac_t *wps_mac)
{
    for (uint8_t i = 0; i < wps_mac->timeslot->auto_connection_count; i++) {
        if (wps_mac->timeslot->connection_auto_reply[i]->tx_flush) {
            flush_tx_frame(wps_mac, wps_mac->timeslot->connection_auto_reply[i],
                           &wps_mac->config.callback_auto);
        }
    }
    if (wps_mac->timeslot->auto_connection_count > 1) {
        wps_mac->auto_connection_id =
            get_highest_priority_conn_index(wps_mac->timeslot->connection_auto_reply,
                                            wps_mac->timeslot->auto_connection_count);
        wps_mac->auto_connection =
            link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                       wps_mac->auto_connection_id);
    }

    wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
    wps_mac->auto_xlayer               = get_xlayer_for_tx(wps_mac, wps_mac->auto_connection);
    update_auto_reply_xlayer_link_parameter(wps_mac, wps_mac->auto_xlayer);
}

/** @brief Prepare auto reply frame reception.
 *
 * This function prepares the MAC for auto reply frame reception.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void prepare_rx_auto(wps_mac_t *wps_mac)
{
    wps_mac->output_signal.auto_signal = MAC_SIGNAL_WPS_PREPARE_DONE;
    wps_mac->auto_xlayer               = get_xlayer_for_rx(wps_mac, wps_mac->auto_connection);
    update_auto_reply_xlayer_link_parameter(wps_mac, wps_mac->auto_xlayer);
}

/** @brief Process next time slot.
 *
 * This state get the next timeslot to handle and executes accordingy.
 *
 *  @param[in] wps_mac  MAC structure.
 */
static void process_next_timeslot(wps_mac_t *wps_mac)
{
    uint8_t inc_count;

    link_scheduler_reset_sleep_time(&wps_mac->scheduler);
    inc_count = link_scheduler_increment_time_slot(&wps_mac->scheduler);
    handle_link_throttle(wps_mac, &inc_count);
    link_channel_hopping_increment_sequence(&wps_mac->channel_hopping, inc_count);

    wps_mac->channel_index   = link_channel_hopping_get_channel(&wps_mac->channel_hopping);
    wps_mac->timeslot        = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
    wps_mac->main_connection = link_scheduler_get_current_main_connection(&wps_mac->scheduler, 0);
    wps_mac->auto_connection = link_scheduler_get_current_auto_connection(&wps_mac->scheduler, 0);

    if (is_current_timeslot_tx(wps_mac)) {
        prepare_tx_main(wps_mac);
    } else {
        prepare_rx_main(wps_mac);
    }
    if (wps_mac->auto_connection != NULL) {
        if (is_current_auto_reply_timeslot_tx(wps_mac)) {
            prepare_tx_auto(wps_mac);
        } else {
            prepare_rx_auto(wps_mac);
        }
    }
}

/** @brief Output if node role is NETWORK_NODE.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Node role is NETWORK_NODE.
 *  @retval False  Node role is not NETWORK_NODE.
 */
static bool is_network_node(wps_mac_t *wps_mac)
{
    return (wps_mac->node_role == NETWORK_NODE);
}

/** @brief Output if current main connection timeslot is TX.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Main connection timeslot is TX.
 *  @retval False  Main connection timeslot is RX.
 */
static bool is_current_timeslot_tx(wps_mac_t *wps_mac)
{
    return (wps_mac->main_connection->source_address == wps_mac->local_address);
}

/** @brief Output if auto-reply connection timeslot is TX.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @retval True   Auto-reply connection timeslot is TX.
 *  @retval False  Auto-reply connection timeslot is RX.
 */
static bool is_current_auto_reply_timeslot_tx(wps_mac_t *wps_mac)
{
    return (wps_mac->auto_connection->source_address == wps_mac->local_address);
}

/** @brief Return if stop and wait is enable or not.
 *
 *  @param wps_mac  MAC structure.
 *  @retval True   Stop and wait ARQ is enable.
 *  @retval False  Stop and wait ARQ is disable.
 */
static bool is_saw_arq_enable(wps_connection_t *connection)
{
    return connection->stop_and_wait_arq.enable;
}

/** @brief Update the xlayer sync module value for PHY.
 *
 *  @param[in]  wps_mac         MAC structure.
 *  @param[out] current_xlayer  Current xlayer node to update.
 */
static void update_xlayer_sync(wps_mac_t *wps_mac, xlayer_cfg_internal_t *xlayer_cfg)
{
    xlayer_cfg->power_up_delay = link_tdma_sync_get_pwr_up(&wps_mac->tdma_sync);
    xlayer_cfg->rx_timeout     = link_tdma_sync_get_timeout(&wps_mac->tdma_sync);
    xlayer_cfg->sleep_time     = link_tdma_sync_get_sleep_cycles(&wps_mac->tdma_sync);
}

/** @brief Update the main connection xlayer gain loop value for PHY.
 *
 *  @param[in] wps_mac MAC structure.
 *  @param[in] xlayer  xlayer node to update.
 */
static void update_main_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer)
{
    xlayer->frame.destination_address = wps_mac->main_connection->destination_address;
    xlayer->frame.source_address      = wps_mac->main_connection->source_address;
}

/** @brief Update the main connection xlayer gain loop value for PHY.
 *
 *  @param[in] wps_mac MAC structure.
 *  @param[in] xlayer  xlayer node to update.
 */
static void update_auto_reply_xlayer_link_parameter(wps_mac_t *wps_mac, xlayer_t *xlayer)
{
    if (xlayer != NULL) {
        xlayer->frame.destination_address = wps_mac->auto_connection->destination_address;
        xlayer->frame.source_address      = wps_mac->auto_connection->source_address;
    }
}

/** @brief Update the main connection xlayer modem feat value for PHY.
 *
 *  @param[in] wps_mac MAC structure.
 *  @param[in] xlayer  xlayer node to update.
 */
static void update_xlayer_modem_feat(wps_mac_t *wps_mac,  xlayer_cfg_internal_t *xlayer_cfg)
{
    xlayer_cfg->fec        = wps_mac->main_connection->frame_cfg.fec;
    xlayer_cfg->modulation = wps_mac->main_connection->frame_cfg.modulation;
}

/** @brief Return the corresponding queue for TX depending on the input connection.
 *
 * For TX timeslot, Application should have enqueue a node
 * inside the queue, so the MAC only need to get the front of
 * the queue in order to get the good node for the process.
 *
 *  @param[in] connection  Queue node connection.
 *  @return Current pending node in queue.
 */
static xlayer_t *get_xlayer_for_tx(wps_mac_t *wps_mac, wps_connection_t *connection)
{
    xlayer_t *free_xlayer;
    xlayer_queue_node_t *node;
    bool unsync = ((wps_mac->tdma_sync.slave_sync_state == STATE_SYNCING) && (wps_mac->node_role == NETWORK_NODE));

    if (connection->currently_enabled) {
        node = xlayer_queue_get_node(&connection->xlayer_queue);
    } else {
        node = NULL;
    }
    if (node == NULL) {
        free_xlayer = NULL;
    } else {
        free_xlayer = &node->xlayer;
    }

    if (free_xlayer == NULL || unsync) {
        if (connection->auto_sync_enable && !unsync) {
            wps_mac->empty_frame_tx.frame.header_memory = overrun_buffer;
            wps_mac->empty_frame_tx.frame.header_end_it = overrun_buffer + connection->header_size;
        } else {
            wps_mac->empty_frame_tx.frame.header_memory = NULL;
            wps_mac->empty_frame_tx.frame.header_end_it = NULL;
        }
        wps_mac->empty_frame_tx.frame.header_begin_it  = wps_mac->empty_frame_tx.frame.header_end_it;
        wps_mac->empty_frame_tx.frame.payload_end_it   = wps_mac->empty_frame_tx.frame.header_end_it;
        wps_mac->empty_frame_tx.frame.payload_begin_it = wps_mac->empty_frame_tx.frame.header_end_it;
        free_xlayer                                    = &wps_mac->empty_frame_tx;
        wps_mac->empty_frame_tx.frame.time_stamp       = connection->get_tick();
    } else {
        free_xlayer->frame.header_begin_it = free_xlayer->frame.header_end_it;
    }

    return free_xlayer;
}

/** @brief Return the corresponding queue for RX depending on the input connection.
 *
 * For RX timeslot, MAC should get the first free slot, WPS
 * should enqueue for application .
 *
 *  @param[in] connection  Queue node connection.
 *  @return Current pending node in queue.
 */
static xlayer_t *get_xlayer_for_rx(wps_mac_t *wps_mac, wps_connection_t *connection)
{
    wps_mac->rx_node = xlayer_queue_get_free_node(connection->free_queue);

    /* if free node is not available, will return an empty frame*/
    if (wps_mac->rx_node == NULL) {
        wps_mac->empty_frame_rx.frame.header_memory       = overrun_buffer;
        wps_mac->empty_frame_rx.frame.header_end_it       = overrun_buffer;
        wps_mac->empty_frame_rx.frame.header_begin_it     = wps_mac->empty_frame_rx.frame.header_end_it;
        wps_mac->empty_frame_rx.frame.payload_end_it      = wps_mac->empty_frame_rx.frame.header_end_it;
        wps_mac->empty_frame_rx.frame.payload_begin_it    = wps_mac->empty_frame_rx.frame.header_end_it;
        wps_mac->empty_frame_rx.frame.payload_memory_size = connection->payload_size;
        wps_mac->empty_frame_rx.frame.header_memory_size  = connection->header_size;
        return &wps_mac->empty_frame_rx;
    }

    wps_mac->rx_node->xlayer.frame.payload_memory_size = connection->payload_size;
    wps_mac->rx_node->xlayer.frame.header_memory_size = connection->header_size;
    return &wps_mac->rx_node->xlayer;
}

/** @brief Extract the header fields from a received main frame.
 *
 *  @param[out] wps_mac        Frame header MAC instance.
 *  @param[in]  current_queue  xlayer header.
 */
static void extract_header_main(wps_mac_t *wps_mac, xlayer_t *current_queue)
{
    find_received_timeslot_and_connection_main(wps_mac);

    /* MAC should always be the first to extract */
    current_queue->frame.header_begin_it = current_queue->frame.header_memory;
    if (current_queue->frame.header_begin_it != NULL) {
        /* First byte should always be the radio automatic response */
        current_queue->frame.header_begin_it++;
        link_protocol_receive_buffer(&wps_mac->main_connection->link_protocol,
                                     wps_mac->main_xlayer->frame.header_begin_it,
                                     wps_mac->main_connection->header_size);
        wps_mac->main_connection =
            link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                       wps_mac->main_connection_id);
        wps_mac->main_xlayer->frame.header_begin_it += wps_mac->main_connection->header_size;

        /* Assign payload pointer as if there is no other layer on top. */
        current_queue->frame.payload_begin_it = current_queue->frame.header_begin_it;
    }
}

/** @brief Extract the header fields from a received auto reply frame.
 *
 *  @param[out] wps_mac        Frame header MAC instance.
 *  @param[in]  current_queue  xlayer header.
 */
static void extract_header_auto(wps_mac_t *wps_mac, xlayer_t *current_queue)
{
    find_received_timeslot_and_connection_auto(wps_mac);

    /* MAC should always be the first to extract */
    current_queue->frame.header_begin_it = current_queue->frame.header_memory;
    if (current_queue->frame.header_begin_it != NULL) {
        /* First byte should always be the radio automatic response */
        current_queue->frame.header_begin_it++;

        link_protocol_receive_buffer(&wps_mac->auto_connection->link_protocol,
                                     wps_mac->auto_xlayer->frame.header_begin_it,
                                     wps_mac->auto_connection->header_size);
        wps_mac->auto_connection =
            link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                       wps_mac->auto_connection_id);
        wps_mac->auto_xlayer->frame.header_begin_it += wps_mac->auto_connection->header_size;

        /* Assign payload pointer as if there is no other layer on top. */
        current_queue->frame.payload_begin_it = current_queue->frame.header_begin_it;
    }
}

/** @brief Fill the header fields for a TX node queue.
 *
 *  @param[in]  connection     Connection.
 *  @param[in]  current_queue  header xlayer.
 */
static void fill_header(wps_connection_t *connection, xlayer_t *current_queue)
{
    uint32_t size = 0;

    current_queue->frame.header_begin_it -= connection->header_size;
    link_protocol_send_buffer(&connection->link_protocol, current_queue->frame.header_begin_it,
                              &size);
}

/** @brief Fill the header fields for a TX node queue.
 *
 *  @param[in] current_queue  Current xlayer.
 *  @retval True   No payload have been received.
 *  @retval False  Payload have been received.
 */
static bool no_payload_received(xlayer_t *current_queue)
{
    return (current_queue->frame.header_begin_it == current_queue->frame.payload_end_it);
}

/** @brief Finish a transmission.
 *
 *  @param[in] connection wps_connection_t instance.
 *  @retval true   On success.
 *  @retval false  On error.
 */
static bool send_done(wps_connection_t *connection)
{
    xlayer_queue_node_t *node;

    connection->tx_flush = false;
    node = xlayer_queue_dequeue_node(&connection->xlayer_queue);
    xlayer_queue_free_node(node);
    if (connection->certification_mode_enabled) {
        certification_send(connection);
    }
    return true;
}

/** @brief  Check and flush timeout frame before sending to PHY.
 *
 *  @param wps_mac  WPS MAC instance.
 */
static void flush_timeout_frames_before_sending(wps_mac_t *wps_mac, wps_connection_t *connection,
                                                xlayer_callback_t *callback)
{
    bool timeout = false;
    xlayer_queue_node_t *xlayer_queue_node;

    do {
        xlayer_queue_node = xlayer_queue_get_node(&connection->xlayer_queue);
        if ((xlayer_queue_node != NULL) && (&xlayer_queue_node->xlayer != NULL)) {
            timeout = link_saw_arq_is_frame_timeout(&connection->stop_and_wait_arq,
                                                    xlayer_queue_node->xlayer.frame.time_stamp,
                                                    xlayer_queue_node->xlayer.frame.retry_count++,
                                                    connection->get_tick());
            if (timeout) {
                callback->callback      = connection->tx_drop_callback;
                callback->parg_callback = connection->tx_drop_parg_callback;
                wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
                wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_TX_DROP;
#ifndef WPS_DISABLE_LINK_STATS
                wps_mac->main_connection->wps_stats.tx_drop++;
                wps_mac->main_connection->total_pkt_dropped++;
#endif /* WPS_DISABLE_LINK_STATS */
                send_done(connection);
            }
        } else {
            timeout = false;
        }
    } while (timeout);
}

/** @brief  Flush the next packet from the wps tx queue.
 *
 *  @param wps_mac  WPS MAC instance.
 */
static void flush_tx_frame(wps_mac_t *wps_mac, wps_connection_t *connection,
                           xlayer_callback_t *callback)
{
    xlayer_t *xlayer;

    xlayer = &xlayer_queue_get_node(&connection->xlayer_queue)->xlayer;

    if (xlayer != NULL) {
        callback->callback      = connection->tx_drop_callback;
        callback->parg_callback = connection->tx_drop_parg_callback;
        wps_callback_enqueue(&wps_mac->callback_queue, &wps_mac->config.callback_main);
        wps_mac->output_signal.main_signal = MAC_SIGNAL_WPS_TX_DROP;
#ifndef WPS_DISABLE_LINK_STATS
        wps_mac->main_connection->wps_stats.tx_drop++;
        wps_mac->main_connection->total_pkt_dropped++;
#endif /* WPS_DISABLE_LINK_STATS */
        send_done(connection);
    }
}

#ifndef WPS_DISABLE_LINK_STATS
/** @brief Update WPS statistics
 *
 *  @param[in] MAC  WPS MAC instance.
 */
static void update_wps_stats_main(wps_mac_t *mac)
{
    switch (mac->output_signal.main_signal) {
    case MAC_SIGNAL_WPS_FRAME_RX_SUCCESS:
        mac->main_connection->wps_stats.rx_received++;
        mac->main_connection->wps_stats.rx_byte_received +=
            (mac->main_xlayer->frame.payload_end_it - mac->main_xlayer->frame.payload_begin_it);
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_OVERRUN:
        mac->main_connection->wps_stats.rx_overrun++;
        break;
    case MAC_SIGNAL_WPS_TX_SUCCESS:
        mac->main_connection->wps_stats.tx_success++;
        mac->main_connection->wps_stats.tx_byte_sent += (mac->main_xlayer->frame.payload_end_it -
                                                         mac->main_xlayer->frame.payload_begin_it);
        if (mac->main_connection->cca.enable) {
            if (mac->config.cca_try_count >= mac->config.cca_max_try_count) {
                mac->main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->main_connection->wps_stats.cca_tx_fail++;
                mac->main_connection->total_cca_tx_fail_count++;
            } else if (mac->main_xlayer->frame.frame_outcome != FRAME_WAIT) {
                mac->main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->main_connection->wps_stats.cca_pass++;
            }
        }
        break;
    case MAC_SIGNAL_WPS_TX_FAIL:

        mac->main_connection->wps_stats.tx_fail++;

        if (mac->main_connection->cca.enable) {
            if (mac->config.cca_try_count >= mac->config.cca_max_try_count) {
                mac->main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->main_connection->wps_stats.cca_tx_fail++;
                mac->main_connection->total_cca_tx_fail_count++;
            } else if (mac->main_xlayer->frame.frame_outcome != FRAME_WAIT) {
                mac->main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->main_connection->wps_stats.cca_pass++;
            }
        }
        break;
    case MAC_SIGNAL_WPS_TX_DROP:
        mac->main_connection->wps_stats.tx_drop++;
        mac->main_connection->total_pkt_dropped++;
        break;
    case MAC_SIGNAL_WPS_EMPTY:
        /* PHY NACK signal occurred but SAW has not yet trigger, handle CCA stats only. */
        if (mac->main_connection->cca.enable) {
            if (mac->config.cca_try_count >= mac->config.cca_max_try_count) {
                mac->main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->main_connection->wps_stats.cca_tx_fail++;
                mac->main_connection->total_cca_tx_fail_count++;
            } else if (mac->main_xlayer->frame.frame_outcome != FRAME_WAIT) {
                mac->main_connection->wps_stats.cca_fail += mac->config.cca_try_count;
                mac->main_connection->total_cca_fail_count += mac->config.cca_try_count;
                mac->main_connection->wps_stats.cca_pass++;
            }
        }
        break;
    default:
        break;
    }
}

/** @brief Update WPS statistics
 *
 *  @param[in] MAC  WPS MAC instance.
 */
static void update_wps_stats_auto(wps_mac_t *mac)
{
    switch (mac->output_signal.auto_signal) {
    case MAC_SIGNAL_WPS_FRAME_RX_SUCCESS:
        mac->auto_connection->wps_stats.rx_received++;
        mac->auto_connection->wps_stats.rx_byte_received +=
            (mac->auto_xlayer->frame.payload_end_it - mac->auto_xlayer->frame.payload_begin_it);
        break;
    case MAC_SIGNAL_WPS_FRAME_RX_OVERRUN:
        mac->auto_connection->wps_stats.rx_overrun++;
        break;
    case MAC_SIGNAL_WPS_TX_SUCCESS:
        mac->auto_connection->wps_stats.tx_success++;
        mac->auto_connection->wps_stats.tx_byte_sent += (mac->auto_xlayer->frame.payload_end_it -
                                                         mac->auto_xlayer->frame.payload_begin_it);
        break;
    case MAC_SIGNAL_WPS_TX_FAIL:
        mac->auto_connection->wps_stats.tx_fail++;
        break;
    case MAC_SIGNAL_WPS_TX_DROP:
        mac->auto_connection->wps_stats.tx_drop++;
        mac->main_connection->total_pkt_dropped++;
        break;
    default:
        break;
    }
}
#endif /* WPS_DISABLE_LINK_STATS */

/** @brief Handle link throttle.
 *
 *  @param[in] wps_mac    WPS MAC instance.
 *  @param[in] inc_count  Increment count.
 */
static void handle_link_throttle(wps_mac_t *wps_mac, uint8_t *inc_count)
{
    wps_connection_t *candidate_connection;
    timeslot_t *time_slot;
    bool ts_enabled;

    do {
        time_slot = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
        for (uint8_t i = 0; i < time_slot->main_connection_count; i++) {
            candidate_connection                    = time_slot->connection_main[i];
            candidate_connection->currently_enabled = true;

            if (candidate_connection->pattern != NULL) {
                candidate_connection->pattern_count = (candidate_connection->pattern_count + 1) %
                                                      candidate_connection->pattern_total_count;

                candidate_connection->currently_enabled =
                    candidate_connection->pattern[candidate_connection->pattern_count];
            }
        }

        for (uint8_t i = 0; i < time_slot->auto_connection_count; i++) {
            candidate_connection                    = time_slot->connection_auto_reply[i];
            candidate_connection->currently_enabled = true;
        }

        ts_enabled = false;
        for (uint8_t i = 0; i < time_slot->main_connection_count; i++) {
            ts_enabled = time_slot->connection_main[i]->currently_enabled;
            if (ts_enabled == true) {
                break;
            }
        }

        if (ts_enabled == false) {
            *inc_count += link_scheduler_increment_time_slot(&wps_mac->scheduler);
        }

    } while (ts_enabled == false);
}

/** @brief Get the event associated with the current connection status.
 *
 *  @param[in] link_connect_status Link connection status module instance.
 *  @retval [WPS_EVENT_CONNECT]    if the status is connected.
 *  @retval [WPS_EVENT_DISCONNECT] if the status is disconnected.
 */
static inline wps_error_t get_status_error(link_connect_status_t *link_connect_status)
{
    return (link_connect_status->status == CONNECT_STATUS_CONNECTED) ? WPS_CONNECT_EVENT : WPS_DISCONNECT_EVENT;
}

/** @brief Get the index of the highest priority connection.
 *
 *  @param[in] connections       Connection table.
 *  @param[in] connection_count  Connection count.
 */
static uint8_t get_highest_priority_conn_index(wps_connection_t **connections, uint8_t connection_count)
{
    xlayer_t *free_xlayer;
    xlayer_queue_node_t *node;
    uint8_t min_prio  = WPS_MAX_CONN_PRIORITY + 1;
    uint8_t min_index = 0;

    for (uint8_t i = 0; i < connection_count; i++) {
        if (connections[i]->currently_enabled) {
            node = xlayer_queue_get_node(&connections[i]->xlayer_queue);
        } else {
            node = NULL;
        }
        if (node == NULL) {
            free_xlayer = NULL;
        } else {
            free_xlayer = &node->xlayer;
        }
        if (free_xlayer != NULL) {
            if (connections[i]->priority < min_prio) {
                min_prio = connections[i]->priority;
                min_index = i;
            }
            if (min_prio == 0) {
                break;
            }

        }
    }
    return min_index;
}

/** @brief Find the received time slot ID and connection ID for main frame.
 *
 *  @param[out] wps_mac  Frame header MAC instance.
 */
static void find_received_timeslot_and_connection_main(wps_mac_t *wps_mac)
{
    uint8_t ts_id_saw;
    uint8_t time_slot_id;
    uint8_t connection_id;
    wps_connection_t *connection;
    uint8_t connection_count;
    uint8_t *conn_id;

    connection       = wps_mac->main_connection;
    connection_count = wps_mac->timeslot->main_connection_count;
    conn_id          = &wps_mac->main_connection_id;
    if (is_network_node(wps_mac)) {
        ts_id_saw = *(
            wps_mac->main_xlayer->frame.header_begin_it +
            link_protocol_get_buffer_offset(&connection->link_protocol, MAC_PROTO_ID_TIMESLOT_SAW));
        time_slot_id = MASK2VAL(ts_id_saw, HEADER_BYTE0_TIME_SLOT_ID_MASK);
        if (time_slot_id < wps_mac->scheduler.schedule.size) {
            if (link_scheduler_get_next_timeslot_index(&wps_mac->scheduler) != time_slot_id) {
                link_scheduler_set_mismatch(&wps_mac->scheduler);
            }
            link_scheduler_set_time_slot_i(&wps_mac->scheduler, time_slot_id);
        }
    }
    if ((!link_tdma_sync_is_slave_synced(&wps_mac->tdma_sync) &&
         (wps_mac->node_role == NETWORK_NODE)) ||
        link_scheduler_get_mismatch(&wps_mac->scheduler)) {
        wps_mac->timeslot = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
        wps_mac->main_connection =
            link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                       wps_mac->main_connection_id);
        wps_mac->auto_connection = link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                                              0);
        connection               = wps_mac->main_connection;
        connection_count         = wps_mac->timeslot->main_connection_count;
    }

    if (connection_count > 1) {
        connection_id = *(wps_mac->main_xlayer->frame.header_begin_it +
                          link_protocol_get_buffer_offset(&connection->link_protocol,
                                                          MAC_PROTO_ID_CONNECTION_ID));
        if (connection_id < connection_count) {
            *conn_id = connection_id;
        } else {
            *conn_id = 0;
        }
    } else {
        *conn_id = 0;
    }
    wps_mac->main_connection =
        link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                   wps_mac->main_connection_id);
    wps_mac->auto_connection =
        link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                   wps_mac->auto_connection_id);
}

/** @brief Find the received time slot ID and connection ID for auto reply frame.
 *
 *  @param[out] wps_mac  Frame header MAC instance.
 */
static void find_received_timeslot_and_connection_auto(wps_mac_t *wps_mac)
{
    uint8_t connection_id;
    wps_connection_t *connection;
    uint8_t connection_count;
    uint8_t *conn_id;

    connection       = wps_mac->auto_connection;
    connection_count = wps_mac->timeslot->auto_connection_count;
    conn_id          = &wps_mac->auto_connection_id;

    if (connection_count > 1) {
        connection_id = *(wps_mac->main_xlayer->frame.header_begin_it +
                          link_protocol_get_buffer_offset(&connection->link_protocol,
                                                          MAC_PROTO_ID_CONNECTION_ID));
        if (connection_id < connection_count) {
            *conn_id = connection_id;
        } else {
            *conn_id = 0;
        }
    } else {
        *conn_id = 0;
    }
    wps_mac->main_connection =
        link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                   wps_mac->main_connection_id);
    wps_mac->auto_connection =
        link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                   wps_mac->auto_connection_id);
}

/** @brief Enqueue initial certification frames.
 *
 *  @param[out] wps_mac  Frame header MAC instance.
 */
static void certification_init(wps_mac_t *wps_mac)
{
    wps_connection_t *connection_main;
    wps_connection_t *connection_auto;
    timeslot_t *time_slot;
    uint8_t initial_index = wps_mac->scheduler.current_time_slot_num;
    uint8_t current_index = 255;

    while (initial_index != current_index) {
        connection_main = link_scheduler_get_current_main_connection(&wps_mac->scheduler,
                                                                     wps_mac->main_connection_id);
        if (connection_main != NULL) {
            if (connection_main->source_address == wps_mac->local_address) {
                connection_main->certification_mode_enabled = true;
                certification_send(connection_main);
            } else if (connection_main->ack_enable) {
                uint16_t temp = connection_main->source_address;

                connection_main->source_address             = wps_mac->local_address;
                connection_main->destination_address        = temp;
                connection_main->payload_size               = 0;
                connection_main->header_size                = 0;
                connection_main->certification_mode_enabled = true;
                certification_send(connection_main);
            }
            connection_main->ack_enable               = true;
            connection_main->stop_and_wait_arq.enable = true;
        }
        connection_auto = link_scheduler_get_current_auto_connection(&wps_mac->scheduler,
                                                                     wps_mac->auto_connection_id);
        if (connection_auto != NULL) {
            if (connection_auto->source_address == wps_mac->local_address) {
                certification_auto_reply_conn_config(connection_main, connection_auto);
                time_slot = link_scheduler_get_current_timeslot(&wps_mac->scheduler);
                time_slot->auto_connection_count            = 0;
                time_slot->main_connection_count            = 1;
                connection_main->certification_mode_enabled = true;
                connection_main->ack_enable                 = true;
                connection_main->stop_and_wait_arq.enable   = true;
                certification_send(connection_main);
            }
        }
        link_scheduler_increment_time_slot(&wps_mac->scheduler);
        current_index = wps_mac->scheduler.current_time_slot_num;
    }
}

/** @brief Configure certification auto-reply connection.
 *
 *  @param[in] conn_main  Connection main.
 *  @param[in] conn_auto  Connection auto-reply.
 */
static void certification_auto_reply_conn_config(wps_connection_t *conn_main,
                                                 wps_connection_t *conn_auto)
{
    memcpy(&conn_auto->frame_cfg, &conn_main->frame_cfg, sizeof(frame_cfg_t));
    memcpy(&conn_auto->channel, &conn_main->channel,
           sizeof(rf_channel_t) * WPS_NB_RF_CHANNEL * WPS_RADIO_COUNT);
    memcpy(conn_main, conn_auto, sizeof(wps_connection_t));
}

/** @brief Send certification frame on connection.
 *
 *  @param[in] connection  Connection.
 */
static void certification_send(wps_connection_t *connection)
{
    wps_error_t wps_err = WPS_NO_ERROR;
    uint8_t *data       = NULL;

    wps_get_free_slot(connection, &data, &wps_err);
    if ((wps_err != WPS_NO_ERROR) || (data == NULL)) {
        /* Queue is full. */
        return;
    }
    for (uint8_t i = 0; i < connection->payload_size; i++) {
        /* Send maximum power */
        if (connection->link_protocol.max_buffer_size % 2 == 0) {
            data[i] = (i % 2 == 0) ? PHY_CERTIF_BYTE0 : PHY_CERTIF_BYTE1;
        } else {
            data[i] = (i % 2 == 0) ? PHY_CERTIF_BYTE1 : PHY_CERTIF_BYTE0;
        }
    }
    wps_send(connection, data, connection->payload_size, &wps_err);
}

/** @brief Process application pending request.
 *
 *  @param[in] request  WPS request info structure.
 */
static void process_pending_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy)
{
    wps_request_info_t *request;

    request = circular_queue_front(&wps_mac->request_queue);
    if (request != NULL) {
        switch (request->type) {
        case REQUEST_MAC_CHANGE_SCHEDULE_RATIO: {
            process_schedule_request(wps_mac, request);
            break;
        }
        case REQUEST_PHY_WRITE_REG: {
            if (WPS_RADIO_COUNT == 1) {
                process_write_request(wps_mac, wps_phy, request);
            }
            break;
        }
        case REQUEST_PHY_READ_REG: {
            if (WPS_RADIO_COUNT == 1) {
                process_read_request(wps_mac, wps_phy, request);
            }
            break;
        }
        case REQUEST_PHY_DISCONNECT:
            process_disconnect_request(wps_mac, wps_phy);
            break;
        default:
            break;
        }
        circular_queue_dequeue(&wps_mac->request_queue);
    }
}

/** @brief Process MAC schedule change.
 *
 *  @note This allow the user to modify the active timeslot
 *        in the schedule of a given connection.
 *
 *  @note This process the request of type REQUEST_MAC_CHANGE_SCHEDULE_RATIO.
 *        Config structure should be of type wps_schedule_ratio_cfg_t.
 *
 *  @param[in] wps_mac  MAC structure.
 *  @param[in] request  WPS request info structure.
 */
static void process_schedule_request(wps_mac_t *wps_mac, wps_request_info_t *request)
{
    wps_schedule_ratio_cfg_t *schedule_ratio_cfg = (wps_schedule_ratio_cfg_t *)request->config;
    bool *pattern                                = schedule_ratio_cfg->pattern_cfg;

    if (pattern != NULL) {
        schedule_ratio_cfg->target_conn->active_ratio = schedule_ratio_cfg->active_ratio;
        schedule_ratio_cfg->target_conn->pattern_total_count =
            schedule_ratio_cfg->pattern_total_count;
        schedule_ratio_cfg->target_conn->pattern_count = schedule_ratio_cfg->pattern_current_count;
        memcpy(schedule_ratio_cfg->target_conn->pattern, pattern,
               schedule_ratio_cfg->pattern_total_count);
        circular_queue_dequeue(wps_mac->schedule_ratio_cfg_queue);
    }
}

/** @brief Process a write register request from application
 *
 *  @param[in] wps     WPS instance.
 *  @param[in] request WPS request info structure.
 */
static void process_write_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy,
                                  wps_request_info_t *request)
{
    wps_write_request_info_t *write_request = (wps_write_request_info_t *)request->config;

    wps_phy_write_register(wps_phy, write_request->target_register, write_request->data);

    circular_queue_dequeue(wps_mac->write_request_queue);
}

/** @brief Process a read register request from application
 *
 *  @param[in] wps     WPS instance.
 *  @param[in] request WPS request info structure.
 */
static void process_read_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy,
                                 wps_request_info_t *request)
{
    wps_read_request_info_t *read_request = (wps_read_request_info_t *)request->config;

    wps_phy_read_register(wps_phy, read_request->target_register, read_request->rx_buffer,
                          read_request->xfer_cmplt);

    circular_queue_dequeue(wps_mac->read_request_queue);
}

/** @brief Process disconnection request.
 *
 *  @param[in] wps  WPS instance.
 */
static void process_disconnect_request(wps_mac_t *wps_mac, wps_phy_t *wps_phy)
{
    wps_phy_disconnect(wps_phy);

    /* Free MAC RX node in case a frame was received after the disconnect request */
    xlayer_queue_free_node(wps_mac->rx_node);

    wps_mac->signal = WPS_DISCONNECT;
}
