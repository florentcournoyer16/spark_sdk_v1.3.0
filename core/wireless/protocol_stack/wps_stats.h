/** @file  wps_stats.h
 *  @brief Wireless Protocol Stack statistics.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef WPS_STATS_H
#define WPS_STATS_H

/* INCLUDES *******************************************************************/
#include "wps.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Number of payloads successfully sent or dropped.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload sent count.
 */
uint32_t wps_stats_get_payload_sent_count(wps_connection_t *connection);

/** @brief Number of payloads successfully sent.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload successfully sent count.
 */
uint32_t wps_stats_get_payload_success_count(wps_connection_t *connection);

/** @brief Number of payloads unsuccessfully sent.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload successfully sent count.
 */
uint32_t wps_stats_get_payload_fail_count(wps_connection_t *connection);

/** @brief Number of payloads dropped.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload dropped count.
 */
uint32_t wps_stats_get_payload_dropped_count(wps_connection_t *connection);

/** @brief Number of payload successfully sent on the total number of payload sent.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload transmission success ratio.
 */
float wps_stats_get_payload_success_ratio(wps_connection_t *connection);

/** @brief Number of payloads received.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload received count.
 */
uint32_t wps_stats_get_payload_received_count(wps_connection_t *connection);

/** @brief Number of payloads dropped because of an RX buffer overload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Payload overrun count.
 */
uint32_t wps_stats_get_payload_overrun_count(wps_connection_t *connection);

/** @brief Number of sync frame sent or the number of empty tx timeslots.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Empty count.
 */
uint32_t wps_stats_get_empty_count(wps_connection_t *connection);

/** @brief Number of sync frame received.
 *
 *  @param[in] connection  WPS connection object.
 *  @return RX sync count.
 */
uint32_t wps_stats_get_rx_sync_count(wps_connection_t *connection);

/** @brief Ratio of TX timeslots with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return TX link usage.
 */
float wps_stats_get_tx_link_usage_ratio(wps_connection_t *connection);

/** @brief Ratio of RX timeslots with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return RX link usage.
 */
float wps_stats_get_rx_link_usage_ratio(wps_connection_t *connection);

/** @brief Average TX datarate in kbps since the last stats reset.
 *
 *  @note The tx datarate is calculated from the acknowledge count,
 *        if ack are disabled the datarate will be 0.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] time_ms     Elapsed time in ms since the last stats reset.
 *  @return RX datarate.
 */
float wps_stats_get_tx_datarate(wps_connection_t *connection, uint32_t time_ms);

/** @brief Average RX datarate in kbps since the last stats reset.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] time_ms     Elapsed time in ms since the last stats reset.
 *  @return TX datarate.
 */
float wps_stats_get_rx_datarate(wps_connection_t *connection, uint32_t time_ms);

/** @brief Get average RSSI on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Average RSSI.
 */
uint32_t wps_stats_get_phy_rssi_avg(wps_connection_t *connection);

/** @brief Get last received RSSI measurement on the given connection.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Instantaneous RSSI.
 */
uint32_t wps_stats_get_phy_rssi(wps_connection_t *connection);

/** @brief Get last received RSSI measurement on the given connection in tenth of dB.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Instantaneous RSSI in tenth of dB.
 */
uint32_t wps_stats_get_inst_phy_rssi_tenth_db(wps_connection_t *connection);

/** @brief Get average RNSI on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Average RNSI.
 */
uint32_t wps_stats_get_phy_rnsi_avg(wps_connection_t *connection);

/** @brief Get last received RNSI measurement on the given connection.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Instantaneous RNSI.
 */
uint32_t wps_stats_get_phy_rnsi(wps_connection_t *connection);

/** @brief Get last received RNSI measurement on the given connection in tenth of dB.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Instantaneous RNSI in tenth of dB.
 */
uint32_t wps_stats_get_inst_phy_rnsi_tenth_db(wps_connection_t *connection);

/** @brief Get link margin on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Link margin.
 */
int32_t wps_stats_get_phy_margin_avg(wps_connection_t *connection);

/** @brief Get the instantaneous link margin on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Link margin.
 */
int32_t wps_stats_get_inst_phy_margin(wps_connection_t *connection);

/** @brief Get the instantaneous link margin on the physical layer without integrity check.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Link margin.
 */
int32_t wps_stats_get_inst_phy_margin_fast(wps_connection_t *connection);

/** @brief Get phase offset instantaneous values.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] index       Index.
 *  @return Phase offset values.
 */
uint32_t wps_stats_get_phy_inst_phase_offset(wps_connection_t *connection, uint8_t index);

/** @brief Get phy sent frame count.
 *
 *  @note This will increment every TX timeslot.
 *
 *  @param[in] connection  WPS connection object.
 *  @return PHY sent count.
 */
uint32_t wps_stats_get_phy_sent_count(wps_connection_t *connection);

/** @brief Get ack frame count on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Ack frame count.
 */
uint32_t wps_stats_get_phy_ack_frame_count(wps_connection_t *connection);

/** @brief Get nack frame count on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Nack frame count.
 */
uint32_t wps_stats_get_phy_nack_frame_count(wps_connection_t *connection);

/** @brief Get received frame count on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Received frame count.
 */
uint32_t wps_stats_get_phy_received_frame_count(wps_connection_t *connection);

/** @brief Get missing frame count on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Missing frame count.
 */
uint32_t wps_stats_get_phy_missing_frame_count(wps_connection_t *connection);

/** @brief Get rejected frame count on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Rejected frame count.
 */
uint32_t wps_stats_get_phy_rejected_frame_count(wps_connection_t *connection);

/** @brief Get duplicated frame count on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Duplicated frame count.
 */
uint32_t wps_stats_get_phy_duplicated_frame_count(wps_connection_t *connection);

/** @brief Get retry frame count on the physical layer.
 *
 *  @param connection  WPS connection object.
 *  @return Number of retries.
 */
uint32_t wps_stats_get_phy_retry_frame_count(wps_connection_t *connection);

/** @brief Get ack frame ratio on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Ack frame ratio.
 */
float wps_stats_get_phy_ack_frame_ratio(wps_connection_t *connection);

/** @brief Get nack frame ratio on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Nack frame ratio.
 */
float wps_stats_get_phy_nack_frame_ratio(wps_connection_t *connection);

/** @brief Get received frame ratio on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Received frame ratio.
 */
float wps_stats_get_phy_received_frame_ratio(wps_connection_t *connection);

/** @brief Get missing frame ratio on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Missing frame ratio.
 */
float wps_stats_get_phy_missing_frame_ratio(wps_connection_t *connection);

/** @brief Get rejected frame ratio on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Rejected frame ratio.
 */
float wps_stats_get_phy_rejected_frame_ratio(wps_connection_t *connection);

/** @brief Get Missing/Reject Ration (MRR) frame ratio on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return MRR frame ratio.
 */
float wps_stats_get_phy_mrr(wps_connection_t *connection);

/** @brief Get Payload Error Rate (PER) on the physical layer.
 *
 *  @param[in] connection  WPS connection object.
 *  @return PER.
 */
float wps_stats_get_phy_per(wps_connection_t *connection);

/** @brief Number CCA pass events.
 *
 *  @param[in] connection  WPS connection object.
 *  @return CCA pass count.
 */
uint32_t wps_stats_get_phy_cca_pass_count(wps_connection_t *connection);

/** @brief Number of timeslots in which all CCA tries failed.
 *
 *  @param[in] connection  WPS connection object.
 *  @return CCA TX fails.
 */
uint32_t wps_stats_get_phy_cca_tx_fail(wps_connection_t *connection);

/** @brief CCA pass ratio.
 *
 *  @param[in] connection  WPS connection object.
 *  @return CCA pass ratio.
 */
float wps_stats_get_phy_cca_pass_ratio(wps_connection_t *connection);

/** @brief CCA fail ratio.
 *
 *  @param[in] connection  WPS connection object.
 *  @return CCA fail ratio.
 */
float wps_stats_get_phy_cca_fail_ratio(wps_connection_t *connection);

/** @brief Get number of CCA fail events.
 *
 *  @param[in] connection  WPS connection object.
 *  @return CCA fail count.
 */
uint32_t wps_stats_get_phy_cca_fail(wps_connection_t *connection);

/** @brief Get average Received Signal Strength Indicator (RSSI) of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Average RSSI.
 */
uint32_t wps_stats_get_rssi_avg(wps_connection_t *connection);

/** @brief Get average Received Noise Strength Indicator (RNSI) of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Average RNSI.
 */
uint32_t wps_stats_get_rnsi_avg(wps_connection_t *connection);

/** @brief Get link margin of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Link margin.
 */
int32_t wps_stats_get_margin_avg(wps_connection_t *connection);

/** @brief Get ACK frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return ACK frame count.
 */
uint32_t wps_stats_get_ack_frame_count(wps_connection_t *connection);

/** @brief Get NACK frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return NACK frame count.
 */
uint32_t wps_stats_get_nack_frame_count(wps_connection_t *connection);

/** @brief Get received frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Received frame count.
 */
uint32_t wps_stats_get_received_frame_count(wps_connection_t *connection);

/** @brief Get missing frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Missing frame count.
 */
uint32_t wps_stats_get_missing_frame_count(wps_connection_t *connection);

/** @brief Get rejected frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Rejected frame count.
 */
uint32_t wps_stats_get_rejected_frame_count(wps_connection_t *connection);

/** @brief Get duplicated frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Duplicated frame count.
 */
uint32_t wps_stats_get_duplicated_frame_count(wps_connection_t *connection);

/** @brief Get retry frame count of frames with payload.
 *
 *  @param connection  WPS connection object.
 *  @return Number of retries.
 */
uint32_t wps_stats_get_retry_frame_count(wps_connection_t *connection);

/** @brief Get ACK frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return ACK frame ratio.
 */
float wps_stats_get_ack_frame_ratio(wps_connection_t *connection);

/** @brief Get NACK frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return NACK frame ratio.
 */
float wps_stats_get_nack_frame_ratio(wps_connection_t *connection);

/** @brief Get received frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Received frame ratio.
 */
float wps_stats_get_received_frame_ratio(wps_connection_t *connection);

/** @brief Get missing frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Missing frame ratio.
 */
float wps_stats_get_missing_frame_ratio(wps_connection_t *connection);

/** @brief Get rejected frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return Rejected frame ratio.
 */
float wps_stats_get_rejected_frame_ratio(wps_connection_t *connection);

/** @brief Get Missing/Rejected Ratio (MRR) of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return MRR frame ratio.
 */
float wps_stats_get_mrr(wps_connection_t *connection);

/** @brief Get PER of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @return PER.
 */
float wps_stats_get_per(wps_connection_t *connection);

/** @brief Reset stats.
 *
 *  @param[in] connection  WPS connection object.
 */
void wps_stats_reset(wps_connection_t *connection);

/** @brief Get average RSSI of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Average RSSI.
 */
uint32_t wps_stats_get_chan_rssi_avg(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get last received RSSI of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Last RSSI measurement.
 */
uint32_t wps_stats_get_chan_rssi(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get average RNSI of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Average RNSI.
 */
uint32_t wps_stats_get_chan_rnsi_avg(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get last received RNSI of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Last RNSI measurment.
 */
uint32_t wps_stats_get_chan_rnsi(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get link margin of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Link margin.
 */
int32_t wps_stats_get_chan_margin_avg(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get ack frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Ack frame count.
 */
uint32_t wps_stats_get_chan_ack_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get nack frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Nack frame count.
 */
uint32_t wps_stats_get_chan_nack_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get received frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Received frame count.
 */
uint32_t wps_stats_get_chan_received_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get missing frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Missing frame count.
 */
uint32_t wps_stats_get_chan_missing_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get rejected frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Rejected frame count.
 */
uint32_t wps_stats_get_chan_rejected_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get duplicated frame count of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Duplicated frame count.
 */
uint32_t wps_stats_get_chan_duplicated_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get retry frame count of frames with payload.
 *
 *  @param connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Number of retries.
 */
uint32_t wps_stats_get_chan_retry_frame_count(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get ack frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Ack frame ratio.
 */
float wps_stats_get_chan_ack_frame_ratio(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get nack frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Nack frame ratio.
 */
float wps_stats_get_chan_nack_frame_ratio(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get received frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Received frame ratio.
 */
float wps_stats_get_chan_received_frame_ratio(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get missing frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Missing frame ratio.
 */
float wps_stats_get_chan_missing_frame_ratio(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get rejected frame ratio of frames with payload.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return Rejected frame ratio.
 */
float wps_stats_get_chan_rejected_frame_ratio(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get Missing/Reject Ration (MRR) of a channel.
 *
 *  @param[in] connection  WPS connection object.
 *  @param[in] channel_idx Channel index.
 *  @return MRR frame ratio.
 */
float wps_stats_get_chan_mrr(wps_connection_t *connection, uint8_t channel_idx);

/** @brief Get Payload Error Rate (PER) of a channel.
 *
 *  @param[in] connection   WPS connection object.
 *  @param[in] channel_idx  Channel index.
 *  @return PER.
 */
float wps_stats_get_chan_per(wps_connection_t *connection, uint8_t channel_idx);


#ifdef __cplusplus
}
#endif

#endif /* WPS_STATS_H */
