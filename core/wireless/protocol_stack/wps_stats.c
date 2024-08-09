/** @file  wps_stats.c
 *  @brief Wireless Protocol Stack statistics.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps_stats.h"
#include <string.h>

/* CONSTANTS ******************************************************************/
#define CHAR_BIT 8

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Compare two objects.
 *
 * @param a            First value.
 * @param b            Second value.
 * @param object_size  Size of the object. Ex : sizeof(struct_t)
 * @retval false  Not equal.
 * @retval true   Equal.
 */
static bool is_equal_object(void *a, void *b, size_t object_size)
{
    int index = object_size / sizeof(int);
    int *in   = (int *)a;
    int *out  = (int *)b;

    do {
        --index;
        if (in[index] != out[index]) {
            return false;
        }
    } while (index);

    return true;
}

/** @brief Check if object is NULL.
 *
 * @param a            First value.
 * @param object_size  Size of the object. Ex : sizeof(struct_t)
 * @retval false Not NULL.
 * @retval true  Is NULL.
 */
static bool is_object_null(void *a, size_t object_size)
{
    int index = object_size / sizeof(int);
    int *in   = (int *)a;

    do {
        --index;
        if (in[index] != 0) {
            return false;
        }
    } while (index);

    return true;
}

/** @brief Copy object from in to out.
 *
 * @param in           Object with original data.
 * @param out          Object where the data is copied.
 * @param object_size  Size of the object. Ex : sizeof(struct_t)
 */
static void save_object(void *in, void *out, size_t object_size)
{
    do {
        memcpy(out, in, object_size);
    } while (!is_equal_object(out, in, object_size));
}

/* PUBLIC FUNCTIONS ***********************************************************/
uint32_t wps_stats_get_payload_success_count(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.tx_success;
}

uint32_t wps_stats_get_payload_fail_count(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.tx_fail;
}

uint32_t wps_stats_get_payload_dropped_count(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.tx_drop;
}

float wps_stats_get_payload_success_ratio(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    if (!(temp.tx_success + temp.tx_fail)) {
        return 0;
    }

    return (float)temp.tx_success / (temp.tx_success + temp.tx_fail);
}

uint32_t wps_stats_get_payload_received_count(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.rx_received;
}

uint32_t wps_stats_get_payload_overrun_count(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.rx_overrun;
}

uint32_t wps_stats_get_empty_count(wps_connection_t *connection)
{
    lqi_t used_frame_lqi_temp;
    lqi_t lqi_temp;

    save_object(&connection->used_frame_lqi, &used_frame_lqi_temp, sizeof(lqi_t));
    save_object(&connection->lqi, &lqi_temp, sizeof(lqi_t));

    return lqi_temp.sent_count - used_frame_lqi_temp.sent_count;
}

uint32_t wps_stats_get_rx_sync_count(wps_connection_t *connection)
{
    lqi_t used_frame_lqi_temp;
    lqi_t lqi_temp;

    save_object(&connection->used_frame_lqi, &used_frame_lqi_temp, sizeof(lqi_t));
    save_object(&connection->lqi, &lqi_temp, sizeof(lqi_t));

    return lqi_temp.received_count - used_frame_lqi_temp.received_count;
}

float wps_stats_get_tx_link_usage_ratio(wps_connection_t *connection)
{
    lqi_t used_frame_lqi_temp;
    lqi_t lqi_temp;

    save_object(&connection->used_frame_lqi, &used_frame_lqi_temp, sizeof(lqi_t));
    save_object(&connection->lqi, &lqi_temp, sizeof(lqi_t));

    if (!lqi_temp.total_count) {
        return 0;
    }

    return (float)used_frame_lqi_temp.sent_count / lqi_temp.total_count;
}

float wps_stats_get_rx_link_usage_ratio(wps_connection_t *connection)
{
    lqi_t used_frame_lqi_temp;
    lqi_t lqi_temp;

    save_object(&connection->used_frame_lqi, &used_frame_lqi_temp, sizeof(lqi_t));
    save_object(&connection->lqi, &lqi_temp, sizeof(lqi_t));

    if (!lqi_temp.total_count) {
        return 0;
    }

    return (float)used_frame_lqi_temp.received_count / lqi_temp.total_count;
}

float wps_stats_get_tx_datarate(wps_connection_t *connection, uint32_t time_ms)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    if (!time_ms) {
        return 0;
    }

    return CHAR_BIT * ((float)temp.tx_byte_sent / time_ms);
}

float wps_stats_get_rx_datarate(wps_connection_t *connection, uint32_t time_ms)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    if (!time_ms) {
        return 0;
    }

    return CHAR_BIT * ((float)temp.rx_byte_received / time_ms);
}

uint32_t wps_stats_get_phy_rssi_avg(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rssi_tenth_db(&temp);
}

uint32_t wps_stats_get_phy_rssi(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_inst_rssi(&temp);
}

uint32_t wps_stats_get_inst_phy_rssi_tenth_db(wps_connection_t *connection)
{
    return link_lqi_get_inst_rssi_tenth_db(&connection->lqi);
}

uint32_t wps_stats_get_phy_rnsi_avg(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rnsi_tenth_db(&temp);
}

uint32_t wps_stats_get_phy_rnsi(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_inst_rnsi(&temp);
}

uint32_t wps_stats_get_inst_phy_rnsi_tenth_db(wps_connection_t *connection)
{
    return link_lqi_get_inst_rnsi_tenth_db(&connection->lqi);
}

int32_t wps_stats_get_phy_margin_avg(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rssi_tenth_db(&temp) - link_lqi_get_avg_rnsi_tenth_db(&temp);
}

int32_t wps_stats_get_inst_phy_margin(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_inst_rssi_tenth_db(&temp) - link_lqi_get_inst_rnsi_tenth_db(&temp);
}

int32_t wps_stats_get_inst_phy_margin_fast(wps_connection_t *connection)
{
    return link_lqi_get_inst_rssi_tenth_db(&connection->lqi) - link_lqi_get_inst_rnsi_tenth_db(&connection->lqi);
}

uint32_t wps_stats_get_phy_inst_phase_offset(wps_connection_t *connection, uint8_t index)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_inst_phase_offset(&temp, index);
}

uint32_t wps_stats_get_phy_sent_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_sent_count(&temp);
}

uint32_t wps_stats_get_phy_ack_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_ack_count(&temp);
}

uint32_t wps_stats_get_phy_nack_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_nack_count(&temp);
}

uint32_t wps_stats_get_phy_received_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_received_count(&temp);
}

uint32_t wps_stats_get_phy_missing_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_lost_count(&temp);
}

uint32_t wps_stats_get_phy_rejected_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_rejected_count(&temp);
}

uint32_t wps_stats_get_phy_duplicated_frame_count(wps_connection_t *connection)
{
    saw_arq_t temp;

    save_object(&connection->stop_and_wait_arq, &temp, sizeof(saw_arq_t));

    return link_saw_arq_get_duplicate_count(&temp);
}

uint32_t wps_stats_get_phy_retry_frame_count(wps_connection_t *connection)
{
    saw_arq_t temp;

    save_object(&connection->stop_and_wait_arq, &temp, sizeof(saw_arq_t));

    return link_saw_arq_get_retry_count(&temp);
}

float wps_stats_get_phy_ack_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_sent_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_ack_count(&temp)) / link_lqi_get_sent_count(&temp);
}

float wps_stats_get_phy_nack_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_sent_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_nack_count(&temp)) / link_lqi_get_sent_count(&temp);
}

float wps_stats_get_phy_received_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_received_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_phy_missing_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&connection->lqi)) {
        return 0;
    }

    return (float)(link_lqi_get_lost_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_phy_rejected_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_rejected_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_phy_mrr(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    uint32_t bad_frame_count = link_lqi_get_rejected_count(&temp) + link_lqi_get_lost_count(&temp);

    if (!bad_frame_count) {
        return 0;
    }
    return (float)(link_lqi_get_lost_count(&temp)) / bad_frame_count;
}

float wps_stats_get_phy_per(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->lqi, &temp, sizeof(lqi_t));

    uint32_t total_frame_count = link_lqi_get_total_count(&temp);

    if (!total_frame_count) {
        return 0;
    }

    return (float)(total_frame_count - link_lqi_get_received_count(&temp)) / total_frame_count;
}

uint32_t wps_stats_get_phy_cca_pass_count(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.cca_pass;
}

uint32_t wps_stats_get_phy_cca_tx_fail(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return temp.cca_tx_fail;
}

float wps_stats_get_phy_cca_pass_ratio(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return (float)(temp.cca_pass) / (temp.cca_pass + temp.cca_tx_fail);
}

float wps_stats_get_phy_cca_fail_ratio(wps_connection_t *connection)
{
    wps_stats_t temp;

    save_object(&connection->wps_stats, &temp, sizeof(wps_stats_t));

    return (float)(temp.cca_tx_fail) / (temp.cca_pass + temp.cca_tx_fail);
}

uint32_t wps_stats_get_phy_cca_fail(wps_connection_t *connection)
{
    return connection->wps_stats.cca_fail;
}

uint32_t wps_stats_get_rssi_avg(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rssi_tenth_db(&temp);
}

uint32_t wps_stats_get_rnsi_avg(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rnsi_tenth_db(&temp);
}

int32_t wps_stats_get_margin_avg(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rssi_tenth_db(&temp) - link_lqi_get_avg_rnsi_tenth_db(&temp);
}

uint32_t wps_stats_get_ack_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_ack_count(&temp);
}

uint32_t wps_stats_get_nack_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_nack_count(&temp);
}

uint32_t wps_stats_get_received_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_received_count(&temp);
}

uint32_t wps_stats_get_missing_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_lost_count(&temp);
}

uint32_t wps_stats_get_rejected_frame_count(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    return link_lqi_get_rejected_count(&temp);
}

uint32_t wps_stats_get_duplicated_frame_count(wps_connection_t *connection)
{
    saw_arq_t temp;

    save_object(&connection->stop_and_wait_arq, &temp, sizeof(saw_arq_t));

    return link_saw_arq_get_duplicate_count(&temp);
}

uint32_t wps_stats_get_retry_frame_count(wps_connection_t *connection)
{
    saw_arq_t temp;

    save_object(&connection->stop_and_wait_arq, &temp, sizeof(saw_arq_t));

    return link_saw_arq_get_retry_count(&temp);
}

float wps_stats_get_ack_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_sent_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_ack_count(&temp)) / link_lqi_get_sent_count(&temp);
}

float wps_stats_get_nack_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_sent_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_nack_count(&temp)) / link_lqi_get_sent_count(&temp);
}

float wps_stats_get_received_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_received_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_missing_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&connection->used_frame_lqi)) {
        return 0;
    }

    return (float)(link_lqi_get_lost_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_rejected_frame_ratio(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_rejected_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_mrr(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    uint32_t bad_frame_count = link_lqi_get_rejected_count(&temp) + link_lqi_get_lost_count(&temp);

    if (!bad_frame_count) {
        return 0;
    }
    return (float)(link_lqi_get_lost_count(&temp)) / bad_frame_count;
}

float wps_stats_get_per(wps_connection_t *connection)
{
    lqi_t temp;

    save_object(&connection->used_frame_lqi, &temp, sizeof(lqi_t));

    uint32_t total_frame_count = link_lqi_get_total_count(&temp);

    if (!total_frame_count) {
        return 0;
    }

    return (float)(total_frame_count - link_lqi_get_received_count(&temp)) / total_frame_count;
}

uint32_t wps_stats_get_chan_rssi_avg(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rssi_tenth_db(&temp);
}

uint32_t wps_stats_get_chan_rssi(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_inst_rssi(&temp);
}

uint32_t wps_stats_get_chan_rnsi_avg(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rnsi_tenth_db(&temp);
}

uint32_t wps_stats_get_chan_rnsi(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_inst_rnsi(&temp);
}

int32_t wps_stats_get_chan_margin_avg(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_avg_rssi_tenth_db(&temp) - link_lqi_get_avg_rnsi_tenth_db(&temp);
}

uint32_t wps_stats_get_chan_ack_frame_count(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_ack_count(&temp);
}

uint32_t wps_stats_get_chan_nack_frame_count(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_nack_count(&temp);
}

uint32_t wps_stats_get_chan_received_frame_count(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_received_count(&temp);
}

uint32_t wps_stats_get_chan_missing_frame_count(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_lost_count(&temp);
}

uint32_t wps_stats_get_chan_rejected_frame_count(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    return link_lqi_get_rejected_count(&temp);
}

float wps_stats_get_chan_ack_frame_ratio(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    if (!link_lqi_get_sent_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_ack_count(&temp)) / link_lqi_get_sent_count(&temp);
}

float wps_stats_get_chan_nack_frame_ratio(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    if (!link_lqi_get_sent_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_nack_count(&temp)) / link_lqi_get_sent_count(&temp);
}

float wps_stats_get_chan_received_frame_ratio(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_received_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_chan_missing_frame_ratio(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&connection->channel_lqi[channel_idx])) {
        return 0;
    }

    return (float)(link_lqi_get_lost_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_chan_rejected_frame_ratio(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    if (!link_lqi_get_total_count(&temp)) {
        return 0;
    }

    return (float)(link_lqi_get_rejected_count(&temp)) / link_lqi_get_total_count(&temp);
}

float wps_stats_get_chan_mrr(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    uint32_t bad_frame_count = link_lqi_get_rejected_count(&temp) + link_lqi_get_lost_count(&temp);

    if (!bad_frame_count) {
        return 0;
    }
    return (float)(link_lqi_get_lost_count(&temp)) / bad_frame_count;
}

float wps_stats_get_chan_per(wps_connection_t *connection, uint8_t channel_idx)
{
    lqi_t temp;

    save_object(&connection->channel_lqi[channel_idx], &temp, sizeof(lqi_t));

    uint32_t total_frame_count = link_lqi_get_total_count(&temp);

    if (!total_frame_count) {
        return 0;
    }

    return (float)(total_frame_count - link_lqi_get_received_count(&temp)) / total_frame_count;
}

void wps_stats_reset(wps_connection_t *connection)
{
    lqi_t *lqi             = &connection->lqi;
    lqi_t *used_frame_lqi  = &connection->used_frame_lqi;
    saw_arq_t *saw_arq     = &connection->stop_and_wait_arq;
    wps_stats_t *wps_stats = &connection->wps_stats;

    do {
        link_lqi_reset(lqi);
    } while (!is_object_null(lqi, sizeof(lqi_t)));

    do {
        link_lqi_reset(used_frame_lqi);
    } while (!is_object_null(used_frame_lqi, sizeof(lqi_t)));

    do {
        memset(wps_stats, 0, sizeof(wps_stats_t));
    } while (!is_object_null(wps_stats, sizeof(wps_stats_t)));

    do {
        link_saw_arq_reset_stats(saw_arq);
    } while (saw_arq->retry_count != 0 && saw_arq->duplicate_count != 0);

    for (size_t i = 0; i < WPS_NB_RF_CHANNEL; i++) {
        lqi_t *channel_lqi = &connection->channel_lqi[i];

        do {
            link_lqi_reset(channel_lqi);
        } while (!is_object_null(channel_lqi, sizeof(lqi_t)));
    }
}
