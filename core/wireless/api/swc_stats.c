/** @file  swc_stats.c
 *  @brief SPARK Wireless Core statistics.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "swc_stats.h"
#include <stdio.h>
#include "swc_api.h"
#include "wps_stats.h"

/* PUBLIC FUNCTIONS ***********************************************************/
swc_statistics_t *swc_connection_update_stats(swc_connection_t *conn)
{
    /* TX stats */
    conn->stats.tx_timeslot_occurrence = wps_stats_get_phy_sent_count(conn->wps_conn_handle);

    conn->stats.packet_sent_and_acked_count     = wps_stats_get_ack_frame_count(conn->wps_conn_handle);
    conn->stats.packet_sent_and_not_acked_count = wps_stats_get_nack_frame_count(conn->wps_conn_handle);

    uint32_t tx_count = conn->stats.packet_sent_and_acked_count + conn->stats.packet_sent_and_not_acked_count;

    /* tx_timeslot_occurrence value can be lower than tx_count value if after updating tx_timeslot_occurrence,
     * and before updating packet_sent_and_acked_count and/or packet_sent_and_not_acked_count, a new packet is transmitted.
     * In such case, adjust tx_timeslot_occurrence to reflect the new TX event(s).
     */
    if (tx_count > conn->stats.tx_timeslot_occurrence) {
        conn->stats.tx_timeslot_occurrence = tx_count;
    }

    conn->stats.no_packet_tranmission_count = conn->stats.tx_timeslot_occurrence - conn->stats.packet_sent_and_acked_count -
                                              conn->stats.packet_sent_and_not_acked_count;

    conn->stats.packet_dropped_count = wps_stats_get_payload_dropped_count(conn->wps_conn_handle);
    conn->stats.tx_used_capacity_pc  = ((float)tx_count / (conn->stats.tx_timeslot_occurrence)) * 100;

    conn->stats.cca_pass_count = wps_stats_get_phy_cca_pass_count(conn->wps_conn_handle);
    conn->stats.cca_fail_count = wps_stats_get_phy_cca_tx_fail(conn->wps_conn_handle);
    conn->stats.cca_try_fail_count = wps_stats_get_phy_cca_fail(conn->wps_conn_handle);

    /* RX stats */
    conn->stats.rx_timeslot_occurrence = wps_stats_get_phy_received_frame_count(conn->wps_conn_handle) +
                                         wps_stats_get_phy_rejected_frame_count(conn->wps_conn_handle) +
                                         wps_stats_get_phy_missing_frame_count(conn->wps_conn_handle);

    conn->stats.packet_successfully_received_count = wps_stats_get_received_frame_count(conn->wps_conn_handle);

    /* rx_timeslot_occurrence value can be lower than packet_successfully_received_count value if after updating
     * rx_timeslot_occurrence, and before updating packet_successfully_received_count, a new packet is received.
     * In such case, adjust rx_timeslot_occurrence to reflect the new RX event(s).
     */
    if (conn->stats.packet_successfully_received_count > conn->stats.rx_timeslot_occurrence) {
        conn->stats.rx_timeslot_occurrence = conn->stats.packet_successfully_received_count;
    }

    conn->stats.no_packet_reception_count = conn->stats.rx_timeslot_occurrence - conn->stats.packet_successfully_received_count;

    conn->stats.packet_duplicated_count = wps_stats_get_duplicated_frame_count(conn->wps_conn_handle);
    conn->stats.packet_rejected_count   = wps_stats_get_phy_rejected_frame_count(conn->wps_conn_handle);
    conn->stats.packet_overrun_count    = wps_stats_get_payload_overrun_count(conn->wps_conn_handle);

    return &conn->stats;
}

int swc_connection_format_stats(swc_connection_t *conn, swc_node_t *node, char *buffer, uint16_t size)
{
    int string_length;

    if (conn->wps_conn_handle->source_address == node->wps_node_handle->cfg.local_address) {
        /* TX stats */
        const char *tx_timeslot_occurrence_str    = "TX Timeslot Occurrence";
        const char *packet_sent_and_acked_str     = "Packet Sent And ACK'd";
        const char *packet_sent_and_not_acked_str = "Packet Sent And Not ACK'd";
        const char *no_packet_transmission_str    = "No Packet Transmission";
        const char *tx_packet_dropped_str         = "Packet Dropped";
        const char *tx_used_capacity_str          = "TX Used Capacity";
        const char *cca_pass_str                  = "CCA Pass";
        const char *cca_fail_str                  = "CCA Fail";
        const char *cca_try_fail_str              = "CCA Try Fail";

        string_length = snprintf(buffer, size,
                                 "<<< %s >>>\r\n"
                                 "%s:\t\t%10lu\r\n"
                                 "  %s:\t%10lu (%05.2f%%)\r\n"
                                 "  %s:\t%10lu (%05.2f%%)\r\n"
                                 "  %s:\t%10lu (%05.2f%%)\r\n"
                                 "%s:\t\t\t%10lu\r\n"
                                 "%s:\t\t%10.2f%%\r\n"
                                 "%s:\t\t\t%10lu\r\n"
                                 "%s:\t\t\t%10lu\r\n"
                                 "%s:\t\t\t%10lu\r\n",
                                 conn->cfg.name,
                                 tx_timeslot_occurrence_str, conn->stats.tx_timeslot_occurrence,
                                 packet_sent_and_acked_str, conn->stats.packet_sent_and_acked_count,
                                                             (double)conn->stats.packet_sent_and_acked_count /
                                                             conn->stats.tx_timeslot_occurrence * 100,
                                 packet_sent_and_not_acked_str, conn->stats.packet_sent_and_not_acked_count,
                                                                 (double)conn->stats.packet_sent_and_not_acked_count /
                                                                 conn->stats.tx_timeslot_occurrence * 100,
                                 no_packet_transmission_str, conn->stats.no_packet_tranmission_count,
                                                             (double)conn->stats.no_packet_tranmission_count /
                                                             conn->stats.tx_timeslot_occurrence * 100,
                                 tx_packet_dropped_str, conn->stats.packet_dropped_count,
                                 tx_used_capacity_str, (double)conn->stats.tx_used_capacity_pc,
                                 cca_pass_str, conn->stats.cca_pass_count,
                                 cca_fail_str, conn->stats.cca_fail_count,
                                 cca_try_fail_str, conn->stats.cca_try_fail_count);
    } else {
        /* RX stats */
        const char *rx_timeslot_occurrence_str             = "RX Timeslot Occurrence";
        const char *packet_successfully_received_count_str = "Packet Successfully Received";
        const char *no_packet_reception_count_str          = "No Packet Reception";
        const char *packet_duplicated_count_str            = "Packet Duplicated";
        const char *packet_rejected_count_str              = "Packet Rejected";
        const char *packet_overrun_count_str               = "Packet Overrun";

        string_length = snprintf(buffer, size,
                                 "<<< %s >>>\r\n"
                                 "%s:\t\t%10lu\r\n"
                                 "  %s:\t%10lu (%05.2f%%)\r\n"
                                 "  %s:\t\t%10lu (%05.2f%%)\r\n"
                                 "%s:\t\t%10lu\r\n"
                                 "%s:\t\t%10lu\r\n"
                                 "%s:\t\t\t%10lu\r\n",
                                 conn->cfg.name, rx_timeslot_occurrence_str, conn->stats.rx_timeslot_occurrence,
                                 packet_successfully_received_count_str, conn->stats.packet_successfully_received_count,
                                 (double)conn->stats.packet_successfully_received_count / conn->stats.rx_timeslot_occurrence * 100,
                                 no_packet_reception_count_str, conn->stats.no_packet_reception_count,
                                 (double)conn->stats.no_packet_reception_count / conn->stats.rx_timeslot_occurrence * 100,
                                 packet_duplicated_count_str, conn->stats.packet_duplicated_count,
                                 packet_rejected_count_str, conn->stats.packet_rejected_count,
                                 packet_overrun_count_str, conn->stats.packet_overrun_count);
    }

    return string_length;
}

void swc_connection_reset_stats(swc_connection_t *conn)
{
    memset(&conn->stats, 0, sizeof(swc_statistics_t));
    wps_stats_reset(conn->wps_conn_handle);
}
