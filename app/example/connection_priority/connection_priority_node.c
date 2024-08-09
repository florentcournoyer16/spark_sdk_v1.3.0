/** @file  connection_priority_node.c
 *  @brief This is a basic example of how to use the Wireless Core connection priority.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include "iface_connection_priority.h"
#include "iface_wireless.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SWC_MEM_POOL_SIZE           12000
#define MAX_BIG_PAYLOAD_SIZE_BYTE   15
#define MAX_SMALL_PAYLOAD_SIZE_BYTE 8
#define STATS_ARRAY_LENGTH          8000
#define TIMER1_PACKET_RATE_US       1333
#define TIMER2_PACKET_RATE_US       5000
#define TIMER_STAT_MS               500

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t hal;
static swc_node_t *node;
static swc_connection_t *rx_cid0;
static swc_connection_t *rx_cid1;
static swc_connection_t *rx_cid2;
static swc_connection_t *tx_cid3;
static swc_connection_t *tx_cid4;

static uint32_t timeslot_us[]       = SCHEDULE;
static uint32_t channel_sequence[]  = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_timeslots[]       = RX_TIMESLOTS;
static int32_t tx_timeslots[]       = TX_TIMESLOTS;

/* ** Application Specific ** */
static uint32_t cid3_sent_count;
static uint32_t cid4_sent_count;
static uint32_t cid3_dropped_count;
static uint32_t cid4_dropped_count;

static volatile bool print_stats_now;
static volatile bool reset_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void cid3_tx_send_callback(void);
static void cid4_tx_send_callback(void);
static void rx_success_callback(void *conn);

static void print_stats(void);
static void reset_stats(void);
static void stats_callback(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    swc_error_t swc_err;

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    swc_connect(&swc_err);

    /* Connection ID 3 (CID3) sends 750 pkt/s */
    iface_packet_rate_timer1_init(TIMER1_PACKET_RATE_US);
    iface_packet_rate_set_timer1_callback(cid3_tx_send_callback);
    iface_packet_rate_timer1_start();

    /* Connection ID 4 (CID4) sends 200 pkt/s */
    iface_packet_rate_timer2_init(TIMER2_PACKET_RATE_US);
    iface_packet_rate_set_timer2_callback(cid4_tx_send_callback);
    iface_packet_rate_timer2_start();

    iface_stats_timer_init(TIMER_STAT_MS);
    iface_stats_set_timer_callback(stats_callback);
    iface_stats_timer_start();

    while (1) {
        iface_button_handling(NULL, reset_stats);

        if (print_stats_now) {
            if (reset_stats_now) {
                swc_connection_reset_stats(rx_cid0);
                swc_connection_reset_stats(rx_cid1);
                swc_connection_reset_stats(rx_cid2);
                swc_connection_reset_stats(tx_cid3);
                swc_connection_reset_stats(tx_cid4);
                cid3_sent_count = 0;
                cid4_sent_count = 0;
                cid3_dropped_count = 0;
                cid4_dropped_count = 0;
                reset_stats_now = false;
            } else {
                print_stats();
            }
            print_stats_now = false;
        }
    }

    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[out] err  Wireless Core error code.
 */
static void app_swc_core_init(swc_error_t *err)
{
    iface_swc_hal_init(&hal);
    iface_swc_handlers_init();

    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE,
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = PAN_ID,
        .coordinator_address = COORDINATOR_ADDRESS,
        .local_address = LOCAL_ADDRESS,
        .sleep_level = SWC_SLEEP_LEVEL,
    };
    node = swc_node_init(node_cfg, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_radio_cfg_t radio_cfg = {
        .irq_polarity = SWC_IRQ_ACTIVE_HIGH,
        .spi_mode = SWC_SPI_STANDARD,
    };
    swc_node_add_radio(node, radio_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT,
    };
    swc_channel_cfg_t tx_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT,
    };

    /* ** Node receiving from Coordinator Connection ID 0 ** */
    swc_connection_cfg_t rx_cfg_cid0 = {
        .name = "RX CID0 from Coordinator",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_BIG_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
        .connection_id_enabled = true,
    };

    rx_cid0 = swc_connection_init(node, rx_cfg_cid0, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_cid0, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    swc_connection_set_rx_success_callback(rx_cid0, rx_success_callback, err);

    /* ** Node receiving from Coordinator Connection ID 1 ** */
    swc_connection_cfg_t rx_cfg_cid1 = {
        .name = "RX CID1 from Coordinator",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_SMALL_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
        .connection_id_enabled = true,
    };

    rx_cid1 = swc_connection_init(node, rx_cfg_cid1, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_cid1, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    swc_connection_set_rx_success_callback(rx_cid1, rx_success_callback, err);

    /* ** Node receiving from Coordinator Connection ID 2 ** */
    swc_connection_cfg_t rx_cfg_cid2 = {
        .name = "RX CID2 from Coordinator",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_BIG_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
        .connection_id_enabled = true,
    };

    rx_cid2 = swc_connection_init(node, rx_cfg_cid2, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_cid2, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    swc_connection_set_rx_success_callback(rx_cid2, rx_success_callback, err);

    /* ** Node sending to Coordinator Connection ID 3 ** */
    swc_connection_cfg_t tx_cfg_cid3 = {
        .name = "TX CID3 to Coordinator",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .max_payload_size = MAX_BIG_PAYLOAD_SIZE_BYTE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
        .connection_id_enabled = true,
        .priority = 0,
    };

    tx_cid3 = swc_connection_init(node, tx_cfg_cid3, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_cid3, node, tx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** Node sending to Coordinator Connection ID 4 ** */
    swc_connection_cfg_t tx_cfg_cid4 = {
        .name = "TX CID4 to Coordinator",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .max_payload_size = MAX_SMALL_PAYLOAD_SIZE_BYTE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
        .connection_id_enabled = true,
        .priority = 1,
    };

    tx_cid4 = swc_connection_init(node, tx_cfg_cid4, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_cid4, node, tx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    swc_setup(node, err);
}

/** @brief Callback function when it is time to send a payload on Connection ID 3 (CID3).
 */
static void cid3_tx_send_callback(void)
{
    swc_error_t swc_err;
    uint8_t *payload_buf;

    swc_connection_get_payload_buffer(tx_cid3, &payload_buf, &swc_err);
    if (payload_buf != NULL) {
        snprintf((char *)payload_buf, MAX_BIG_PAYLOAD_SIZE_BYTE, "%s", "CID3");
        swc_connection_send(tx_cid3, payload_buf, MAX_BIG_PAYLOAD_SIZE_BYTE, &swc_err);
        cid3_sent_count++;
    } else {
        cid3_dropped_count++;
    }
}

/** @brief Callback function when it is time to send a payload on Connection ID 4 (CID4).
 */
static void cid4_tx_send_callback(void)
{
    swc_error_t swc_err;
    uint8_t *payload_buf;

    swc_connection_get_payload_buffer(tx_cid4, &payload_buf, &swc_err);
    if (payload_buf != NULL) {
        snprintf((char *)payload_buf, MAX_SMALL_PAYLOAD_SIZE_BYTE, "%s", "CID4");
        swc_connection_send(tx_cid4, payload_buf, MAX_SMALL_PAYLOAD_SIZE_BYTE, &swc_err);
        cid4_sent_count++;
    } else {
        cid4_dropped_count++;
    }
}

/** @brief Callback function when a frame has been successfully received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void rx_success_callback(void *conn)
{
    swc_error_t err;

    swc_connection_receive_complete(conn, &err);
}

/** @brief Print the available statistics.
 */
static void print_stats(void)
{
    char stats_string[STATS_ARRAY_LENGTH];
    uint32_t total_payload_count;
    int string_length = 0;

    const char *device_str          = "\n\r<  NODE  >\n\r";
    const char *app_stats_str       = "<<  Connection Priority App Statistics  >>\n\r";
    const char *data_rate_str       = "<<< Connections Transmission Rate >>>\n\r";
    const char *total_cid3_str      = "Payload Generated on CID3:\t%10lu\n\r";
    const char *total_cid4_str      = "Payload Generated on CID4:\t%10lu\n\r";
    const char *payload_sent_str    = "  Payload Sent:\t\t\t%10lu (%05.2f%%)\n\r";
    const char *payload_dropped_str = "  Payload Dropped:\t\t%10lu (%05.2f%%)\n\r";
    const char *overview_str        = "<<< Connections Transmission Overview >>>\n\r";
    const char *cid3_sent_str       = "Payload Sent on CID3:\t\t%10lu (%05.2f%%)\n\r";
    const char *cid4_sent_str       = "Payload Sent on CID4:\t\t%10lu (%05.2f%%)\n\r";
    const char *wireless_stats_str  = "<<  Wireless Core Statistics  >>\n\r";

    swc_connection_update_stats(rx_cid0);
    swc_connection_update_stats(rx_cid1);
    swc_connection_update_stats(rx_cid2);
    swc_connection_update_stats(tx_cid3);
    swc_connection_update_stats(tx_cid4);

    total_payload_count = (cid3_sent_count + cid4_sent_count);

    /* Device role */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

    /* Application statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, app_stats_str);
    /* Connection transmission rate */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, data_rate_str);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, total_cid3_str,
                              cid3_sent_count + cid3_dropped_count);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, payload_sent_str, cid3_sent_count,
                              (double)cid3_sent_count * 100 / (cid3_sent_count + cid3_dropped_count));
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, payload_dropped_str, cid3_dropped_count,
                              (double)cid3_dropped_count * 100 / (cid3_sent_count + cid3_dropped_count));
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, total_cid4_str,
                              cid4_sent_count + cid4_dropped_count);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, payload_sent_str, cid4_sent_count,
                              (double)cid4_sent_count * 100 / (cid4_sent_count + cid4_dropped_count));
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, payload_dropped_str, cid4_dropped_count,
                              (double)cid4_dropped_count * 100 / (cid4_sent_count + cid4_dropped_count));
    /* Link capacity utilization */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, overview_str);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, cid3_sent_str, cid3_sent_count,
                              (double)cid3_sent_count * 100 / total_payload_count);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, cid4_sent_str, cid4_sent_count,
                              (double)cid4_sent_count * 100 / total_payload_count);

    /* Wireless statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, wireless_stats_str);
    string_length += swc_connection_format_stats(rx_cid0, node, stats_string + string_length, sizeof(stats_string) - string_length);
    string_length += swc_connection_format_stats(rx_cid1, node, stats_string + string_length, sizeof(stats_string) - string_length);
    string_length += swc_connection_format_stats(rx_cid2, node, stats_string + string_length, sizeof(stats_string) - string_length);
    string_length += swc_connection_format_stats(tx_cid3, node, stats_string + string_length, sizeof(stats_string) - string_length);
    string_length += swc_connection_format_stats(tx_cid4, node, stats_string + string_length, sizeof(stats_string) - string_length);

    iface_print_string(stats_string);
}

/** @brief Reset the TX and RX statistics.
 */
static void reset_stats(void)
{
    if (reset_stats_now == false) {
        reset_stats_now = true;
    }
}

/** @brief Callback handling when the stats have to be printed.
 */
static void stats_callback(void)
{
    print_stats_now = true;
}
