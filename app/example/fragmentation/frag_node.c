/** @file  frag_node.c
 *  @brief This is a basic example of how to use the SPARK fragmentation.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include "iface_fragmentation.h"
#include "iface_wireless.h"
#include "pseudo_data.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SWC_MEM_POOL_SIZE          12000
#define MAX_PAYLOAD_SIZE_BYTE      500
#define MAX_SWC_PAYLOAD_SIZE_BYTE  124
#define STATS_ARRAY_LENGTH         2000
#define SEQ_NUM_OFFSET             1

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t hal;
static swc_node_t *node;
static swc_connection_t *rx_conn;
static swc_connection_t *tx_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_timeslots[] = RX_TIMESLOTS;
static int32_t tx_timeslots[] = TX_TIMESLOTS;

/* ** Application Specific ** */
static uint8_t tx_payload[MAX_PAYLOAD_SIZE_BYTE];
static uint8_t rx_payload[MAX_PAYLOAD_SIZE_BYTE];
static uint32_t tx_count;
static uint8_t tx_payload_id;
static uint32_t valid_payload_count;
static uint32_t invalid_payload_count;
static uint32_t valid_crc_count;
static uint32_t invalid_crc_count;
static bool print_stats_now;
static bool reset_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_tx_success_callback(void *conn);
static void conn_rx_success_callback(void *conn);
static void print_stats(void);
static void reset_stats(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    swc_error_t swc_err;

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    /* Fill application payload */
    tx_payload[0] = tx_payload_id;
    tx_payload_id++;
    pseudo_data_generate((uint8_t *)&tx_payload[SEQ_NUM_OFFSET], MAX_PAYLOAD_SIZE_BYTE - SEQ_NUM_OFFSET);

    swc_connect(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    /* Make the first send */
    swc_connection_send(tx_conn, (uint8_t *)tx_payload, MAX_PAYLOAD_SIZE_BYTE, &swc_err);
    while (1) {
        if (print_stats_now) {
            if (reset_stats_now) {
                swc_connection_reset_stats(tx_conn);
                swc_connection_reset_stats(rx_conn);
                valid_payload_count   = 0;
                invalid_payload_count = 0;
                valid_crc_count       = 0;
                invalid_crc_count     = 0;
                reset_stats_now       = false;
            } else {
                print_stats();
            }
            print_stats_now = false;
        }
        iface_button_handling(reset_stats, NULL);
    }
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
        .timeslot_sequence        = timeslot_us,
        .timeslot_sequence_length = ARRAY_SIZE(timeslot_us),
        .channel_sequence         = channel_sequence,
        .channel_sequence_length  = ARRAY_SIZE(channel_sequence),
        .memory_pool              = swc_memory_pool,
        .memory_pool_size         = SWC_MEM_POOL_SIZE,
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role                = NETWORK_ROLE,
        .pan_id              = PAN_ID,
        .coordinator_address = COORD_ADDRESS,
        .local_address       = LOCAL_ADDRESS,
        .sleep_level         = SWC_SLEEP_LEVEL,
    };
    node = swc_node_init(node_cfg, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_radio_cfg_t radio_cfg = {
        .irq_polarity = SWC_IRQ_ACTIVE_HIGH,
        .spi_mode     = SWC_SPI_STANDARD,
    };
    swc_node_add_radio(node, radio_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    /* ** TX Connection ** */
    /*
     * A connection using auto-reply timeslots (i.e. AUTO_TIMESLOT()) needs
     * only a subset of the configuration since it mostly reuses the configuration
     * of the main connection (RX Connection here). It also does not need any
     * channel to be added to it, since, again, it will reuse the main connection ones.
     */
    swc_connection_cfg_t tx_conn_cfg = {
        .name                    = "TX Connection",
        .source_address          = LOCAL_ADDRESS,
        .destination_address     = REMOTE_ADDRESS,
        .queue_size              = TX_DATA_QUEUE_SIZE,
        .max_payload_size        = MAX_SWC_PAYLOAD_SIZE_BYTE,
        .timeslot_id             = tx_timeslots,
        .timeslot_count          = ARRAY_SIZE(tx_timeslots),
        .fragmentation_enabled   = true,
    };
    tx_conn = swc_connection_init(node, tx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_connection_set_tx_success_callback(tx_conn, conn_tx_success_callback, err);

    /* ** RX Connection ** */
    swc_connection_cfg_t rx_conn_cfg = {
        .name                       = "RX Connection",
        .source_address             = REMOTE_ADDRESS,
        .destination_address        = LOCAL_ADDRESS,
        .max_payload_size           = MAX_SWC_PAYLOAD_SIZE_BYTE,
        .queue_size                 = RX_DATA_QUEUE_SIZE,
        .modulation                 = SWC_MODULATION,
        .fec                        = SWC_FEC_LEVEL,
        .timeslot_id                = rx_timeslots,
        .timeslot_count             = ARRAY_SIZE(rx_timeslots),
        .ack_enabled                = true,
        .arq_enabled                = true,
        .auto_sync_enabled          = true,
        .arq_settings.try_count     = 0,
        .arq_settings.time_deadline = 0,
        .fragmentation_enabled      = true,
    };
    rx_conn = swc_connection_init(node, rx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_AUTO_REPLY_PULSE_COUNT,
        .tx_pulse_width = TX_AUTO_REPLY_PULSE_WIDTH,
        .tx_pulse_gain  = TX_AUTO_REPLY_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT,
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_conn, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_conn, conn_rx_success_callback, err);

    swc_setup(node, err);
}

/** @brief Callback function when a previously sent frame has been ACK'd.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_success_callback(void *conn)
{
    (void)conn;
    swc_error_t swc_err;

    iface_tx_conn_status();
    tx_payload[0] = tx_payload_id;
    tx_payload_id++;
    pseudo_data_generate((uint8_t *)&tx_payload[SEQ_NUM_OFFSET], MAX_PAYLOAD_SIZE_BYTE - SEQ_NUM_OFFSET);
    swc_connection_send(tx_conn, (uint8_t *)tx_payload, MAX_PAYLOAD_SIZE_BYTE, &swc_err);
    /* Print stats every time the required number of samples has been sent */
    if ((tx_count++ % 100) == 0) {
        print_stats_now = true;
    }
}

/** @brief Callback function when a frame has been successfully received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_success_callback(void *conn)
{
    (void)conn;
    swc_error_t err;
    uint8_t current_rx_payload_id;
    static uint8_t previous_rx_payload_id;

    /* Get new payload */
    swc_connection_receive_to_buffer(rx_conn, rx_payload, MAX_PAYLOAD_SIZE_BYTE, &err);
    /* Look for count matching*/
    current_rx_payload_id = rx_payload[0];
    if (((uint8_t)(previous_rx_payload_id + 1)) == current_rx_payload_id) {
        valid_payload_count++;
    } else {
        invalid_payload_count++;
    }
    previous_rx_payload_id = rx_payload[0];

    if (pseudo_data_is_crc_populated(MAX_PAYLOAD_SIZE_BYTE - SEQ_NUM_OFFSET)) {
        if (pseudo_data_validate((uint8_t *)&rx_payload[SEQ_NUM_OFFSET], MAX_PAYLOAD_SIZE_BYTE - SEQ_NUM_OFFSET)) {
            valid_crc_count++;
        } else {
            invalid_crc_count++;
        }
    }

    iface_rx_conn_status();
}

/** @brief Print the ranging and wireless statistics.
 */
static void print_stats(void)
{
    char stats_string[STATS_ARRAY_LENGTH];
    int string_length = 0;
    const char *device_str = "\n\r<  NODE  >\n\r";
    const char *app_stats_str = "<<  Fragmentation App Statistics  >>\n\r";
    const char *valid_sequence_count_str = "Valid Payload Sequence Count:\t%10d\n\r";
    const char *invalid_sequence_count_str = "Invalid Payload Sequence Count:\t%10d\n\r";
    const char *valid_crc_count_str = "Valid Payload CRC Count:\t%10d\n\r";
    const char *invalid_crc_count_str = "Invalid Payload CRC Count:\t%10d\n\r";
    const char *wireless_stats_str = "<<  Wireless Core Statistics  >>\n\r";

    /* Device Prelude */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

    /* Application statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string), app_stats_str);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string), valid_sequence_count_str, valid_payload_count);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string), invalid_sequence_count_str, invalid_payload_count);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string), valid_crc_count_str, valid_crc_count);
    string_length += snprintf(stats_string + string_length, sizeof(stats_string), invalid_crc_count_str, invalid_crc_count);

    /* Wireless statistics */
    /* TX */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, wireless_stats_str);
    swc_connection_update_stats(tx_conn);
    string_length += swc_connection_format_stats(tx_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);

    /* RX */
    swc_connection_update_stats(rx_conn);
    string_length += swc_connection_format_stats(rx_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);

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
