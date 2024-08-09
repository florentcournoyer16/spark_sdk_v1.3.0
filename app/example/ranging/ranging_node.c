/** @file  ranging_node.c
 *  @brief This is a basic example of how to use the SPARK Ranging Core.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include "iface_ranging.h"
#include "iface_wireless.h"
#include "srac_api.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SWC_MEM_POOL_SIZE          6900
#define MAX_PAYLOAD_SIZE_BYTE      20
#define STATS_ARRAY_LENGTH         2000
#define ENDING_NULL_CHARACTER_SIZE 1
#define RANGING_SAMPLE_COUNT       128

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
static uint32_t tx_count;
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

    uint8_t str_counter  = 0;
    uint8_t *ranging_buf = NULL;

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    swc_connect(&swc_err);

    while (1) {
        swc_connection_get_payload_buffer(tx_conn, &ranging_buf, &swc_err);
        if (ranging_buf != NULL) {
            size_t tx_payload_size = snprintf((char *)ranging_buf, MAX_PAYLOAD_SIZE_BYTE, "SPARK! %d\n\r", str_counter++);

            swc_connection_send(tx_conn, ranging_buf, tx_payload_size + ENDING_NULL_CHARACTER_SIZE, &swc_err);
        }

        /* Print stats every time the required number of samples has been sent */
        if (print_stats_now) {
            if (reset_stats_now) {
                swc_connection_reset_stats(tx_conn);
                swc_connection_reset_stats(rx_conn);
                reset_stats_now = false;
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
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = ARRAY_SIZE(channel_sequence),
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE,
        .rdo_enabled = true,
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = PAN_ID,
        .coordinator_address = COORD_ADDRESS,
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

    /* ** TX Connection ** */
    /*
     * A connection using auto-reply timeslots (i.e. AUTO_TIMESLOT()) needs
     * only a subset of the configuration since it mostly reuses the configuration
     * of the main connection (RX Connection here). It also does not need any
     * channel to be added to it, since, again, it will reuse the main connection ones.
     */
    swc_connection_cfg_t tx_conn_cfg = {
        .name = "TX Connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
        .ranging_enabled = true,
    };
    tx_conn = swc_connection_init(node, tx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_connection_set_tx_success_callback(tx_conn, conn_tx_success_callback, err);

    /* ** RX Connection ** */
    swc_connection_cfg_t rx_conn_cfg = {
        .name = "RX Connection",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .ranging_enabled = true,
        .cca_enabled = true,
        .cca_settings.threshold = 25,
        .cca_settings.try_count = 6,
        .cca_settings.retry_time = 819, /* 40 us */
        .cca_settings.fail_action = SWC_CCA_FORCE_TX,
        .rdo_enabled = true,
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

    iface_tx_conn_status();

    /* Print stats every time the required number of samples has been sent */
    tx_count++;
    if ((tx_count % RANGING_SAMPLE_COUNT) == 0) {
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
    uint8_t *payload = NULL;

    /* Get new payload */
    swc_connection_receive(rx_conn, &payload, &err);

    /* Do anything with the newly obtained payload */

    /* Free the payload memory */
    swc_connection_receive_complete(rx_conn, &err);

    iface_rx_conn_status();
}

/** @brief Print the ranging and wireless statistics.
 */
static void print_stats(void)
{
    static char stats_string[STATS_ARRAY_LENGTH];
    int string_length = 0;

    const char *device_str         = "\n\r<  NODE  >\n\r";
    const char *wireless_stats_str = "<<  Wireless Core Statistics  >>\n\r";

    /* Device Prelude */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

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
