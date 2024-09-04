/** @file  cortical_implant_coord.c
 *  @brief This is a basic example of how to use the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_cortical_implant.h"
#include "iface_wireless.h"
#include "swc_api.h"
#include "swc_cfg_coord.h"
#include "swc_stats.h"
#include "hw_cfg.h"
#include <stdio.h>
#include <zephyr/kernel.h>

/* CONSTANTS ******************************************************************/
#define SWC_MEM_POOL_SIZE 6000
#define MAX_PAYLOAD_SIZE_BYTE 20
#define ENDING_NULL_CHARACTER_SIZE 1
#define STATS_ARRAY_LENGTH 1024

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
static uint8_t *swc_memory_pool_ptr;
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
static char rx_payload[MAX_PAYLOAD_SIZE_BYTE];
static uint32_t tx_count;
static bool print_stats_now;
static bool reset_stats_now;
static swc_error_t swc_err;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_tx_success_callback(void *conn);
static void conn_tx_fail_callback(void *conn);
static void conn_rx_success_callback(void *conn);
static void print_stats(void);
static void reset_stats(void);

void init_cortical_implant(void)
{  
    swc_memory_pool_ptr = k_malloc(SWC_MEM_POOL_SIZE);

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE)
    {
        while (1)
            ;
    }
    printk("INIT : COORD Initialisation complete.\n");
}
/* PUBLIC FUNCTIONS ***********************************************************/
extern void cortical_implant_routine(void)
{
    uint8_t str_counter = 0;
    uint8_t *cortical_implant_buf = NULL;
    pair_device();
    print_stats_now = false;

    while (!print_stats_now)
    {
        swc_connection_get_payload_buffer(tx_conn, &cortical_implant_buf, &swc_err);
        if (cortical_implant_buf != NULL)
        {
            size_t tx_payload_size = snprintf((char *)cortical_implant_buf, MAX_PAYLOAD_SIZE_BYTE, "Hello, World! %d\n\r", str_counter++);

            swc_connection_send(tx_conn, cortical_implant_buf, tx_payload_size + ENDING_NULL_CHARACTER_SIZE, &swc_err);
        }
    }
    if (reset_stats_now)
    {
        swc_connection_reset_stats(tx_conn);
        swc_connection_reset_stats(rx_conn);
        reset_stats_now = false;
    }
    else
    {
        iface_print_string(rx_payload);
        print_stats();
    }

    print_stats_now = false;
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
        .timeslot_sequence_length = ARRAY_SIZE_SPARK(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = ARRAY_SIZE_SPARK(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool_ptr,
        .memory_pool_size = SWC_MEM_POOL_SIZE,
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE)
    {
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
    if (*err != SWC_ERR_NONE)
    {
        return;
    }

    swc_radio_cfg_t radio_cfg = {
        .irq_polarity = SWC_IRQ_ACTIVE_HIGH,
        .spi_mode = SWC_SPI_STANDARD,
    };
    swc_node_add_radio(node, radio_cfg, &hal, err);
    if (*err != SWC_ERR_NONE)
    {
        return;
    }

    /* ** TX Connection ** */
    swc_connection_cfg_t tx_conn_cfg = {
        .name = "TX Connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE_SPARK(tx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
    };
    tx_conn = swc_connection_init(node, tx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE)
    {
        return;
    }

    swc_channel_cfg_t tx_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT,
    };
    for (uint8_t i = 0; i < ARRAY_SIZE_SPARK(channel_sequence); i++)
    {
        tx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_conn, node, tx_channel_cfg, err);
        if (*err != SWC_ERR_NONE)
        {
            return;
        }
    }
    swc_connection_set_tx_success_callback(tx_conn, conn_tx_success_callback, err);
    swc_connection_set_tx_fail_callback(tx_conn, conn_tx_fail_callback, err);

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
        .timeslot_count = ARRAY_SIZE_SPARK(rx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false,
    };

    rx_conn = swc_connection_init(node, rx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE)
    {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT,
    };
    for (uint8_t i = 0; i < ARRAY_SIZE_SPARK(channel_sequence); i++)
    {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_conn, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE)
        {
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

    /* Print stats every 1000 transmissions */
    tx_count++;
    if ((tx_count % 1000) == 0)
    {
        print_stats_now = true;
    }
}

/** @brief Callback function when a previously sent frame has not been ACK'd.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_fail_callback(void *conn)
{
    (void)conn;

    /* Print stats every 1000 transmissions */
    tx_count++;
    if ((tx_count % 1000) == 0)
    {
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
    memcpy(rx_payload, payload, sizeof(rx_payload));

    /* Free the payload memory */
    swc_connection_receive_complete(rx_conn, &err);

    iface_rx_conn_status();
}

/** @brief Print the TX and RX statistics.
 */
static void print_stats(void)
{
    static char stats_string[STATS_ARRAY_LENGTH];
    int string_length = 0;

    swc_connection_update_stats(tx_conn);
    string_length = swc_connection_format_stats(tx_conn, node, stats_string, sizeof(stats_string));
    swc_connection_update_stats(rx_conn);
    swc_connection_format_stats(rx_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);
    iface_print_string(stats_string);
}

/** @brief Reset the TX and RX statistics.
 */
static void reset_stats(void)
{
    if (reset_stats_now == false)
    {
        reset_stats_now = true;
    }
}

void unpair_device(void)
{
    swc_disconnect(&swc_err);
    if ((swc_err != SWC_ERR_NONE) && (swc_err != SWC_ERR_NOT_CONNECTED))
    {
        while (1)
            ;
    }

    gpio_pin_set_dt(&led2, 0);
}

void pair_device(void)
{
    swc_connect(&swc_err);
    if ((swc_err != SWC_ERR_NONE) && (swc_err != SWC_ERR_NOT_CONNECTED))
    {
        while (1)
            ;
    }
    gpio_pin_set_dt(&led2, 0);
}
