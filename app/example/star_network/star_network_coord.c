/** @file  star_network_coord.c
 *  @brief This is a basic example on how to use a SPARK star network.
 *         This example uses the SPARK SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include "iface_star_network.h"
#include "iface_wireless.h"
#include "swc_api.h"
#include "swc_cfg_coord.h"

/* CONSTANTS ******************************************************************/
#define SWC_MEM_POOL_SIZE     8700
#define MAX_PAYLOAD_SIZE_BYTE 12
#define BUTTON_PRESSED        0x01

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t hal;
static swc_node_t *node;
static swc_connection_t *tx_to_node1_conn;
static swc_connection_t *rx_from_node1_conn;
static swc_connection_t *tx_to_node2_conn;
static swc_connection_t *rx_from_node2_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t tx_to_node1_timeslots[] = TX_TO_NODE1_TIMESLOTS;
static int32_t rx_from_node1_timeslots[] = RX_FROM_NODE1_TIMESLOTS;
static int32_t tx_to_node2_timeslots[] = TX_TO_NODE2_TIMESLOTS;
static int32_t rx_from_node2_timeslots[] = RX_FROM_NODE2_TIMESLOTS;

/* ** Application Specific ** */
static unsigned long inc_node1;
static unsigned long inc_node2;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_node1_rx_success_callback(void *conn);
static void conn_node2_rx_success_callback(void *conn);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    swc_error_t swc_err;
    uint8_t *star_network_buf;

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    swc_connect(&swc_err);

    while (1) {

        swc_connection_get_payload_buffer(tx_to_node1_conn, &star_network_buf, &swc_err);
        if (star_network_buf != NULL) {
            /* Send the payload through the Wireless Core */
            snprintf((char *)star_network_buf, MAX_PAYLOAD_SIZE_BYTE, "x%lu", inc_node1++);
            star_network_buf[0] = iface_read_button_status(BUTTON_A);
            /* Payload must include the terminating NULL */
            swc_connection_send(tx_to_node1_conn, star_network_buf, MAX_PAYLOAD_SIZE_BYTE, &swc_err);
        }

        swc_connection_get_payload_buffer(tx_to_node2_conn, &star_network_buf, &swc_err);
        if (star_network_buf != NULL) {
            /* Send the payload through the Wireless Core */
            snprintf((char *)star_network_buf, MAX_PAYLOAD_SIZE_BYTE, "x%lu", inc_node2++);
            star_network_buf[0] = iface_read_button_status(BUTTON_B);
            /* Payload must include the terminating NULL */
            swc_connection_send(tx_to_node2_conn, star_network_buf, MAX_PAYLOAD_SIZE_BYTE, &swc_err);
        }
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
        .fast_sync_enabled = true,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE
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
        .sleep_level = SWC_SLEEP_LEVEL
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

    /* ** Coordinator sending to Node1 connection ** */
    swc_connection_cfg_t tx_to_node1_conn_cfg = {
        .name = "Coordinator to Node1 connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS_1,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_to_node1_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_to_node1_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    tx_to_node1_conn = swc_connection_init(node, tx_to_node1_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_to_node1_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_to_node1_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_to_node1_conn, node, tx_to_node1_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** Coordinator receiving from Node1 connection ** */
    swc_connection_cfg_t rx_from_node1_conn_cfg = {
        .name = "Node1 to Coordinator connection",
        .source_address = REMOTE_ADDRESS_1,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_from_node1_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_from_node1_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    rx_from_node1_conn = swc_connection_init(node, rx_from_node1_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_node1_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_from_node1_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_from_node1_conn, node, rx_from_node1_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_from_node1_conn, conn_node1_rx_success_callback, err);

    /* ** Coordinator sending to Node2 connection ** */
    swc_connection_cfg_t tx_to_node2_conn_cfg = {
        .name = "Coordinator to Node2 connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS_2,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_to_node2_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_to_node2_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    tx_to_node2_conn = swc_connection_init(node, tx_to_node2_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_to_node2_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_to_node2_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_to_node2_conn, node, tx_to_node2_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** Coordinator receiving from Node2 connection ** */
    swc_connection_cfg_t rx_from_node2_conn_cfg = {
        .name = "Node2 to Coordinator connection",
        .source_address = REMOTE_ADDRESS_2,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_from_node2_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_from_node2_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    rx_from_node2_conn = swc_connection_init(node, rx_from_node2_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_node2_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_from_node2_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_from_node2_conn, node, rx_from_node2_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_from_node2_conn, conn_node2_rx_success_callback, err);

    swc_setup(node, err);
}

/** @brief Callback function when a frame has been successfully received from Node1.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_node1_rx_success_callback(void *conn)
{
    (void)conn;

    swc_error_t err;
    uint8_t *payload = NULL;

    /* Get new payload */
    swc_connection_receive(rx_from_node1_conn, &payload, &err);

    if (payload[0] == BUTTON_PRESSED) {
        iface_payload_sent_status();
    } else {
        iface_empty_payload_sent_status();
    }

    iface_usb_printf("Received from Node1 : %s\n\r", &payload[1]);

    /* Notify the Wireless Core that the new payload has been read */
    swc_connection_receive_complete(rx_from_node1_conn, &err);
}

/** @brief Callback function when a frame has been successfully received from Node2.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_node2_rx_success_callback(void *conn)
{
    (void)conn;

    swc_error_t err;
    uint8_t *payload = NULL;

    /* Get new payload */
    swc_connection_receive(rx_from_node2_conn, &payload, &err);

    if (payload[0] == BUTTON_PRESSED) {
        iface_payload_received_status();
    } else {
        iface_empty_payload_received_status();
    }

    iface_usb_printf("Received from Node2 : %s\n\r", &payload[1]);

    /* Notify the Wireless Core that the new payload has been read */
    swc_connection_receive_complete(rx_from_node2_conn, &err);
}
