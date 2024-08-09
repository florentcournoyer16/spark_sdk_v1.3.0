/** @file  pairing_basic_node.c
 *  @brief This application implements a basic pairing procedure to allow pairing two devices in a point-to-point configuration.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "iface_pairing_basic.h"
#include "iface_wireless.h"
#include "pairing_api.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_utils.h"

/* CONSTANTS ******************************************************************/
/* Memory reserved for the SPARK Wireless Core. */
#define SWC_MEM_POOL_SIZE          5100
/* The timeout in second after which the pairing procedure will abort. */
#define PAIRING_TIMEOUT_IN_SECONDS 10
/* The pairing device role is used for the coordinator's pairing discovery list. */
#define PAIRING_DEVICE_ROLE        1
/* The application code prevents unwanted devices from pairing with this application. */
#define PAIRING_APP_CODE           0x12345678

/* TYPES **********************************************************************/
/** @brief The application level device states.
 */
typedef enum device_pairing_state {
    /*! The device is not paired with the coordinator. */
    DEVICE_UNPAIRED,
    /*! The device is paired with the coordinator. */
    DEVICE_PAIRED,
} device_pairing_state_t;

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
static int32_t tx_timeslots[] = TX_TIMESLOTS;
static int32_t rx_timeslots[] = RX_TIMESLOTS;

/* ** Application Specific ** */
static device_pairing_state_t device_pairing_state;
static pairing_cfg_t app_pairing_cfg;
static pairing_assigned_address_t pairing_assigned_address;
static uint8_t button_status;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_configuration(pairing_assigned_address_t *app_pairing, swc_error_t *err);
static void conn_rx_success_callback(void *conn);

static void send_button_status(void);
static void enter_pairing_mode(void);
static void unpair_device(void);

static void pairing_application_callback(void);
static void abort_pairing_procedure(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int cortical_implant_routine(void)
{
    swc_memory_pool_ptr = malloc(SWC_MEM_POOL_SIZE);
    /* Initialize the board and the SPARK Wireless Core. */
    iface_board_init();
    iface_swc_hal_init(&hal);
    iface_swc_handlers_init();

    /* The device starts by default in the unpaired state. */
    device_pairing_state = DEVICE_UNPAIRED;

    while (1) {
        if (device_pairing_state == DEVICE_UNPAIRED) {
            /* Since the device is unpaired the send_button_status should do nothing */
            enter_pairing_mode();
        } else if (device_pairing_state == DEVICE_PAIRED) {
            //iface_button_handling(send_button_status, unpair_device);
        }
    }
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[in]  pairing_assigned_address  Configure the Wireless Core with the pairing values.
 *  @param[out] err                       Wireless Core error code.
 */
static void app_swc_configuration(pairing_assigned_address_t *app_pairing, swc_error_t *err)
{
    uint16_t node_address;
    uint16_t coordinator_address;

    node_address = app_pairing->node_address;
    coordinator_address = app_pairing->coordinator_address;

    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = SWC_ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = SWC_ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool_ptr,
        .memory_pool_size = SWC_MEM_POOL_SIZE
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = app_pairing->pan_id,
        .coordinator_address = coordinator_address,
        .local_address = node_address,
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

    /* ** TX Connection ** */
    swc_connection_cfg_t tx_conn_cfg = {
        .name = "TX Connection",
        .source_address = node_address,
        .destination_address = coordinator_address,
        .max_payload_size = 1,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_timeslots,
        .timeslot_count = SWC_ARRAY_SIZE(tx_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = true,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    tx_conn = swc_connection_init(node, tx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        tx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_conn, node, tx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** RX Connection ** */
    swc_connection_cfg_t rx_conn_cfg = {
        .name = "RX Connection",
        .source_address = coordinator_address,
        .destination_address = node_address,
        .max_payload_size = 1,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = SWC_ARRAY_SIZE(rx_timeslots),
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
    rx_conn = swc_connection_init(node, rx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_conn, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_conn, conn_rx_success_callback, err);

    swc_setup(node, err);
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

    /*
     * When paired, the device receives the BTN2 status from the other
     * device and applies this state to the LED2.
     */
    if (payload[0] == 0) {
        iface_empty_payload_received_status();
    } else {
        iface_payload_received_status();
    }

    swc_connection_receive_complete(rx_conn, &err);
}

/** @brief Toggle a button variable and send it through the SWC.
 */
static void send_button_status(void)
{
    swc_error_t err;

    button_status = !button_status;

    /* Send the payload over the air */
    swc_connection_send(tx_conn, (uint8_t *)&button_status, sizeof(button_status), &err);
}

/** @brief Enter in Pairing Mode using the Pairing Module.
 */
static void enter_pairing_mode(void)
{
    swc_error_t* swc_err = malloc(sizeof(swc_error_t));
    pairing_error_t* pairing_err = malloc(sizeof(pairing_error_t));
    *pairing_err = PAIRING_ERR_NONE;

    pairing_event_t* pairing_event = malloc(sizeof(pairing_event_t));

    iface_notify_enter_pairing();

    /* The wireless core must be stopped before starting the pairing procedure. */
    if (swc_get_status() == SWC_STATUS_RUNNING) {
        swc_disconnect(swc_err);
        if ((*swc_err != SWC_ERR_NONE) && (*swc_err != SWC_ERR_NOT_CONNECTED)) {
            while (1);
        }
    }

    /* Give the information to the Pairing Module. */
    app_pairing_cfg.app_code = PAIRING_APP_CODE;
    app_pairing_cfg.timeout_sec = PAIRING_TIMEOUT_IN_SECONDS;
    app_pairing_cfg.hal = &hal;
    app_pairing_cfg.application_callback = pairing_application_callback;
    app_pairing_cfg.memory_pool = swc_memory_pool_ptr;
    app_pairing_cfg.memory_pool_size = SWC_MEM_POOL_SIZE;
    app_pairing_cfg.uwb_regulation = SWC_REGULATION_FCC;
    *pairing_event = pairing_node_start(&app_pairing_cfg, &pairing_assigned_address, PAIRING_DEVICE_ROLE, pairing_err);
    if (*pairing_err != PAIRING_ERR_NONE) {
        while (1);
    }

    /* Handle the pairing events. */
    switch (*pairing_event) {
    case PAIRING_EVENT_SUCCESS:
        iface_notify_pairing_successful();

        app_swc_configuration(&pairing_assigned_address, swc_err);
        if (*swc_err != SWC_ERR_NONE) {
            while (1);
        }
        swc_connect(swc_err);

        device_pairing_state = DEVICE_PAIRED;

        break;
    case PAIRING_EVENT_TIMEOUT:
    case PAIRING_EVENT_INVALID_APP_CODE:
    case PAIRING_EVENT_ABORT:
    default:
        /* Indicate that the pairing process was unsuccessful */
        iface_notify_not_paired();

        device_pairing_state = DEVICE_UNPAIRED;
    }
    free(swc_err);
    free(pairing_err);
    free(pairing_event);
}

/** @brief Put the device in the unpaired state and disconnect it from the network.
 */
static void unpair_device(void)
{
    swc_error_t swc_err;

    device_pairing_state = DEVICE_UNPAIRED;

    swc_disconnect(&swc_err);
    if ((swc_err != SWC_ERR_NONE) && (swc_err != SWC_ERR_NOT_CONNECTED)) {
        while (1);
    }

    iface_notify_not_paired();
}

/** @brief Application callback called during pairing.
 */
static void pairing_application_callback(void)
{
    /*
     * Note: The button press will only be detected when the pairing module
     *       executes the registered application callback, which might take
     *       a variable amount of time.
     */
    //iface_button_handling(NULL, abort_pairing_procedure);
}

/** @brief Abort the pairing procedure once started.
 */
static void abort_pairing_procedure(void)
{
    pairing_abort();
}
