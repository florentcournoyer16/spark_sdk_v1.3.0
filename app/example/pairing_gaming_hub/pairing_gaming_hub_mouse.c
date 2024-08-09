/** @file  pairing_gaming_hub_mouse.c
 *  @brief The mouse device is part of the gaming hub to demonstrate a point to multipoint pairing application.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_audio.h"
#include "iface_pairing_gaming_hub.h"
#include "iface_wireless.h"
#include "pairing_api.h"
#include "sac_api.h"
#include "swc_api.h"
#include "swc_cfg_mouse.h"
#include "swc_utils.h"

/* CONSTANTS ******************************************************************/
/* Memory reserved for the SPARK Wireless Core. */
#define SWC_MEM_POOL_SIZE          5600
/* Wireless payload size for the mouse device. */
#define MOUSE_PAYLOAD_SIZE         1

/* The device role is used by the coordinator's pairing discovery list. */
#define PAIRING_DEVICE_ROLE        2
/* The application code prevents unwanted devices from pairing with this application. */
#define PAIRING_APP_CODE           0x87654321
/* The timeout in second after which the pairing procedure will abort. */
#define PAIRING_TIMEOUT_IN_SECONDS 10

/* TYPES **********************************************************************/
/** @brief The application level device states.
 */
typedef enum device_pairing_state {
    /*! The device is not paired with the node. */
    DEVICE_UNPAIRED,
    /*! The device is paired with the node. */
    DEVICE_PAIRED,
} device_pairing_state_t;

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *rx_from_beacon_conn;
static swc_connection_t *tx_to_dongle_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_from_beacon_timeslots[] = RX_FROM_BEACON_TIMESLOTS;
static int32_t tx_to_dongle_timeslots[] = TX_TO_DONGLE_TIMESLOTS;

/* ** Application Specific ** */
static device_pairing_state_t device_pairing_state;
static pairing_cfg_t app_pairing_cfg;
static pairing_assigned_address_t pairing_assigned_address;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_configuration(pairing_assigned_address_t *pairing_assigned_address, swc_error_t *err);

static void send_mouse_pressed_state(void);
static void send_mouse_released_state(void);

static void enter_pairing_mode(void);
static void unpair_device(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    /* Initialize the board and the SPARK wireless core. */
    iface_board_init();
    iface_swc_hal_init(&swc_hal);
    iface_swc_handlers_init();

    /* The device starts by default in the unpaired state. */
    device_pairing_state = DEVICE_UNPAIRED;

    while (1) {
        if (device_pairing_state == DEVICE_UNPAIRED) {
            /* Since the device is unpaired the send_mouse_pressed_state should do nothing */
            iface_button_handling_osr(NULL, enter_pairing_mode);
        } else if (device_pairing_state == DEVICE_PAIRED) {
            iface_button_handling_osr(send_mouse_pressed_state, unpair_device);
            iface_button_handling_osf(send_mouse_released_state, NULL);
        }
    }

    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[in]  pairing_assigned_address  Addresses received from the pairing process.
 *  @param[out] err                       Wireless Core error code.
 */
static void app_swc_configuration(pairing_assigned_address_t *pairing_assigned_address, swc_error_t *err)
{
    uint8_t local_address;
    uint8_t remote_address;

    local_address = pairing_assigned_address->node_address;
    remote_address = pairing_assigned_address->coordinator_address;

    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = SWC_ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = SWC_ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE
    };
    swc_init(core_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = pairing_assigned_address->pan_id,
        .coordinator_address = pairing_assigned_address->coordinator_address,
        .local_address = local_address,
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
    swc_node_add_radio(node, radio_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    /* ** Mouse receiving Beacon from Dongle connection ** */
    swc_connection_cfg_t rx_from_beacon_conn_cfg;

    rx_from_beacon_conn_cfg = swc_get_beacon_connection_config(node, remote_address, rx_from_beacon_timeslots,
                                                               SWC_ARRAY_SIZE(rx_from_beacon_timeslots));
    rx_from_beacon_conn = swc_connection_init(node, rx_from_beacon_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_beacon_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        rx_from_beacon_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_from_beacon_conn, node, rx_from_beacon_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** Mouse sending to Dongle connection ** */
    swc_connection_cfg_t tx_to_dongle_conn_cfg = {
        .name = "Mouse to Dongle connection",
        .source_address = local_address,
        .destination_address = remote_address,
        .max_payload_size = MOUSE_PAYLOAD_SIZE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_to_dongle_timeslots,
        .timeslot_count = SWC_ARRAY_SIZE(tx_to_dongle_timeslots),
        .ack_enabled = true,
        .arq_enabled = true,
        .arq_settings.try_count = 0,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = false,
        .cca_enabled = false,
        .rdo_enabled = false
    };
    tx_to_dongle_conn = swc_connection_init(node, tx_to_dongle_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_to_dongle_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        tx_to_dongle_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_to_dongle_conn, node, tx_to_dongle_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_setup(node, err);
}


/** @brief Send the mouse pressed state to the Dongle.
 */
static void send_mouse_pressed_state(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    uint8_t *mouse_press_status = NULL;

    swc_connection_get_payload_buffer(tx_to_dongle_conn, &mouse_press_status, &swc_err);
    if (mouse_press_status != NULL) {
        *mouse_press_status = 0x01;
        swc_connection_send(tx_to_dongle_conn, mouse_press_status, MOUSE_PAYLOAD_SIZE, &swc_err);
    }
    iface_mouse_button_latched_status();
}

/** @brief Send the state of the button on a button release event.
 */
static void send_mouse_released_state(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    uint8_t *mouse_press_status = NULL;

    swc_connection_get_payload_buffer(tx_to_dongle_conn, &mouse_press_status, &swc_err);
    if (mouse_press_status != NULL) {
        *mouse_press_status = 0x00;
        swc_connection_send(tx_to_dongle_conn, mouse_press_status, MOUSE_PAYLOAD_SIZE, &swc_err);
    }
    iface_mouse_button_unlatched_status();
}

/** @brief Enter in Pairing Mode using the Pairing Module.
 */
static void enter_pairing_mode(void)
{
    swc_error_t swc_err;
    pairing_error_t pairing_err = PAIRING_ERR_NONE;
    pairing_event_t pairing_event;

    iface_notify_enter_pairing();

    /* The wireless core must be stopped before starting the pairing procedure. */
    if (swc_get_status() == SWC_STATUS_RUNNING) {
        swc_disconnect(&swc_err);
        if ((swc_err != SWC_ERR_NONE) && (swc_err != SWC_ERR_NOT_CONNECTED)) {
            while (1);
        }
    }

    /* Give the information to the Pairing Application */
    app_pairing_cfg.app_code = PAIRING_APP_CODE;
    app_pairing_cfg.timeout_sec = PAIRING_TIMEOUT_IN_SECONDS;
    app_pairing_cfg.hal = &swc_hal;
    app_pairing_cfg.memory_pool = swc_memory_pool;
    app_pairing_cfg.memory_pool_size = SWC_MEM_POOL_SIZE;
    app_pairing_cfg.uwb_regulation = SWC_REGULATION_FCC;
    pairing_event = pairing_node_start(&app_pairing_cfg, &pairing_assigned_address, PAIRING_DEVICE_ROLE, &pairing_err);
    if (pairing_err != PAIRING_ERR_NONE) {
        while (1);
    }

    /* Handle the pairing events. */
    switch (pairing_event) {
    case PAIRING_EVENT_SUCCESS:
        /* Reconfigure the Coordinator and Node addresses */
        app_swc_configuration(&pairing_assigned_address, &swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }
        swc_connect(&swc_err);

        /* Indicate that the pairing process was successful */
        iface_notify_pairing_successful();

        device_pairing_state = DEVICE_PAIRED;

        break;
    case PAIRING_EVENT_TIMEOUT:
    case PAIRING_EVENT_INVALID_APP_CODE:
    case PAIRING_EVENT_ABORT:
    default:
        /* Indicate that the pairing process was unsuccessful */
        iface_notify_not_paired();

        device_pairing_state = DEVICE_UNPAIRED;

        break;
    }
}

/** @brief Put the device in the unpaired state and disconnect if from the network.
 */
static void unpair_device(void)
{
    swc_error_t swc_err;

    swc_disconnect(&swc_err);
    if ((swc_err != SWC_ERR_NONE) && (swc_err != SWC_ERR_NOT_CONNECTED)) {
        while (1);
    }

    device_pairing_state = DEVICE_UNPAIRED;

    /* Indicate that the device is unpaired */
    iface_notify_not_paired();
}
