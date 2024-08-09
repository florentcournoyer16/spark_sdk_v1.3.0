/** @file  pairing_gaming_hub_dongle.c
 *  @brief The dongle device is part of the gaming hub to demonstrate a point to multipoint pairing application.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES ******************************************************************/
#include "iface_audio.h"
#include "iface_pairing_gaming_hub.h"
#include "iface_wireless.h"
#include "pairing_api.h"
#include "sac_api.h"
#include "swc_api.h"
#include "swc_cfg_dongle.h"
#include "swc_utils.h"

/* MACROS *********************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

/* CONSTANTS ******************************************************************/
/* Memory reserved for the SPARK Audio Core. */
#define SAC_MEM_POOL_SIZE           5200

#define SAC_PAYLOAD_SIZE            84
#define SAC_LATENCY_QUEUE_SIZE      18

/* Memory reserved for the SPARK Wireless Core. */
#define SWC_MEM_POOL_SIZE           10000
/* Wireless payload size for the mouse device. */
#define MOUSE_PAYLOAD_SIZE          1
/* Wireless payload size for the keyboard device. */
#define KEYBOARD_PAYLOAD_SIZE       1

/* The device roles are used for the pairing discovery list. */
#define DEVICE_ROLE_DONGLE          0
#define DEVICE_ROLE_HEADSET         1
#define DEVICE_ROLE_MOUSE           2
#define DEVICE_ROLE_KEYBOARD        3
/* The discovery list includes the dongle(coordinator), headset, mouse and keyboard devices. */
#define PAIRING_DISCOVERY_LIST_SIZE 4
/* The application code prevents unwanted devices from pairing with this application. */
#define PAIRING_APP_CODE            0x87654321
/* The timeout in second after which the pairing procedure will abort. */
#define PAIRING_TIMEOUT_IN_SECONDS  10

/* PRIVATE GLOBALS ************************************************************/
/* ** Audio Core ** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
static sac_hal_t sac_hal;
static sac_pipeline_t *sac_pipeline;
static sac_endpoint_t *max98091_producer;
static ep_swc_instance_t swc_consumer_instance;
static sac_endpoint_t *swc_consumer;

/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *tx_beacon_conn;
static swc_connection_t *tx_to_headset_conn;
static swc_connection_t *rx_from_mouse_conn;
static swc_connection_t *rx_from_keyboard_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t tx_beacon_timeslots[] = TX_BEACON_TIMESLOTS;
static int32_t tx_to_headset_timeslots[] = TX_TO_HEADSET_TIMESLOTS;
static int32_t rx_from_mouse_timeslots[] = RX_FROM_MOUSE_TIMESLOTS;
static int32_t rx_from_keyboard_timeslots[] = RX_FROM_KEYBOARD_TIMESLOTS;

/* ** Application Specific ** */
static pairing_cfg_t app_pairing_cfg;
static pairing_assigned_address_t pairing_assigned_address;
static pairing_discovery_list_t discovery_list[PAIRING_DISCOVERY_LIST_SIZE];
static uint8_t rx_keyboard_status[KEYBOARD_PAYLOAD_SIZE];
static uint8_t rx_mouse_status[MOUSE_PAYLOAD_SIZE];

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_configuration(pairing_assigned_address_t *pairing_assigned_address, swc_error_t *err);
static void conn_tx_to_headset_success_callback(void *conn);
static void conn_tx_to_headset_fail_callback(void *conn);
static void conn_rx_from_mouse_success_callback(void *conn);
static void conn_rx_from_keyboard_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void audio_i2s_rx_complete_cb(void);

static void enter_pairing_mode(void);
static void unpair_device(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    sac_error_t sac_err;

    /* Initialize the board and the SPARK Wireless Core SPARK and Audio Core. */
    iface_board_init();
    iface_audio_coord_init();
    iface_swc_hal_init(&swc_hal);
    iface_swc_handlers_init();

    while (1) {
        iface_button_handling_osr(unpair_device, enter_pairing_mode);

        if (sac_pipeline != NULL) {
            sac_pipeline_process(sac_pipeline, &sac_err);
            sac_pipeline_consume(sac_pipeline, &sac_err);
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
    uint8_t dongle_address;
    uint8_t headset_address;
    uint8_t mouse_address;
    uint8_t keyboard_address;

    /* Assign the address from the discovered list to the devices. */
    dongle_address = discovery_list[DEVICE_ROLE_DONGLE].node_address;
    headset_address = discovery_list[DEVICE_ROLE_HEADSET].node_address;
    mouse_address = discovery_list[DEVICE_ROLE_MOUSE].node_address;
    keyboard_address = discovery_list[DEVICE_ROLE_KEYBOARD].node_address;

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
        .local_address = dongle_address,
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

    /* ** Beacon TX Connection ** */
    swc_connection_cfg_t tx_beacon_conn_cfg;

    tx_beacon_conn_cfg = swc_get_beacon_connection_config(node, dongle_address, tx_beacon_timeslots, SWC_ARRAY_SIZE(tx_beacon_timeslots));
    tx_beacon_conn = swc_connection_init(node, tx_beacon_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_beacon_channel_cfg = {
        .tx_pulse_count = TX_BEACON_PULSE_COUNT,
        .tx_pulse_width = TX_BEACON_PULSE_WIDTH,
        .tx_pulse_gain  = TX_BEACON_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        tx_beacon_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_beacon_conn, node, tx_beacon_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    if (headset_address != 0x00) {
        /* ** Headset TX Connection ** */
        swc_connection_cfg_t tx_to_headset_conn_cfg = {
            .name = "Headset TX Connection",
            .source_address = dongle_address,
            .destination_address = headset_address,
            .max_payload_size = SAC_PAYLOAD_SIZE + sizeof(sac_header_t),
            .queue_size = TX_DATA_QUEUE_SIZE,
            .modulation = SWC_MODULATION,
            .fec = SWC_FEC_LEVEL,
            .timeslot_id = tx_to_headset_timeslots,
            .timeslot_count = SWC_ARRAY_SIZE(tx_to_headset_timeslots),
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
        tx_to_headset_conn = swc_connection_init(node, tx_to_headset_conn_cfg, &swc_hal, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }

        swc_channel_cfg_t tx_to_headset_channel_cfg = {
            .tx_pulse_count = TX_DATA_PULSE_COUNT,
            .tx_pulse_width = TX_DATA_PULSE_WIDTH,
            .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
            .rx_pulse_count = RX_ACK_PULSE_COUNT
        };
        for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
            tx_to_headset_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(tx_to_headset_conn, node, tx_to_headset_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }
        }
        swc_connection_set_tx_success_callback(tx_to_headset_conn, conn_tx_to_headset_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
        swc_connection_set_tx_fail_callback(tx_to_headset_conn, conn_tx_to_headset_fail_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    if (mouse_address != 0x00) {
        /* ** Mouse RX Connection ** */
        swc_connection_cfg_t rx_from_mouse_conn_cfg = {
            .name = "Mouse RX Connection",
            .source_address = mouse_address,
            .destination_address = dongle_address,
            .max_payload_size = MOUSE_PAYLOAD_SIZE,
            .queue_size = RX_DATA_QUEUE_SIZE,
            .modulation = SWC_MODULATION,
            .fec = SWC_FEC_LEVEL,
            .timeslot_id = rx_from_mouse_timeslots,
            .timeslot_count = SWC_ARRAY_SIZE(rx_from_mouse_timeslots),
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
        rx_from_mouse_conn = swc_connection_init(node, rx_from_mouse_conn_cfg, &swc_hal, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }

        swc_channel_cfg_t rx_from_mouse_channel_cfg = {
            .tx_pulse_count = TX_ACK_PULSE_COUNT,
            .tx_pulse_width = TX_ACK_PULSE_WIDTH,
            .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
            .rx_pulse_count = RX_DATA_PULSE_COUNT
        };
        for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
            rx_from_mouse_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(rx_from_mouse_conn, node, rx_from_mouse_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }
        }

        swc_connection_set_rx_success_callback(rx_from_mouse_conn, conn_rx_from_mouse_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    if (keyboard_address != 0x00) {
        /* ** Keyboard RX Connection ** */
        swc_connection_cfg_t rx_from_keyboard_conn_cfg = {
            .name = "Keyboard RX Connection",
            .source_address = keyboard_address,
            .destination_address = dongle_address,
            .max_payload_size = KEYBOARD_PAYLOAD_SIZE,
            .queue_size = RX_DATA_QUEUE_SIZE,
            .modulation = SWC_MODULATION,
            .fec = SWC_FEC_LEVEL,
            .timeslot_id = rx_from_keyboard_timeslots,
            .timeslot_count = SWC_ARRAY_SIZE(rx_from_keyboard_timeslots),
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
        rx_from_keyboard_conn = swc_connection_init(node, rx_from_keyboard_conn_cfg, &swc_hal, err);

        swc_channel_cfg_t rx_from_keyboard_channel_cfg = {
            .tx_pulse_count = TX_ACK_PULSE_COUNT,
            .tx_pulse_width = TX_ACK_PULSE_WIDTH,
            .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
            .rx_pulse_count = RX_DATA_PULSE_COUNT
        };
        for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
            rx_from_keyboard_channel_cfg.frequency = channel_frequency[i];
            swc_connection_add_channel(rx_from_keyboard_conn, node, rx_from_keyboard_channel_cfg, err);
            if (*err != SWC_ERR_NONE) {
                return;
            }
        }
        swc_connection_set_rx_success_callback(rx_from_keyboard_conn, conn_rx_from_keyboard_success_callback, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    swc_setup(node, err);
}

/** @brief Callback function when a frame has been successfully send and an ACK is received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_to_headset_success_callback(void *conn)
{
    (void)conn;

    iface_tx_conn_status();
}

/** @brief Callback function when a frame has not been successfully transmitted on the headset connection
 *
 * @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_to_headset_fail_callback(void *conn)
{
    (void)conn;

    iface_reset_tx_conn_status();
}

/** @brief Callback function when a frame has been successfully received from the Mouse.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_from_mouse_success_callback(void *conn)
{
    (void)conn;
    swc_error_t err;
    uint8_t *payload = NULL;
    uint8_t rx_size = 0;

    /* Get new payload */
    rx_size = swc_connection_receive(rx_from_mouse_conn, &payload, &err);

    if (rx_size != 0) {
        memcpy(&rx_mouse_status, payload, sizeof(rx_mouse_status));

        if (rx_mouse_status[0] == 0x01) {
            iface_mouse_button_latched_status();
        } else {
            iface_mouse_button_unlatched_status();
        }
    }

    /* Notify the SWC that the new payload has been read */
    swc_connection_receive_complete(rx_from_mouse_conn, &err);
}

/** @brief Callback function when a frame has been successfully received from the Keyboard.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_from_keyboard_success_callback(void *conn)
{
    (void)conn;
    swc_error_t err;
    uint8_t *payload = NULL;
    uint8_t rx_size = 0;

    /* Get new payload */
    rx_size = swc_connection_receive(rx_from_keyboard_conn, &payload, &err);

    if (rx_size != 0) {
        memcpy(&rx_keyboard_status, payload, sizeof(rx_keyboard_status));

        if (rx_keyboard_status[0] == 0x01) {
            iface_keyboard_button_latched_status();
        } else {
            iface_keyboard_button_unlatched_status();
        }
    }
    /* Notify the SWC that the new payload has been read */
    swc_connection_receive_complete(rx_from_keyboard_conn, &err);
}

/** @brief Initialize the Audio Core.
 *
 *  @param[out] err  Audio Core error code.
 */
static void app_audio_core_init(sac_error_t *sac_err)
{
    sac_endpoint_interface_t max98091_producer_iface;
    sac_endpoint_interface_t swc_consumer_iface;

    if (tx_to_headset_conn != NULL) {

        iface_sac_hal_init(&sac_hal);
        iface_audio_swc_endpoint_init(NULL, &swc_consumer_iface);
        iface_audio_max98091_endpoint_init(&max98091_producer_iface, NULL);
        iface_set_sai_complete_callback(NULL, audio_i2s_rx_complete_cb);

        swc_consumer_instance.connection = tx_to_headset_conn;

        sac_cfg_t core_cfg = {
            .memory_pool = audio_memory_pool,
            .memory_pool_size = SAC_MEM_POOL_SIZE
        };
        sac_init(core_cfg, &sac_hal, sac_err);

        /*
         * Audio Pipeline
         * ==============
         *
         * Input:      Stereo stream of 42 samples @ 48 kHz/16 bits is recorded using the MAX98091 hardware audio codec.
         * Processing: None.
         * Output:     Stereo stream of 42 samples @ 48 kHz/16 bits is sent over the air to the Node.
         *
         * +-------+    +-----+
         * | Codec | -> | SWC |
         * +-------+    +-----+
         */
        sac_endpoint_cfg_t max98091_producer_cfg = {
            .use_encapsulation  = false,
            .delayed_action     = true,
            .channel_count      = 2,
            .bit_depth          = SAC_16BITS,
            .audio_payload_size = SAC_PAYLOAD_SIZE,
            .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE
        };
        max98091_producer = sac_endpoint_init(NULL, "MAX98091 EP (Producer)",
                                              max98091_producer_iface,
                                              max98091_producer_cfg,
                                              sac_err);
        if (*sac_err != SAC_ERR_NONE) {
            return;
        }

        sac_endpoint_cfg_t swc_consumer_cfg = {
            .use_encapsulation  = true,
            .delayed_action     = false,
            .channel_count      = 2,
            .bit_depth          = SAC_16BITS,
            .audio_payload_size = SAC_PAYLOAD_SIZE,
            .queue_size         = SAC_LATENCY_QUEUE_SIZE
        };
        swc_consumer = sac_endpoint_init((void *)&swc_consumer_instance,
                                         "SWC EP (Consumer)",
                                         swc_consumer_iface,
                                         swc_consumer_cfg,
                                         sac_err);
        if (*sac_err != SAC_ERR_NONE) {
            return;
        }

        sac_pipeline_cfg_t sac_pipeline_cfg = {
            .cdc_enable = false,
            .do_initial_buffering = true,
        };
        sac_pipeline = sac_pipeline_init("Codec -> SWC",
                                         max98091_producer,
                                         sac_pipeline_cfg,
                                         swc_consumer,
                                         sac_err);
        if (*sac_err != SAC_ERR_NONE) {
            return;
        }

        sac_pipeline_setup(sac_pipeline, sac_err);
        if (*sac_err != SAC_ERR_NONE) {
            return;
        }
    }
}

/** @brief SAI DMA RX complete callback.
 *
 *  This receives audio packets from the codec. It needs to be
 *  executed every time a DMA transfer from the codec is completed
 *  in order to keep recording audio.
 */
static void audio_i2s_rx_complete_cb(void)
{
    sac_error_t sac_err;

    sac_pipeline_produce(sac_pipeline, &sac_err);
}

/** @brief Enter in Pairing Mode using the Pairing Module.
 */
static void enter_pairing_mode(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    sac_error_t sac_err = SAC_ERR_NONE;
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
    pairing_event = pairing_coordinator_start(&app_pairing_cfg, &pairing_assigned_address,
                                              discovery_list, PAIRING_DISCOVERY_LIST_SIZE, &pairing_err);
    if (pairing_err != PAIRING_ERR_NONE) {
        while (1);
    }

    /* Handle the pairing events. */
    switch (pairing_event) {
    case PAIRING_EVENT_SUCCESS:
        /* Indicate that the pairing process was successful */
        iface_notify_pairing_successful();

        /* Reconfigure the Coordinator and Node addresses */
        app_swc_configuration(&pairing_assigned_address, &swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }

        if (discovery_list[DEVICE_ROLE_HEADSET].node_address != 0x00) {
            app_audio_core_init(&sac_err);
            if (sac_err != SAC_ERR_NONE) {
                while (1);
            }

            sac_pipeline_start(sac_pipeline, &sac_err);
            if (sac_err != SAC_ERR_NONE) {
                while (1);
            }
        }

        swc_connect(&swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }
        break;
    case PAIRING_EVENT_TIMEOUT:
    case PAIRING_EVENT_INVALID_APP_CODE:
    case PAIRING_EVENT_ABORT:
    default:
        /* Indicate that the pairing process was unsuccessful */
        iface_notify_not_paired();
        break;
    }
}

/** @brief Unpair the device, this will put its connection addresses back to default value.
 */
static void unpair_device(void)
{
    swc_error_t swc_err;
    sac_error_t sac_err;

    memset(discovery_list, 0, SWC_ARRAY_SIZE(discovery_list));

    if (sac_pipeline != NULL) {
        sac_pipeline_stop(sac_pipeline, &sac_err);
        if (sac_err != SAC_ERR_NONE) {
            while (1);
        }
        memset(sac_pipeline, 0, sizeof(sac_pipeline));
    }

    /* Disconnect the Wireless Core. */
    swc_disconnect(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    /* Indicate that the device is unpaired */
    iface_notify_not_paired();
}
