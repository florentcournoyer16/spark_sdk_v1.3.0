/** @file  pairing_gaming_hub_headset.c
 *  @brief The headset device is part of the gaming hub to demonstrate a point to multipoint pairing application.
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
#include "sac_cdc.h"
#include "sac_mute_on_underflow.h"
#include "sac_utils.h"
#include "swc_api.h"
#include "swc_cfg_headset.h"
#include "swc_utils.h"

/* CONSTANTS ******************************************************************/
/* Memory reserved for the SPARK Audio Core. */
#define SAC_MEM_POOL_SIZE          5200

#define SAC_PAYLOAD_SIZE           84
#define SAC_NB_CHANNEL             2
#define SAC_LATENCY_QUEUE_SIZE     18
#define SAC_SAMPLING_RATE          48000
#define SAC_BIT_DEPTH              SAC_16BITS

/* Memory reserved for the SPARK Wireless Core. */
#define SWC_MEM_POOL_SIZE          5200

/* The device role is used for the coordinator's pairing discovery list. */
#define PAIRING_DEVICE_ROLE        1
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
/* ** Audio Core ** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
static sac_hal_t sac_hal;
static sac_pipeline_t *sac_pipeline;
static sac_endpoint_t *max98091_consumer;
static ep_swc_instance_t swc_producer_instance;
static sac_endpoint_t *swc_producer;
static sac_cdc_instance_t cdc_instance;
static sac_processing_t *cdc_processing;
static sac_mute_on_underflow_instance_t mute_on_underflow_instance;
static sac_processing_t *mute_on_underflow_processing;

/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *rx_from_dongle_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_from_dongle_timeslots[] = RX_FROM_DONGLE_TIMESLOTS;

/* ** Application Specific ** */
static device_pairing_state_t device_pairing_state;
static pairing_cfg_t app_pairing_cfg;
static pairing_assigned_address_t pairing_assigned_address;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_configuration(pairing_assigned_address_t *pairing_assigned_address, swc_error_t *err);
static void conn_rx_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void app_audio_core_cdc_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_mute_on_underflow_interface_init(sac_processing_interface_t *iface);
static void audio_i2s_tx_complete_cb(void);

static void enter_pairing_mode(void);
static void unpair_device(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    sac_error_t sac_err;

    /* Initialize the board and the SPARK Wireless Core SPARK and Audio Core. */
    iface_board_init();
    iface_audio_node_init();
    iface_swc_hal_init(&swc_hal);
    iface_swc_handlers_init();

    while (1) {
        if (device_pairing_state == DEVICE_UNPAIRED) {
            iface_button_handling_osr(NULL, enter_pairing_mode);
        } else if (device_pairing_state == DEVICE_PAIRED) {
            iface_button_handling_osr(NULL, unpair_device);
            sac_pipeline_process(sac_pipeline, &sac_err);
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

    /* ** RX from dongle Connection ** */
    swc_connection_cfg_t rx_from_dongle_conn_cfg = {
        .name = "RX from dongle Connection",
        .source_address = remote_address,
        .destination_address = local_address,
        .max_payload_size = SAC_PAYLOAD_SIZE + sizeof(sac_header_t),
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_from_dongle_timeslots,
        .timeslot_count = SWC_ARRAY_SIZE(rx_from_dongle_timeslots),
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
    rx_from_dongle_conn = swc_connection_init(node, rx_from_dongle_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_dongle_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < SWC_ARRAY_SIZE(channel_sequence); i++) {
        rx_from_dongle_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_from_dongle_conn, node, rx_from_dongle_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_from_dongle_conn, conn_rx_success_callback, err);

    swc_setup(node, err);
}

/** @brief Callback function when a frame has been successfully received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_success_callback(void *conn)
{
    (void)conn;
    sac_error_t sac_err;

    iface_rx_conn_status();

    sac_pipeline_produce(sac_pipeline, &sac_err);
}

/** @brief Initialize the Audio Core.
 *
 *  @param[out] err  Audio Core error code.
 */
static void app_audio_core_init(sac_error_t *sac_err)
{
    sac_endpoint_interface_t max98091_consumer_iface;
    sac_endpoint_interface_t swc_producer_iface;
    sac_processing_interface_t cdc_iface;
    sac_processing_interface_t mute_on_underflow_iface;

    iface_sac_hal_init(&sac_hal);
    iface_audio_swc_endpoint_init(&swc_producer_iface, NULL);
    iface_audio_max98091_endpoint_init(NULL, &max98091_consumer_iface);
    iface_set_sai_complete_callback(audio_i2s_tx_complete_cb, NULL);

    app_audio_core_cdc_interface_init(&cdc_iface);
    app_audio_core_mute_on_underflow_interface_init(&mute_on_underflow_iface);

    swc_producer_instance.connection = rx_from_dongle_conn;

    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE
    };
    sac_init(core_cfg, &sac_hal, sac_err);

    /*
     * Audio Pipeline
     * ==============
     *
     * Input:      Stereo stream of 42 samples @ 48 kHz/16 bits is received over the air from the Coordinator.
     * Processing: Clock drift compensation followed by mute on glitch.
     * Output:     Stereo stream of 42 samples @ 48 kHz/16 bits is played using the MAX98091 hardware audio codec.
     *
     * +-----+    +-----+    +----------------+    +-------+
     * | SWC | -> | CDC | -> | Mute on Glitch | -> | Codec |
     * +-----+    +-----+    +----------------+    +-------+
     */
    sac_endpoint_cfg_t swc_producer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = SAC_NB_CHANNEL,
        .bit_depth          = SAC_BIT_DEPTH,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE
    };
    swc_producer = sac_endpoint_init((void *)&swc_producer_instance,
                                     "SWC EP (Producer)",
                                     swc_producer_iface,
                                     swc_producer_cfg,
                                     sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    cdc_instance.cdc_resampling_length = 1440;
    cdc_instance.cdc_queue_avg_size = 1000;
    cdc_processing = sac_processing_stage_init((void *)&cdc_instance, "CDC", cdc_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    mute_on_underflow_instance.reload_value = sac_get_nb_packets_in_x_ms(30,
                                                                         SAC_PAYLOAD_SIZE,
                                                                         SAC_NB_CHANNEL,
                                                                         SAC_BIT_DEPTH,
                                                                         SAC_SAMPLING_RATE);

    mute_on_underflow_processing = sac_processing_stage_init((void *)&mute_on_underflow_instance,
                                                             "Mute on underflow",
                                                             mute_on_underflow_iface,
                                                             sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t max98091_consumer_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = true,
        .channel_count      = SAC_NB_CHANNEL,
        .bit_depth          = SAC_BIT_DEPTH,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };
    max98091_consumer = sac_endpoint_init(NULL,
                                          "MAX98091 EP (Consumer)",
                                          max98091_consumer_iface,
                                          max98091_consumer_cfg,
                                          sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t pipeline_cfg = {
        .cdc_enable = true,
        .do_initial_buffering = false,
    };
    sac_pipeline = sac_pipeline_init("Codec -> SWC",
                                     swc_producer,
                                     pipeline_cfg,
                                     max98091_consumer,
                                     sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline, cdc_processing, sac_err);
    sac_pipeline_add_processing(sac_pipeline, mute_on_underflow_processing, sac_err);

    sac_pipeline_setup(sac_pipeline, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }
}

/** @brief Initialize the clock drift compensation audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_cdc_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_cdc_init;
    iface->ctrl = sac_cdc_ctrl;
    iface->process = sac_cdc_process;
    iface->gate = NULL;
}

/** @brief Initialize the mute on underflow audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_mute_on_underflow_interface_init(sac_processing_interface_t *iface)
{
    iface->init    = sac_mute_on_underflow_init;
    iface->ctrl    = NULL;
    iface->process = sac_mute_on_underflow_process;
    iface->gate    = NULL;
}

/** @brief SAI DMA TX complete callback.
 *
 *  This feeds the codec with audio packets. It needs to be
 *  executed every time a DMA transfer to the codec is completed
 *  in order to keep the audio playing.
 */
static void audio_i2s_tx_complete_cb(void)
{
    sac_error_t sac_err;

    sac_pipeline_consume(sac_pipeline, &sac_err);
}

/** @brief Enter in Pairing Mode using the Pairing Module.
 */
static void enter_pairing_mode(void)
{
    swc_error_t swc_err;
    sac_error_t sac_err;
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

    /* Give the information to the Pairing Application. */
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
        /* Indicate that the pairing process was successful */
        iface_notify_pairing_successful();

        /* Reconfigure the Coordinator and Node addresses */
        app_swc_configuration(&pairing_assigned_address, &swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }

        app_audio_core_init(&sac_err);
        if (sac_err != SAC_ERR_NONE) {
            while (1);
        }

        sac_pipeline_start(sac_pipeline, &sac_err);
        if (sac_err != SAC_ERR_NONE) {
            while (1);
        }

        swc_connect(&swc_err);
        if (swc_err != SWC_ERR_NONE) {
            while (1);
        }

        device_pairing_state = DEVICE_PAIRED;

        break;
    case PAIRING_EVENT_TIMEOUT:
    case PAIRING_EVENT_INVALID_APP_CODE:
    case PAIRING_EVENT_ABORT:
    default:

        device_pairing_state = DEVICE_UNPAIRED;

        /* Indicate that the pairing process was unsuccessful */
        iface_notify_not_paired();
    }
}

/** @brief Unpair the device, this will put its connection addresses back to default value.
 */
static void unpair_device(void)
{
    swc_error_t swc_err;
    sac_error_t sac_err;

    device_pairing_state = DEVICE_UNPAIRED;

    sac_pipeline_stop(sac_pipeline, &sac_err);
    if (sac_err != SAC_ERR_NONE) {
        while (1);
    }

    /* Disconnect the Wireless Core. */
    swc_disconnect(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    /* Indicate that the device is unpaired */
    iface_notify_not_paired();
}
