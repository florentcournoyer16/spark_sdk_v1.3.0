/** @file  audio_mixer_coord.c
 *  @brief Application example to stream two audio sources and mix them into one audio output.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES ******************************************************************/
#include <stdio.h>
#include "sac_mixer_endpoint.h"
#include "sac_mixer_module.h"
#include "iface_audio.h"
#include "iface_audio_mixer.h"
#include "iface_wireless.h"
#include "sac_api.h"
#include "sac_cdc.h"
#include "sac_stats.h"
#include "sac_volume.h"
#include "swc_api.h"
#include "swc_cfg_coord.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SAC_MEM_POOL_SIZE      10000
#define SAC_PAYLOAD_SIZE       74
#define SAC_LATENCY_QUEUE_SIZE 7
#define SWC_MEM_POOL_SIZE      9000
#define STATS_ARRAY_LENGTH     2500

/* PRIVATE GLOBALS ************************************************************/
/* ** Audio Core ** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
static sac_hal_t sac_hal;

/* First audio input pipeline */
static sac_pipeline_t *sac_pipeline_input1;
static ep_swc_instance_t ep_swc_input1_instance;
static sac_endpoint_t *ep_swc_producer_input1;
static sac_endpoint_t *ep_mixer_consumer_input1;

static sac_volume_instance_t volume_instance;
static sac_processing_t *volume_processing;

static sac_cdc_instance_t cdc_instance1;
static sac_processing_t *cdc_processing1;

/* Second audio input pipeline */
static sac_pipeline_t *sac_pipeline_input2;
static ep_swc_instance_t ep_swc_input2_instance;
static sac_endpoint_t *ep_swc_producer_input2;
static sac_endpoint_t *ep_mixer_consumer_input2;

static sac_cdc_instance_t cdc_instance2;
static sac_processing_t *cdc_processing2;

/* Mixer pipeline that takes the first and second input pipelines and mixes them */
static sac_pipeline_t *sac_pipeline_mixer;
static sac_endpoint_t *ep_mixer_producer1;
static sac_endpoint_t *ep_mixer_producer2;
static sac_endpoint_t *ep_max98091_consumer;

/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *tx_beacon_conn;
static swc_connection_t *rx_from_node1_conn;
static swc_connection_t *rx_from_node2_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t tx_beacon_timeslots[] = TX_BEACON_TIMESLOTS;
static int32_t rx_from_node1_timeslots[] = RX_FROM_NODE1_TIMESLOTS;
static int32_t rx_from_node2_timeslots[] = RX_FROM_NODE2_TIMESLOTS;

/* ** Application Specific ** */
static bool mix_audio;
static volatile bool print_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_node1_rx_success_callback(void *conn);
static void conn_node2_rx_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void app_audio_core_volume_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_cdc_interface_init(sac_processing_interface_t *iface);
static void audio_i2s_tx_complete_cb(void);

static void volume_up(void);
static void volume_down(void);

static void audio_process_callback(void);
static void stats_callback(void);
static void print_stats(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    swc_error_t swc_err;
    sac_error_t sac_err;

    iface_board_init();

    app_swc_core_init(&swc_err);
    if (swc_err != SWC_ERR_NONE) {
        while (1);
    }

    app_audio_core_init(&sac_err);
    if (sac_err != SAC_ERR_NONE) {
        while (1);
    }

    iface_audio_coord_init();

    sac_pipeline_start(sac_pipeline_input1, &sac_err);
    sac_pipeline_start(sac_pipeline_input2, &sac_err);
    sac_pipeline_start(sac_pipeline_mixer, &sac_err);

    swc_connect(&swc_err);

    iface_audio_process_timer_init(100);
    iface_audio_process_set_timer_callback(audio_process_callback);
    iface_audio_process_timer_start();

    iface_stats_timer_init(1000);
    iface_stats_set_timer_callback(stats_callback);
    iface_stats_timer_start();

    mix_audio = true;

    while (1) {
        iface_button_handling(volume_up, volume_down);

        if (print_stats_now) {
            print_stats();
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
    iface_swc_hal_init(&swc_hal);
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
    swc_init(core_cfg, &swc_hal, err);
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
    swc_node_add_radio(node, radio_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_connection_cfg_t tx_beacon_conn_cfg;

    tx_beacon_conn_cfg = swc_get_beacon_connection_config(node, COORDINATOR_ADDRESS, tx_beacon_timeslots, ARRAY_SIZE(tx_beacon_timeslots));
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
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_beacon_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_beacon_conn, node, tx_beacon_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** RX from Node1 Connection ** */
    swc_connection_cfg_t rx_from_node1_conn_cfg = {
        .name = "RX from Node1 Connection",
        .source_address = REMOTE_ADDRESS_1,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = SAC_PAYLOAD_SIZE + sizeof(sac_header_t),
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_from_node1_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_from_node1_timeslots),
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
    rx_from_node1_conn = swc_connection_init(node, rx_from_node1_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_node1_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_from_node1_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_from_node1_conn, node, rx_from_node1_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_from_node1_conn, conn_node1_rx_success_callback, err);

    /* ** RX from Node 2 Connection ** */
    swc_connection_cfg_t rx_from_node2_conn_cfg = {
        .name = "RX from Node 2 Connection",
        .source_address = REMOTE_ADDRESS_2,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = SAC_PAYLOAD_SIZE + sizeof(sac_header_t),
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_from_node2_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_from_node2_timeslots),
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
    rx_from_node2_conn = swc_connection_init(node, rx_from_node2_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_node2_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
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

/** @brief Callback function when a frame has been successfully received from the Node 1.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_node1_rx_success_callback(void *conn)
{
    (void)conn;
    sac_error_t sac_err;

    iface_node1_rx_conn_status();
    sac_pipeline_produce(sac_pipeline_input1, &sac_err);
}

/** @brief Callback function when a frame has been successfully received from the Node 2.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_node2_rx_success_callback(void *conn)
{
    (void)conn;
    sac_error_t sac_err;

    iface_node2_rx_conn_status();
    sac_pipeline_produce(sac_pipeline_input2, &sac_err);
}

/** @brief Initialize the Audio Core.
 *
 *  @param[out] err  Audio Core error code.
 */
static void app_audio_core_init(sac_error_t *sac_err)
{
    sac_endpoint_interface_t ep_iface_swc_producer;
    sac_endpoint_interface_t ep_iface_mixer_consumer;
    sac_endpoint_interface_t ep_iface_mixer_producer;
    sac_endpoint_interface_t ep_iface_max98091_consumer;
    sac_processing_interface_t volume_iface;
    sac_processing_interface_t cdc_iface;
    sac_mixer_module_cfg_t sac_mixer_module_cfg;

    ep_iface_mixer_producer.action = ep_mixer_produce;
    ep_iface_mixer_producer.start = ep_mixer_start;
    ep_iface_mixer_producer.stop = ep_mixer_stop;
    ep_iface_mixer_consumer.action = ep_mixer_consume;
    ep_iface_mixer_consumer.start = ep_mixer_start;
    ep_iface_mixer_consumer.stop = ep_mixer_stop;

    iface_sac_hal_init(&sac_hal);
    iface_audio_swc_endpoint_init(&ep_iface_swc_producer, NULL);
    iface_audio_max98091_endpoint_init(NULL, &ep_iface_max98091_consumer);
    iface_set_sai_complete_callback(audio_i2s_tx_complete_cb, NULL);

    app_audio_core_volume_interface_init(&volume_iface);
    app_audio_core_cdc_interface_init(&cdc_iface);

    ep_swc_input1_instance.connection = rx_from_node1_conn;
    ep_swc_input2_instance.connection = rx_from_node2_conn;

    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE
    };
    sac_init(core_cfg, &sac_hal, sac_err);

    /*
     * Audio Mixer Pipelines
     * =====================
     *  Input 1 Pipeline
     * +-----+    +-----+    +-------+
     * | SWC | -> | CDC | -> | Mixer |--\   Output Mixer Pipeline
     * +-----+    +-----+    +-------+   \ +------ +    +-------+
     *  Input 2 Pipeline                  -| Mixer | -> | Codec |
     * +-----+    +-----+    +-------+   / +------ +    +-------+
     * | SWC | -> | CDC | -> | Mixer |--/
     * +-----+    +-----+    +-------+
     */

    sac_mixer_module_cfg.bit_depth = SAC_16BITS;
    sac_mixer_module_cfg.nb_of_inputs = 2;
    sac_mixer_module_cfg.payload_size = SAC_PAYLOAD_SIZE;

    sac_mixer_init(sac_mixer_module_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        while (1);
    }

    /*
     * Input 1 Pipeline
     * ================
     *
     * Input:      Mono stream of 37 samples @ 32 kHz/16 bits is received over the air from the Coordinator.
     * Processing: CDC.
     * Output:     Mono stream of 37 samples @ 32 kHz/16 bits is stored in the mixer queue.
     *
     * +-----+    +-----+    +-------+
     * | SWC | -> | CDC | -> | Mixer |
     * +-----+    +-----+    +-------+
     */
    sac_endpoint_cfg_t swc_input1_producer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE
    };
    ep_swc_producer_input1 = sac_endpoint_init((void *)&ep_swc_input1_instance, "SWC EP (Producer)",
                                               ep_iface_swc_producer, swc_input1_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    cdc_instance1.cdc_resampling_length = 1440;
    cdc_instance1.cdc_queue_avg_size = 1000;
    cdc_processing1 = sac_processing_stage_init((void *)&cdc_instance1, "CDC Input 1", cdc_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t ep_mixer_consumer_input1_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };
    ep_mixer_consumer_input1 = sac_endpoint_init(NULL, "Mixer 1 EP (Consumer)",
                                                 ep_iface_mixer_consumer, ep_mixer_consumer_input1_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t input1_pipeline_cfg = {
        .cdc_enable = true,
        .do_initial_buffering = false,
        .mixer_option.input_mixer_pipeline = true
    };

    sac_pipeline_input1 = sac_pipeline_init("SWC 1 -> Mixer 1", ep_swc_producer_input1,
                                              input1_pipeline_cfg, ep_mixer_consumer_input1, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline_input1, cdc_processing1, sac_err);

    sac_pipeline_setup(sac_pipeline_input1, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    /*
     * Input 2 Pipeline
     * ================
     *
     * Input:      Mono stream of 37 samples @ 32 kHz/16 bits is received over the air from the Coordinator.
     * Processing: CDC.
     * Output:     Mono stream of 37 samples @ 32 kHz/16 bits is stored in the mixer queue.
     *
     * +-----+    +-----+    +-------+
     * | SWC | -> | CDC | -> | Mixer |
     * +-----+    +-----+    +-------+
     */
    sac_endpoint_cfg_t swc_input2_producer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE
    };
    ep_swc_producer_input2 = sac_endpoint_init((void *)&ep_swc_input2_instance, "SWC EP (Producer)",
                                               ep_iface_swc_producer, swc_input2_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    cdc_instance2.cdc_resampling_length = 1440;
    cdc_instance2.cdc_queue_avg_size = 1000;
    cdc_processing2 = sac_processing_stage_init((void *)&cdc_instance2, "CDC Input 2", cdc_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t ep_mixer_consumer_input2_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };
    ep_mixer_consumer_input2 = sac_endpoint_init(NULL, "Mixer 2 EP (Consumer)",
                                                 ep_iface_mixer_consumer, ep_mixer_consumer_input2_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t input2_pipeline_cfg = {
        .cdc_enable = true,
        .do_initial_buffering = false,
        .mixer_option.input_mixer_pipeline = true
    };

    sac_pipeline_input2 = sac_pipeline_init("SWC 2 -> Mixer 2", ep_swc_producer_input2,
                                              input2_pipeline_cfg, ep_mixer_consumer_input2, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline_input2, cdc_processing2, sac_err);

    sac_pipeline_setup(sac_pipeline_input2, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    /*
     * Output Mixer Audio Pipeline
     * ===========================
     *
     * Input:      Mixer.
     * Processing: None.
     * Output:     Codec.
     *
     * +-------+    +-------+
     * | Mixer | -> | Codec |
     * +-------+    +-------+
     */
    sac_endpoint_cfg_t ep_mixer_producer_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };

    ep_mixer_producer1 = sac_endpoint_init(NULL, "Mixer 1 EP (Producer)",
                                           ep_iface_mixer_producer, ep_mixer_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    ep_mixer_producer2 = sac_endpoint_init(NULL, "Mixer 2 EP (Producer)",
                                           ep_iface_mixer_producer, ep_mixer_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    volume_instance.initial_volume_level = 100;
    volume_instance.bit_depth = SAC_16BITS;

    volume_processing = sac_processing_stage_init((void *)&volume_instance, "Digital Volume Control", volume_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t ep_max98091_consumer_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = true,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = 3
    };
    ep_max98091_consumer = sac_endpoint_init(NULL, "MAX98091 EP (Consumer)",
                                             ep_iface_max98091_consumer, ep_max98091_consumer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t mixer_pipeline_cfg = {
        .cdc_enable = false,
        .do_initial_buffering = false,
        .mixer_option.output_mixer_pipeline = true
    };
    sac_pipeline_mixer = sac_pipeline_init("Mixer -> Codec", ep_mixer_producer1,
                                             mixer_pipeline_cfg, ep_max98091_consumer, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    /* Add the second Mixer Producer */
    sac_pipeline_add_extra_producer(sac_pipeline_mixer, ep_mixer_producer2, sac_err);

    /* Link the Mixer Producer 1 Endpoint queue with the Input1 Consumer Endpoint */
    sac_pipeline_add_input_pipeline(sac_pipeline_mixer, sac_pipeline_input1, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    /* Link the Mixer Producer 2 Endpoint queue with the Input2 Consumer Endpoint */
    sac_pipeline_add_input_pipeline(sac_pipeline_mixer, sac_pipeline_input2, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline_mixer, volume_processing, sac_err);

    sac_pipeline_setup(sac_pipeline_mixer, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }
}

/** @brief Initialize the digital volume control audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_volume_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_volume_init;
    iface->ctrl = sac_volume_ctrl;
    iface->process = sac_volume_process;
    iface->gate = NULL;
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

/** @brief SAI DMA TX complete callback.
 *
 *  This feeds the codec with audio packets. It needs to be
 *  executed every time a DMA transfer to the codec is completed
 *  in order to keep the audio playing.
 */
static void audio_i2s_tx_complete_cb(void)
{
    sac_error_t sac_err;

    mix_audio = true;

    sac_pipeline_consume(sac_pipeline_mixer, &sac_err);
}

/** @brief Increase the audio output volume level.
 *
 *  This affects the audio pipeline the digital volume
 *  processing stage is added to.
 */
static void volume_up(void)
{
    sac_error_t err;

    sac_processing_ctrl(volume_processing, sac_pipeline_mixer, SAC_VOLUME_INCREASE, SAC_NO_ARG, &err);
}

/** @brief Decrease the audio output volume level.
 *
 *  This affects the audio pipeline the digital volume
 *  processing stage is added to.
 */
static void volume_down(void)
{
    sac_error_t err;

    sac_processing_ctrl(volume_processing, sac_pipeline_mixer, SAC_VOLUME_DECREASE, SAC_NO_ARG, &err);
}

/** @brief Callback handling the audio process that triggers with the app timer.
 */
static void audio_process_callback(void)
{
    sac_error_t sac_err;

    sac_pipeline_process(sac_pipeline_input1, &sac_err);
    sac_pipeline_process(sac_pipeline_input2, &sac_err);

    if (mix_audio) {
        mix_audio = false;
        sac_pipeline_process(sac_pipeline_mixer, &sac_err);
    }
}

/** @brief Callback handling when the stats have to be printed.
 */
static void stats_callback(void)
{
    print_stats_now = true;
}

/** @brief Print the audio and wireless statistics.
 */
static void print_stats(void)
{
    static char stats_string[STATS_ARRAY_LENGTH];
    int string_length = 0;

    const char *device_str = "\n<   COORDINATOR   >\n\r";
    const char *audio_stats_str = "\n<<  Audio Core Statistics  >>\n\r";
    const char *wireless_stats_str = "\n<<  Wireless Core Statistics  >>\n\r";

    /* Device Prelude */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

    /* Audio statistics */
        /* sac_pipeline_input1 */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, audio_stats_str);
    sac_pipeline_update_stats(sac_pipeline_input1);
    string_length += sac_pipeline_format_stats(sac_pipeline_input1, stats_string + string_length, sizeof(stats_string) - string_length);

        /* sac_pipeline_input2 */
    sac_pipeline_update_stats(sac_pipeline_input2);
    string_length += sac_pipeline_format_stats(sac_pipeline_input2, stats_string + string_length, sizeof(stats_string) - string_length);

        /* sac_pipeline_mixer */
    sac_pipeline_update_stats(sac_pipeline_mixer);
    string_length += sac_pipeline_format_stats(sac_pipeline_mixer, stats_string + string_length, sizeof(stats_string) - string_length);

    /* Wireless statistics */
        /* RX from node 1 */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, wireless_stats_str);
    swc_connection_update_stats(rx_from_node1_conn);
    string_length += swc_connection_format_stats(rx_from_node1_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);

        /* RX from node 2 */
    swc_connection_update_stats(rx_from_node2_conn);
    string_length += swc_connection_format_stats(rx_from_node2_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);

    iface_print_string(stats_string);
}
