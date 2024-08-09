/** @file  audio_compression_node.c
 *  @brief This application creates a unidirectional uncompressed audio stream
 *         from the MAX98091 audio codec Line-In of the coordinator to the
 *         Headphone-Out of the node. If the link quality degrades, the audio
 *         stream becomes compressed.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include "sac_compression.h"
#include "sac_fallback_gate.h"
#include "sac_fallback_module.h"
#include "iface_audio.h"
#include "iface_audio_compression.h"
#include "iface_wireless.h"
#include "sac_api.h"
#include "sac_cdc.h"
#include "sac_mute_on_underflow.h"
#include "sac_stats.h"
#include "sac_utils.h"
#include "sac_volume.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SAC_MEM_POOL_SIZE      10000

#define SAC_PAYLOAD_SIZE       68
#define SAC_NB_CHANNEL         2
#define SAC_LATENCY_QUEUE_SIZE 22
#define SAC_SAMPLING_RATE      48000
#define SAC_BIT_DEPTH          SAC_16BITS

#define SWC_MEM_POOL_SIZE      10000
#define STATS_ARRAY_LENGTH     1500
#define LINK_MARGIN_SIZE_BYTES 1

/* PRIVATE GLOBALS ************************************************************/
/* ** Audio Core ** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
static sac_hal_t sac_hal;
static sac_pipeline_t *sac_pipeline;

static sac_volume_instance_t volume_instance;
static sac_processing_t *volume_processing;

static sac_compression_instance_t decompression_instance;
static sac_processing_t *decompression_processing;

static sac_cdc_instance_t cdc_instance;
static sac_processing_t *cdc_processing;

static sac_mute_on_underflow_instance_t mute_on_underflow_instance;
static sac_processing_t *mute_on_underflow_processing;

static sac_endpoint_t *max98091_consumer;

static ep_swc_instance_t swc_producer_instance;
static sac_endpoint_t *swc_producer;

static swc_fallback_info_t fallback_info;

/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *tx_conn;
static swc_connection_t *rx_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t tx_timeslots[] = TX_TIMESLOTS;
static int32_t rx_timeslots[] = RX_TIMESLOTS;
static uint8_t rx_link_margin_data_tx;

/* ** Application Specific ** */
static volatile bool print_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_tx_success_callback(void *conn);
static void conn_rx_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void app_audio_core_compression_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_volume_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_cdc_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_mute_on_underflow_interface_init(sac_processing_interface_t *iface);
static void audio_i2s_tx_complete_cb(void);

static void volume_up(void);
static void volume_down(void);

static void fallback_led_handler(void);
static void fallback_send_rx_link_margin(void);

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

    iface_audio_node_init();

    sac_pipeline_start(sac_pipeline, &sac_err);

    swc_connect(&swc_err);

    iface_audio_process_timer_init(100);
    iface_audio_process_set_timer_callback(audio_process_callback);
    iface_audio_process_timer_start();

    iface_stats_timer_init(1000);
    iface_stats_set_timer_callback(stats_callback);
    iface_stats_timer_start();

    while (1) {
        iface_button_handling(volume_up, volume_down);
        fallback_led_handler();

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
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = true,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE,
        .rdo_enabled = true,
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
        .max_payload_size = LINK_MARGIN_SIZE_BYTES,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
        .arq_enabled = false,
    };
    tx_conn = swc_connection_init(node, tx_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_connection_set_tx_success_callback(tx_conn, conn_tx_success_callback, err);

    /* ** RX Connection ** */
    swc_connection_cfg_t rx_conn_cfg = {
        .name = "RX Connection",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = SAC_PAYLOAD_SIZE + sizeof(sac_header_t),
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
        .cca_enabled = true,
        .cca_settings.threshold = 25,
        .cca_settings.try_count = 3,
        .cca_settings.retry_time = 512, /* 25 us */
        .cca_settings.fail_action = SWC_CCA_FORCE_TX,
        .rdo_enabled = true,
        .fallback_enabled = false,
    };
    rx_conn = swc_connection_init(node, rx_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_AUTO_REPLY_PULSE_COUNT,
        .tx_pulse_width = TX_AUTO_REPLY_PULSE_WIDTH,
        .tx_pulse_gain  = TX_AUTO_REPLY_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
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
    fallback_send_rx_link_margin();
}

/** @brief Function for node to send the current link margin to the coordinator via the auto reply.
 */
static void fallback_send_rx_link_margin(void)
{
    swc_error_t swc_err;

    fallback_info = swc_connection_get_fallback_info(rx_conn, &swc_err);
    rx_link_margin_data_tx = (fallback_info.link_margin > UINT8_MAX) ? UINT8_MAX : fallback_info.link_margin;
    swc_connection_send(tx_conn, &rx_link_margin_data_tx, LINK_MARGIN_SIZE_BYTES, &swc_err);
}

/** @brief Initialize the Audio Core.
 *
 *  @param[out] err  Audio Core error code.
 */
static void app_audio_core_init(sac_error_t *sac_err)
{
    sac_endpoint_interface_t max98091_consumer_iface;
    sac_endpoint_interface_t swc_producer_iface;
    sac_processing_interface_t volume_iface;
    sac_processing_interface_t decompression_iface;
    sac_processing_interface_t cdc_iface;
    sac_processing_interface_t mute_on_underflow_iface;

    iface_sac_hal_init(&sac_hal);
    iface_audio_swc_endpoint_init(&swc_producer_iface, NULL);
    iface_audio_max98091_endpoint_init(NULL, &max98091_consumer_iface);
    iface_set_sai_complete_callback(audio_i2s_tx_complete_cb, NULL);

    app_audio_core_volume_interface_init(&volume_iface);
    app_audio_core_compression_interface_init(&decompression_iface);
    app_audio_core_cdc_interface_init(&cdc_iface);
    app_audio_core_mute_on_underflow_interface_init(&mute_on_underflow_iface);

    swc_producer_instance.connection = rx_conn;

    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE
    };
    sac_init(core_cfg, &sac_hal, sac_err);

    /*
     * Audio Pipeline
     * ==============
     *
     ***** NORMAL MODE *****
     * Input:      Stereo stream of 34 samples @ 48 kHz/16 bits is received over the air from the Coordinator.
     * Processing: Digital volume control and clock drift compensation and mute on glitch.
     * Output:     Stereo stream of 34 samples @ 48 kHz/16 bits is played using the MAX98091 hardware audio codec.
     *
     * +-----+    +----------------+    +-----+    +----------------+    +-------+
     * | SWC | -> | Digital Volume | -> | CDC | -> | Mute on Glitch | -> | Codec |
     * +-----+    +----------------+    +-----+    +----------------+    +-------+
     *
     ***** FALLBACK MODE *****
     * Input:      Stereo stream of 34 samples @ 48 kHz/4 bits is received over the air from the Coordinator.
     * Processing: Decompression of samples compressed with ADPCM, digital volume control and clock drift compensation and mute on glitch.
     * Output:     Stereo stream of 34 samples @ 48 kHz/16 bits is played using the MAX98091 hardware audio codec.
     *
     * +-----+    +---------------------+    +----------------+    +-----+    +----------------+    +-------+
     * | SWC | -> | ADPCM Decompression | -> | Digital Volume | -> | CDC | -> | Mute on Glitch | -> | Codec |
     * +-----+    +---------------------+    +----------------+    +-----+    +----------------+    +-------+
     */
    sac_endpoint_cfg_t swc_producer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = SAC_NB_CHANNEL,
        .bit_depth          = SAC_BIT_DEPTH,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE
    };
    swc_producer = sac_endpoint_init((void *)&swc_producer_instance, "SWC EP (Producer)",
                                     swc_producer_iface, swc_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    volume_instance.initial_volume_level = 100;
    volume_instance.bit_depth = SAC_16BITS;
    volume_processing = sac_processing_stage_init((void *)&volume_instance, "Digital Volume Control",
                                                  volume_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    decompression_instance.compression_mode = SAC_COMPRESSION_UNPACK_STEREO;
    decompression_instance.bit_depth = SAC_16BITS;
    decompression_processing = sac_processing_stage_init((void *)&decompression_instance, "Audio Decompression",
                                                         decompression_iface, sac_err);
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
    max98091_consumer = sac_endpoint_init(NULL, "MAX98091 EP (Consumer)",
                                          max98091_consumer_iface, max98091_consumer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t pipeline_cfg = {
        .cdc_enable = true,
        .do_initial_buffering = false,
    };
    sac_pipeline = sac_pipeline_init("SWC -> Codec", swc_producer,
                                       pipeline_cfg, max98091_consumer, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline, decompression_processing, sac_err);
    sac_pipeline_add_processing(sac_pipeline, volume_processing, sac_err);
    sac_pipeline_add_processing(sac_pipeline, cdc_processing, sac_err);
    sac_pipeline_add_processing(sac_pipeline, mute_on_underflow_processing, sac_err);

    sac_pipeline_setup(sac_pipeline, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    /* Fallback module for the node is needed only to manage the fallback flag */
    sac_fallback_module_cfg_t audio_fallback_cfg = { 0 };

    sac_fallback_init(&audio_fallback_cfg, sac_err);
}

/** @brief Initialize the compression audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_compression_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_compression_init;
    iface->ctrl = sac_compression_ctrl;
    iface->process = sac_compression_process;
    iface->gate = sac_fallback_gate_fallback_on_detect;
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

/** @brief Increase the audio output volume level.
 *
 *  This affects the audio pipeline the digital volume
 *  processing stage is added to.
 */
static void volume_up(void)
{
    sac_error_t err;

    sac_processing_ctrl(volume_processing, sac_pipeline, SAC_VOLUME_INCREASE, SAC_NO_ARG, &err);
}

/** @brief Decrease the audio output volume level.
 *
 *  This affects the audio pipeline the digital volume
 *  processing stage is added to.
 */
static void volume_down(void)
{
    sac_error_t err;

    sac_processing_ctrl(volume_processing, sac_pipeline, SAC_VOLUME_DECREASE, SAC_NO_ARG, &err);
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

/** @brief Update the fallback LED indication.
 */
static void fallback_led_handler(void)
{
    iface_fallback_status(sac_fallback_is_active());
}

/** @brief Callback handling the audio process that triggers with the app timer.
 */
static void audio_process_callback(void)
{
    sac_error_t sac_err;

    sac_pipeline_process(sac_pipeline, &sac_err);
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

    const char *device_str = "\n<   NODE   >\n\r";
    const char *audio_stats_str = "\n<<  Audio Core Statistics  >>\n\r";
    const char *wireless_stats_str = "\n<<  Wireless Core Statistics  >>\n\r";

    /* Device Prelude */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

    /* Audio statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, audio_stats_str);
    sac_pipeline_update_stats(sac_pipeline);
    string_length += sac_pipeline_format_stats(sac_pipeline, stats_string + string_length, sizeof(stats_string) - string_length);

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
