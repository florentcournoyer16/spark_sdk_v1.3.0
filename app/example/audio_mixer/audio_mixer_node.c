/** @file  audio_mixer_node.c
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
#include "iface_audio.h"
#include "iface_audio_mixer.h"
#include "iface_wireless.h"
#include "sac_api.h"
#include "sac_stats.h"
#include "swc_api.h"
#include "swc_stats.h"

#if defined(NODE1)
#include "swc_cfg_node1.h"
#elif defined(NODE2)
#include "swc_cfg_node2.h"
#endif

/* CONSTANTS ******************************************************************/
#define SAC_MEM_POOL_SIZE      2000
#define SAC_PAYLOAD_SIZE       74
#define SAC_LATENCY_QUEUE_SIZE 7
#define SWC_MEM_POOL_SIZE      7000
#define STATS_ARRAY_LENGTH     1500

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
static swc_connection_t *rx_beacon_conn;
static swc_connection_t *tx_to_coord_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_beacon_timeslots[] = RX_BEACON_TIMESLOTS;
static int32_t tx_to_coord_timeslots[] = TX_TO_COORD_TIMESLOTS;

/* ** Application Specific ** */
static volatile bool print_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_tx_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void audio_i2s_rx_complete_cb(void);

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

    swc_connection_cfg_t rx_beacon_conn_cfg;

    rx_beacon_conn_cfg = swc_get_beacon_connection_config(node, COORDINATOR_ADDRESS, rx_beacon_timeslots, ARRAY_SIZE(rx_beacon_timeslots));
    rx_beacon_conn = swc_connection_init(node, rx_beacon_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_from_beacon_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_from_beacon_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_beacon_conn, node, rx_from_beacon_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }

    /* ** TX to Coordinator Connection ** */
    swc_connection_cfg_t tx_to_coord_conn_cfg = {
        .name = "TX to Coord Connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .max_payload_size = SAC_PAYLOAD_SIZE + sizeof(sac_header_t),
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_to_coord_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_to_coord_timeslots),
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
    tx_to_coord_conn = swc_connection_init(node, tx_to_coord_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_to_coord_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_to_coord_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_to_coord_conn, node, tx_to_coord_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_tx_success_callback(tx_to_coord_conn, conn_tx_success_callback, err);

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

/** @brief Initialize the Audio Core.
 *
 *  @param[out] err  Audio Core error code.
 */
static void app_audio_core_init(sac_error_t *sac_err)
{
    sac_endpoint_interface_t max98091_producer_iface;
    sac_endpoint_interface_t swc_consumer_iface;

    iface_sac_hal_init(&sac_hal);
    iface_audio_swc_endpoint_init(NULL, &swc_consumer_iface);
    iface_audio_max98091_endpoint_init(&max98091_producer_iface, NULL);
    iface_set_sai_complete_callback(NULL, audio_i2s_rx_complete_cb);

    swc_consumer_instance.connection = tx_to_coord_conn;

    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE
    };
    sac_init(core_cfg, &sac_hal, sac_err);

    /*
     * Audio Pipeline
     * ==============
     *
     * Input:      Mono stream of 37 samples @ 32 kHz/16 bits is recorded using the MAX98091 hardware audio codec.
     * Processing: None.
     * Output:     Mono stream of 37 samples @ 32 kHz/16 bits is sent over the air to the Node.
     *
     * +-------+    +-----+
     * | Codec | -> | SWC |
     * +-------+    +-----+
     */
    sac_endpoint_cfg_t max98091_producer_cfg = {
        .use_encapsulation  = false,
        .delayed_action     = true,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_MIN_PRODUCER_QUEUE_SIZE
    };
    max98091_producer = sac_endpoint_init(NULL, "MAX98091 EP (Producer)",
                                          max98091_producer_iface, max98091_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t swc_consumer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = 1,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };
    swc_consumer = sac_endpoint_init((void *)&swc_consumer_instance, "SWC EP (Consumer)",
                                     swc_consumer_iface, swc_consumer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t sac_pipeline_cfg = {
        .cdc_enable = false,
        .do_initial_buffering = true,
    };
    sac_pipeline = sac_pipeline_init("Codec -> SWC", max98091_producer,
                                       sac_pipeline_cfg, swc_consumer, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_setup(sac_pipeline, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
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

/** @brief Callback handling the audio process that triggers with the app timer.
 */
static void audio_process_callback(void)
{
    sac_error_t sac_err;

    sac_pipeline_process(sac_pipeline, &sac_err);
    sac_pipeline_consume(sac_pipeline, &sac_err);
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

    const char *device_str = NULL;
    const char *audio_stats_str = "\n<<  Audio Core Statistics  >>\n\r";
    const char *wireless_stats_str = "\n<<  Wireless Core Statistics  >>\n\r";
    #if defined(NODE1)
        device_str = "\n<   NODE 1   >\n\r";
    #elif defined(NODE2)
        device_str = "\n<   NODE 2   >\n\r";
    #endif

    /* Device Prelude */
    string_length = snprintf(stats_string, sizeof(stats_string), device_str);

    /* Audio statistics */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, audio_stats_str);
    sac_pipeline_update_stats(sac_pipeline);
    string_length += sac_pipeline_format_stats(sac_pipeline, stats_string + string_length, sizeof(stats_string) - string_length);

    /* Wireless statistics */
        /* TX */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, wireless_stats_str);
    swc_connection_update_stats(tx_to_coord_conn);
    string_length += swc_connection_format_stats(tx_to_coord_conn, node, stats_string + string_length, sizeof(stats_string) - string_length);

    iface_print_string(stats_string);
}
