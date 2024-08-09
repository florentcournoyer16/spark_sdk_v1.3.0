/** @file  audio_user_data_coord.c
 *  @brief This application creates a unidirectional uncompressed audio stream
 *         from the MAX98091 audio codec Line-In of the coordinator to the
 *         Headphone-Out of the node. The stream also carries user data.
 *
 *         This application demonstrates the possibility to add a custom audio processing
 *         stage at the application level.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES ******************************************************************/
#include <stdio.h>
#include "iface_audio.h"
#include "iface_audio_user_data.h"
#include "iface_wireless.h"
#include "sac_api.h"
#include "sac_stats.h"
#include "swc_api.h"
#include "swc_cfg_coord.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
#define SAC_MEM_POOL_SIZE        3600
#define SAC_PAYLOAD_SIZE         68
#define SAC_LATENCY_QUEUE_SIZE   24
#define SWC_MEM_POOL_SIZE        3700
#define CUSTOM_DATA_PAYLOAD_SIZE 1
#define STATS_ARRAY_LENGTH       1500

#define LED_STATE_OFF 0
#define LED_STATE_ON  1

/* PRIVATE GLOBALS ************************************************************/
/* ** Audio Core ** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
static sac_hal_t sac_hal;
static sac_pipeline_t *sac_pipeline;

static sac_processing_t *user_data_tx_processing;
bool    user_data_is_valid;
uint8_t user_data;

static sac_endpoint_t *max98091_producer;

static ep_swc_instance_t swc_consumer_instance;
static sac_endpoint_t *swc_consumer;

/* ** Wireless Core ** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t swc_hal;
static swc_node_t *node;
static swc_connection_t *tx_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t tx_timeslots[] = TX_TIMESLOTS;

/* ** Application Specific ** */
static volatile bool print_stats_now;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_tx_success_callback(void *conn);

static void app_audio_core_init(sac_error_t *sac_err);
static void app_audio_core_user_data_interface_init(sac_processing_interface_t *iface);
static void audio_i2s_rx_complete_cb(void);
static uint16_t app_audio_user_data_process_tx(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in,
                                               uint16_t size, uint8_t *data_out, sac_error_t *err);

static void send_led_state_on(void);
static void send_led_state_off(void);

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

    sac_pipeline_start(sac_pipeline, &sac_err);

    swc_connect(&swc_err);

    iface_audio_process_timer_init(100);
    iface_audio_process_set_timer_callback(audio_process_callback);
    iface_audio_process_timer_start();

    iface_stats_timer_init(1000);
    iface_stats_set_timer_callback(stats_callback);
    iface_stats_timer_start();

    while (1) {
        iface_button_handling(send_led_state_on, send_led_state_off);
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

    /* ** TX Connection ** */
    swc_connection_cfg_t tx_conn_cfg = {
        .name = "TX Connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .max_payload_size = SAC_PAYLOAD_SIZE + CUSTOM_DATA_PAYLOAD_SIZE + sizeof(sac_header_t),
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
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
    tx_conn = swc_connection_init(node, tx_conn_cfg, &swc_hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_conn, node, tx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_tx_success_callback(tx_conn, conn_tx_success_callback, err);

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
    sac_processing_interface_t user_data_tx_iface;

    iface_sac_hal_init(&sac_hal);
    iface_audio_swc_endpoint_init(NULL, &swc_consumer_iface);
    iface_audio_max98091_endpoint_init(&max98091_producer_iface, NULL);
    iface_set_sai_complete_callback(NULL, audio_i2s_rx_complete_cb);

    app_audio_core_user_data_interface_init(&user_data_tx_iface);

    swc_consumer_instance.connection = tx_conn;

    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE
    };
    sac_init(core_cfg, &sac_hal, sac_err);

    /*
     * Audio Pipeline
     * ==============
     *
     * Input:      Stereo stream of 34 samples @ 48 kHz/16 bits is recorded using the MAX98091 hardware audio codec.
     * Processing: A byte of trailing user data is added to the audio packet.
     * Output:     Stereo stream of 34 samples @ 48 kHz/16 bits is sent over the air to the Node.
     *
     * +-------+    +-----------------------+    +-----+
     * | Codec | -> | Append User Data Byte | -> | SWC |
     * +-------+    +-----------------------+    +-----+
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
                                          max98091_producer_iface, max98091_producer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    user_data_tx_processing = sac_processing_stage_init(NULL, "User Data TX", user_data_tx_iface, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_endpoint_cfg_t swc_consumer_cfg = {
        .use_encapsulation  = true,
        .delayed_action     = false,
        .channel_count      = 2,
        .bit_depth          = SAC_16BITS,
        .audio_payload_size = SAC_PAYLOAD_SIZE + CUSTOM_DATA_PAYLOAD_SIZE,
        .queue_size         = SAC_LATENCY_QUEUE_SIZE
    };
    swc_consumer = sac_endpoint_init((void *)&swc_consumer_instance, "SWC EP (Consumer)",
                                     swc_consumer_iface, swc_consumer_cfg, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_cfg_t pipeline_cfg = {
        .cdc_enable = false,
        .do_initial_buffering = true,
    };
    sac_pipeline = sac_pipeline_init("Codec -> SWC", max98091_producer,
                                       pipeline_cfg, swc_consumer, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }

    sac_pipeline_add_processing(sac_pipeline, user_data_tx_processing, sac_err);

    sac_pipeline_setup(sac_pipeline, sac_err);
    if (*sac_err != SAC_ERR_NONE) {
        return;
    }
}

/** @brief Initialize the user data audio processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_user_data_interface_init(sac_processing_interface_t *iface)
{
    iface->init = NULL;
    iface->ctrl = NULL;
    iface->process = app_audio_user_data_process_tx;
    iface->gate = NULL;
}

/** @brief Process the user data processing stage.
 *
 *  @param[in]  instance  User data instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  header    Audio packet's header.
 *  @param[in]  data_in   Audio payload to process.
 *  @param[in]  size      Size in bytes of the audio payload.
 *  @param[out] data_out  Audio payload that has been processed.
 *  @param[out] err       Error code.
 *
 *  @return Size in bytes of the processed samples, 0 if no processing happened.
 */
static uint16_t app_audio_user_data_process_tx(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in,
                                               uint16_t size, uint8_t *data_out, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)header;
    uint16_t output_size = 0;

    *err = SAC_ERR_NONE;

    if (user_data_is_valid) {
        data_in[size] = user_data;
        output_size = size + 1;
        memcpy(data_out, data_in, output_size);
        user_data_is_valid = false;
    }

    return output_size;
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

/** @brief Set the audio user data byte value to 1.
 *
 *  This affects the audio pipeline the audio user data
 *  processing stage is added to.
 */
static void send_led_state_on(void)
{
    user_data = LED_STATE_ON;
    user_data_is_valid = true;
}

/** @brief Set the audio user data byte value to 0.
 *
 *  This affects the audio pipeline the audio user data
 *  processing stage is added to.
 */
static void send_led_state_off(void)
{
    user_data = LED_STATE_OFF;
    user_data_is_valid = true;
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

    const char *device_str = "\n<   COORDINATOR   >\n\r";
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

    iface_print_string(stats_string);
}
