/** @file  sac_api.h
 *  @brief SPARK Audio Core Application Programming Interface.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_API_H_
#define SAC_API_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "crc4_itu.h"
#include "mem_pool.h"
#include "queue.h"
#include "resampling.h"
#include "sac_error.h"
#include "sac_mixer_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
/*! Maximum of audio channels supported in audio core. */
#define SAC_MAX_CHANNEL_COUNT          2
/*! Placeholder to be used in a `sac_processing_ctrl` function call when no arguments are required. */
#define SAC_NO_ARG                     0
/*! Position of the audio payload in the audio packet. */
#define SAC_NODE_PAYLOAD_SIZE_OFFSET   0
/*! Size of the audio payload variable. */
#define SAC_NODE_PAYLOAD_SIZE_VAR_SIZE sizeof(uint16_t)
/*! Position of the audio header in the audio packet. */
#define SAC_PACKET_HEADER_OFFSET       (SAC_NODE_PAYLOAD_SIZE_OFFSET + SAC_NODE_PAYLOAD_SIZE_VAR_SIZE)
/*! Position of the packet data in the audio packet. */
#define SAC_PACKET_DATA_OFFSET         (SAC_PACKET_HEADER_OFFSET + sizeof(sac_header_t))
/*! Minimum queue size to use when initializing a producer audio endpoint. */
#define SAC_MIN_PRODUCER_QUEUE_SIZE    3

/* MACROS *********************************************************************/
/*! Get the audio payload size in the audio packet. */
#define sac_node_get_payload_size(node) \
        (*((uint16_t *)(queue_get_data_ptr(node, SAC_NODE_PAYLOAD_SIZE_OFFSET))))

/*! Set the audio payload size in the audio packet. */
#define sac_node_set_payload_size(node, payload_size) \
        ((*((uint16_t *)(queue_get_data_ptr(node, SAC_NODE_PAYLOAD_SIZE_OFFSET)))) = (payload_size))

/*! Get a pointer to the audio header in the audio packet. */
#define sac_node_get_header(node) \
        ((sac_header_t *)(queue_get_data_ptr(node, SAC_PACKET_HEADER_OFFSET)))

/*! Get a pointer to the packet data in the audio packet. */
#define sac_node_get_data(node) \
        ((uint8_t *)(queue_get_data_ptr(node, SAC_PACKET_DATA_OFFSET)))

/*! Return an array size aligned on a specific type. */
#define sac_align_data_size(current_size, type_to_align) \
        (sizeof(type_to_align) - ((current_size) % sizeof(type_to_align)))

/* TYPES **********************************************************************/
typedef struct sac_pipeline sac_pipeline_t;

/** @brief Audio Core Configuration.
 */
typedef struct sac_cfg {
    /*! Memory pool instance from which memory allocation is done. */
    uint8_t *memory_pool;
    /*! Memory pool size in bytes. */
    size_t memory_pool_size;
} sac_cfg_t;

/** @brief Audio Core Hardware Abstraction Layer.
 */
typedef struct sac_hal {
    /*! Function the audio core uses to enter a critical section of the code. */
    void (*enter_critical)(void);
    /*! Function the audio core uses to exit a critical section of the code. */
    void (*exit_critical)(void);
} sac_hal_t;

/** @brief Audio Core Bit Depth.
 */
typedef enum sac_bit_depth {
    /*! 16-bit PCM samples. */
    SAC_16BITS = 16,
    /*! 18-bit PCM samples. */
    SAC_18BITS = 18,
    /*! 20-bit PCM samples. */
    SAC_20BITS = 20,
    /*! 24-bit PCM samples. */
    SAC_24BITS = 24,
    /*! 32-bit PCM samples. */
    SAC_32BITS = 32,
} sac_bit_depth_t;

/** @brief Audio Core Word Size.
 */
typedef enum sac_word_size {
    /*! 2-byte word for 16-bit PCM samples. */
    SAC_16BITS_BYTE = 2,
    /*! 4-byte word for 18-bit PCM samples. */
    SAC_18BITS_BYTE = 4,
    /*! 4-byte word for 20-bit PCM samples. */
    SAC_20BITS_BYTE = 4,
    /*! 4-byte word for 24-bit PCM samples. */
    SAC_24BITS_BYTE = 4,
    /*! 4-byte word for 32-bit PCM samples. */
    SAC_32BITS_BYTE = 4,
} sac_word_size_t;

/** @brief Audio Core Header.
 */
typedef struct sac_header {
    /*! For clock drift compensation. Used by the recorder to notify the player that its TX audio buffer is filling up. */
    uint8_t tx_queue_level_high:1;
    /*! Indicates a fallback packet. */
    uint8_t fallback:1;
    /*! Reserved for future use. */
    uint8_t reserved:2;
    /*! CRC4 of the header. */
    uint8_t crc4:4;
    /*! Size of the payload (audio samples) expressed in bytes. */
    uint8_t payload_size;
} sac_header_t;

/** @brief Processing Interface.
 */
typedef struct sac_processing_interface {
    /*! Function the audio core uses to execute any processing stage initialization sequence. */
    void (*init)(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err);
    /*! Function the audio application uses to interact with the processing stage. */
    uint32_t (*ctrl)(void *instance, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t args, sac_error_t *err);
    /*! Function the audio core uses to do processing on audio samples. */
    uint16_t (*process)(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in,
                        uint16_t size, uint8_t *data_out, sac_error_t *err);
    /*! Function called by process_samples prior to process to determine if process will be executed or not. */
    bool (*gate)(void *instance, sac_pipeline_t *pipeline,  sac_header_t *header,
                 uint8_t *data_in, uint16_t size, sac_error_t *err);
} sac_processing_interface_t;

/** @brief Audio Core Processing.
 */
typedef struct sac_processing {
    /*! Pointer to the processing stage's specific instance. */
    void *instance;
    /*! Character string describing the processing stage. */
    const char *name;
    /*! Interface the processing stage must comply to. */
    sac_processing_interface_t iface;
    /*! Pointer to the next processing state. */
    struct sac_processing *next_process;
} sac_processing_t;

/** @brief Endpoint Interface.
 */
typedef struct sac_endpoint_interface {
    /*! Function the audio core uses to send or receive audio samples depending if the endpoint produces or consumes. */
    uint16_t (*action)(void *instance, uint8_t *samples, uint16_t size);
    /*! Function the audio core uses to execute any endpoint startup sequence. */
    void (*start)(void *instance);
    /*! Function the audio core uses to stop any endpoint operations. */
    void (*stop)(void *instance);
} sac_endpoint_interface_t;

/** @brief Add Audio Core Mixer's specific options when using pipelines to mix packets.
 */
typedef struct sac_mixer_option {
    /*! True if it is the input pipeline of the mixing stage. */
    bool input_mixer_pipeline;
    /*! True if it is the output pipeline of the mixing stage. */
    bool output_mixer_pipeline;
} sac_mixer_option_t;

/** @brief Audio Core Endpoint Configuration.
 */
typedef struct sac_endpoint_cfg {
    /*!
     * True if the endpoint produces or consumes audio packets (SAC header + audio payload),
     * False for only audio payloads (audio samples).
     */
    bool use_encapsulation;
    /*! True if the endpoint requires a complete cycle to produce or consume data. False if the endpoint produces or consumes instantly. */
    bool delayed_action;
    /*! 1 if the endpoint produces or consumes mono audio payloads and 2 for interleaved stereo. */
    uint8_t channel_count;
    /*! Bit depth of samples the endpoint produces or consumes. */
    sac_bit_depth_t bit_depth;
    /*! Size in bytes of the audio payload. */
    uint16_t audio_payload_size;
    /*! Size in number of audio packets the endpoint's queue can contain. */
    uint8_t queue_size;
} sac_endpoint_cfg_t;

/** @brief Audio Core Endpoint.
 */
typedef struct sac_endpoint {
    /*! Pointer to endpoint's specific instance. */
    void *instance;
    /*! Character string describing the endpoint. */
    const char *name;
    /*! Interface the endpoint must comply to. */
    sac_endpoint_interface_t iface;
    /*! SAC endpoint configuration. */
    sac_endpoint_cfg_t cfg;
    /*! Pointer to the next endpoint. */
    struct sac_endpoint *next_endpoint;
    struct {
        /*! Internal: queue the endpoint will use to store or retrieve audio packets. */
        queue_t *queue;
        /*! Internal: Pointer to the free queue the endpoint will retrieve free nodes from. */
        queue_t *free_queue;
        /*! Internal: pointer to the queue node the endpoint is working with at the moment. */
        queue_node_t *current_node;
        /*! Internal: Whether or not the initial audio buffering has been completed. */
        bool buffering_complete;
    } _internal;
} sac_endpoint_t;

/** @brief Audio Core Pipeline Configuration.
 */
typedef struct sac_pipeline_cfg {
    /*! Set to true if an audio clock drift compensation processing stage will be added to this pipeline. */
    bool cdc_enable;
    /*! Wait for the consumer queue (TX audio buffer) to be full before starting to consume. */
    bool do_initial_buffering;
    /*! Configure the pipeline with mixer's specific options. */
    sac_mixer_option_t mixer_option;
} sac_pipeline_cfg_t;

/** @brief Audio Core Statistics.
 */
typedef struct sac_statistics {
    /*! Number of audio packets currently in the producer queue. */
    uint32_t producer_buffer_load;
    /*! Maximum number of audio packets the producer queue can hold. */
    uint16_t producer_buffer_size;
    /*! Number of times the producer queue has overflowed. */
    uint32_t producer_buffer_overflow_count;
    /*! Number of corrupted packets received from the coord. */
    uint32_t producer_packets_corrupted_count;
    /*! Number of audio packets currently in the consumer queue. */
    uint32_t consumer_buffer_load;
    /*! Maximum number of audio packets the consumer queue can hold. */
    uint16_t consumer_buffer_size;
    /*! Number of times the consumer queue has overflowed. */
    uint32_t consumer_buffer_overflow_count;
    /*! Number of times the consumer queue has underflowed. */
    uint32_t consumer_buffer_underflow_count;
    /*! Consumer queue peak load. */
    uint32_t consumer_queue_peak_buffer_load;
    /*! Consumer link margin min. */
    uint32_t consumer_link_margin_min_peak;
    /*! Consumer CCA count peak. */
    uint32_t consumer_cca_fail_count_peak;
    /*! Number of packets inflated by the CDC. */
    uint32_t cdc_inflated_packets_count;
    /*! Number of packets deflated by the CDC. */
    uint32_t cdc_deflated_packets_count;
} sac_statistics_t;

/** @brief Audio Core Pipeline.
 */
typedef struct sac_pipeline {
    /*! Name of the pipeline. */
    const char *name;
    /*! Pipeline inputting audio samples when doing audio mixing. */
    sac_pipeline_t *input_pipeline[MAX_NB_OF_INPUTS];
    /*! Pointer to the SAC endpoint that will produce audio samples to this SAC pipeline. */
    sac_endpoint_t *producer;
    /*! List of processing stages that will sequentially be produced samples before they are consumed. */
    sac_processing_t *process;
    /*! Pointer to the SAC endpoint that will consume audio samples from this SAC pipeline. */
    sac_endpoint_t *consumer;
    /*! SAC pipeline configuration. */
    sac_pipeline_cfg_t cfg;
    /*! SAC pipeline statistics. */
    sac_statistics_t _statistics;
    struct {
        /*! Internal: The number of audio packets to buffer before considering the initial buffering complete. */
        uint8_t buffering_threshold;
        /*! Internal: Size in bytes of samples produced but not yet consumed. */
        uint32_t samples_buffered_size;
    } _internal;
} sac_pipeline_t;

/** @brief Forward declaration of the fallback module configuration structure.
 */
typedef struct sac_fallback_module_cfg sac_fallback_module_cfg_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the Audio Core.
 *
 *  @param[in]  cfg  Audio Core configuration.
 *  @param[in]  hal  Board specific functions.
 *  @param[out] err  Error code.
 */
void sac_init(sac_cfg_t cfg, sac_hal_t *hal, sac_error_t *err);

/** @brief Initialize the SAC Mixer Module.
 *
 *  @note Only call this initialization when the Mixer Module is needed.
 *
 *  @param[in]  cfg  Configuration of the SAC Mixer Module.
 *  @param[out] err  Error code.
 */
void sac_mixer_init(sac_mixer_module_cfg_t cfg, sac_error_t *err);

/** @brief Initialize the SAC Fallback Module using the SAC mem_pool.
 *
 *  @note Only call this initialization when the Fallback Module is needed.
 *
 *  @param[in]  cfg  Pointer to the configuration of the module.
 *  @param[out] err  Error code.
 */
void sac_fallback_init(sac_fallback_module_cfg_t *cfg, sac_error_t *err);

/** @brief Initialize a SAC pipeline.
 *
 *  @param[in]  name      Name of the pipeline.
 *  @param[in]  producer  Producer endpoint.
 *  @param[in]  cfg       Pipeline configuration.
 *  @param[in]  consumer  Consumer endpoint.
 *  @param[out] err       Error code.
 *  @return Reference to the initialized SAC pipeline.
 */
sac_pipeline_t *sac_pipeline_init(const char *name, sac_endpoint_t *producer, sac_pipeline_cfg_t cfg,
                                  sac_endpoint_t *consumer, sac_error_t *err);

/** @brief Initialize an Audio Core endpoint.
 *
 *  @param[in]  instance  Endpoint instance.
 *  @param[in]  name      Name of the endpoint.
 *  @param[in]  iface     Interface of the endpoint.
 *  @param[in]  cfg       Endpoint configuration.
 *  @param[out] err       Error code.
 *  @return Reference to the initialized endpoint.
 */
sac_endpoint_t *sac_endpoint_init(void *instance, const char *name, sac_endpoint_interface_t iface,
                                  sac_endpoint_cfg_t cfg, sac_error_t *err);

/** @brief Link the queue of a consumer endpoint with a producer endpoint.
 *
 *  @note This feature is used for audio mixing.
 *
 *  @param[in]  consumer  Consumer endpoint instance to be linked.
 *  @param[in]  producer  Producer endpoint instance to be linked.
 *  @param[out] err       Error code.
 */
void sac_endpoint_link(sac_endpoint_t *consumer, sac_endpoint_t *producer, sac_error_t *err);

/** @brief Initialize an Audio Core processing stage.
 *
 *  @param[in]  instance  Processing stage instance.
 *  @param[in]  name      Name of the processing stage.
 *  @param[in]  iface     Interface of the processing stage.
 *  @param[out] err       Error code.
 *  @return Reference to the initialized processing stage.
 */
sac_processing_t *sac_processing_stage_init(void *instance, const char *name, sac_processing_interface_t iface, sac_error_t *err);

/** @brief Add a processing stage to the pipeline.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  process   Pointer to processing structure.
 *  @param[out] err       Error code.
 */
void sac_pipeline_add_processing(sac_pipeline_t *pipeline, sac_processing_t *process, sac_error_t *err);

/** @brief Add an extra consumer endpoint to the pipeline.
 *
 *  @param[in]  pipeline       Pipeline instance.
 *  @param[in]  next_consumer  Extra consumer endpoint.
 *  @param[out] err            Error code.
 */
void sac_pipeline_add_extra_consumer(sac_pipeline_t *pipeline, sac_endpoint_t *next_consumer, sac_error_t *err);

/** @brief Add an extra producer endpoint to the pipeline.
 *
 *  @note This feature is used for Audio Mixing.
 *
 *  @param[in]  pipeline       Pipeline instance.
 *  @param[in]  next_producer  Extra producer endpoint.
 *  @param[out] err            Error code.
 */
void sac_pipeline_add_extra_producer(sac_pipeline_t *pipeline, sac_endpoint_t *next_producer, sac_error_t *err);

/** @brief Add an input pipeline to a pipeline doing audio mixing.
 *
 *  @param[in]  pipeline        Pipeline instance.
 *  @param[in]  input_pipeline  Input pipeline instance.
 *  @param[out] err             Error code.
 */
void sac_pipeline_add_input_pipeline(sac_pipeline_t *pipeline, sac_pipeline_t *input_pipeline, sac_error_t *err);

/** @brief Setup the Audio Core pipeline.
 *
 *  This makes the pipeline ready to use. It must be called last,
 *  after every other initialization functions.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
void sac_pipeline_setup(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Start the Audio Core pipeline.
 *
 *  @param[in] pipeline  Pipeline instance.
 *  @param[out] err      Error code.
 */
void sac_pipeline_start(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Stop the Audio Core pipeline.
 *
 *  @param[in] pipeline  Pipeline instance.
 *  @param[out] err      Error code.
 */
void sac_pipeline_stop(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Execute process specific control.
 *
 *  @param[in]  sac_processing    SAC processing structure.
 *  @param[in]  pipeline          Pipeline instance.
 *  @param[in]  cmd               Command specific to the processing stage.
 *  @param[in]  arg               Argument specific to the processing stage.
 *  @param[out] err               Error code.
 *  @return A value specific to the control function.
 */
uint32_t sac_processing_ctrl(sac_processing_t *sac_processing, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t arg, sac_error_t *err);

/** @brief Execute the Audio Core processing stages.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
void sac_pipeline_process(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Execute the produce endpoint.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
void sac_pipeline_produce(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Execute the consumer endpoint(s).
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
void sac_pipeline_consume(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Execute all the consume endpoints that contain data in their queue.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
void sac_pipeline_consume_all(sac_pipeline_t *pipeline, sac_error_t *err);

/** @brief Get the number of bytes allocated in the memory pool.
 *
 *  @return Number of bytes allocated in the memory pool.
 */
uint32_t sac_get_allocated_bytes(void);

#ifdef __cplusplus
}
#endif

#endif /* SAC_API_H_ */
