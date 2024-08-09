/** @file  sac_api.c
 *  @brief SPARK Audio Core Application Programming Interface.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_api.h"
#include <string.h>
#include "sac_fallback_module.h"

/* CONSTANTS ******************************************************************/
#define CDC_QUEUE_DATA_SIZE_INFLATION      (SAC_MAX_CHANNEL_COUNT * SAC_32BITS_BYTE)
#define CDC_QUEUE_SIZE_INFLATION           3
#define TX_QUEUE_HIGH_LEVEL                2

/* MACROS *********************************************************************/
#define CHECK_ERROR(cond, err_ptr, err_code, ret) \
    do {                                          \
        if (cond) {                               \
            *(err_ptr) = (err_code);              \
            ret;                                  \
        }                                         \
    } while (0)

/* PRIVATE GLOBALS ************************************************************/
static mem_pool_t mem_pool;
static sac_mixer_module_t *sac_mixer_module;
static void (*enter_critical)(void);
static void (*exit_critical)(void);

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void init_audio_queues(sac_pipeline_t *pipeline, sac_error_t *err);
static void init_audio_free_queue(sac_endpoint_t *endpoint, const char *queue_name,
                                  uint16_t queue_data_size, uint8_t queue_size, sac_error_t *err);
static void move_audio_packet_to_consumer_queue(sac_pipeline_t *pipeline, queue_node_t *node);
static queue_node_t *process_samples(sac_pipeline_t *pipeline, queue_node_t *node, sac_error_t *err);
static void enqueue_producer_node(sac_pipeline_t *pipeline, sac_error_t *err);
static uint16_t produce(sac_pipeline_t *pipeline, sac_error_t *err);
static uint16_t consume(sac_pipeline_t *pipeline, sac_endpoint_t *consumer, sac_error_t *err);
static void consume_no_delay(sac_pipeline_t *pipeline, sac_endpoint_t *consumer, sac_error_t *err);
static void consume_delay(sac_pipeline_t *pipeline, sac_endpoint_t *consumer, sac_error_t *err);
static queue_node_t *start_mixing_process(sac_pipeline_t *pipeline, queue_node_t *node);
static void endpoint_link(sac_endpoint_t *consumer, sac_endpoint_t *producer, sac_error_t *err);

/* PUBLIC FUNCTIONS ***********************************************************/
void sac_init(sac_cfg_t cfg, sac_hal_t *hal, sac_error_t *err)
{
    queue_critical_cfg_t queue_critical;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(hal == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(hal->enter_critical == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(hal->exit_critical == NULL, err, SAC_ERR_NULL_PTR, return);

    enter_critical = hal->enter_critical;
    exit_critical  = hal->exit_critical;

    queue_critical.enter_critical = hal->enter_critical;
    queue_critical.exit_critical  = hal->exit_critical;

    queue_init(queue_critical);

    mem_pool_init(&mem_pool, cfg.memory_pool, cfg.memory_pool_size);
}

void sac_mixer_init(sac_mixer_module_cfg_t cfg, sac_error_t *err)
{
    *err = SAC_ERR_NONE;

    sac_mixer_module = sac_mixer_module_init(cfg, &mem_pool, err);
}

void sac_fallback_init(sac_fallback_module_cfg_t *cfg, sac_error_t *err)
{
    *err = SAC_ERR_NONE;

    sac_fallback_module_init(*cfg, &mem_pool, err);
}

sac_pipeline_t *sac_pipeline_init(const char *name, sac_endpoint_t *producer, sac_pipeline_cfg_t cfg,
                                  sac_endpoint_t *consumer, sac_error_t *err)
{
    sac_pipeline_t *pipeline;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(name == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR(producer == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR(consumer == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR((cfg.mixer_option.input_mixer_pipeline) &&
                (cfg.mixer_option.output_mixer_pipeline), err, SAC_ERR_MIXER_OPTION, return NULL);

    pipeline = mem_pool_malloc(&mem_pool, sizeof(sac_pipeline_t));
    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    pipeline->name = name;
    pipeline->producer = producer;
    pipeline->consumer = consumer;
    pipeline->cfg = cfg;

    return pipeline;
}

sac_endpoint_t *sac_endpoint_init(void *instance, const char *name, sac_endpoint_interface_t iface,
                                  sac_endpoint_cfg_t cfg, sac_error_t *err)
{
    sac_endpoint_t *endpoint;
    queue_t *queue;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(name == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR(iface.action == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR(iface.start == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR(iface.stop == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR((cfg.bit_depth != SAC_16BITS) &&
                (cfg.bit_depth != SAC_18BITS) &&
                (cfg.bit_depth != SAC_20BITS) &&
                (cfg.bit_depth != SAC_24BITS) &&
                (cfg.bit_depth != SAC_32BITS), err, SAC_ERR_BIT_DEPTH, return NULL);
    CHECK_ERROR((cfg.channel_count != 1) &&
                (cfg.channel_count != 2), err, SAC_ERR_CHANNEL_COUNT, return NULL);

    endpoint = mem_pool_malloc(&mem_pool, sizeof(sac_endpoint_t));
    CHECK_ERROR(endpoint == NULL, err, SAC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    queue = mem_pool_malloc(&mem_pool, sizeof(queue_t));
    CHECK_ERROR(queue == NULL, err, SAC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    endpoint->instance = instance;
    endpoint->name = name;
    endpoint->iface = iface;
    endpoint->cfg = cfg;
    endpoint->next_endpoint = NULL;
    endpoint->_internal.queue = queue;
    endpoint->_internal.free_queue = NULL;
    endpoint->_internal.current_node = NULL;
    endpoint->_internal.buffering_complete = false;

    return endpoint;
}

sac_processing_t *sac_processing_stage_init(void *instance, const char *name, sac_processing_interface_t iface, sac_error_t *err)
{
    sac_processing_t *process;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(name == NULL, err, SAC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR(iface.process == NULL, err, SAC_ERR_NULL_PTR, return NULL);

    process = mem_pool_malloc(&mem_pool, sizeof(sac_processing_t));
    CHECK_ERROR(process == NULL, err, SAC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    process->instance = instance;
    process->name = name;
    process->iface = iface;

    return process;
}

void sac_pipeline_add_processing(sac_pipeline_t *pipeline, sac_processing_t *process, sac_error_t *err)
{
    sac_processing_t *current_process;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(process == NULL, err, SAC_ERR_NULL_PTR, return);

    current_process = pipeline->process;

    if (current_process == NULL) {
        pipeline->process = process;
        return;
    }

    /* Find the last processing stage in the chain. */
    while (current_process->next_process != NULL) {
        current_process = current_process->next_process;
    }

    current_process->next_process = process;
}

void sac_pipeline_add_extra_consumer(sac_pipeline_t *pipeline, sac_endpoint_t *next_consumer, sac_error_t *err)
{
    sac_endpoint_t *current_consumer;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(next_consumer == NULL, err, SAC_ERR_NULL_PTR, return);

    current_consumer = pipeline->consumer;

    /* Find the last consumer in the chain. */
    while (current_consumer->next_endpoint != NULL) {
        current_consumer = current_consumer->next_endpoint;
    }

    current_consumer->next_endpoint = next_consumer;
}

void sac_pipeline_add_extra_producer(sac_pipeline_t *pipeline, sac_endpoint_t *next_producer, sac_error_t *err)
{
    sac_endpoint_t *current_producer;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(next_producer == NULL, err, SAC_ERR_NULL_PTR, return);

    current_producer = pipeline->producer;

    /* Find the last producer in the chain. */
    while (current_producer->next_endpoint != NULL) {
        current_producer = current_producer->next_endpoint;
    }

    current_producer->next_endpoint = next_producer;
}

void sac_pipeline_add_input_pipeline(sac_pipeline_t *pipeline, sac_pipeline_t *input_pipeline, sac_error_t *err)
{
    sac_endpoint_t *producer;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(input_pipeline == NULL, err, SAC_ERR_NULL_PTR, return);
    CHECK_ERROR(pipeline->input_pipeline[MAX_NB_OF_INPUTS - 1] != NULL, err, SAC_ERR_MAXIMUM_REACHED, return);

    producer = pipeline->producer;

    for (uint8_t i = 0; i < MAX_NB_OF_INPUTS; i++) {
        if (pipeline->input_pipeline[i] == NULL) {
            pipeline->input_pipeline[i] = input_pipeline;
            endpoint_link(input_pipeline->consumer, producer, err);
            return;
        }
        producer = producer->next_endpoint; /* TODO: Add error check. */
    }
}

void sac_pipeline_setup(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_processing_t *process;
    sac_endpoint_t *consumer;
    sac_endpoint_t *producer;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);

    process  = pipeline->process;
    consumer = pipeline->consumer;
    producer = pipeline->producer;

    /* Initialize processing stages. */
    while (process != NULL) {
        if (process->iface.init != NULL) {
            process->iface.init(process->instance, pipeline, &mem_pool, err);
            if (*err != SAC_ERR_NONE) {
                return;
            }
        }
        process = process->next_process;
    }

    /* Initialize audio queues. */
    init_audio_queues(pipeline, err);
    if (*err != SAC_ERR_NONE) {
        return;
    }

    /* Initialize stats. */
    pipeline->_statistics.producer_buffer_size = queue_get_limit(producer->_internal.queue);
    pipeline->_statistics.consumer_buffer_size = queue_get_limit(consumer->_internal.queue);
}

void sac_pipeline_produce(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_endpoint_t *producer;
    uint16_t size;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);

    producer = pipeline->producer;

    if (pipeline->producer->cfg.delayed_action) {
        if (producer->_internal.current_node != NULL) {
            /* Enqueue previous node. */
            enqueue_producer_node(pipeline, err);
            if (*err != SAC_ERR_NONE) {
                return;
            }
        }
        /* Start production of next node. */
        produce(pipeline, err);
        if (*err != SAC_ERR_NONE) {
            return;
        }
    } else {
        /* Start production of next node. */
        size = produce(pipeline, err);
        if (*err != SAC_ERR_NONE) {
            return;
        }
        if (size > 0) {
            /* Endpoint produced the node, so enqueue it. */
            enqueue_producer_node(pipeline, err);
            if (*err != SAC_ERR_NONE) {
                return;
            }
        } else {
            /* Error: producer returned no data, so free the current node. */
            queue_free_node(producer->_internal.current_node);
            producer->_internal.current_node = NULL;
            pipeline->_statistics.producer_packets_corrupted_count++;
        }
    }
}

void sac_pipeline_consume(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_endpoint_t *consumer;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);

    consumer = pipeline->consumer;

    if (consumer->cfg.delayed_action) {
        consume_delay(pipeline, consumer, err);
    } else {
        do {
            if (queue_get_length(consumer->_internal.queue) > 0) {
                consume_no_delay(pipeline, consumer, err);
            }
            consumer = consumer->next_endpoint;
        } while (consumer != NULL);
    }
}

void sac_pipeline_start(sac_pipeline_t *pipeline, sac_error_t *err)
{
    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);

    /*
     * If buffering is enabled, the consumer will only be started once the
     * consumer queue is about to be full. Otherwise, the consumer is started
     * as soon as a packet is in the queue.
     */
    pipeline->_internal.buffering_threshold = (pipeline->cfg.do_initial_buffering) ?
                                      pipeline->consumer->cfg.queue_size - 1 : 1;

    /* Start producing samples. */
    pipeline->producer->iface.start(pipeline->producer->instance);
}

void sac_pipeline_stop(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_endpoint_t *consumer;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);

    consumer = pipeline->consumer;

    /* Stop endpoints. */
    do {
        pipeline->consumer->iface.stop(consumer->instance);
        consumer = consumer->next_endpoint;
    } while (consumer != NULL);

    pipeline->producer->iface.stop(pipeline->producer->instance);

    /* Free current node. */
    queue_free_node(pipeline->producer->_internal.current_node);
    pipeline->producer->_internal.current_node = NULL;
}

uint32_t sac_processing_ctrl(sac_processing_t *sac_processing, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t arg, sac_error_t *err)
{
    *err = SAC_ERR_NONE;

    return sac_processing->iface.ctrl(sac_processing->instance, pipeline, cmd, arg, err);
}

void sac_pipeline_process(sac_pipeline_t *pipeline, sac_error_t *err)
{
    queue_node_t *node1;
    queue_node_t *node2;
    queue_node_t *temp_node;
    sac_endpoint_t *consumer;
    sac_endpoint_t *producer;
    uint8_t crc;

    *err = SAC_ERR_NONE;

    CHECK_ERROR(pipeline == NULL, err, SAC_ERR_NULL_PTR, return);

    consumer = pipeline->consumer;
    producer = pipeline->producer;

    /* Prevent the mixer to make the buffering before the mixing. */
    if ((!pipeline->cfg.mixer_option.input_mixer_pipeline) &&
        (!pipeline->cfg.mixer_option.output_mixer_pipeline)) {
        do {
            if (!consumer->_internal.buffering_complete) {
                if (queue_get_length(consumer->_internal.queue) >= (pipeline->_internal.buffering_threshold)) {
                    /* Buffering threshold reached. */
                    consumer->_internal.buffering_complete = true;
                    consumer->iface.start(consumer->instance);
                }
            }
            consumer = consumer->next_endpoint;
        } while (consumer != NULL);
    }

    /*
     * If it's a Mixing Pipeline get the mixed packet of all Output Producer Endpoints.
     * Otherwise, get the packet from the single producer endpoints.
     */
    if (pipeline->cfg.mixer_option.output_mixer_pipeline) {
        temp_node = queue_get_free_node(consumer->_internal.free_queue);
        node1 = start_mixing_process(pipeline, temp_node);
    } else {
        /* Get a node with audio samples that need processing from the producer queue. */
        node1 = queue_dequeue_node(pipeline->producer->_internal.queue);
        if (node1 == NULL) {
            *err = SAC_ERR_NO_SAMPLES_TO_PROCESS;
            return;
        }
    }

    /*
     * Check if payload size in audio header is what is expected. If not, packet may have
     * been corrupted. In which case, set it to expected value to avoid queue node overflow
     * when using this packet as data source for memcpy().
     */
    if (producer->cfg.use_encapsulation) {
        crc = sac_node_get_header(node1)->crc4;
        sac_node_get_header(node1)->crc4 = 0;
        sac_node_get_header(node1)->reserved = 0;
        if (crc4itu(0, (uint8_t *)sac_node_get_header(node1), sizeof(sac_header_t)) != crc) {
            /* Audio packet is corrupted, set it to a known value. */
            sac_node_set_payload_size(node1, producer->cfg.audio_payload_size);
            sac_node_get_header(node1)->fallback = 0;
            sac_node_get_header(node1)->tx_queue_level_high = 0;
            pipeline->_statistics.producer_packets_corrupted_count++;
        }
    }

    if (pipeline->process != NULL) {
        /* Apply all processing stages on audio packet. */
        node2 = process_samples(pipeline, node1, err);
        if (*err != SAC_ERR_NONE) {
            return;
        }
    } else {
        /* No processing to be done. */
        node2 = node1;
    }

    move_audio_packet_to_consumer_queue(pipeline, node2);

    /*
     * Start the Mixer Output Pipeline as soon as the first mixed audio packet is ready.
     * The Mixer Output Pipeline consumer will never be stopped after this point
     * since the mixer will always produce audio packets to be consumed.
     */
    if (pipeline->cfg.mixer_option.output_mixer_pipeline) {
        if (consumer->_internal.buffering_complete == false) {
            consumer->_internal.buffering_complete = true;
            consumer->iface.start(consumer->instance);
        }
    }
    queue_free_node(node2);
}

uint32_t sac_get_allocated_bytes(void)
{
    return mem_pool_get_allocated_bytes(&mem_pool);
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize audio queues.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
static void init_audio_queues(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_endpoint_t *consumer = pipeline->consumer;
    sac_endpoint_t *producer = pipeline->producer;
    uint16_t queue_data_inflation_size;
    uint16_t queue_data_size;
    uint8_t queue_size;

    *err = SAC_ERR_NONE;

    /* Calculate required queue data inflation size. */
    queue_data_inflation_size = SAC_NODE_PAYLOAD_SIZE_VAR_SIZE;
    queue_data_inflation_size += sizeof(sac_header_t);
    queue_data_inflation_size += CDC_QUEUE_DATA_SIZE_INFLATION;

    /* Calculate producer initial queue data size. */
    if (consumer->cfg.audio_payload_size > producer->cfg.audio_payload_size) {
        /*
         * If consumer queue is bigger than producer queue,
         * then the audio processing will require more space than initial size.
         */
        queue_data_size = consumer->cfg.audio_payload_size;
    } else {
        queue_data_size = producer->cfg.audio_payload_size;
    }
    queue_data_size += queue_data_inflation_size;
    queue_data_size += sac_align_data_size(queue_data_size, uint32_t); /* Align nodes on 32bits. */

    /*
     * Initialize producer queue.
     * The Mixer Output Pipeline producer queue will be initialized in
     * endpoint_link() as it shares the Mixer Input Pipeline consumer queue.
     */
    if (!pipeline->cfg.mixer_option.output_mixer_pipeline) {
        if (producer->cfg.queue_size < SAC_MIN_PRODUCER_QUEUE_SIZE) {
            producer->cfg.queue_size = SAC_MIN_PRODUCER_QUEUE_SIZE;
        }
        queue_size = producer->cfg.queue_size;
        init_audio_free_queue(producer, "Processing Free Queue",
                              queue_data_size, queue_size, err);
        if (*err != SAC_ERR_NONE) {
            return;
        }
        if (producer->cfg.delayed_action) {
            /* When using delayed action, one of the node will not be available. */
            queue_size--;
        }
        queue_init_queue(producer->_internal.queue, queue_size, "Processing Queue");
    }

    /* Calculate producer initial queue data size.  */
    queue_data_size = consumer->cfg.audio_payload_size;
    queue_data_size += queue_data_inflation_size;
    queue_data_size += sac_align_data_size(queue_data_size, uint32_t); /* Align nodes on 32bits. */

    /* Initialize consumer free queue. */
    queue_size = consumer->cfg.queue_size;
    if (pipeline->cfg.cdc_enable) {
        queue_size += CDC_QUEUE_SIZE_INFLATION;
    }

    /*
     * Since the Mixer Output Pipeline producer and the Mixer Input Pipeline consumer share the
     * same queue, the Mixer Output Pipeline does not have a dedicated producer queue for
     * audio processing. To account for that, their shared free queue has 3 extra nodes used
     * for audio processing.
     */
    if (pipeline->cfg.mixer_option.input_mixer_pipeline) {
        queue_size += 3;
    }

    init_audio_free_queue(consumer, "Audio Buffer Free Queue",
                          queue_data_size, queue_size, err);
    if (*err != SAC_ERR_NONE) {
        return;
    }

    /* Initialize consumer queues. */
    do {
        consumer->_internal.free_queue = pipeline->consumer->_internal.free_queue; /* All consumers use the same memory space. */
        queue_size = consumer->_internal.free_queue->limit;
        if (consumer->cfg.delayed_action) {
            /* When using delayed action, one of the node will not be available. */
            queue_size--;
        }
        queue_init_queue(consumer->_internal.queue, queue_size, "Audio Buffer");
        consumer = consumer->next_endpoint;
    } while (consumer != NULL);
}

/** @brief Initialize an audio free queue.
 *
 *  @param[in]  endpoint         Pointer to the queue's endpoint.
 *  @param[in]  queue_name       Name of the queue.
 *  @param[in]  queue_data_size  Size in bytes of the data in a node.
 *  @param[in]  queue_size       Number of nodes in the queue.
 *  @param[out] err              Error code.
 */
static void init_audio_free_queue(sac_endpoint_t *endpoint, const char *queue_name,
                                  uint16_t queue_data_size, uint8_t queue_size, sac_error_t *err)
{
    uint8_t *pool_ptr;

    *err = SAC_ERR_NONE;

    pool_ptr = mem_pool_malloc(&mem_pool, QUEUE_NB_BYTES_NEEDED(queue_size, queue_data_size));
    if (pool_ptr == NULL) {
        *err = SAC_ERR_NOT_ENOUGH_MEMORY;
        return;
    }
    endpoint->_internal.free_queue = mem_pool_malloc(&mem_pool, sizeof(queue_t));
    if (endpoint->_internal.free_queue == NULL) {
        *err = SAC_ERR_NOT_ENOUGH_MEMORY;
        return;
    }

    queue_init_pool(pool_ptr,
                    endpoint->_internal.free_queue,
                    queue_size,
                    queue_data_size,
                    queue_name);
}

/** @brief Copy data from a node of the producer queue to a node of the consumer queue.
 *
 *  @param[in] pipeline  Pipeline instance.
 *  @param[in] node1     Node from the producer queue.
 */
static void move_audio_packet_to_consumer_queue(sac_pipeline_t *pipeline, queue_node_t *node1)
{
    uint16_t length;
    queue_node_t *node2;
    sac_endpoint_t *consumer = pipeline->consumer;

    /* Detect overflow */
    do {
        if ((queue_get_length(consumer->_internal.queue) == queue_get_limit(consumer->_internal.queue)) &&
            (queue_get_length(consumer->_internal.free_queue) == 0)) {
            pipeline->_statistics.consumer_buffer_overflow_count++;
            node2 = queue_dequeue_node(consumer->_internal.queue);
            enter_critical();
            if (pipeline->cfg.mixer_option.output_mixer_pipeline) {
                for (uint8_t i = 0; i < MAX_NB_OF_INPUTS; i++) {
                    if (pipeline->input_pipeline[i] != NULL) {
                        pipeline->input_pipeline[i]->_internal.samples_buffered_size -= sac_node_get_payload_size(consumer->_internal.current_node);
                    }
                }
            } else {
                pipeline->_internal.samples_buffered_size -= sac_node_get_payload_size(node2); /* FIXME: This only works for a single consumer. */
            }
            exit_critical();
            queue_free_node(node2);
        }
        consumer = consumer->next_endpoint;
    } while (consumer != NULL);

    /* Move audio packet into a consumer node. */
    node2 = queue_get_free_node(pipeline->consumer->_internal.free_queue);
    memcpy(node2->data, node1->data, SAC_PACKET_HEADER_OFFSET + sizeof(sac_header_t) +
           sac_node_get_payload_size(node1));

    /* Enqueue node for all consumers. */
    consumer = pipeline->consumer;
    while (1) {
        queue_enqueue_node(consumer->_internal.queue, node2);
        enter_critical();
        pipeline->_internal.samples_buffered_size += sac_node_get_payload_size(node2); /* FIXME: This only works for a single consumer. */
        exit_critical();
        consumer = consumer->next_endpoint;
        if (consumer != NULL) {
            queue_inc_copy_count(node2);
        } else {
            break;
        }
    }
    length = queue_get_length(pipeline->consumer->_internal.queue);
    if (length > pipeline->_statistics.consumer_queue_peak_buffer_load) {
        pipeline->_statistics.consumer_queue_peak_buffer_load = length;
    }
}

/** @brief Apply all processing stages to a producer queue node.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  node1     Node from the producer queue.
 *  @param[out] err       Error code.
 *  @return Pointer to a producer queue node containing the processed data.
 */
static queue_node_t *process_samples(sac_pipeline_t *pipeline, queue_node_t *node1, sac_error_t *err)
{
    uint16_t rv;
    queue_node_t *node2, *node_tmp;
    sac_processing_t *process = pipeline->process;

    *err = SAC_ERR_NONE;

    /* Get a process destination node. */
    node2 = queue_get_free_node(pipeline->producer->_internal.free_queue);
    do {
        /* Execute gate function if present. */
        if ((process->iface.gate == NULL) || process->iface.gate(process->instance,
                                                                 pipeline,
                                                                 sac_node_get_header(node1),
                                                                 sac_node_get_data(node1),
                                                                 sac_node_get_payload_size(node1),
                                                                 err)) {
            if (*err != SAC_ERR_NONE) {
                return NULL;
            }

            /* node1 is the source node. */
            rv = process->iface.process(process->instance,
                                        pipeline,
                                        sac_node_get_header(node1),
                                        sac_node_get_data(node1),
                                        sac_node_get_payload_size(node1),
                                        sac_node_get_data(node2),
                                        err);
            if (*err != SAC_ERR_NONE) {
                return NULL;
            }
            if (rv != 0) { /* != 0 means processing happened. */
                /* Copy the header from the old source. */
                memcpy(sac_node_get_header(node2), sac_node_get_header(node1), sizeof(sac_header_t));
                /* Update the size. */
                sac_node_set_payload_size(node2, rv);
                /* Swap node1 and node2. */
                node_tmp = node1;
                node1 = node2;
                node2 = node_tmp;
            }
        }
        process = process->next_process;
    } while (process != NULL);
    queue_free_node(node2);

    return node1;
}

/** @brief Enqueue the current producer queue node.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 */
static void enqueue_producer_node(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_endpoint_t *producer = pipeline->producer;
    *err = SAC_ERR_NONE;

    if (producer->cfg.use_encapsulation) {
        /* If produced audio is encapsulated, save the payload size locally. */
        sac_node_set_payload_size(producer->_internal.current_node, sac_node_get_header(producer->_internal.current_node)->payload_size);
    }

    /*
     * Before enqueuing a node, make sure that the producer queue has a minimum
     * of SAC_MIN_PRODUCER_QUEUE_SIZE free nodes available.
     * If it does not, the processing hasn't started, so get rid of the oldest node
     * before enqueuing the new one.
     */
    if (queue_get_length(producer->_internal.queue) > (producer->cfg.queue_size - SAC_MIN_PRODUCER_QUEUE_SIZE)) {
        queue_free_node(queue_dequeue_node(producer->_internal.queue));
        pipeline->_statistics.producer_buffer_overflow_count++;
        *err = SAC_ERR_PRODUCER_Q_FULL;
    }
    queue_enqueue_node(producer->_internal.queue, producer->_internal.current_node);
    /* The current node is no longer been used by the producer. */
    producer->_internal.current_node = NULL;
}

/** @brief Get a free producer queue node and apply the producer endpoint action.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[out] err       Error code.
 *  @return The amount of bytes produces.
 */
static uint16_t produce(sac_pipeline_t *pipeline, sac_error_t *err)
{
    sac_endpoint_t *producer = pipeline->producer;
    uint8_t *payload;
    uint16_t payload_size;

    *err = SAC_ERR_NONE;

    producer->_internal.current_node = queue_get_free_node(producer->_internal.free_queue);
    if (producer->_internal.current_node == NULL) {
        pipeline->_statistics.producer_buffer_overflow_count++;
        *err = SAC_ERR_PRODUCER_Q_FULL;
        return 0;
    }

    payload_size = producer->cfg.audio_payload_size;
    if (producer->cfg.use_encapsulation) {
        payload = (uint8_t *)sac_node_get_header(producer->_internal.current_node);
        payload_size += sizeof(sac_header_t);
    } else {
        payload = (uint8_t *)sac_node_get_data(producer->_internal.current_node);
        sac_node_set_payload_size(producer->_internal.current_node, payload_size);
    }

    return producer->iface.action(producer->instance, payload, payload_size);
}

/** @brief Apply the consumer endpoint action on the current node.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  consumer  Pointer to the consumer endpoint.
 *  @param[out] err       Error code.
 *  @return The amount of bytes consumed.
 */
static uint16_t consume(sac_pipeline_t *pipeline, sac_endpoint_t *consumer, sac_error_t *err)
{
    uint8_t *payload;
    uint16_t payload_size, crc;

    *err = SAC_ERR_NONE;

    if (consumer->_internal.current_node == NULL) {
        pipeline->_statistics.consumer_buffer_underflow_count++;
        consumer->_internal.buffering_complete = false;
        *err = SAC_ERR_CONSUMER_Q_EMPTY;
        return 0;
    } else {
        payload_size = sac_node_get_payload_size(consumer->_internal.current_node);
        if (consumer->cfg.use_encapsulation) {
            payload = (uint8_t *)sac_node_get_header(consumer->_internal.current_node);
            /* Update audio header's payload size before sending the packet. */
            sac_node_get_header(consumer->_internal.current_node)->payload_size = (uint8_t)payload_size;
            payload_size += sizeof(sac_header_t);
            ((sac_header_t *)payload)->tx_queue_level_high = (queue_get_length(consumer->_internal.queue) < TX_QUEUE_HIGH_LEVEL) ? 0 : 1;

            /* Update CRC. */
            ((sac_header_t *)payload)->crc4 = 0;
            ((sac_header_t *)payload)->reserved = 0;
            crc = crc4itu(0, payload, sizeof(sac_header_t));
            ((sac_header_t *)payload)->crc4 = crc;
        } else {
            payload = sac_node_get_data(consumer->_internal.current_node);
        }
    }

    return consumer->iface.action(consumer->instance, payload, payload_size);
}

/** @brief Execute the specified not delayed action consumer endpoint.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  consumer  Consumer instance.
 *  @param[out] err       Error code.
 */
static void consume_no_delay(sac_pipeline_t *pipeline, sac_endpoint_t *consumer, sac_error_t *err)
{
    uint16_t size;
    queue_node_t *node;

    *err = SAC_ERR_NONE;

    if (consumer->_internal.buffering_complete == false) {
        *err = SAC_ERR_BUFFERING_NOT_COMPLETE;
        return;
    }

    /* Get the next node, if available, without dequeuing. */
    consumer->_internal.current_node = queue_get_node(consumer->_internal.queue);
    /* Start consumption of the node. */
    size = consume(pipeline, consumer, err);
    if (size > 0) {
        /* Consumed successfully, so dequeue and free. */
        node = queue_dequeue_node(consumer->_internal.queue);
        enter_critical();
        pipeline->_internal.samples_buffered_size -= sac_node_get_payload_size(node);
        exit_critical();
        queue_free_node(node);
    }
    consumer->_internal.current_node = NULL;
}

/** @brief Execute the specified delayed action consumer endpoint.
 *
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  consumer  Consumer instance.
 *  @param[out] err       Error code.
 */
static void consume_delay(sac_pipeline_t *pipeline, sac_endpoint_t *consumer, sac_error_t *err)
{
    *err = SAC_ERR_NONE;

    if (consumer->_internal.buffering_complete == false) {
        *err = SAC_ERR_BUFFERING_NOT_COMPLETE;
        return;
    }

    /* Free previous node. */
    queue_free_node(consumer->_internal.current_node);
    /* Get new node. */
    consumer->_internal.current_node = queue_dequeue_node(consumer->_internal.queue);
    if (consumer->_internal.current_node != NULL) {
        enter_critical();
        if (pipeline->cfg.mixer_option.output_mixer_pipeline) {
            for (uint8_t i = 0; i < MAX_NB_OF_INPUTS; i++) {
                if (pipeline->input_pipeline[i] != NULL) {
                    pipeline->input_pipeline[i]->_internal.samples_buffered_size -= sac_node_get_payload_size(consumer->_internal.current_node);
                }
            }
        } else {
            pipeline->_internal.samples_buffered_size -= sac_node_get_payload_size(consumer->_internal.current_node);
        }
        exit_critical();
    }
    /* Start consumption of new node. */
    consume(pipeline, consumer, err);
}

/** @brief Mix the producers' audio packet.
 *
 *  @param[in] pipeline  Pipeline instance.
 *  @param[in] node      Empty node.
 *  @return Pointer to a queue node containing the mixed samples.
 */
static queue_node_t *start_mixing_process(sac_pipeline_t *pipeline, queue_node_t *node)
{
    queue_node_t *temp_node;
    sac_endpoint_t *producer = pipeline->producer;
    uint8_t producer_index = 0;

    /* Loop on all the Output Producer Endpoints and load the Input Samples Queues. */
    do {
        /* Loop until there are enough samples to create an audio payload. */
        do {
            /* Verify if the producer has a packet. */
            if (queue_get_length(producer->_internal.queue) > 0) {
                /* If the queue has enough samples to make a payload no need to enqueue and append. */
                if (sac_mixer_module->input_samples_queue[producer_index].current_size < sac_mixer_module->cfg.payload_size) {
                    /* Dequeue the packet and append it to the Input Samples Queue. */
                    temp_node = queue_dequeue_node(producer->_internal.queue);
                    sac_mixer_module_append_samples(&sac_mixer_module->input_samples_queue[producer_index],
                                                    sac_node_get_data(temp_node),
                                                    sac_node_get_payload_size(temp_node));
                    queue_free_node(temp_node);
                }
            } else {
                /* If no packet is present, append silent samples to the Input Samples Queue. */
                uint8_t silent_samples_size = sac_mixer_module->cfg.payload_size - sac_mixer_module->input_samples_queue[producer_index].current_size;

                sac_mixer_module_append_silence(&sac_mixer_module->input_samples_queue[producer_index], silent_samples_size);

                enter_critical();
                pipeline->input_pipeline[producer_index]->_internal.samples_buffered_size += silent_samples_size;
                exit_critical();
            }
        } while (sac_mixer_module->input_samples_queue[producer_index].current_size < sac_mixer_module->cfg.payload_size);

        producer_index++;
        producer = producer->next_endpoint;
    } while (producer != NULL);

    /* Once the Input Samples Queues are filled, mix them into the Output Packet Queue. */
    sac_mixer_module_mix_packets(sac_mixer_module);

    /* Apply the Output Packet to the node and return it to the processing stage. */
    memcpy(sac_node_get_data(node), sac_mixer_module->output_packet_buffer, sac_mixer_module->cfg.payload_size);
    sac_node_set_payload_size(node, sac_mixer_module->cfg.payload_size);

    return node;
}

/** @brief Link the queue of two endpoints.
 *
 *  This is used for audio mixing, where the consumer's queue from
 *  an input pipeline must be linked to a producer's queue from
 *  the output pipeline.
 *
 *  @param[in]  consumer  Consumer endpoint from the input pipeline.
 *  @param[in]  producer  Producer endpoint from the output pipeline.
 *  @param[out] err       Error code.
 */
static void endpoint_link(sac_endpoint_t *consumer, sac_endpoint_t *producer, sac_error_t *err)
{
    *err = SAC_ERR_NONE;

    if (consumer == NULL || producer == NULL) {
        *err = SAC_ERR_NULL_PTR;
    } else {
        producer->_internal.queue = consumer->_internal.queue;
        producer->_internal.free_queue = consumer->_internal.free_queue;
    }
}
