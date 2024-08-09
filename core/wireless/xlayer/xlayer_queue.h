/** @file xlayer_queue.h
 *  @brief Queue management.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef XLAYER_QUEUE_H_
#define XLAYER_QUEUE_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "xlayer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define HEADER_MAX_SIZE               10
#define XLAYER_QUEUE_EXTRA_NODES      2
#define XLAYER_QUEUE_LIMIT_UNLIMITED  0xFFFF
#define XLAYER_QUEUE_NB_BYTES_NEEDED(num_nodes, data_size) ((num_nodes + XLAYER_QUEUE_EXTRA_NODES) * \
                                                ((sizeof(xlayer_queue_node_t) + (HEADER_MAX_SIZE + data_size))))

/* TYPES **********************************************************************/
/** @brief Cross layer queue node.
 */
typedef struct xlayer_queue_node {
    uint8_t           *data;                     /*!< Pointer to data */
    xlayer_t           xlayer;                   /*!< Cross layer structure */
    struct xlayer_queue      *home_xlayer_queue; /*!< Pointer to free queue */
    struct xlayer_queue_node *next;              /*!< Pointer to next node */
    uint8_t           copy_count;                /*!< Node copy count */
} xlayer_queue_node_t;

/** @brief Cross layer queue.
 */
typedef struct xlayer_queue {
    xlayer_queue_node_t *head;              /*!< Pointer to head */
    xlayer_queue_node_t *tail;              /*!< Pointer to tail */
    uint16_t     size;                      /*!< Queue size */
    uint16_t     max_size;                  /*!< Queue maximum size */
    bool         free_xlayer_queue_type;    /*!< Free queue type flag */
    const char   *q_name;                   /*!< Queue name */
    struct xlayer_queue *prev_xlayer_queue; /*!< Pointer to previous queue */
} xlayer_queue_t;

/** @brief Cross layer queue stats.
 */
typedef struct xlayer_queue_stats {
    uint16_t xlayer_queue_size;      /*!< Queue size */
    uint16_t xlayer_queue_max_size;  /*!< Queue maximum size */
    char     *xlayer_queue_name;     /*!< Queue name */
    bool     xlayer_queue_free_type; /*!< Free queue type flag */
 } xlayer_queue_stats_t;

/* MACROS *********************************************************************/
/** @brief Return pointer to data + offset for the specified node.
 *
 *  @param[in] _node    Specified node.
 *  @param[in] _offset  Offset added to the data pointer.
 */
#define xlayer_queue_get_data_ptr(_node, _offset) (&(_node)->data[_offset])

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize a new node pool.
 *
 *  @param[in] pool                   Pool containing nodes and data.
 *  @param[in] new_free_xlayer_queue  Queue where new nodes will be stored.
 *  @param[in] num_nodes              Number of nodes in this free pool.
 *  @param[in] data_size              Data size of each node.
 *  @param[in] xlayer_queue_name      Queue name.
 *  @return Amount of memory consumed by this pool. This value rounded up to the nearest 4-byte multiple.
 */
uint32_t xlayer_queue_init_pool(uint8_t *pool, xlayer_queue_t *new_free_xlayer_queue, uint16_t num_nodes,
                                uint16_t data_size, const char *xlayer_queue_name);

/** @brief Initialize a new xlayer_queue.
 *
 *  @param[in] xlayer_queue   Queue to be initialized.
 *  @param[in] max_size   Queue maximum size.
 *  @param[in] q_name  Queue name.
 */
void xlayer_queue_init_queue(xlayer_queue_t *xlayer_queue, uint16_t max_size, const char *q_name);

/** @brief Get a free buffer from the xlayer_queue.
 *
 *  @param[in] xlayer_queue  Queue containing free nodes.
 *  @return Address of the free node, or NULL if xlayer_queue is empty.
 */
xlayer_queue_node_t *xlayer_queue_get_free_node(xlayer_queue_t *xlayer_queue);

/** @brief Return node to its free xlayer_queue.
 *
 *  @param[in] node  Node to be freed.
 */
void xlayer_queue_free_node(xlayer_queue_node_t *node);

/** @brief Get a node from a xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @return Address of the node.
 */
xlayer_queue_node_t *xlayer_queue_dequeue_node(xlayer_queue_t *xlayer_queue);

/** @brief Add a node to a xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @param[in] node   Address of the node.
 *  @return true if the node was successfully enxlayer_queued, false otherwise.
 */
bool xlayer_queue_enqueue_node(xlayer_queue_t *xlayer_queue, xlayer_queue_node_t *node);

/** @brief Replace the head of a xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @param[in] node   Address of the node.
 *  @return true if the head was successfully replaced, false otherwise.
 */
bool xlayer_queue_enqueue_at_head(xlayer_queue_t *xlayer_queue, xlayer_queue_node_t *node);

/** @brief Get the address of the head node without removing it from the xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @return Address of the node.
 */
xlayer_queue_node_t *xlayer_queue_get_node(xlayer_queue_t *xlayer_queue);

/** @brief Get the size of desired xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @return Size of the xlayer_queue.
 */
uint16_t xlayer_queue_get_size(xlayer_queue_t *xlayer_queue);

/** @brief Get the maximum size of desired xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @return Maximum size of the xlayer_queue.
 */
uint16_t xlayer_queue_get_max_size(xlayer_queue_t *xlayer_queue);

/** @brief Get free space desired xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue.
 *  @return Number of free space in the xlayer_queue.
 */
uint16_t xlayer_queue_get_free_space(xlayer_queue_t *xlayer_queue);

/** @brief Free any existing nodes in the xlayer_queue.
 *
 *  @param[in] xlayer_queue  Desired xlayer_queue to flush.
 */
void xlayer_queue_flush(xlayer_queue_t *xlayer_queue_to_flush);

/** @brief Unlink the xlayer_queue from the linked list of xlayer_queues.
 *
 *  @param[in] xlayer_queue_to_flush  Desired xlayer_queue.
 */
void xlayer_queue_unlink(xlayer_queue_t *xlayer_queue);

/** @brief Get the xlayer_queue statistics.
 *
 *  @param[in]  first        True for first xlayer_queue stats, then false for the remaining.
 *  @param[out] xlayer_queue_stats  Location for requested stats.
 *  @return True if valid stats returned, false if no more xlayer_queues.
 */
bool xlayer_queue_get_stats(bool first, xlayer_queue_stats_t *xlayer_queue_stats);

/** @brief Increment the copy count value of a node.
 *
 *  This is to indicate that the node is in another xlayer_queue
 *  beyond the first and that it should not be freed until
 *  every user is done with it.
 *
 *  @param[in] node  Address of the node.
 */
void xlayer_queue_inc_copy_count(xlayer_queue_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* XLAYER_QUEUE_H_ */
