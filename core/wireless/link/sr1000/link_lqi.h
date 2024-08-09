/** @file link_lqi.h
 *  @brief LQI module.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_LQI_H_
#define LINK_LQI_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include <string.h>
#include "link_gain_loop.h"
#include "link_utils.h"
#include "sr1000_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
typedef enum lqi_mode {
    LQI_MODE_0, /**< Consider lost and rejected frames as having weakest RSSI possible and typical RNSI */
    LQI_MODE_1, /**< Don't consider rejected and lost frames in RSSI and RNSI calculation */
} lqi_mode_t;

typedef struct lqi {
    lqi_mode_t mode;              /**< LQI object mode */
    uint64_t rssi_total_tenth_db; /**< RSSI in tenths of dB */
    uint64_t rnsi_total_tenth_db; /**< RNSI in tenths of dB */
    uint32_t sent_count;          /**< Sent frame count */
    uint32_t ack_count;           /**< ACKed frame count */
    uint32_t nack_count;          /**< NACKed frame count */
    uint32_t received_count;      /**< Received frame count */
    uint32_t rejected_count;      /**< Rejected frame count */
    uint32_t lost_count;          /**< Lost frame count */
    uint32_t total_count;         /**< Total frame count */
    uint8_t inst_rssi;            /**< Instantaneous RSSI measurement */
    uint8_t inst_rnsi;            /**< Instantaneous RNSI measurement */
    uint16_t inst_rssi_tenth_db;  /**< Instantaneous RSSI measurement tenths of dB*/
    uint16_t inst_rnsi_tenth_db;  /**< Instantaneous RNSI measurement tenths of dB*/
} lqi_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize LQI object.
 *
 *  @param[in] lqi  LQI object.
 *  @param[in] mode LQI mode.
 *  @return None.
 */
void link_lqi_init(lqi_t *lqi, lqi_mode_t mode);

/** @brief Get sent frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return Sent frame count.
 */
static inline uint32_t link_lqi_get_sent_count(lqi_t *lqi)
{
    return lqi->sent_count;
}

/** @brief Get acked frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return ACKed frame count.
 */
static inline uint32_t link_lqi_get_ack_count(lqi_t *lqi)
{
    return lqi->ack_count;
}

/** @brief Get nacked frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return NACKed frame count.
 */
static inline uint32_t link_lqi_get_nack_count(lqi_t *lqi)
{
    return lqi->nack_count;
}

/** @brief Get received frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return Received frame count.
 */
static inline uint32_t link_lqi_get_received_count(lqi_t *lqi)
{
    return lqi->received_count;
}

/** @brief Get rejected frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return Rejected frame count.
 */
static inline uint32_t link_lqi_get_rejected_count(lqi_t *lqi)
{
    return lqi->rejected_count;
}

/** @brief Get lost frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return Lost frame count.
 */
static inline uint32_t link_lqi_get_lost_count(lqi_t *lqi)
{
    return lqi->lost_count;
}

/** @brief Get total frame count.
 *
 *  @param[in] lqi  LQI object.
 *  @return Total frame count.
 */
static inline uint32_t link_lqi_get_total_count(lqi_t *lqi)
{
    return lqi->total_count;
}

/** @brief Get RSSI average.
 *
 *  @param[in] lqi  LQI object.
 *  @return RSSI average.
 */
uint16_t link_lqi_get_avg_rssi_tenth_db(lqi_t *lqi);

/** @brief Get RNSI average.
 *
 *  @param[in] lqi  LQI object.
 *  @return RNSI average.
 */
uint16_t link_lqi_get_avg_rnsi_tenth_db(lqi_t *lqi);

/** @brief Get the last received RNSI measurement.
 *
 *  @param[in] lqi  LQI object.
 *  @return Last received RNSI measurement.
 */
static inline uint16_t link_lqi_get_inst_rnsi(lqi_t *lqi)
{
    return lqi->inst_rnsi;
}

/** @brief Get the last received RNSI measurement in tenths of dB.
 *
 *  @param[in] lqi  LQI object.
 *  @return Last received RNSI measurement in tenths of dB.
 */
static inline uint16_t link_lqi_get_inst_rnsi_tenth_db(lqi_t *lqi)
{
    return lqi->inst_rnsi_tenth_db;
}

/** @brief Get the last received RSSI measurement.
 *
 *  @param[in] lqi  LQI object.
 *  @return Last received RSSI measurement.
 */
static inline uint16_t link_lqi_get_inst_rssi(lqi_t *lqi)
{
    return lqi->inst_rssi;
}

/** @brief Get the last received RSSI measurement in tenths of dB.
 *
 *  @param[in] lqi  LQI object.
 *  @return Last received RSSI measurement in tenths of dB.
 */
static inline uint16_t link_lqi_get_inst_rssi_tenth_db(lqi_t *lqi)
{
    return lqi->inst_rssi_tenth_db;
}

/** @brief Get instantaneous phase offset data (SR11XX feature only).
 *
 *  @param[in] lqi    LQI object.
 *  @param[in] index  Index.
 *  @return Phase offset data.
 */
static inline uint8_t link_lqi_get_inst_phase_offset(lqi_t *lqi, uint8_t index)
{
    (void)lqi;
    (void)index;

    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Update LQI mode 0 (Consider lost and rejected frames as having weakest RSSI possible and typical RNSI).
 *
 *  @param[in] frame_outcome  Outcome of the frame.
 *  @param[in] rssi           Receiver signal strength indicator.
 *  @param[in] rnsi           Receiver noise strength indicator.
 *  @param[in] gain_index     Gain index.
 *  @param[in] lqi            LQI object.
 *  @return None.
 */
static inline void link_lqi_update_mode_0(lqi_t *lqi, uint8_t gain_index, frame_outcome_t frame_outcome, uint8_t rssi, uint8_t rnsi)
{
    lqi->total_count++;
    switch (frame_outcome) {
    case FRAME_RECEIVED:
        lqi->received_count++;
        lqi->inst_rnsi          = rnsi;
        lqi->inst_rssi          = rssi;
        lqi->inst_rnsi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rnsi);
        lqi->inst_rssi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rssi);
        lqi->rssi_total_tenth_db += lqi->inst_rssi_tenth_db;
        lqi->rnsi_total_tenth_db += lqi->inst_rnsi_tenth_db;
        break;
    case FRAME_REJECTED:
        lqi->rejected_count++;
        lqi->rssi_total_tenth_db += link_gain_loop_get_min_tenth_db(gain_index);
        lqi->rnsi_total_tenth_db += link_gain_loop_get_rnsi_tenth_db(gain_index);
        break;
    case FRAME_LOST:
        lqi->lost_count++;
        lqi->rssi_total_tenth_db += link_gain_loop_get_min_tenth_db(gain_index);
        lqi->rnsi_total_tenth_db += link_gain_loop_get_rnsi_tenth_db(gain_index);
        break;
    case FRAME_SENT_ACK:
        lqi->sent_count++;
        lqi->ack_count++;
        lqi->received_count++;
        lqi->inst_rnsi          = rnsi;
        lqi->inst_rssi          = rssi;
        lqi->inst_rnsi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rnsi);
        lqi->inst_rssi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rssi);
        lqi->rssi_total_tenth_db += lqi->inst_rssi_tenth_db;
        lqi->rnsi_total_tenth_db += lqi->inst_rnsi_tenth_db;
        break;
    case FRAME_SENT_ACK_LOST:
    case FRAME_SENT_ACK_REJECTED:
        lqi->sent_count++;
        lqi->nack_count++;
        lqi->rssi_total_tenth_db += link_gain_loop_get_min_tenth_db(gain_index);
        lqi->rnsi_total_tenth_db += link_gain_loop_get_rnsi_tenth_db(gain_index);
        break;
    case FRAME_WAIT:
        lqi->sent_count++;
    default:
        break;
    }
}

/** @brief Update LQI mode 1 (Don't consider rejected and lost frames in RSSI and RNSI calculation).
 *
 *  @param[in] frame_outcome  Outcome of the frame.
 *  @param[in] rssi           Receiver signal strength indicator.
 *  @param[in] rnsi           Receiver noise strength indicator.
 *  @param[in] gain_index     Gain index.
 *  @param[in] lqi            LQI object.
 *  @return None.
 */
static inline void link_lqi_update_mode_1(lqi_t *lqi, uint8_t gain_index, frame_outcome_t frame_outcome, uint8_t rssi, uint8_t rnsi)
{
    lqi->total_count++;
    switch (frame_outcome) {
    case FRAME_RECEIVED:
        lqi->received_count++;
        lqi->inst_rnsi          = rnsi;
        lqi->inst_rssi          = rssi;
        lqi->inst_rnsi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rnsi);
        lqi->inst_rssi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rssi);
        lqi->rssi_total_tenth_db += lqi->inst_rssi_tenth_db;
        lqi->rnsi_total_tenth_db += lqi->inst_rnsi_tenth_db;
        break;
    case FRAME_REJECTED:
        lqi->rejected_count++;
        break;
    case FRAME_LOST:
        lqi->lost_count++;
        break;
    case FRAME_SENT_ACK:
        lqi->sent_count++;
        lqi->ack_count++;
        lqi->received_count++;
        lqi->inst_rnsi          = rnsi;
        lqi->inst_rssi          = rssi;
        lqi->inst_rnsi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rnsi);
        lqi->inst_rssi_tenth_db = calculate_normalized_gain(link_gain_loop_get_min_tenth_db(gain_index), rssi);
        lqi->rssi_total_tenth_db += lqi->inst_rssi_tenth_db;
        lqi->rnsi_total_tenth_db += lqi->inst_rnsi_tenth_db;
        break;
    case FRAME_SENT_ACK_LOST:
    case FRAME_SENT_ACK_REJECTED:
        lqi->sent_count++;
        lqi->nack_count++;
        break;
    case FRAME_WAIT:
        lqi->sent_count++;
    default:
        break;
    }
}

/** @brief Reset LQI object.
 *
 *  @param[in] lqi  LQI object.
 *  @return None.
 */
static inline void link_lqi_reset(lqi_t *lqi)
{
    memset(lqi, 0, sizeof(lqi_t));
}

/** @brief Update LQI.
 *
 *  @param[in] frame_outcome  Outcome of the frame.
 *  @param[in] rssi           Receiver signal strength indicator.
 *  @param[in] rnsi           Receiver noise strength indicator.
 *  @param[in] gain_index     Gain index.
 *  @param[in] lqi            LQI object.
 *  @return None.
 */
static inline void link_lqi_update(lqi_t *lqi, uint8_t gain_index, frame_outcome_t frame_outcome, uint8_t rssi, uint8_t rnsi,
                                   uint8_t *phase_offset)
{
    (void)phase_offset;
    switch (lqi->mode) {
    case LQI_MODE_0:
        link_lqi_update_mode_0(lqi, gain_index, frame_outcome, rssi, rnsi);
        break;
    case LQI_MODE_1:
        link_lqi_update_mode_1(lqi, gain_index, frame_outcome, rssi, rnsi);
        break;
    default:
        link_lqi_update_mode_0(lqi, gain_index, frame_outcome, rssi, rnsi);
    }

    /* Total count overflow */
    if (!lqi->total_count) {
        link_lqi_reset(lqi);
    }
}

#ifdef __cplusplus
}
#endif
#endif /* LINK_LQI_H_ */
