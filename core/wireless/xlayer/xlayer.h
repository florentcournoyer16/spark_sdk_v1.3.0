/** @file  xlayer.h
 *  @brief SPARK cross layer queue.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef XLAYER_H
#define XLAYER_H

/* INCLUDES *******************************************************************/
#include "link_cca.h"
#include "link_gain_loop.h"
#include "link_phase.h"
#include "sr_def.h"
#include "sr_spectral.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief Cross layer callback structure.
 */
typedef struct xlayer_callback {
    void (*callback)(void *parg); /*!< Function called when the frame is fully processed */
    void *parg_callback;          /*!< callback void pointer argument*/
} xlayer_callback_t;

/** @brief Cross layer configuration - internal WPS use.
 */
typedef struct xlayer_cfg_internal {
    /* Layer 2 */
    bool expect_ack;              /*!< Expect ACK? */

    /* Layer 1 */
    modulation_t modulation;                       /*!< modulation */
    fec_level_t fec;                               /*!< FEC level */
    rf_channel_t *channel;                         /*!< Current channel information */
    gain_loop_t *gain_loop;                        /*!< Gain loop */
    packet_cfg_t packet_cfg;                       /*!< Packet configuration */
    uint16_t power_up_delay;                       /*!< Power up delay */
    uint16_t rx_timeout;                           /*!< RX timeout */
    uint32_t sleep_time;                           /*!< Sleep time in PLL cycles */
    uint16_t rx_wait_time;                         /*!< RX wait time */
    uint8_t rx_constgain;                          /*!< Receiver constant gain */
    uint8_t cca_threshold;                         /*!< Clear Channel Assessment threshold */
    uint16_t cca_retry_time;                       /*!< CCA retry time */
    uint8_t cca_max_try_count;                     /*!< CCA max try count */
    uint8_t phase_offset[PHASE_OFFSET_BYTE_COUNT]; /*!< Phase offset */
    cca_fail_action_t cca_fail_action;             /*!< CCA fail action */
    uint8_t cca_try_count;                         /*!< CCA try count */
    uint32_t rnsi_raw;                             /*!< RNSI in 1/10 dB */
    uint32_t rssi_raw;                             /*!< RSSI in 1/10 dB */
    sleep_lvl_t sleep_level;                       /*!< Sleep Level */
    phase_info_t *phases_info;                     /*!< phase info */
    uint8_t rx_cca_retry_count;                    /*!< RX CCA retry count */
    isi_mitig_t isi_mitig;                         /*!< ISI mitigation level */
    bool certification_header_en;                  /*!< Certification header usage flag */

    /* Callback */
    xlayer_callback_t callback_main;              /*!< Main connection callback */
    xlayer_callback_t callback_auto;              /*!< Main connection callback */

} xlayer_cfg_internal_t;

/** @brief Cross layer configuration.
 */
typedef struct xlayer_cfg {

    uint32_t rnsi_raw;         /*!< RNSI in 1/10 dB */
    uint32_t rssi_raw;         /*!< RSSI in 1/10 dB */
    phase_info_t phases_info;  /*!< phase info */

} xlayer_cfg_t;

/** @brief Cross layer frame.
 */
typedef struct xlayer_frame {
    /* Layer 2 */
    uint16_t source_address;      /*!< Source address */
    uint16_t destination_address; /*!< Destination address */

    /* Frame */
    uint8_t *header_memory;     /*!< Header's buffer memory, point to index 0 */
    uint8_t header_memory_size; /*!< Header's buffer size */
    uint8_t *header_begin_it;   /*!< Header's begin iterator */
    uint8_t *header_end_it;     /*!< Header's end iterator */
    uint64_t time_stamp;        /*!< Frame's timestamps */
    uint16_t retry_count;       /*!< Frame's retry count */

    uint8_t *payload_memory;     /*!< Payload's buffer memory, point to index 0 */
    uint8_t payload_memory_size; /*!< Payload's buffer size */
    uint8_t *payload_begin_it;   /*!< Payload's begin iterator */
    uint8_t *payload_end_it;     /*!< Payload's end iterator */

    frame_outcome_t frame_outcome; /*!< Frame outcome */
} xlayer_frame_t;

/** @brief Cross layer.
 */
typedef struct xlayer {
    xlayer_cfg_t   config; /*!< Configuration */
    xlayer_frame_t frame;  /*!< Frame */
} xlayer_t;

#ifdef __cplusplus
}
#endif

#endif /* XLAYER_H */
