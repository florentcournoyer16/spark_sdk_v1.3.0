/** @file  sac_fallback_module.h
 *  @brief SPARK Audio Core Fallback Module is used to manage audio fallback.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_FALLBACK_MODULE_H_
#define SAC_FALLBACK_MODULE_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "mem_pool.h"
#include "sac_api.h"
#include "sac_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
/*! Array size holding buffer load values to calculate a rolling average */
#define SAC_FALLBACK_QUEUE_ARRAY_LENGTH 3

/** @brief The SPARK Audio Core Fallback Module States.
 */
typedef enum sac_fallback_state {
    /*! Normal state, monitor the TX audio buffer to switch to wait threshold. */
    SAC_FB_STATE_NORMAL,
    /*! Link degrading, measure link margin for ~250 ms to determine return from fallback threshold. */
    SAC_FB_STATE_WAIT_THRESHOLD,
    /*! Fallback mode due to degraded link, monitor link margin to return to normal mode. */
    SAC_FB_STATE_FALLBACK,
    /*! Fallback mode due to disconnected link, threshold set to fixed value,
     *   monitor link margin to return to normal mode.
     */
    SAC_FB_STATE_FALLBACK_DISCONNECT
} sac_fallback_state_t;

/* TYPES **********************************************************************/
/** @brief The SPARK Audio Core Fallback Module Configuration.
 */
typedef struct sac_fallback_module_cfg {
    /*! Pipeline on which the fallback module is instantiated. */
    sac_pipeline_t *pipeline;
    /*! Set to true if instantiated for an audio transmitting pipeline. */
    bool is_tx_device;
    /*! Default average link margin threshold to allow disabling fallback. */
    uint8_t link_margin_threshold;
    /*! Link margin threshold hysteresis. */
    uint8_t link_margin_threshold_hysteresis;
    /*! Amount of time in seconds the link margin must be higher than the threshold
     *   to allow disabling fallback.
     */
    uint32_t link_margin_good_time_s;
    /*! Maximum number of CCA tries possible on this connection. */
    uint16_t cca_max_try_count;
    /*! Average CCA try count threshold in percent of the maximum number of
     *   CCA tries possible on this connection.
     */
    uint8_t cca_try_count_threshold_perc;
    /*! Amount of time in seconds the CCA try count must be higher than the threshold
     *   to allow disabling fallback.
     */
    uint32_t cca_good_time_s;
    /*! Audio transmitting pipeline consumer buffer load above which fallback is triggered.
     *   Value should be multiplied by 10. (ex: 1.3 is 13).
     */
    uint32_t consumer_buffer_load_threshold_tenths;
    /*! Number of used packets per second on the connection sending link margin information.
     *   Ex: A connection with a capacity of 3000 packets / second and 25% retransmission ratio
             will use (1 - 0.25) * 3000 = 2250 packet per seconds.
     */
    uint32_t link_margin_conn_packets_per_second;
    /*! Number of used packets per second on the connection sending audio.
     *   Ex: A connection with a capacity of 3000 packets / second and 25% retransmission ratio
             will use (1 - 0.25) * 3000 = 2250 packet per seconds.
     */
    uint32_t main_conn_packets_per_second;
    /*! Callback function called on a fallback state change. */
    void (*fallback_state_change_callback)(bool enabled);
} sac_fallback_module_cfg_t;

/** @brief The SPARK Audio Core Fallback Module Queue Metrics.
 */
typedef struct sac_fallback_queue_metrics {
    /*! Queue length averaging array. */
    uint8_t  queue_length_arr[SAC_FALLBACK_QUEUE_ARRAY_LENGTH];
    /*! Queue length averaging array index. */
    uint8_t  queue_length_arr_idx;
    /*! Queue length rolling average sum. */
    uint16_t queue_length_sum;
    /*! Queue length rolling average. */
    uint8_t  queue_length_avg;
} sac_fallback_queue_metrics_t;

/** @brief The SPARK Audio Core Fallback Module Link Margin Metrics.
 */
typedef struct sac_fallback_link_margin_metrics {
    /*! Number of packets per 250 ms period on the connection sending link margin information. */
    uint16_t packets_per_period;
    /*! Current link margin threshold to return to normal. */
    uint8_t  threshold;
    /*! Default link margin threshold. */
    uint8_t  threshold_default;
    /*! Link margin threshold hysteresis. */
    uint8_t  threshold_hysteresis;
    /*! Accumulation of link margin values over a nominal 250 ms. */
    uint32_t accumulator;
    /*! Number of link margin values accumulated. */
    uint16_t accumulator_count;
    /*! Link margin average in 250 ms calculated from accumulator. */
    uint8_t  accumulator_average;
    /*! Number of 250 ms continuous periods where the link margin values were above the threshold. */
    uint8_t  good_count;
    /*! Number of 250 ms continuous periods where the link margin must be higher than
     *   the threshold to allow disabling fallback.
     */
    uint16_t good_count_threshold;
    /*! Flag indicating that rx_link_margin_data data is valid. */
    bool rx_data_valid;
    /*! Link margin data from the node. */
    uint8_t rx_data;
} sac_fallback_link_margin_metrics_t;

/** @brief The SPARK Audio Core Fallback Module CCA Metrics.
 */
typedef struct sac_fallback_cca_metrics {
    /*! Number CCA fail count allowed in 250 ms. */
    uint32_t fail_count_threshold;
    /*! CCA fail count value at the start of the fallback processing. */
    uint32_t fail_count_start;
    /*! Current CCA fail count value. */
    uint32_t fail_count_current;
    /*! Number of CCA fails in 250 ms. */
    uint32_t fail_count_value;
    /*! Number of 250 ms continuous periods where the CCA fail count value was below the threshold. */
    uint16_t good_count;
    /*! Number of 250 ms continuous periods where the CCA try count must be higher than
     *   the threshold to allow disabling fallback.
     */
    uint16_t good_count_threshold;
} sac_fallback_cca_metrics_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Get the default values of the SPARK Audio Core Fallback Module.
 *
 *  @note These parameters still need to be set by user on the tx device:
 *         - pipeline
 *         - is_tx_device
 *         - link_margin_conn_packets_per_second
 *         - main_conn_packets_per_second
 *        Other parameters should be reviewed by user but will work with their default value.
 *
 *  @return Default fallback configuration structure.
 */
sac_fallback_module_cfg_t sac_fallback_module_get_defaults(void);

/** @brief Initialize the SPARK Audio Core Fallback Module.
 *
 *  @param[in]  cfg       Fallback configuration.
 *  @param[in]  mem_pool  Memory pool.
 *  @param[out] err       Error code.
 */
void sac_fallback_module_init(sac_fallback_module_cfg_t cfg, mem_pool_t *mem_pool, sac_error_t *err);

/** @brief Set the time the link margin has to be good to get out of fallback mode.
 *
 *  @param[in] time  Time in seconds.
 */
void sac_fallback_set_link_margin_good_time(uint32_t time);

/** @brief Set the time the CCA has to be good to get out of fallback mode.
 *
 *  @param[in] time  Time in seconds.
 */
void sac_fallback_set_cca_good_time(uint32_t time);

/** @brief Set the rx'ed rx link margin value from the node.
 *
 *  @param[in] rx_lm  RX link margin value.
 */
void sac_fallback_set_rx_link_margin(uint8_t rx_lm);

/** @brief Function for coordinator to update the fallback state machine. Called from inside the main background loop.
 */
void sac_fallback_update_state(void);

/** @brief Update the txq and lm stats.
 *
 *  @param[in] cca_fail_count  CCA fail count.
 */
void sac_fallback_update_stats(uint32_t cca_fail_count);

/** @brief Return status of fallback flag.
 *
 *  @retval true If active.
 *  @retval false If not active.
 */
bool sac_fallback_is_active(void);

/** @brief Set fallback flag.
 */
void sac_fallback_set_fallback_flag(void);

/** @brief Clear fallback flag.
 */
void sac_fallback_clear_fallback_flag(void);

/** @brief Get fallback count.
 *
 *  @return Fallback count.
 */
uint32_t sac_fallback_get_fallback_count(void);

/** @brief Get CCA metrics.
 *
 *  @return Pointer to CCA metrics.
 */
sac_fallback_cca_metrics_t *sac_fallback_get_cca_metrics(void);

/** @brief Get RX link margin.
 *
 *  @return RX link margin.
 */
uint8_t sac_fallback_get_rx_link_margin(void);

/** @brief Format the fallback statistics as a string of characters.
 *
 *  @param[out] buffer    Buffer where to put the formatted string.
 *  @param[in]  size      Size of the buffer.
 *  @return The formatted string length, excluding the NULL terminator.
 */
int sac_fallback_module_format_stats(char *buffer, uint16_t size);

/** @brief Reset fallback statistics.
 */
void sac_fallback_module_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* SAC_FALLBACK_MODULE_H_ */

