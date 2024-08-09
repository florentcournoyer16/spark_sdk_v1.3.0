/** @file  sac_fallback_module.c
 *  @brief SPARK Audio Core Fallback Module is used to manage audio fallback.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_fallback_module.h"
#include <stddef.h>
#include <stdio.h>
#include "sac_api.h"

/* CONSTANTS ******************************************************************/
/*! Decimal factor used for tx queue length calculation. */
#define BUF_SIZE_DECIMAL_FACTOR   10
/*! Number of sampling intervals in 1 second. */
#define SAMPLING_INTERVAL_DIVISOR 4

/* TYPES **********************************************************************/
/** @brief The SPARK Audio Core Fallback Module Instance.
 */
typedef struct sac_fallback_module_instance {
    /*! Pipeline on which the fallback module is instantiated. */
    sac_pipeline_t *pipeline;
    /*! Set to true if instantiated for an audio transmitting pipeline. */
    bool is_tx_device;
    /*! Fallback state. */
    sac_fallback_state_t fallback_state;
    /*! Number of times fallback was triggered. */
    uint32_t fallback_count;
    /*! Maximum size of the audio transmitting consumer buffer multiplied by 10. */
    uint32_t consumer_buffer_size_tenths;
    /*! Audio transmitting pipeline consumer buffer load above which fallback is triggered;
     *   Value should be multiplied by 10 (e.g.: 1.3 is 13).
     */
    uint32_t consumer_buffer_load_threshold_tenths;
    /*! Fallback mode flag. */
    bool fallback_flag;
    /*! Audio transmitting pipeline consumer queue metrics. */
    sac_fallback_queue_metrics_t consumer_queue_metrics;
    /*! Link margin metrics. */
    sac_fallback_link_margin_metrics_t link_margin_metrics;
    /*! CCA metrics. */
    sac_fallback_cca_metrics_t cca_metrics;
    /*! Callback function called on a fallback state change. */
    void (*fallback_state_change_callback)(bool enabled);
} sac_fallback_module_instance_t;

/* PRIVATE GLOBALS ************************************************************/
static sac_fallback_module_instance_t *fallback_instance;

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void init_consumer_queue_metrics(void);
static void init_link_stats(void);
static void update_consumer_queue_metrics(void);
static void update_link_stats(uint32_t cca_fail_count);
static bool is_link_good(void);

/* PUBLIC FUNCTIONS ***********************************************************/
sac_fallback_module_cfg_t sac_fallback_module_get_defaults(void)
{
    sac_fallback_module_cfg_t sac_fallback_default_cfg = {
        .pipeline = NULL,
        .is_tx_device = false,
        .link_margin_threshold = 50,
        .link_margin_threshold_hysteresis = 20,
        .link_margin_good_time_s = 5,
        .cca_max_try_count = 0,
        .cca_try_count_threshold_perc = 5,
        .cca_good_time_s = 30,
        .consumer_buffer_load_threshold_tenths = 13,
        .link_margin_conn_packets_per_second = 0,
        .main_conn_packets_per_second = 0,
    };

    return sac_fallback_default_cfg;
}

void sac_fallback_module_init(sac_fallback_module_cfg_t cfg, mem_pool_t *mem_pool, sac_error_t *err)
{
    *err = SAC_ERR_NONE;
    uint32_t consumer_buffer_size_tenths;
    uint32_t cca_fail_count_max_per_period;

    fallback_instance = (sac_fallback_module_instance_t *)mem_pool_malloc(mem_pool, sizeof(sac_fallback_module_instance_t));
    if (fallback_instance == NULL) {
        *err = SAC_ERR_NOT_ENOUGH_MEMORY;
        return;
    }

    /* Start system in fallback mode. */
    fallback_instance->fallback_flag = true;
    fallback_instance->fallback_state = SAC_FB_STATE_FALLBACK_DISCONNECT;
    fallback_instance->fallback_count = 0;
    fallback_instance->is_tx_device = cfg.is_tx_device;

    if (cfg.is_tx_device) {
        if (cfg.pipeline == NULL) {
            *err = SAC_ERR_FALLBACK_INIT_FAILURE;
            return;
        }
        /* consumer_buffer_load_threshold_tenths is x10. */
        consumer_buffer_size_tenths = cfg.pipeline->consumer->cfg.queue_size * BUF_SIZE_DECIMAL_FACTOR;
        if ((cfg.consumer_buffer_load_threshold_tenths == 0) ||
            (cfg.consumer_buffer_load_threshold_tenths >= consumer_buffer_size_tenths) ||
            (consumer_buffer_size_tenths == 0)) {
            *err = SAC_ERR_FALLBACK_INIT_FAILURE;
            return;
        }

        if ((cfg.link_margin_conn_packets_per_second == 0) || (cfg.main_conn_packets_per_second == 0)) {
            *err = SAC_ERR_FALLBACK_INIT_FAILURE;
            return;
        }

        fallback_instance->link_margin_metrics.good_count_threshold = cfg.link_margin_good_time_s * SAMPLING_INTERVAL_DIVISOR;
        fallback_instance->cca_metrics.good_count_threshold = cfg.cca_good_time_s * SAMPLING_INTERVAL_DIVISOR;
        fallback_instance->link_margin_metrics.threshold_default = cfg.link_margin_threshold;
        fallback_instance->link_margin_metrics.threshold = cfg.link_margin_threshold;
        fallback_instance->link_margin_metrics.threshold_hysteresis = cfg.link_margin_threshold_hysteresis;
        fallback_instance->pipeline = cfg.pipeline;
        fallback_instance->consumer_buffer_size_tenths = consumer_buffer_size_tenths;
        fallback_instance->link_margin_metrics.packets_per_period = cfg.link_margin_conn_packets_per_second / SAMPLING_INTERVAL_DIVISOR;
        cca_fail_count_max_per_period = cfg.cca_max_try_count * cfg.main_conn_packets_per_second / SAMPLING_INTERVAL_DIVISOR;
        fallback_instance->cca_metrics.fail_count_threshold = ((cca_fail_count_max_per_period * cfg.cca_try_count_threshold_perc) / 100);
        fallback_instance->consumer_buffer_load_threshold_tenths = cfg.consumer_buffer_load_threshold_tenths;
        fallback_instance->cca_metrics.good_count = fallback_instance->cca_metrics.good_count_threshold;
    }

    init_consumer_queue_metrics();
    init_link_stats();

    fallback_instance->fallback_state_change_callback = cfg.fallback_state_change_callback;
}

void sac_fallback_set_link_margin_good_time(uint32_t time)
{
    if (fallback_instance == NULL) {
        /* ERROR: Fallback not initialized. */
        while (1);
    }

    fallback_instance->link_margin_metrics.good_count_threshold = time * SAMPLING_INTERVAL_DIVISOR;
}

void sac_fallback_set_cca_good_time(uint32_t time)
{
    if (fallback_instance == NULL) {
        /* ERROR: Fallback not initialized. */
        while (1);
    }

    fallback_instance->cca_metrics.good_count_threshold = time * SAMPLING_INTERVAL_DIVISOR;
}

void sac_fallback_set_rx_link_margin(uint8_t rx_lm)
{
    if (fallback_instance == NULL) {
        /* ERROR: Fallback not initialized. */
        while (1);
    }

    fallback_instance->link_margin_metrics.rx_data = rx_lm;
    fallback_instance->link_margin_metrics.rx_data_valid = true;
}

void sac_fallback_update_state(void)
{
    sac_fallback_link_margin_metrics_t *link_margin_metrics = &fallback_instance->link_margin_metrics;
    sac_fallback_queue_metrics_t *consumer_queue_metrics = &fallback_instance->consumer_queue_metrics;

    if (fallback_instance == NULL) {
        /* ERROR: Fallback not initialized. */
        while (1);
    }

    if (fallback_instance->is_tx_device) {
        /* Fallback state machine only runs on the tx device. */
        switch (fallback_instance->fallback_state) {
        case SAC_FB_STATE_NORMAL:
            if (consumer_queue_metrics->queue_length_avg == fallback_instance->consumer_buffer_size_tenths) {
                /* TX Queue is full => Link is disconnected. */
                fallback_instance->link_margin_metrics.threshold = fallback_instance->link_margin_metrics.threshold_default;
                init_link_stats(); /* Clear the lm stats since they are from non-fallback mode. */
                sac_fallback_set_fallback_flag();
                fallback_instance->fallback_state = SAC_FB_STATE_FALLBACK_DISCONNECT;
            } else if ((consumer_queue_metrics->queue_length_avg > fallback_instance->consumer_buffer_load_threshold_tenths) && !fallback_instance->fallback_flag) {
                /* Buffer load threshold has been reached. Measure the current link margin to use it as a threshold. */
                init_link_stats(); /* Clear the lm stats since they are from non-fallback mode. */
                sac_fallback_set_fallback_flag();
                fallback_instance->fallback_state = SAC_FB_STATE_WAIT_THRESHOLD;
            }
            break;
        case SAC_FB_STATE_WAIT_THRESHOLD:
            /* State entered due to a degrading link, waiting to measure return to normal threshold. */
            if (consumer_queue_metrics->queue_length_avg == fallback_instance->consumer_buffer_size_tenths) {
                /* TX Queue is full => Link is disconnected. */
                fallback_instance->link_margin_metrics.threshold = fallback_instance->link_margin_metrics.threshold_default;
                fallback_instance->fallback_state = SAC_FB_STATE_FALLBACK_DISCONNECT;
            } else if (link_margin_metrics->accumulator_average > 0) {
                /* Averaging is complete. Use this value as a threshold to return to normal. */
                fallback_instance->link_margin_metrics.threshold = link_margin_metrics->accumulator_average;
                /* Make sure threshold is reasonable. */
                if ((fallback_instance->link_margin_metrics.threshold >
                    (fallback_instance->link_margin_metrics.threshold_default + fallback_instance->link_margin_metrics.threshold_hysteresis)) ||
                    (fallback_instance->link_margin_metrics.threshold <
                    (fallback_instance->link_margin_metrics.threshold_default - fallback_instance->link_margin_metrics.threshold_hysteresis))) {
                    fallback_instance->link_margin_metrics.threshold = fallback_instance->link_margin_metrics.threshold_default;
                }
                fallback_instance->fallback_state = SAC_FB_STATE_FALLBACK;
            }
            break;
        case SAC_FB_STATE_FALLBACK:
            /* State entered due to a degraded link. */
            if (consumer_queue_metrics->queue_length_avg == fallback_instance->consumer_buffer_size_tenths) {
                /* TX Queue is full => Link is disconnected. */
                fallback_instance->link_margin_metrics.threshold = fallback_instance->link_margin_metrics.threshold_default;
                fallback_instance->fallback_state = SAC_FB_STATE_FALLBACK_DISCONNECT;
            } else if (is_link_good()) {
                /* LM is continuously above threshold for 3 seconds, switch to normal. */
                sac_fallback_clear_fallback_flag();
                fallback_instance->fallback_state = SAC_FB_STATE_NORMAL;
            }
            break;
        case SAC_FB_STATE_FALLBACK_DISCONNECT:
            /* State entered due to a disconnected link. */
            if (is_link_good()) {
                /* LM is continuously above threshold for 3 seconds, switch to normal. */
                sac_fallback_clear_fallback_flag();
                fallback_instance->fallback_state = SAC_FB_STATE_NORMAL;
            }
            break;
        }
    }
}

void sac_fallback_update_stats(uint32_t cca_fail_count)
{
    if (fallback_instance == NULL) {
        /* ERROR: Fallback not initialized. */
        while (1);
    }

    if (fallback_instance->is_tx_device) {
        /* Fallback state machine only runs on the tx device. */
        update_consumer_queue_metrics();
        update_link_stats(cca_fail_count);
    }
}

bool sac_fallback_is_active(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized. */
        return false;
    }

    return fallback_instance->fallback_flag;
}

void sac_fallback_set_fallback_flag(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return;
    }

    if (!fallback_instance->fallback_flag) {
        fallback_instance->fallback_flag = true;
        fallback_instance->fallback_count++;
        if (fallback_instance->fallback_state_change_callback != NULL) {
            fallback_instance->fallback_state_change_callback(fallback_instance->fallback_flag);
        }
    }
}

void sac_fallback_clear_fallback_flag(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return;
    }

    if (fallback_instance->fallback_flag) {
        fallback_instance->fallback_flag = false;
        if (fallback_instance->fallback_state_change_callback != NULL) {
            fallback_instance->fallback_state_change_callback(fallback_instance->fallback_flag);
        }
    }
}

uint32_t sac_fallback_get_fallback_count(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return 0;
    }

    return fallback_instance->fallback_count;
}

sac_fallback_cca_metrics_t *sac_fallback_get_cca_metrics(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return NULL;
    }

    return &fallback_instance->cca_metrics;
}

uint8_t sac_fallback_get_rx_link_margin(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return 0;
    }

    return fallback_instance->link_margin_metrics.rx_data;
}

int sac_fallback_module_format_stats(char *buffer, uint16_t size)
{
    int string_length = 0;

    const char *is_active_str = "Fallback State";
    const char *activation_count_str = "Fallback Activation Count";
    const char *link_margin_str = "Link Margin Value";
    const char *link_margin_thr_str = "Link Margin Threshold";
    const char *cca_fail_count_str = "CCA Fail Count Value";
    const char *cca_fail_count_thr_str = "CCA Fail Count Threshold";

    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return 0;
    }

    if (fallback_instance->is_tx_device) {
        string_length = snprintf(buffer, size,
                                 "%s:\t\t\t%10s\r\n"
                                 "%s:\t%10lu\r\n"
                                 "<<< Link Margin >>>\r\n"
                                 "  %s:\t\t%10i\r\n"
                                 "  %s:\t%10i\r\n"
                                 "<<< Clear Channel Assessment >>>\r\n"
                                 "  %s:\t\t%10lu\r\n"
                                 "  %s:\t%10lu\r\n",
                                 is_active_str,
                                 fallback_instance->fallback_flag ? "Active" : "Inactive",
                                 activation_count_str,
                                 fallback_instance->fallback_count,
                                 link_margin_str,
                                 fallback_instance->link_margin_metrics.accumulator_average,
                                 link_margin_thr_str,
                                 fallback_instance->link_margin_metrics.threshold,
                                 cca_fail_count_str,
                                 fallback_instance->cca_metrics.fail_count_value,
                                 cca_fail_count_thr_str,
                                 fallback_instance->cca_metrics.fail_count_threshold);
    } else {
        string_length = snprintf(buffer, size,
                                 "%s:\t\t\t%10s\r\n",
                                 is_active_str,
                                 fallback_instance->fallback_flag ? "Active" : "Inactive");
    }

    return string_length;
}

void sac_fallback_module_reset_stats(void)
{
    if (fallback_instance == NULL) {
        /* Fallback not initialized, do nothing. */
        return;
    }

    fallback_instance->fallback_count = 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Clear the consumer queue metrics.
 */
static void init_consumer_queue_metrics(void)
{
    memset(&fallback_instance->consumer_queue_metrics, 0, sizeof(fallback_instance->consumer_queue_metrics));
}

/** @brief Clear the link stats.
 */
static void init_link_stats(void)
{
    fallback_instance->link_margin_metrics.accumulator = 0;
    fallback_instance->link_margin_metrics.accumulator_count = 0;
    fallback_instance->link_margin_metrics.accumulator_average = 0;
    fallback_instance->link_margin_metrics.good_count = 0;
    fallback_instance->cca_metrics.fail_count_start = fallback_instance->cca_metrics.fail_count_current;
}

/** @brief Update the consumer queue metrics.
 */
static void update_consumer_queue_metrics(void)
{
    sac_fallback_queue_metrics_t *consumer_queue_metrics = &fallback_instance->consumer_queue_metrics;

    if (fallback_instance->pipeline->consumer->_internal.buffering_complete) {
        consumer_queue_metrics->queue_length_sum -= consumer_queue_metrics->queue_length_arr[consumer_queue_metrics->queue_length_arr_idx];
        consumer_queue_metrics->queue_length_arr[consumer_queue_metrics->queue_length_arr_idx] = queue_get_length(fallback_instance->pipeline->consumer->_internal.queue);
        consumer_queue_metrics->queue_length_sum += consumer_queue_metrics->queue_length_arr[consumer_queue_metrics->queue_length_arr_idx++];
        consumer_queue_metrics->queue_length_avg = (consumer_queue_metrics->queue_length_sum * BUF_SIZE_DECIMAL_FACTOR) / SAC_FALLBACK_QUEUE_ARRAY_LENGTH;
        consumer_queue_metrics->queue_length_arr_idx %= SAC_FALLBACK_QUEUE_ARRAY_LENGTH;
    }
}

/** @brief Update the link stats.
 *
 *  Call inside the main background loop.
 */
static void update_link_stats(uint32_t cca_fail_count)
{
    sac_fallback_link_margin_metrics_t *link_margin_metrics = &fallback_instance->link_margin_metrics;
    sac_fallback_cca_metrics_t *cca_metrics = &fallback_instance->cca_metrics;

    cca_metrics->fail_count_current = cca_fail_count;
    if (fallback_instance->link_margin_metrics.rx_data_valid) {
        link_margin_metrics->accumulator += fallback_instance->link_margin_metrics.rx_data;
        if (fallback_instance->link_margin_metrics.rx_data < fallback_instance->pipeline->_statistics.consumer_link_margin_min_peak) {
            fallback_instance->pipeline->_statistics.consumer_link_margin_min_peak = fallback_instance->link_margin_metrics.rx_data;
        }
        /* Received link margin from node. */
        fallback_instance->link_margin_metrics.rx_data_valid = false;
        if (++link_margin_metrics->accumulator_count >= fallback_instance->link_margin_metrics.packets_per_period) {
            /* Calculate 250ms average. */
            link_margin_metrics->accumulator_average =
                        link_margin_metrics->accumulator / link_margin_metrics->accumulator_count;
            link_margin_metrics->accumulator = 0;
            link_margin_metrics->accumulator_count = 0;
            if (link_margin_metrics->accumulator_average >= (fallback_instance->link_margin_metrics.threshold +
                                                             fallback_instance->link_margin_metrics.threshold_hysteresis) &&
                                                             fallback_instance->fallback_flag) {
                /* Average above threshold, increment the 3 second count. */
                link_margin_metrics->good_count = (link_margin_metrics->good_count + 1) < fallback_instance->link_margin_metrics.good_count_threshold ?
                                                  (link_margin_metrics->good_count + 1) : fallback_instance->link_margin_metrics.good_count_threshold;
            } else {
                link_margin_metrics->good_count = 0; /* Below average, restart count. */
            }

            if (cca_metrics->fail_count_current < cca_metrics->fail_count_start) {
                /* Adjust for roll over */
                cca_metrics->fail_count_value = (UINT32_MAX - cca_metrics->fail_count_start) + cca_metrics->fail_count_current;
            } else {
                cca_metrics->fail_count_value = cca_metrics->fail_count_current - cca_metrics->fail_count_start;
            }
            cca_metrics->fail_count_start = cca_fail_count;

            if (cca_metrics->fail_count_value > fallback_instance->pipeline->_statistics.consumer_cca_fail_count_peak) {
                fallback_instance->pipeline->_statistics.consumer_cca_fail_count_peak = cca_metrics->fail_count_value;
            }

            if (cca_metrics->fail_count_value <= cca_metrics->fail_count_threshold) {
                cca_metrics->good_count = (cca_metrics->good_count + 1) < cca_metrics->good_count_threshold ?
                                          (cca_metrics->good_count + 1) : cca_metrics->good_count_threshold;
            } else {
                cca_metrics->good_count = 0;
            }
        }
    }
}

/** @brief Return if link is good enough to switch to normal mode.
 *
 *  @return true if the link is good, false otherwise.
 */
static bool is_link_good(void)
{
   return ((fallback_instance->link_margin_metrics.good_count >= fallback_instance->link_margin_metrics.good_count_threshold) &&
           (fallback_instance->cca_metrics.good_count >= fallback_instance->cca_metrics.good_count_threshold));
}
