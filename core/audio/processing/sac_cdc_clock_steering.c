/** @file  sac_cdc_clock_steering.c
 *  @brief Clock drift compensation processing stage using audio buffer load averaging for
 *         detecting the drift and clock steering for correcting it.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_cdc_clock_steering.h"

/* CONSTANTS ******************************************************************/
#define DECIMAL_FACTOR 100

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void update_queue_avg(sac_cdc_clock_steering_instance_t *cdc, sac_pipeline_t *pipeline);
static void detect_drift(sac_cdc_clock_steering_instance_t *cdc, sac_pipeline_t *pipeline, sac_header_t *header);
static void steer_clock(sac_pipeline_t *pipeline, sac_cdc_clock_steering_instance_t *cdc);

/* PUBLIC FUNCTIONS ***********************************************************/
void sac_cdc_clock_steering_init(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err)
{
    sac_cdc_clock_steering_instance_t *cdc = instance;

    *err = SAC_ERR_NONE;

    if (cdc == NULL) {
        *err = SAC_ERR_NULL_PTR;
        return;
    }

    if (cdc->cdc_decrease_clock == NULL || cdc->cdc_increase_clock == NULL) {
        /* Can't use clock steering as compensation mechanism without providing proper clock adjustment functions. */
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }

    cdc->_internal.avg_sum = 0;
    cdc->_internal.avg_val = 0;
    cdc->_internal.avg_idx = 0;
    cdc->_internal.queue_avg_size = cdc->cdc_queue_avg_size;

    /* Allocate rolling average memory. */
    cdc->_internal.avg_arr = (uint16_t *)mem_pool_malloc(mem_pool, cdc->_internal.queue_avg_size * sizeof(uint16_t));
    if (cdc->_internal.avg_arr == NULL) {
        *err = SAC_ERR_NOT_ENOUGH_MEMORY;
        return;
    }
    memset(cdc->_internal.avg_arr, 0, cdc->_internal.queue_avg_size);

    /* Initialize resampling configuration. */
    switch (pipeline->consumer->cfg.bit_depth) {
    case SAC_16BITS:
        cdc->_internal.size_of_buffer_type = SAC_16BITS_BYTE;
        break;
    case SAC_20BITS:
        cdc->_internal.size_of_buffer_type = SAC_20BITS_BYTE;
        break;
    case SAC_24BITS:
        cdc->_internal.size_of_buffer_type = SAC_24BITS_BYTE;
        break;
    case SAC_32BITS:
        cdc->_internal.size_of_buffer_type = SAC_32BITS_BYTE;
        break;
    default:
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }

    /* Initial state. */
    cdc->_internal.state = SAC_CDC_STATE_WAIT_QUEUE_FULL;
    cdc->_internal.correction = SAC_CDC_CORRECTION_NONE;

    /* Configure threshold. */
    cdc->_internal.sample_amount = pipeline->consumer->cfg.audio_payload_size /
                                  (pipeline->consumer->cfg.channel_count * cdc->_internal.size_of_buffer_type);
    cdc->_internal.normal_queue_size = pipeline->consumer->cfg.queue_size * cdc->_internal.sample_amount * DECIMAL_FACTOR;
    cdc->_internal.max_queue_offset = cdc->cdc_detect_resolution * DECIMAL_FACTOR;
}

uint32_t sac_cdc_clock_steering_ctrl(void *instance, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t arg, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)cmd;
    (void)arg;

    *err = SAC_ERR_NONE;

    return 0;
}

uint16_t sac_cdc_process_clock_steering(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in,
                                        uint16_t size, uint8_t *data_out, sac_error_t *err)
{
    sac_cdc_clock_steering_instance_t *cdc = instance;

    (void)data_in;
    (void)data_out;
    (void)size;

    *err = SAC_ERR_NONE;

    detect_drift(cdc, pipeline, header);
    steer_clock(pipeline, cdc);

    /* Clock steering does not alter data, return 0 to indicate no data processing was done. */
    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Detect an audio clock drift based on the average audio queue load.
 *
 *  @param[in] cdc       CDC instance.
 *  @param[in] pipeline  Pipeline instance.
 */
static void update_queue_avg(sac_cdc_clock_steering_instance_t *cdc, sac_pipeline_t *pipeline)
{
    uint16_t current_queue_length = pipeline->_internal.samples_buffered_size / (pipeline->consumer->cfg.channel_count *
                                                                        cdc->_internal.size_of_buffer_type);
    uint16_t avg_idx = cdc->_internal.avg_idx;

    /* Update Rolling Avg */
    cdc->_internal.avg_sum -= cdc->_internal.avg_arr[avg_idx]; /* Remove oldest value. */
    cdc->_internal.avg_arr[avg_idx] = current_queue_length;
    cdc->_internal.avg_sum += cdc->_internal.avg_arr[avg_idx]; /* Add new value. */
    if (++avg_idx >= cdc->_internal.queue_avg_size) {
        avg_idx = 0;
    }
    cdc->_internal.avg_val = (cdc->_internal.avg_sum * DECIMAL_FACTOR) / cdc->_internal.queue_avg_size;
    cdc->_internal.avg_idx = avg_idx;
}

/** @brief Detect an audio clock drift based on the average audio queue load.
 *
 *  @param[in] cdc       CDC instance.
 *  @param[in] pipeline  Pipeline instance.
 *  @param[in] header    Audio packet's header.
 */
static void detect_drift(sac_cdc_clock_steering_instance_t *cdc, sac_pipeline_t *pipeline, sac_header_t *header)
{
    /* Calculate average queue length only if audio link is stable. */
    if (header->tx_queue_level_high == 0) {
        update_queue_avg(cdc, pipeline);
    }

    if ((header->tx_queue_level_high == 1) &&
        (cdc->_internal.state == SAC_CDC_STATE_IDLE)) {
        cdc->_internal.wait_for_queue_full = true;
    }

    if (cdc->_internal.state == SAC_CDC_STATE_WAIT_QUEUE_FULL) {
        if (header->tx_queue_level_high == 0) {
            cdc->_internal.state = SAC_CDC_STATE_IDLE;
            cdc->_internal.wait_for_queue_full = false;
        }
    } else if (cdc->_internal.state == SAC_CDC_STATE_IDLE) {
        /* Verify if queue is increasing or depleting. */
        if (cdc->_internal.wait_for_queue_full) {
            cdc->_internal.state = SAC_CDC_STATE_WAIT_QUEUE_FULL;
        } else {
            if (cdc->_internal.count > cdc->_internal.queue_avg_size) {
                if (cdc->_internal.avg_val > (cdc->_internal.normal_queue_size + cdc->_internal.max_queue_offset)) {
                    cdc->_internal.state = SAC_CDC_STATE_START;
                    cdc->_internal.correction = SAC_CDC_CORRECTION_REMOVE;
                    cdc->_internal.count = 0;
                } else if (cdc->_internal.avg_val < (cdc->_internal.normal_queue_size - cdc->_internal.max_queue_offset)) {
                    cdc->_internal.state = SAC_CDC_STATE_START;
                    cdc->_internal.correction = SAC_CDC_CORRECTION_ADD;
                    cdc->_internal.count = 0;
                }
            } else {
                /* Give time to the avg to stabilize before checking. */
                cdc->_internal.count++;
            }
        }
    }
}

/** @brief Once started (the first call to this function while the CDC state
 *         is SAC_CDC_STATE_START and cdc->correction has been set), the
 *         CDC state is changed to SAC_CDC_STATE_RUNNING and an action is
 *         performed for this call and every subsequent call to this function.
 *         The action depends on the correction to be applied (ADD_SAMPLE
 *         which is equivalent to slowing the clock down or REMOVE_SAMPLE
 *         which is equivalent to accelerating the clock). Each call to this
 *         function will increase or decrease the PLL value (functions defined
 *         and provided by the BSP) up to a certain value (±max_clock_increment)
 *         then return to the original PLL value. Once the PLL has returned to its
 *         original value, a permanent PLL offset is applied and the resampling
 *         state & correction are reset.
 *
 *  @param[in] pipeline  Audio pipeline in which clock steering must be applied.
 *  @param[in] cdc       CDC instance.
 */
static void steer_clock(sac_pipeline_t *pipeline, sac_cdc_clock_steering_instance_t *cdc)
{
    /* This implementation is specific to the STM32U575 microcontroller using its fractional PLL2.
     * This leads to a temporary ±410ppm clock offset (max_clock_increment = 42).
     * The calculation is as follows:
     *      refclock * (N + (FRACN / 2^13)) / P / FS_DIV = FS
     * max_clock_increment represent the maximum number of times FRACN will be
     * incremented or decremented before returning to the initial value.
     * Note that this depends on PLL2's configuration. Changing the PLL2 config
     * will need to be taken into account in the calculation.
     *  PLL2 settings used:
     *      M = 5
     *      refclock = 20576000 / M = 4915200
     *      N = 12
     *      P = 5
     *      FRACN = 4100
     * max_clock_increment may need to be adjusted depending on the audio time
     * per packet.
     */
    const uint8_t max_clock_increment = 42;
    const uint8_t max_permanent_offset = 42;
    static int32_t original_pllfracn_offset;

    if (cdc->_internal.state == SAC_CDC_STATE_START || cdc->_internal.state == SAC_CDC_STATE_RUNNING) {
        cdc->_internal.state = SAC_CDC_STATE_RUNNING;
        switch (cdc->_internal.correction) {
        case SAC_CDC_CORRECTION_ADD:
            /* Logic to decrease sampling rate, equivalent to adding a sample. */
            if (cdc->_internal.return_to_original_clock == false) {
                cdc->cdc_decrease_clock();
                cdc->_internal.current_offset++;
                if (cdc->_internal.current_offset == max_clock_increment) {
                    cdc->_internal.return_to_original_clock = true;
                }
            } else {
                cdc->cdc_increase_clock();
                cdc->_internal.current_offset--;
                if (cdc->_internal.current_offset == 0) {
                    cdc->_internal.state = SAC_CDC_STATE_IDLE;
                    cdc->_internal.correction = SAC_CDC_CORRECTION_NONE;
                    cdc->_internal.return_to_original_clock = false;
                    pipeline->_statistics.cdc_inflated_packets_count++;
                    /* Step PLLFRACN to permanently reduce gap between source & sink. */
                    if (original_pllfracn_offset >= -max_permanent_offset) {
                        cdc->cdc_decrease_clock();
                        original_pllfracn_offset--;
                    }
                }
            }
            break;
        case SAC_CDC_CORRECTION_REMOVE:
            /* Logic to increase sampling rate, equivalent to removing a sample. */
            if (cdc->_internal.return_to_original_clock == false) {
                cdc->cdc_increase_clock();
                cdc->_internal.current_offset++;
                if (cdc->_internal.current_offset == max_clock_increment) {
                    cdc->_internal.return_to_original_clock = true;
                }
            } else {
                cdc->cdc_decrease_clock();
                cdc->_internal.current_offset--;
                if (cdc->_internal.current_offset == 0) {
                    cdc->_internal.state = SAC_CDC_STATE_IDLE;
                    cdc->_internal.correction = SAC_CDC_CORRECTION_NONE;
                    cdc->_internal.return_to_original_clock = false;
                    pipeline->_statistics.cdc_deflated_packets_count++;
                    /* Step PLLFRACN to permanently reduce gap between source & sink. */
                    if (original_pllfracn_offset <= max_permanent_offset) {
                        cdc->cdc_increase_clock();
                        original_pllfracn_offset++;
                    }
                }
            }
            break;
        default:
            break;
        }
    }
}

