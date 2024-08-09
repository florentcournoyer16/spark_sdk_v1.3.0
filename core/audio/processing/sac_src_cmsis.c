/** @file  sac_src_cmsis.c
 *  @brief Sampling rate converter processing stage using the CMSIS DSP software library.
 *
 *  @note This processing stage requires an Arm Cortex-M processor based device.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_src_cmsis.h"
#include <string.h>

/* CONSTANTS ******************************************************************/
#define FIR_NUMTAPS 24 /* Must be dividable by all sac_src_factor since fir phaseLength is (NumTaps / ratio). */

/* Half of initial sampling frequency */
static const q15_t fir_n24_c0_35_w_hamming[FIR_NUMTAPS] = {
    5, -82, -137, 21, 422, 545, -253, -1567, -1561, 1434, 6681, 10874,
    10874, 6681, 1434, -1561, -1567, -253, 545, 422, 21, -137, -82, 5
};

/*
 * Same filter with coefficient multiplied by a factor of 2
 * Used to compensate for the gain loss due to the interpolation zero-stuffing.
 */
static const q15_t fir_n24_c0_35_w_hamming_x2_gain[FIR_NUMTAPS] = {
    11, -164, -275, 43, 845, 1091, -506, -3134, -3123, 2869, 13362, 21749,
    21749, 13362, 2869, -3123, -3134, -506, 1091, 845, 43, -275, -164, 11
};

/* Third of initial sampling frequency */
static const q15_t fir_n24_c0_20_w_hamming[FIR_NUMTAPS] = {
    58, 29, -49, -223, -454, -577, -333, 495, 1933, 3725, 5388, 6391,
    6391, 5388, 3725, 1933, 495, -333, -577, -454, -223, -49, 29, 58
};

/*
 * Same filter with coefficient multiplied by a factor of 3
 * Used to compensate for the gain loss due to the interpolation zero-stuffing.
 */
static const q15_t fir_n24_c0_20_w_hamming_x3_gain[FIR_NUMTAPS] = {
    174, 87, -147, -669, -1362, -1731, -999, 1485, 5799, 11175, 16164, 19173,
    19173, 16164, 11175, 5799, 1485, -999, -1731, -1362, -669, -147, 87, 174
};

/* Sixth of initial sampling frequency. */
static const q15_t fir_n24_c0_10_w_hamming[FIR_NUMTAPS] = {
    -36, -17, 28, 139, 358, 707, 1185, 1760, 2368, 2930, 3363, 3599,
    3599, 3363, 2930, 2368, 1760, 1185, 707, 358, 139, 28, -17, -36
};

/* Same filter with coefficient multiplied by a factor of 6
 * Used to compensate for the gain loss due to the interpolation zero-stuffing.
 */
static const q15_t fir_n24_c0_10_w_hamming_x6_gain[FIR_NUMTAPS] = {
    -216, -102, 168, 834, 2148, 4242, 7110, 10560, 14208, 17580, 20178, 21594,
    21594, 20178, 17580, 14208, 10560, 7110, 4242, 2148, 834, 168, -102, -216
};

/* PUBLIC FUNCTIONS ***********************************************************/
void sac_src_cmsis_init(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err)
{
    (void)pipeline;

    int16_t *fir_state;
    const int16_t *fir_coeff_decimation;
    const int16_t *fir_coeff_interpolation;
    src_cmsis_instance_t *src_instance = instance;
    uint32_t block_size;

    *err = SAC_ERR_NONE;

    if (src_instance == NULL) {
        *err = SAC_ERR_NULL_PTR;
        return;
    }

    if ((src_instance->cfg.payload_size == 0) || (src_instance->cfg.bit_depth == 0)) {
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }

    if ((src_instance->cfg.multiply_ratio == 1) && (src_instance->cfg.divide_ratio == 1)) {
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }

    switch (src_instance->cfg.multiply_ratio) {
    case SAC_SRC_ONE:
        fir_coeff_interpolation = NULL;
        break;
    case SAC_SRC_TWO:
        fir_coeff_interpolation = fir_n24_c0_35_w_hamming_x2_gain;
        break;
    case SAC_SRC_SIX:
        fir_coeff_interpolation = fir_n24_c0_10_w_hamming_x6_gain;
        break;
    case SAC_SRC_THREE:
        fir_coeff_interpolation = fir_n24_c0_20_w_hamming_x3_gain;
        break;
    default:
        /* Invalid ratio. */
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }

    if (src_instance->cfg.multiply_ratio > SAC_SRC_ONE) {
        block_size = src_instance->cfg.payload_size / (src_instance->cfg.bit_depth / 8);
        fir_state = mem_pool_malloc(mem_pool, sizeof(int16_t) * (FIR_NUMTAPS + block_size));
        if (fir_state == NULL) {
            *err = SAC_ERR_NOT_ENOUGH_MEMORY;
            return;
        }
        arm_fir_interpolate_init_q15(&src_instance->_internal.interpolate_instance, src_instance->cfg.multiply_ratio,
                                     FIR_NUMTAPS, fir_coeff_interpolation, fir_state, block_size);
    }

    switch (src_instance->cfg.divide_ratio) {
    case SAC_SRC_ONE:
        fir_coeff_decimation = NULL;
        break;
    case SAC_SRC_TWO:
        fir_coeff_decimation = fir_n24_c0_35_w_hamming;
        break;
    case SAC_SRC_SIX:
        fir_coeff_decimation = fir_n24_c0_10_w_hamming;
        break;
    case SAC_SRC_THREE:
        fir_coeff_decimation = fir_n24_c0_20_w_hamming;
        break;
    default:
        /* Invalid ratio. */
        *err = SAC_ERR_PROCESSING_STAGE_INIT;
        return;
    }

    if (src_instance->cfg.divide_ratio > SAC_SRC_ONE) {
        if (src_instance->cfg.multiply_ratio > SAC_SRC_ONE) {
            src_instance->_internal.multiply_out_buffer = mem_pool_malloc(mem_pool, sizeof(int16_t) * (src_instance->cfg.payload_size * src_instance->cfg.multiply_ratio));
            if (src_instance->_internal.multiply_out_buffer == NULL) {
                *err = SAC_ERR_NOT_ENOUGH_MEMORY;
                return;
            }
        }
        block_size = (src_instance->cfg.payload_size * src_instance->cfg.multiply_ratio) / (src_instance->cfg.bit_depth / 8);
        fir_state = mem_pool_malloc(mem_pool, sizeof(int16_t) * (FIR_NUMTAPS + block_size));
        if (fir_state == NULL) {
            *err = SAC_ERR_NOT_ENOUGH_MEMORY;
            return;
        }
        arm_fir_decimate_init_q15(&src_instance->_internal.decimate_instance, FIR_NUMTAPS, src_instance->cfg.divide_ratio,
                                  fir_coeff_decimation, fir_state, block_size);
    }
}

uint16_t sac_src_cmsis_process(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in, uint16_t size,
                               uint8_t *data_out, sac_error_t *err)
{
    (void)pipeline;
    (void)header;

    uint16_t sample_count_out          = 0;
    uint16_t sample_count_in           = size / sizeof(q15_t);
    src_cmsis_instance_t *src_instance = instance;
    int16_t *audio_in;
    int16_t *audio_out;

    *err = SAC_ERR_NONE;

    if (src_instance->cfg.multiply_ratio > SAC_SRC_ONE) {
        audio_in = (q15_t *)data_in;
        if (src_instance->cfg.divide_ratio > SAC_SRC_ONE) {
            audio_out = src_instance->_internal.multiply_out_buffer;
        } else {
            audio_out = (q15_t *)data_out;
        }
        arm_fir_interpolate_q15(&src_instance->_internal.interpolate_instance, audio_in, audio_out, sample_count_in);
        sample_count_out = sample_count_in * src_instance->cfg.multiply_ratio;
    }

    if (src_instance->cfg.divide_ratio > SAC_SRC_ONE) {
        if (src_instance->cfg.multiply_ratio > SAC_SRC_ONE) {
            sample_count_in = sample_count_out;
            audio_in = src_instance->_internal.multiply_out_buffer;
        } else {
            audio_in = (q15_t *)data_in;
        }
        audio_out = (q15_t *)data_out;
        arm_fir_decimate_q15(&src_instance->_internal.decimate_instance, audio_in, audio_out, sample_count_in);
        sample_count_out = sample_count_in / src_instance->cfg.divide_ratio;
    }

    return (sample_count_out * sizeof(q15_t));
}
