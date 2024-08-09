/** @file  sac_src_cmsis.h
 *  @brief Sampling rate converter processing stage using the CMSIS DSP software library.
 *
 *  @note This processing stage requires an Arm Cortex-M processor based device.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_SRC_CMSIS_H_
#define SAC_SRC_CMSIS_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"
#include "sac_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief SRC CMSIS Ratio.
 */
typedef enum src_cmsis_ratio {
    /*! Ratio of 1 between original and resulting sampling rate */
    SAC_SRC_ONE   = 1,
    /*! Ratio of 2 between original and resulting sampling rate */
    SAC_SRC_TWO   = 2,
    /*! Ratio of 3 between original and resulting sampling rate. */
    SAC_SRC_THREE = 3,
    /*! Ratio of 6 between original and resulting sampling rate. */
    SAC_SRC_SIX   = 6
} src_cmsis_ratio_t;

/** @brief SRC CMSIS Configuration.
 */
typedef struct src_cmsis_cfg {
    /*! Multiply ratio to use for the SRC interpolation. */
    src_cmsis_ratio_t multiply_ratio;
    /*! Divide ratio to use for the SRC decimation. */
    src_cmsis_ratio_t divide_ratio;
    /*! Size of the payload in bytes expected at input. */
    uint16_t payload_size;
    /*! Bit depth of each sample in the payload. */
    uint8_t bit_depth;
} src_cmsis_cfg_t;

/** @brief SRC CMSIS Instance.
 */
typedef struct src_cmsis_instance {
    /*! SRC CMSIS user configuration. */
    src_cmsis_cfg_t cfg;
    struct {
        /*! Internal: Instance for the arm_fir interpolation. */
        arm_fir_interpolate_instance_q15 interpolate_instance;
        /*! Internal: Instance for the arm_fir decimation. */
        arm_fir_decimate_instance_q15 decimate_instance;
        /*! Internal: Audio buffer to be used between multiply and divide process. */
        int16_t *multiply_out_buffer;
    } _internal;
} src_cmsis_instance_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the SRC CMSIS processing stage.
 *
 *  @param[in]  instance  SRC CMSIS instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  mem_pool  Memory pool handle.
 *  @param[out] err       Error code.
 */
void sac_src_cmsis_init(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err);

/** @brief Process SRC on an audio packet.
 *
 *  @param[in]  instance  SRC CMSIS instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  header    Audio packet's header.
 *  @param[in]  data_in   Audio payload to process.
 *  @param[in]  size      Size in bytes of the audio payload.
 *  @param[out] data_out  Audio payload that has been processed.
 *  @param[out] err       Error code.
 *
 *  @return Size in bytes of the processed samples, 0 if no processing happened
 */
uint16_t sac_src_cmsis_process(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in, uint16_t size,
                               uint8_t *data_out, sac_error_t *err);

#ifdef __cplusplus
}
#endif

#endif /* SAC_SRC_CMSIS_H_ */
