/** @file  srac_api.h
 *  @brief SPARK Ranging Core Application Programming Interface.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SRAC_API_H_
#define SRAC_API_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include "srac_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define SRAC_DEFAULT_OFFSET_MM 1061400 /*!< Default calibration offset value (millimeters) */

/* TYPES **********************************************************************/
/** @brief Phases value, last received preamble phase correlation data.
 */
typedef struct srac_data {
    /*! Receiver waited time (MSB) */
    uint8_t receiver_waited_time_1;
    /*! Receiver waited time (LSB) */
    uint8_t receiver_waited_time_0;
    /*! phase correlation metric 1 */
    int8_t phase_correlation_metric_1;
    /*! phase correlation metric 2 */
    int8_t phase_correlation_metric_2;
    /*! phase correlation metric 3 */
    int8_t phase_correlation_metric_3;
    /*! phase correlation metric 4 */
    int8_t phase_correlation_metric_4;
} srac_data_t;

/** @brief Phases value, last received preamble phase correlation data.
 */
typedef struct srac_data_set {
    /*! Local ranging data */
    srac_data_t local_data;
    /*! Remote ranging data */
    srac_data_t remote_data;
} srac_data_set_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Calculate distance from provided data.
 *
 *  @param[in]  ranging_data        Ranging data buffer.
 *  @param[in]  sample_count        SPARK Ranging Core sample count.
 *  @param[in]  calibration_offset  Distance calibration offset.
 *  @param[out] err                 Ranging Core error code.
 *
 *  @return Distance in centimeters.
 */
uint32_t srac_process_distance(srac_data_set_t *ranging_data, uint8_t sample_count, uint32_t calibration_offset, srac_error_t *err);

/** @brief Calibrate distance measurement at a predefined distance of 50 cm between both devices.
 *
 *  @note Distance is calculated without any offset and then the calibration value is determined.
 *
 *  @param[in]  ranging_data  Ranging data buffer.
 *  @param[in]  sample_count  SPARK Ranging Core sample count.
 *  @param[out] err           Ranging Core error code.
 *
 *  @return Calibration offset in centimeters.
 */
uint32_t srac_calibrate(srac_data_set_t *ranging_data, uint8_t sample_count, srac_error_t *err);


#ifdef __cplusplus
}
#endif

#endif /* SRAC_API_H_ */
