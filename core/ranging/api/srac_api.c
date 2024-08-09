/** @file  srac_api.c
 *  @brief SPARK Ranging Core Application Programming Interface.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "srac_api.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "mem_pool.h"
#include "srac_error.h"

/* CONSTANTS ******************************************************************/
#define SRAC_RADIO_RANGE_MM                 100000 /*!< The theoretical SPARK radio range limit is around 100 meters */
#define SRAC_SCALING_FACTOR                 1000   /*!< Scaling factor used for integer operations */
#define SRAC_COEFFICIENT_MM                 14638  /*!< Speed_of_light (mm/s) / pll_clock_frequency (Hz) = 299792458000 / 20.48e6 */
#define SRAC_CALIBRATION_DISTANCE_MM        500    /*!< Distance between two devices when calibrating (millimeters) */
enum {
    /*! One eighth of 1000 clock cycle fractions */
    SRAC_ONE_EIGHTH = 125,
    /*! One quarter of 1000 clock cycle fractions */
    SRAC_ONE_QUARTER = 250,
    /*! One half of 1000 clock cycle fractions */
    SRAC_ONE_HALF = 500,
    /*! A thousand PLL clock cycle fractions used to correct delay outliers */
    SRAC_THOUSAND_CLOCK_CYCLE_FRACTIONS = 1000,
    /*! Threshold used to detect delay outliers */
    SRAC_OUTLIER_DETECTION_THRESHOLD = 500,
};
enum {
    /*! Minimum supported sample count */
    SRAC_MIN_SAMPLE_COUNT = 8,
    /*! Maximum supported sample count */
    SRAC_MAX_SAMPLE_COUNT = 128,
};

/* MACROS *********************************************************************/
#define CHECK_ERROR(cond, err_ptr, err_code, ret) \
    do {                                          \
        if (cond) {                               \
            *(err_ptr) = (err_code);              \
            ret;                                  \
        }                                         \
    } while (0)

/* PRIVATE GLOBALS ************************************************************/
static uint32_t round_trip_delay[SRAC_MAX_SAMPLE_COUNT];

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void correct_ranging_delay(uint32_t delay_average, uint8_t sample_count);
static void get_round_trip_delay_array(srac_data_set_t *srac_data, uint16_t sample_count, uint32_t *raw_delay);
static uint32_t calculate_average_delay(uint32_t *data, uint8_t sample_count);
static uint32_t convert_delay_to_distance(uint32_t round_trip_delay, uint32_t calibration_offset);
static uint32_t get_round_trip_delay(srac_data_set_t *srac_data);
static int32_t get_fine_tuned_delay(srac_data_t *metrics);
static int32_t get_value_sign(int32_t value);

/* PUBLIC FUNCTIONS ***********************************************************/
uint32_t srac_process_distance(srac_data_set_t *ranging_data, uint8_t sample_count, uint32_t calibration_offset, srac_error_t *err)
{
    int32_t distance;
    uint32_t delay_average;
    uint32_t corrected_delay_average;

    *err = SRAC_ERR_NONE;

    CHECK_ERROR(((sample_count < SRAC_MIN_SAMPLE_COUNT) || (sample_count > SRAC_MAX_SAMPLE_COUNT)), err, SRAC_ERR_UNSUPPORTED_SAMPLE_COUNT,
                return 0);
    CHECK_ERROR(ranging_data == NULL, err, SRAC_ERR_NULL_PTR, return 0);

    get_round_trip_delay_array(ranging_data, sample_count, round_trip_delay);
    delay_average = calculate_average_delay(round_trip_delay, sample_count);
    correct_ranging_delay(delay_average, sample_count);
    corrected_delay_average = calculate_average_delay(round_trip_delay, sample_count);
    distance = convert_delay_to_distance(corrected_delay_average, calibration_offset);

    /* When the system is properly calibrated, negative distance values would not make sense and they are replaced by 0 */
    if (distance < 0) {
        distance = 0;
    }

    return distance;
}

uint32_t srac_calibrate(srac_data_set_t *ranging_data, uint8_t sample_count, srac_error_t *err)
{
    uint32_t distance;
    uint32_t calibration_offset = 0;

    *err = SRAC_ERR_NONE;

    distance = srac_process_distance(ranging_data, sample_count, calibration_offset, err);
    CHECK_ERROR((distance == 0) || (distance - SRAC_DEFAULT_OFFSET_MM  > SRAC_RADIO_RANGE_MM), err, SRAC_ERR_CALIBRATION, return 0);
    calibration_offset = (distance - SRAC_CALIBRATION_DISTANCE_MM);

    return calibration_offset;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Detect outliers and correct them.
 *
 * @param[in]  delay_average  SPARK Ranging Core error code.
 * @param[in]  sample_count   SPARK Ranging Core sample count.
 */
static void correct_ranging_delay(uint32_t delay_average, uint8_t sample_count)
{
    for (uint16_t i = 0; i < sample_count; i++) {
        if (round_trip_delay[i] < (delay_average - SRAC_OUTLIER_DETECTION_THRESHOLD)) {
            round_trip_delay[i] = round_trip_delay[i] + SRAC_THOUSAND_CLOCK_CYCLE_FRACTIONS;
        } else if (round_trip_delay[i] > (delay_average + SRAC_OUTLIER_DETECTION_THRESHOLD)) {
            round_trip_delay[i] = round_trip_delay[i] - SRAC_THOUSAND_CLOCK_CYCLE_FRACTIONS;
        }
    }
}

/** @brief Get delay given an array of ranging data.
 *
 *  @param[in]  srac_data         Ranging data array.
 *  @param[in]  sample_count      SPARK Ranging Core sample count.
 *  @param[out] round_trip_delay  Round trip delay array.
 */
static void get_round_trip_delay_array(srac_data_set_t *srac_data, uint16_t sample_count, uint32_t *round_trip_delay)
{
    for (uint16_t i = 0; i < sample_count; i++) {
        round_trip_delay[i] = get_round_trip_delay(srac_data + i);
    }
}

/** @brief Get an average value of a data set.
 *
 *  @param[in] data          Data to average.
 *  @param[in] sample_count  SPARK Ranging Core sample count.
 *
 *  @return Average of data.
 */
static uint32_t calculate_average_delay(uint32_t *data, uint8_t sample_count)
{
    uint32_t sum = 0;

    for (int i = 0; i < sample_count; ++i) {
        sum += data[i];
    }

    return sum / sample_count;
}

/** @brief Get delay for distance calculation.
 *
 *  @param[in] round_trip_delay    Round trip delay value.
 *  @param[in] calibration_offset  Distance calibration offset.
 *
 *  @return Calibrated distance in millimeters.
 */
static uint32_t convert_delay_to_distance(uint32_t round_trip_delay, uint32_t calibration_offset)
{
    uint32_t one_trip_delay = round_trip_delay / 2;
    uint32_t uncalibrated_distance = one_trip_delay * SRAC_COEFFICIENT_MM;
    uint32_t calibrated_distance = uncalibrated_distance / SRAC_SCALING_FACTOR - calibration_offset;

    return calibrated_distance;
}

/** @brief Get round trip delay for distance calculation.
 *
 *  @param[in] srac_data  Ranging Core data.
 *
 *  @return Round trip delay value.
 */
static uint32_t get_round_trip_delay(srac_data_set_t *srac_data)
{
    int32_t local_fine_tuned_delay;
    int32_t remote_fine_tuned_delay;
    uint16_t local_receiver_waited_time;
    uint32_t round_trip_delay;

    local_fine_tuned_delay = get_fine_tuned_delay(&srac_data->local_data);
    remote_fine_tuned_delay = get_fine_tuned_delay(&srac_data->remote_data);
    local_receiver_waited_time = srac_data->local_data.receiver_waited_time_0 | (srac_data->local_data.receiver_waited_time_1 << 8);
    round_trip_delay = local_fine_tuned_delay + remote_fine_tuned_delay + (uint32_t)local_receiver_waited_time * SRAC_SCALING_FACTOR;

    return round_trip_delay;
}

/** @brief Calculate the additional fine-tuned delay value.
 * *
 *  @param[in]  metrics  Preamble Phase Correlation metrics.
 *
 *  @return Additional fine-tuned delay value.
 */
static int32_t get_fine_tuned_delay(srac_data_t *metrics)
{
    int32_t window_energy[4];
    int32_t energy_difference;
    int32_t initial_delay;
    int32_t energy_ratio;
    int32_t extra_delay;
    int32_t fine_delay;

    if ((metrics->phase_correlation_metric_1 + metrics->phase_correlation_metric_2) > (metrics->phase_correlation_metric_3 +
        metrics->phase_correlation_metric_4)) {
        if ((metrics->phase_correlation_metric_1) < 0 && (metrics->phase_correlation_metric_2 < 0)) {
            window_energy[0] = -metrics->phase_correlation_metric_1;
            window_energy[1] = -metrics->phase_correlation_metric_2;
            window_energy[2] = -metrics->phase_correlation_metric_3;
            window_energy[3] = -metrics->phase_correlation_metric_4;
            initial_delay = 0;
        } else if ((metrics->phase_correlation_metric_1) >= 0 && (metrics->phase_correlation_metric_2 >= 0)) {
            window_energy[0] = -metrics->phase_correlation_metric_3;
            window_energy[1] = -metrics->phase_correlation_metric_4;
            window_energy[2] =  metrics->phase_correlation_metric_1;
            window_energy[3] =  metrics->phase_correlation_metric_2;
            initial_delay = SRAC_ONE_HALF;
        } else {
            window_energy[0] = -metrics->phase_correlation_metric_2;
            window_energy[1] = -metrics->phase_correlation_metric_3;
            window_energy[2] = -metrics->phase_correlation_metric_4;
            window_energy[3] =  metrics->phase_correlation_metric_1;
            initial_delay = SRAC_ONE_QUARTER;
        }
    } else {
        if ((metrics->phase_correlation_metric_3) < 0 && (metrics->phase_correlation_metric_4 < 0)) {
            window_energy[0] = -metrics->phase_correlation_metric_1;
            window_energy[1] = -metrics->phase_correlation_metric_2;
            window_energy[2] = -metrics->phase_correlation_metric_3;
            window_energy[3] = -metrics->phase_correlation_metric_4;
            initial_delay = 0;
        } else if ((metrics->phase_correlation_metric_3) >= 0 && (metrics->phase_correlation_metric_4 >= 0)) {
            window_energy[0] =  metrics->phase_correlation_metric_3;
            window_energy[1] =  metrics->phase_correlation_metric_4;
            window_energy[2] = -metrics->phase_correlation_metric_1;
            window_energy[3] = -metrics->phase_correlation_metric_2;
            initial_delay = -SRAC_ONE_HALF;
        } else {
            window_energy[0] =  metrics->phase_correlation_metric_4;
            window_energy[1] = -metrics->phase_correlation_metric_1;
            window_energy[2] = -metrics->phase_correlation_metric_2;
            window_energy[3] = -metrics->phase_correlation_metric_3;
            initial_delay = -SRAC_ONE_QUARTER;
        }
    }

    if (window_energy[1] > window_energy[2]) {
        energy_difference = window_energy[2] - window_energy[0];
        if (abs(energy_difference) >= abs(window_energy[1])) {
            extra_delay = SRAC_ONE_EIGHTH * get_value_sign(energy_difference);
            fine_delay = initial_delay + extra_delay;
        } else {
            energy_difference = energy_difference * SRAC_SCALING_FACTOR;
            energy_ratio = energy_difference / window_energy[1];
            extra_delay = SRAC_ONE_EIGHTH * energy_ratio;
            extra_delay = extra_delay / SRAC_SCALING_FACTOR;
            fine_delay = initial_delay + extra_delay;
        }
    } else {
        energy_difference = window_energy[1] - window_energy[3];
        if (abs(energy_difference) >= abs(window_energy[2])) {
            extra_delay = SRAC_ONE_EIGHTH * get_value_sign(energy_difference);
            extra_delay = initial_delay - extra_delay;
            fine_delay = extra_delay + SRAC_ONE_QUARTER;
        } else {
            energy_difference = energy_difference * SRAC_SCALING_FACTOR;
            energy_ratio = energy_difference / window_energy[2];
            extra_delay = SRAC_ONE_EIGHTH * energy_ratio;
            extra_delay = extra_delay / SRAC_SCALING_FACTOR;
            fine_delay = initial_delay - extra_delay + SRAC_ONE_QUARTER;
        }
    }

    return fine_delay;
}

/** @brief Determine if value is positive or negative.
 *
 *  @param[in] value  Value to evaluate.
 *
 *  @return 1 or -1.
 */
static int32_t get_value_sign(int32_t value)
{
    return (value > 0) - (value < 0);
}
