/** @file  sr_calib.h
 *  @brief SR1000 calibration module.
 *
 *  Functions to calibrate many aspects of the radio such
 *  as the delay-line and the VCRO.
 *
 *  @copyright Copyright (C) 2018-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_CALIB_H_
#define SR_CALIB_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sr_api.h"
#include "sr_nvm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define DCRO_CODE_COUNT_MAX 32

/* TYPES **********************************************************************/

/** @brief Digitally-controlled ring oscillator calibration structure.
 */
typedef struct {
    bool lower_than_target; /**< Flag use to know if calibration value is better from a list */
    uint8_t code;           /**< DCRO calibration code */
} dcro_calib_t;

/** @brief Calibration variables structure.
 */
typedef struct {
    uint64_t chip_id;                               /**< Chip ID of the transceiver */
    uint8_t phy_model;                              /**< Model of the transceiver */
    uint8_t phy_package;                            /**< Package of the transceiver */
    uint8_t phy_version;                            /**< Product id version */
    uint16_t binning_setup_code;                    /**< Code identifying the binning setup used for this chip */
    uint8_t resistune;                              /**< Resistance tuning value to calibrate the PLL and band gap*/
    const uint16_t *lna_peak_table;                 /**< Low noise amplifier peak frequency table */
    uint8_t new_dcro_codes_tx[DCRO_CODE_COUNT_MAX]; /**< Corrected DCRO codes to use for TX */
    uint8_t new_dcro_codes_rx[DCRO_CODE_COUNT_MAX]; /**< Corrected DCRO codes to use for RX */
    uint8_t new_dcro_codes_rx_size;                 /**< Number of corrected DCRO codes to use for RX */
    uint8_t new_dcro_codes_tx_size;                 /**< Number of Corrected DCRO codes to use for TX */
    uint16_t nvm_vcro_shift;                        /**< VCRO shift value stored in NVM */
    uint16_t freq_table_tx[DCRO_CODE_COUNT_MAX];    /**< TX frequency table */
    uint16_t freq_table_rx[DCRO_CODE_COUNT_MAX];    /**< RX frequency table */
    int8_t pulse_width_offset;                      /**< Pulse width offset power tuning */
    int8_t vref_tune_offset;                        /**< Vref tune offset power tuning */
} calib_vars_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the calibration module.
 *
 */
void sr_calib_init(calib_vars_t *calib_vars);

/** @brief Perform the SPARK radio calibration.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] calib_vars  Calibration variable structure, allocated.
 *  @param[in] nvm         NVM structure, allocated and initialized.
 */
void sr_calibrate(radio_t *radio, calib_vars_t *calib_vars, nvm_t *nvm);

/** @brief Set vref tune offset.
 *
 *  This function adds the vref tune offset to the vref tune value.
 *
 *  @param[in] radio  Radio's instance.
 *  @return Resistune register value with vref tune adjusted.
 */
uint8_t sr_calib_vref_tune_offset(calib_vars_t *calib_vars);

/** @brief Calibrate the delay of the delay-line.
 *
 *  This is a software calibration of the delay-line. It verifies if the delay-line
 *  is too fast or too slow compared to a 25ns or 50ns clock period and adjusts it accordingly.
 *
 *  @param[in] radio  Radio's instance.
 *  @return Delay line tuning 4-bit value.
 */
uint8_t sr_calib_tune_delay_line(radio_t *radio);

/** @brief Run the frequency calibration.
 *
 *  This makes use of the on-chip frequency calibration block to
 *  populate a lookup table of all the band frequency values.
 *  The index of the lookup table is the 4-bit DCRO code.
 *
 *  @param[in] radio       Radio's instance.
 *  @param[in] calib_vars  Calibration variables.
 */
void sr_calib_run_frequency_calibration(radio_t *radio, calib_vars_t *calib_vars);

/** @brief Get the frequency associated with a 4-bit DCRO code.
 *
 *  @param[in] calib_vars  Calibration variables.
 *  @param[in] code        4-bit DCRO code.
 *  @param[in] rx_table    If true, use RX table, otherwise use TX table.
 *  @return Frequency in multiples of 40.96 MHz.
 */
uint16_t sr_calib_code_to_frequency(calib_vars_t *calib_vars, uint8_t code, bool rx_table);

/** @brief Get the closest 4-bit DCRO code from a given frequency.
 *
 *  @param[in] calib_vars  Calibration variables.
 *  @param[in] freq        Frequency in multiples of 40.96 MHz.
 *  @param[in] rx_table    If true, use RX table, otherwise use TX table.
 *  @return DCRO calibration structure.
 */
dcro_calib_t sr_calib_frequency_to_code(calib_vars_t *calib_vars, uint16_t freq, bool rx_table);

/** @brief Get the LNA peak value from a 4-bit frequency code.
 *
 *  @param[in] calib_vars  Calibration variables.
 *  @param[in] freq        Frequency in multiples of 40.96 MHz.
 *  @return LNA peak value.
 */
uint8_t sr_calib_get_lna_peak(calib_vars_t *calib_vars, uint16_t freq);

/** @brief Get the next valid DCRO code.
 *
 *  @param[in] calib_vars  Calibration variables.
 *  @param[in] code        Initial DCRO code.
 *  @param[in] rx_table    If true, use RX table, otherwise use TX table.
 *  @return Next DCRO code.
 */
uint8_t sr_calib_get_next_dcro_code(calib_vars_t *calib_vars, uint8_t code, bool rx_table);

/** @brief Check which calibration algorithm is in use.
 *
 *  @retval True   Version 1 is used.
 *  @retval False  Version 2 is used.
 */
uint8_t sr_calib_v1_in_use(calib_vars_t *calib_vars);

#ifdef __cplusplus
}
#endif
#endif /* SR_CALIB_H_ */
