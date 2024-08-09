/** @file sr_spectral.h
 *   @brief SR1000 spectral.
 *
 *   Functions related to radio RF spectrum shaping.
 *
 *   @copyright Copyright (C) 2020-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *   @author    SPARK FW Team.
 */
#ifndef SR_SPECTRAL_H_
#define SR_SPECTRAL_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sr_calib.h"
#include "sr_def.h"
#include "wps_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
typedef struct {
    uint8_t rx_filter;                         /*!< RX filter settings */
    uint8_t tx_pattern[TX_PATTERN_BYTE_COUNT]; /*!< TX pulse pattern */
    uint8_t param;                             /*!< Transmitter parameters */
} rf_channel_settings_t;

typedef struct {
    rf_channel_settings_t channel; /*!< Radio channel settings */
    uint8_t integgain;             /*!< Receiver integrators gain */
    uint8_t pulse_size;            /*!< Size of the TX pulse pattern */
} rf_channel_t;

typedef struct {
    uint8_t pulse_count;
    uint8_t pulse_width;
    uint8_t tx_gain;
} tx_power_settings_t;

typedef struct {
    tx_power_settings_t *power;
    uint16_t frequency;
    uint8_t pulse_spacing;
    uint8_t pulse_start_pos;
    rnd_phase_t rdn_phase_enable;
    uint8_t integgain;
    bool freq_shift_enable;
} channel_cfg_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Configure radio spectrum.
 *
 *  @param[in]  calib_vars  Calibration variables.
 *  @param[in]  frequency   Channel's center frequency.
 *  @param[in]  tx_power    Power level.
 *  @param[out] rf_channel  RF channel.
 */
void config_spectrum(calib_vars_t *calib_vars, uint16_t frequency, tx_power_t tx_power, rf_channel_t *rf_channel);

/** @brief Configure radio spectrum.
 *
 *  @param[in]  calib_vars   Calibration variables.
 *  @param[in]  channel_cfg  Channel's configuration.
 *  @param[out] rf_channel   RF channel.
 */
void config_spectrum_advance(calib_vars_t *calib_vars, channel_cfg_t *channel_cfg, rf_channel_t *rf_channel);

#ifdef __cplusplus
}
#endif
#endif /* SR_SPECTRAL_H_ */
