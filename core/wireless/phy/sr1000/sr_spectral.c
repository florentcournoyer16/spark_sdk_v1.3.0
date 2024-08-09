/** @file sr_spectral.c
 *  @brief SR1000 spectral.
 *
 *  Functions related to radio RF spectrum shaping.
 *
 *  @copyright Copyright (C) 2020-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sr_spectral.h"

/* CONSTANTS ******************************************************************/
static const tx_power_settings_t tx_power_lookup[] = {
    {2, 2, 3}, /* -9.9 dBFs */
    {2, 3, 3}, /* -9.0 dBFs */
    {2, 3, 1}, /* -8.1 dBFs */
    {2, 3, 0}, /* -7.2 dBFs */
    {2, 4, 0}, /* -6.3 dBFs */
    {2, 5, 0}, /* -5.4 dBFs */
    {2, 6, 0}, /* -4.5 dBFs */
    {3, 5, 0}, /* -3.6 dBFs */
    {3, 6, 0}, /* -2.7 dBFs */
    {4, 6, 0}, /* -1.8 dBFs */
    {5, 6, 0}, /* -0.9 dBFs */
    {6, 6, 0}, /* -0.0 dBFs */
    {1, 0, 1}, /* Recommended short-range power setting (Ranging) */
    {3, 3, 0}, /* Recommended long-range power setting (Ranging) */
};

enum spectral_pulse_position {
    COMM_POSITION    = 2, /* Follow datasheet convention, pulses starts at 1 and ends at 12 */
    RANGING_POSITION = 11
};

enum spectral_pulse_spacing {
    PULSE_SPACING_COMM         = 1,
    PULSE_SPACING_RANGING      = 1,
    PULSE_SPACING_RANGING_LONG = 0
};

#define PULSE_WIDTH_MAX_VAL            7
#define PULSE_WIDTH_MIN_VAL            2
#define RANGING_POWER_INDEX_START      12
#define LONG_RANGE_POWER_SETTING_INDEX 13

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void set_pulses(calib_vars_t *calib_vars, uint8_t *pattern, uint8_t *used_pulse_size, uint8_t start_pos, uint8_t pulse_count,
                       uint8_t pulse_spacing, uint8_t pulse_width, uint8_t frequency, bool freq_shift_enable);

static uint8_t set_pulses_increment(calib_vars_t *calib_vars, uint8_t *pattern, uint8_t start_pos, uint8_t pulse_count,
                                    uint8_t pulse_spacing, uint8_t pulse_width, uint8_t frequency, bool freq_shift_enable);
static void set_pulses_decrement(calib_vars_t *calib_vars, uint8_t *pattern, uint8_t start_pos, uint8_t pulse_count, uint8_t pulse_spacing,
                                 uint8_t pulse_width, uint8_t frequency, bool freq_shift_enable);

/* PUBLIC FUNCTIONS ***********************************************************/
void config_spectrum(calib_vars_t *calib_vars, uint16_t frequency, tx_power_t tx_power, rf_channel_t *rf_channel)
{
    dcro_calib_t dcro_calib_rx;
    uint8_t lna_peak_value;
    uint8_t start_pos;
    uint8_t pulse_count;
    uint8_t pulse_spacing;
    int8_t pulse_width;

    /* Pulses */
    if (tx_power >= RANGING_POWER_INDEX_START) {
        start_pos     = RANGING_POSITION;
        tx_power      = tx_power - RANGING_POWER_INDEX_START;
        pulse_spacing = PULSE_SPACING_RANGING;
        if (tx_power == LONG_RANGE_POWER_SETTING_INDEX) {
            pulse_spacing = PULSE_SPACING_RANGING_LONG;
        }
    } else {
        start_pos     = COMM_POSITION;
        pulse_spacing = PULSE_SPACING_COMM;
    }

    pulse_count = tx_power_lookup[tx_power].pulse_count;

    if (pulse_count == 1) {
        rf_channel->integgain = 1;
    } else {
        rf_channel->integgain = 0;
    }

    if (tx_power_lookup[tx_power].pulse_width <= PULSE_WIDTH_MIN_VAL) {
        pulse_width = tx_power_lookup[tx_power].pulse_width;
    } else {
        pulse_width = (int8_t)tx_power_lookup[tx_power].pulse_width + calib_vars->pulse_width_offset;
        if (pulse_width > PULSE_WIDTH_MAX_VAL) {
            pulse_width = PULSE_WIDTH_MAX_VAL;
        }
    }

    set_pulses(calib_vars, rf_channel->channel.tx_pattern, &rf_channel->pulse_size, start_pos, pulse_count, pulse_spacing,
               (uint8_t)pulse_width, frequency, true);

    /* RX filter */
    dcro_calib_rx                 = sr_calib_frequency_to_code(calib_vars, frequency, true);
    lna_peak_value                = sr_calib_get_lna_peak(calib_vars, frequency);
    rf_channel->channel.rx_filter = MOV2MASK(lna_peak_value, BITS_LNAPEAK) | MOV2MASK(dcro_calib_rx.code, BITS_RFFILFREQ);

    /* Transmitter parameters */
    rf_channel->channel.param = BIT_RNDPHASE | (tx_power_lookup[tx_power].tx_gain << 4);
}

void config_spectrum_advance(calib_vars_t *calib_vars, channel_cfg_t *channel_cfg, rf_channel_t *rf_channel)
{
    dcro_calib_t dcro_calib_rx;
    uint8_t lna_peak_value;
    int8_t pulse_width;

    rf_channel->integgain = channel_cfg->integgain;

    if (channel_cfg->power->pulse_width <= PULSE_WIDTH_MIN_VAL) {
        pulse_width = channel_cfg->power->pulse_width;
    } else {
        pulse_width = (int8_t)channel_cfg->power->pulse_width + calib_vars->pulse_width_offset;
        if (pulse_width > PULSE_WIDTH_MAX_VAL) {
            pulse_width = PULSE_WIDTH_MAX_VAL;
        }
    }

    set_pulses(calib_vars, rf_channel->channel.tx_pattern, &rf_channel->pulse_size, channel_cfg->pulse_start_pos,
               channel_cfg->power->pulse_count, channel_cfg->pulse_spacing, (uint8_t)pulse_width, channel_cfg->frequency,
               channel_cfg->freq_shift_enable);

    /* RX filter */
    dcro_calib_rx                 = sr_calib_frequency_to_code(calib_vars, channel_cfg->frequency, true);
    lna_peak_value                = sr_calib_get_lna_peak(calib_vars, channel_cfg->frequency);
    rf_channel->channel.rx_filter = MOV2MASK(lna_peak_value, BITS_LNAPEAK) | MOV2MASK(dcro_calib_rx.code, BITS_RFFILFREQ);

    /* Transmitter parameters */
    rf_channel->channel.param = channel_cfg->rdn_phase_enable | MOV2MASK(channel_cfg->power->tx_gain, BITS_TXPOWER);
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Generate pulses pattern.
 *
 *  @param[in]  calib_vars        Calibration variables.
 *  @param[out] pattern           Pulses pattern.
 *  @param[in]  start_pos         First pulse position.
 *  @param[in]  pulse_count       Number of pulses.
 *  @param[in]  pulse_spacing     Spacing between pulses.
 *  @param[in]  pulse_width       Pulse width.
 *  @param[in]  frequency         Channel's center frequency.
 *  @param[in]  freq_shift_enable Enable frequency shifting.
 */
static void set_pulses(calib_vars_t *calib_vars, uint8_t *pattern, uint8_t *used_pulse_size, uint8_t start_pos, uint8_t pulse_count,
                       uint8_t pulse_spacing, uint8_t pulse_width, uint8_t frequency, bool freq_shift_enable)
{
    /* Clear spectrum */
    memset(pattern, 0, NB_PULSES);

    if (pulse_count > NB_PULSES) {
        pulse_count = NB_PULSES;
    }

    /* Check if we need to increment of decrement through the pulse index */
    if (start_pos <= (NB_PULSES / 2)) {
        *used_pulse_size = set_pulses_increment(calib_vars, pattern, start_pos, pulse_count, pulse_spacing, pulse_width, frequency,
                                                freq_shift_enable);
    } else {
        set_pulses_decrement(calib_vars, pattern, start_pos, pulse_count, pulse_spacing, pulse_width, frequency, freq_shift_enable);
        *used_pulse_size = 0;
    }
}

/** @brief Generate pulses pattern, increment position.
 *
 *  @param[in]  calib_vars        Calibration variables.
 *  @param[out] pattern           Pulses pattern.
 *  @param[in]  start_pos         First pulse position.
 *  @param[in]  pulse_count       Number of pulses.
 *  @param[in]  pulse_spacing     Spacing between pulses.
 *  @param[in]  pulse_width       Pulse width.
 *  @param[in]  frequency         Channel's center frequency.
 *  @param[in]  freq_shift_enable Enable frequency shifting.
 */
static uint8_t set_pulses_increment(calib_vars_t *calib_vars, uint8_t *pattern, uint8_t start_pos, uint8_t pulse_count,
                                    uint8_t pulse_spacing, uint8_t pulse_width, uint8_t frequency, bool freq_shift_enable)
{
    dcro_calib_t dcro_calib_tx;
    uint8_t pulse_freq_code_tx;
    int8_t i_begin;
    int8_t i_end;
    int8_t i_decrement;
    bool do_freq_shift = false;

    /* i(11) correspond to pulse 1, need to decrement index to increment pulse position */
    i_begin = TX_PATTERN_BYTE_COUNT - start_pos;
    i_end   = (i_begin) - ((pulse_count - 1) * (pulse_spacing + 1));

    if (i_end < 0) {
        i_end = 0;
    }

    i_decrement = (1 + pulse_spacing);

    /* TX pulse pattern */
    dcro_calib_tx = sr_calib_frequency_to_code(calib_vars, frequency, false);
    for (int i = i_begin; i >= i_end; i -= i_decrement) {
        if (do_freq_shift & freq_shift_enable) {
            pulse_freq_code_tx = sr_calib_get_next_dcro_code(calib_vars, dcro_calib_tx.code, false);
        } else {
            pulse_freq_code_tx = dcro_calib_tx.code;
        }
        do_freq_shift = !do_freq_shift;

        pattern[i] = MOV2MASK(pulse_width, BITS_PULSEWID) | MOV2MASK(pulse_freq_code_tx, BITS_PULSEFREQ);
    }

    return i_end;
}

/** @brief Generate pulses pattern, decrement position.
 *
 *  @param[in]  calib_vars        Calibration variables.
 *  @param[out] pattern           Pulses pattern.
 *  @param[in]  start_pos         First pulse position.
 *  @param[in]  pulse_count       Number of pulses.
 *  @param[in]  pulse_spacing     Spacing between pulses.
 *  @param[in]  pulse_width       Pulse width.
 *  @param[in]  frequency         Channel's center frequency.
 *  @param[in]  freq_shift_enable Enable frequency shifting.
 */
static void set_pulses_decrement(calib_vars_t *calib_vars, uint8_t *pattern, uint8_t start_pos, uint8_t pulse_count, uint8_t pulse_spacing,
                                 uint8_t pulse_width, uint8_t frequency, bool freq_shift_enable)
{
    dcro_calib_t dcro_calib_tx;
    int8_t pulse_freq_code_tx;
    int8_t i_begin;
    int8_t i_end;
    int8_t i_increment;
    bool do_freq_shift = false;

    /* i(0) correspond to pulse 12, need to increment index to decrement pulse position */
    i_begin = TX_PATTERN_BYTE_COUNT - start_pos;
    i_end   = i_begin + ((pulse_count - 1) * (1 + pulse_spacing)) + 1;

    if (i_end > NB_PULSES) {
        i_end = NB_PULSES;
    }

    i_increment = (1 + pulse_spacing);

    /* TX pulse pattern */
    dcro_calib_tx = sr_calib_frequency_to_code(calib_vars, frequency, false);
    for (int i = i_begin; i < i_end; i += i_increment) {
        if (do_freq_shift & freq_shift_enable) {
            pulse_freq_code_tx = sr_calib_get_next_dcro_code(calib_vars, dcro_calib_tx.code, false);
        } else {
            pulse_freq_code_tx = dcro_calib_tx.code;
        }
        do_freq_shift = !do_freq_shift;

        pattern[i] = MOV2MASK(pulse_width, BITS_PULSEWID) | MOV2MASK(pulse_freq_code_tx, BITS_PULSEFREQ);
    }
}
