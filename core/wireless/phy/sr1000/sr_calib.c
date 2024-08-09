/** @file  sr_calib.c
 *  @brief SR1000 calibration module.
 *
 *  Functions to calibrate many aspects of the radio such
 *  as the delay-line and the VCRO.
 *
 *  @copyright Copyright (C) 2018 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sr_calib.h"
#include <string.h>
#include "sr_access.h"
#include "sr_def.h"¸

#include <zephyr/kernel.h>

/* CONSTANTS ******************************************************************/
#define DCRO_CODE_AVERAGING_COUNT 100
#define DCRO_FREQ_RESOLUTION      40.96
#define DL_TUNE_VALUE_COUNT       16
#define LNA_PEAK_TABLE_SIZE       8
#define VREF_TUNE_MAX_VAL         15
#define LOWEST_VALID_DCRO_FREQ    50
#define RX_GAIN_VALUE             0x88
/* MACROS *********************************************************************/
#define BINNING_WAS_DONE_WITH_ATE(binning_setup_code) ((binning_setup_code) == ATE_BINNING_SETUP_CODE)

/* PRIVATE GLOBALS ************************************************************/
static const uint16_t lna_peak_table_sr1010[] = {
    5290 / DCRO_FREQ_RESOLUTION, 4900 / DCRO_FREQ_RESOLUTION, 4510 / DCRO_FREQ_RESOLUTION, 4240 / DCRO_FREQ_RESOLUTION,
    3970 / DCRO_FREQ_RESOLUTION, 3790 / DCRO_FREQ_RESOLUTION, 3610 / DCRO_FREQ_RESOLUTION, 3460 / DCRO_FREQ_RESOLUTION,
};

/* Add 120MHz to handle RX frequency offset */
static const uint16_t lna_peak_table_sr1020[] = {
    (8840 + 120) / DCRO_FREQ_RESOLUTION, (8420 + 120) / DCRO_FREQ_RESOLUTION, (8000 + 120) / DCRO_FREQ_RESOLUTION,
    (7700 + 120) / DCRO_FREQ_RESOLUTION, (7340 + 120) / DCRO_FREQ_RESOLUTION, (7040 + 120) / DCRO_FREQ_RESOLUTION,
    (6800 + 120) / DCRO_FREQ_RESOLUTION, (6620 + 120) / DCRO_FREQ_RESOLUTION,
};

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void sr_calib_fill_freq_table(radio_t *radio, uint16_t *freq_table, bool rx);
static uint8_t sr_calib_v1_find_new_dcro_code(uint16_t *freq_table, uint8_t code, int8_t offset);
static void sr_calib_v1_fill_new_dcro_codes_table(calib_vars_t *calib_vars, uint16_t *freq_table, bool rx, uint8_t *new_dcro_codes);
static void sr_calib_v1_apply_dcro_codes_offsets(uint16_t *freq_table, uint8_t *new_dcro_codes);
static uint8_t sr_calib_v2_find_next_dcro_code(uint8_t code, uint8_t *new_dcro_codes, uint8_t new_dcro_codes_count);
static uint8_t sr_calib_v2_clean_up_freq_table(uint16_t *freq_table, uint8_t *new_dcro_codes);
static void sr_calib_v2_apply_vcro_offsets(calib_vars_t *calib_vars, uint16_t *freq_table, bool rx);
static int8_t offset_from_bits(uint8_t bits);
static bool use_calib_v1(calib_vars_t *calib_vars);

/* PUBLIC FUNCTIONS ***********************************************************/
void sr_calib_init(calib_vars_t *calib_vars)
{
    if (calib_vars->phy_model == PHY_MODEL_SR1020) {
        calib_vars->lna_peak_table = lna_peak_table_sr1020;
    } else {
        calib_vars->phy_model      = PHY_MODEL_SR1010;
        calib_vars->lna_peak_table = lna_peak_table_sr1010;
    }
}

void sr_calibrate(radio_t *radio, calib_vars_t *calib_vars, nvm_t *nvm)
{
    uint8_t reg_resistune;

    /* Get values from NVM */
    calib_vars->chip_id            = sr_nvm_get_serial_number_chip_id(nvm);
    calib_vars->phy_model          = sr_nvm_get_product_id_model(nvm);
    calib_vars->phy_package        = sr_nvm_get_product_id_package(nvm);
    calib_vars->phy_version        = sr_nvm_get_product_id_version(nvm);
    calib_vars->binning_setup_code = sr_nvm_get_serial_number_binning_setup_code(nvm);
    calib_vars->resistune          = sr_nvm_get_calibration(nvm);
    calib_vars->nvm_vcro_shift     = sr_nvm_get_vcro_shift(nvm);
    calib_vars->pulse_width_offset = sr_nvm_get_vref_adjust_pulse_width_offset(nvm);
    calib_vars->vref_tune_offset   = sr_nvm_get_vref_adjust_vref_tune_offset(nvm);

    sr_calib_init(calib_vars);

    reg_resistune = sr_calib_vref_tune_offset(calib_vars) | BIT_LNAIMPED;

    uwb_set_resistune(radio, reg_resistune);
    uwb_transfer_blocking(radio);

    sr_calib_tune_delay_line(radio);
    sr_calib_run_frequency_calibration(radio, calib_vars);

    uwb_write_register_8(radio, REG_CONSTGAINS, RX_GAIN_VALUE);
    uwb_write_register_8(radio, REG_DEBUGMODEM, BITS_MANUPHASE);
    uwb_transfer_blocking(radio);

    radio->phy_version    = calib_vars->phy_version;
    calib_vars->resistune = reg_resistune;
}

uint8_t sr_calib_vref_tune_offset(calib_vars_t *calib_vars)
{
    int8_t vref_tune = (int8_t)(calib_vars->resistune & 0x0F);

    vref_tune += calib_vars->vref_tune_offset;
    if (vref_tune > VREF_TUNE_MAX_VAL) {
        vref_tune = VREF_TUNE_MAX_VAL;
    } else if (vref_tune < 0) {
        vref_tune = 0;
    }
    uint8_t reg_resistune = (calib_vars->resistune & 0xF0) | (vref_tune & 0x0F);

    return reg_resistune;
}

uint8_t sr_calib_tune_delay_line(radio_t *radio)
{
    uint8_t dl_tune;
    uint8_t *read_reg;

    uwb_write_register_8(radio, REG_SLEEPCONF, 0x00);
    uwb_write_register_8(radio, REG_TIMERCONF, 0x00);
    /*
     * Must be in RX mode but the calibration will apply
     * for both transmission and reception of frames.
     */
    uwb_write_register_8(radio, REG_ACTIONS, BIT_RXMODE);

    /* Wait until radio is awake. */
    do {
        read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
        uwb_transfer_blocking(radio);
    } while (!(*read_reg & BIT_AWAKE));

    /*
     * Each increment of the delay line tuning value corresponds to an
     * increase of each delay by approximately 110 picoseconds.
     */
    for (dl_tune = 0x0; dl_tune < DL_TUNE_VALUE_COUNT; dl_tune++) {
        /* Set new delay line tuning value. */
        uwb_write_register_8(radio, REG_DLLTUNING, MOV2MASK(dl_tune, BITS_DLTUNE));
        /*
         * Stop tuning when the delay line starts
         * to lag the symbol rate in frequency (slower).
         */
        read_reg = uwb_read_register_8(radio, REG_DLLTUNING);
        uwb_transfer_blocking(radio);
        if (*read_reg & BIT_LEADLAG) {
            break;
        }
    }
    /* Override RF GAIN and IFOA GAIN to max gain value for Carrier Sensing and set INTEGGAIN value to default */
    uwb_set_dll_tuning(radio, SET_DLLTUNING(DLL_TUNING_CLEAR, DLTUNE_RAW_TO_REG(dl_tune), ECO_ENABLE, INTEGGAIN_RAW_TO_REG(0b01)));
    uwb_write_register_8(radio, REG_ACTIONS, BIT_GOTOSLP);
    uwb_transfer_blocking(radio);

    /* Make sure we return only 4 bits */
    return (dl_tune & 0x0f);
}

uint16_t sr_calib_code_to_frequency(calib_vars_t *calib_vars, uint8_t code, bool rx_table)
{
    uint16_t freq = 0;
    uint8_t *new_dcro_codes;
    uint8_t dcro_code_count;
    uint16_t *freq_table;

    if (rx_table) {
        freq_table      = calib_vars->freq_table_rx;
        new_dcro_codes  = calib_vars->new_dcro_codes_rx;
        dcro_code_count = calib_vars->new_dcro_codes_rx_size;
    } else {
        freq_table      = calib_vars->freq_table_tx;
        new_dcro_codes  = calib_vars->new_dcro_codes_tx;
        dcro_code_count = calib_vars->new_dcro_codes_tx_size;
    }

    if (use_calib_v1(calib_vars)) {
        freq = freq_table[code];
    } else {
        for (uint8_t i = 0; i < dcro_code_count; i++) {
            if (code == new_dcro_codes[i]) {
                freq = freq_table[i];
                break;
            }
        }
    }

    return freq;
}

dcro_calib_t sr_calib_frequency_to_code(calib_vars_t *calib_vars, uint16_t freq, bool rx_table)
{
    dcro_calib_t dcro_calib;
    uint8_t *new_dcro_codes;
    uint8_t dcro_code_count;
    uint16_t *freq_table;

    if (rx_table) {
        freq           -= 3;
        freq_table      = calib_vars->freq_table_rx;
        new_dcro_codes  = calib_vars->new_dcro_codes_rx;
        dcro_code_count = calib_vars->new_dcro_codes_rx_size;
    } else {
        freq_table      = calib_vars->freq_table_tx;
        new_dcro_codes  = calib_vars->new_dcro_codes_tx;
        dcro_code_count = calib_vars->new_dcro_codes_tx_size;
    }

    if (use_calib_v1(calib_vars)) {
        for (dcro_calib.code = 1; dcro_calib.code < dcro_code_count; dcro_calib.code++) {
            if (freq > freq_table[dcro_calib.code]) {
                if ((freq - freq_table[dcro_calib.code]) > (freq_table[dcro_calib.code - 1] - freq)) {
                    /* Previous code is better */
                    dcro_calib.code--;
                    dcro_calib.lower_than_target = true;
                } else {
                    /* Current code is better */
                    dcro_calib.lower_than_target = false;
                }
                break;
            } else if (dcro_calib.code == (dcro_code_count - 1)) {
                /* Reached the end of the table, so use current code */
                dcro_calib.lower_than_target = false;
                break;
            }
        }
    } else {
        for (uint8_t i = 1; i < dcro_code_count; i++) {
            dcro_calib.code = new_dcro_codes[i];
            if (freq > freq_table[i]) {
                if ((freq - freq_table[i]) > (freq_table[i - 1] - freq)) {
                    /* Previous code is better */
                    dcro_calib.code              = new_dcro_codes[i - 1];
                    dcro_calib.lower_than_target = true;
                } else {
                    /* Current code is better */
                    dcro_calib.lower_than_target = false;
                }
                break;
            } else if (i == (dcro_code_count - 1)) {
                /* Reached the end of the table, so use current code */
                dcro_calib.lower_than_target = false;
                break;
            }
        }
    }

    return dcro_calib;
}

void sr_calib_run_frequency_calibration(radio_t *radio, calib_vars_t *calib_vars)
{
    uint8_t dcro_code;
    uint8_t *read_reg;

    /* Wake up in idle mode */
    uwb_write_register_8(radio, REG_ACTIONS, 0);
    do {
        read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
        uwb_transfer_blocking(radio);
    } while (!(*read_reg & BIT_AWAKE));
    sr_calib_fill_freq_table(radio, calib_vars->freq_table_tx, false);

    /* Wake up in RX mode */
    uwb_write_register_8(radio, REG_ACTIONS, BIT_RXMODE);
    do {
        read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
        uwb_transfer_blocking(radio);
    } while (!(*read_reg & BIT_AWAKE));
    sr_calib_fill_freq_table(radio, calib_vars->freq_table_rx, true);

    if (use_calib_v1(calib_vars)) {
        if (calib_vars->phy_model == PHY_MODEL_SR1010) {
            /* Extrapolate TX from idle and RX frequencies */
            for (dcro_code = 0; dcro_code < DCRO_CODE_COUNT_MAX; dcro_code++) {
                calib_vars->freq_table_tx[dcro_code] = calib_vars->freq_table_rx[dcro_code];
            }
        } else if (calib_vars->phy_model == PHY_MODEL_SR1020) {
            /* Check for overflow at frequencies > 10 GHz */
            for (dcro_code = 0; dcro_code < DCRO_CODE_COUNT_MAX / 2 - 1; dcro_code++) {
                if (calib_vars->freq_table_tx[dcro_code] < calib_vars->freq_table_tx[DCRO_CODE_COUNT_MAX / 2 - 1]) {
                    calib_vars->freq_table_tx[dcro_code] = 0xff;
                }
                if (calib_vars->freq_table_rx[dcro_code] < calib_vars->freq_table_rx[DCRO_CODE_COUNT_MAX / 2 - 1]) {
                    calib_vars->freq_table_rx[dcro_code] = 0xff;
                }
            }
        } else { /* Should never go there */
            /* TODO: Create assert() in the SR1000 HAL */
            while (1)
                ;
        }

        sr_calib_v1_fill_new_dcro_codes_table(calib_vars, calib_vars->freq_table_tx, false, calib_vars->new_dcro_codes_tx);
        sr_calib_v1_apply_dcro_codes_offsets(calib_vars->freq_table_tx, calib_vars->new_dcro_codes_tx);
        sr_calib_v1_fill_new_dcro_codes_table(calib_vars, calib_vars->freq_table_rx, true, calib_vars->new_dcro_codes_rx);
        sr_calib_v1_apply_dcro_codes_offsets(calib_vars->freq_table_rx, calib_vars->new_dcro_codes_rx);

        calib_vars->new_dcro_codes_tx_size = DCRO_CODE_COUNT_MAX;
        calib_vars->new_dcro_codes_rx_size = DCRO_CODE_COUNT_MAX;
    } else {
        if (calib_vars->phy_model == PHY_MODEL_SR1010) {
            /* Extrapolate TX from idle and RX frequencies */
            for (dcro_code = 0; dcro_code < DCRO_CODE_COUNT_MAX; dcro_code++) {
                calib_vars->freq_table_tx[dcro_code] = calib_vars->freq_table_rx[dcro_code];
            }
        }

        sr_calib_v2_apply_vcro_offsets(calib_vars, calib_vars->freq_table_tx, false);
        calib_vars->new_dcro_codes_tx_size = sr_calib_v2_clean_up_freq_table(calib_vars->freq_table_tx, calib_vars->new_dcro_codes_tx);
        sr_calib_v2_apply_vcro_offsets(calib_vars, calib_vars->freq_table_rx, true);
        calib_vars->new_dcro_codes_rx_size = sr_calib_v2_clean_up_freq_table(calib_vars->freq_table_rx, calib_vars->new_dcro_codes_rx);
    }
}

uint8_t sr_calib_get_lna_peak(calib_vars_t *calib_vars, uint16_t freq)
{
    uint16_t min_freq;
    uint16_t max_freq;
    uint8_t lna_peak = 0;

    /* Determine the LNA peak value */
    for (uint8_t i = 0; i < LNA_PEAK_TABLE_SIZE; i++) {
        if ((freq == calib_vars->lna_peak_table[i]) || (freq > calib_vars->lna_peak_table[0])) {
            /* freq is equal or greater than the first entry */
            lna_peak = i;
            break;
        } else if (freq > calib_vars->lna_peak_table[i]) {
            min_freq = calib_vars->lna_peak_table[i - 1] - freq;
            max_freq = freq - calib_vars->lna_peak_table[i];
            if (min_freq < max_freq) {
                lna_peak = i - 1;
            } else {
                lna_peak = i;
            }
            break;
        } else {
            lna_peak = LNA_PEAK_TABLE_SIZE - 1;
        }
    }

    return lna_peak;
}

uint8_t sr_calib_get_next_dcro_code(calib_vars_t *calib_vars, uint8_t code, bool rx_table)
{
    uint8_t next_code;
    uint16_t *freq_table;
    uint8_t *code_table;
    uint8_t code_table_size;

    if (use_calib_v1(calib_vars)) {
        if (rx_table) {
            freq_table = calib_vars->freq_table_rx;
        } else {
            freq_table = calib_vars->freq_table_tx;
        }
        next_code = sr_calib_v1_find_new_dcro_code(freq_table, code, 1);
    } else {
        if (rx_table) {
            code_table      = calib_vars->new_dcro_codes_rx;
            code_table_size = calib_vars->new_dcro_codes_rx_size;
        } else {
            code_table      = calib_vars->new_dcro_codes_tx;
            code_table_size = calib_vars->new_dcro_codes_tx_size;
        }
        next_code = sr_calib_v2_find_next_dcro_code(code, code_table, code_table_size);
    }

    return next_code;
}

uint8_t sr_calib_v1_in_use(calib_vars_t *calib_vars)
{
    return use_calib_v1(calib_vars);
}

/* PRIVATE FUNCTIONS **********************************************************/
static void sr_calib_fill_freq_table(radio_t *radio, uint16_t *freq_table, bool rx)
{
    uint8_t dcro_code;
    uint8_t *calib_res;
    uint16_t average_dcro;
    bool invalid_dcro_code;

    for (dcro_code = 0; dcro_code < DCRO_CODE_COUNT_MAX; dcro_code++) {
        invalid_dcro_code = false;
        average_dcro      = 0;
        for (uint8_t compt_count = 0; compt_count < DCRO_CODE_AVERAGING_COUNT; compt_count++) {
            uwb_write_register_8(radio, REG_CALIBREQUEST, dcro_code);
            if (rx) {
                uwb_write_register_8(radio, REG_ACTIONS, BIT_CALIBRAT | BIT_RXMODE);
            } else {
                uwb_write_register_8(radio, REG_ACTIONS, BIT_CALIBRAT);
            }
            uwb_transfer_blocking(radio);
            
            /*
             * Value is ready after ~100ns. We should be fine
             * without explicitly calling a delay function since
             * SPI transaction takes time. To be verified.
             */
            k_sleep(K_USEC(1));
            calib_res = uwb_read_register_8(radio, REG_CALIBRESULT);
            uwb_transfer_blocking(radio);
            /*
             * Flag the DCRO code as invalid if its corresponding
             * frequency is lower than a certain value. This check is
             * to catch overflowing SR1020 DCRO codes (i.e., DCRO codes
             * that would correspond to a frequency higher than 255).
             */
            if (*calib_res < LOWEST_VALID_DCRO_FREQ) {
                invalid_dcro_code = true;
            }
            average_dcro += *calib_res;
        }
        if (invalid_dcro_code) {
            freq_table[dcro_code] = 0;
        } else {
            average_dcro /= DCRO_CODE_AVERAGING_COUNT;
            freq_table[dcro_code] = average_dcro;
        }
    }
}

static uint8_t sr_calib_v1_find_new_dcro_code(uint16_t *freq_table, uint8_t code, int8_t offset)
{
    int8_t new_code = code;

    /*
     * Special treatment is required when crossing the boundary
     * between low and high codes.
     */
    if (((code == 0xf) || (code == 0x0e)) && (offset > 0)) {
        if ((code == 0x0e) && (offset == 2)) {
            code++;
            offset = 1;
        }
        for (uint8_t i = code; i < DCRO_CODE_COUNT_MAX; i++) {
            if ((freq_table[i + 1] - freq_table[code]) < -1) {
                new_code = i + offset;
                break;
            }
        }
    } else if (((code == 0x10) || (code == 0x11)) && (offset < 0)) {
        if ((code == 0x11) && (offset == -2)) {
            code--;
            offset = -1;
        }
        for (int8_t i = code; i >= 0x0; i--) {
            if ((freq_table[i - 1] - freq_table[code]) > 1) {
                new_code = i + offset;
                break;
            }
        }
    } else { /* For other codes, just apply the regular offset */
        new_code = code + offset;
    }

    if (new_code < 0) {
        new_code = 0;
    } else if (new_code > (DCRO_CODE_COUNT_MAX - 1)) {
        new_code = (DCRO_CODE_COUNT_MAX - 1);
    }

    return new_code;
}

static void sr_calib_v1_apply_dcro_codes_offsets(uint16_t *freq_table, uint8_t *new_dcro_codes)
{
    uint8_t new_code;
    uint16_t temp_freq_table[DCRO_CODE_COUNT_MAX];

    for (uint8_t i = 0; i < DCRO_CODE_COUNT_MAX; i++) {
        new_code           = new_dcro_codes[i];
        temp_freq_table[i] = freq_table[new_code];
    }

    memcpy(freq_table, temp_freq_table, 2 * DCRO_CODE_COUNT_MAX); /* Flawfinder: ignore */
}

static void sr_calib_v1_fill_new_dcro_codes_table(calib_vars_t *calib_vars, uint16_t *freq_table, bool rx, uint8_t *new_dcro_codes)
{
    uint8_t high_codes_bits;
    uint8_t low_codes_bits;
    int8_t offset;

    /*
     * The offset is different for TX and RX, but also for DCRO codes from
     * 0x0 to 0xf and from 0x10 to 0x1f.
     */
    if (rx) {
        high_codes_bits = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_RX_1) >> POS_FREQ_SHIFT_RX_1;
        low_codes_bits  = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_RX_0) >> POS_FREQ_SHIFT_RX_0;
    } else { /* tx */
        high_codes_bits = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_TX_1) >> POS_FREQ_SHIFT_TX_1;
        low_codes_bits  = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_TX_0) >> POS_FREQ_SHIFT_TX_0;
    }

    /* Apply the offset to lower codes */
    offset = offset_from_bits(low_codes_bits);
    for (uint8_t i = 0x0; i < DCRO_CODE_COUNT_MAX / 2; i++) {
        new_dcro_codes[i] = sr_calib_v1_find_new_dcro_code(freq_table, i, offset);
    }

    /* Apply the offset to higher codes */
    offset = offset_from_bits(high_codes_bits);
    for (uint8_t i = DCRO_CODE_COUNT_MAX / 2; i < DCRO_CODE_COUNT_MAX; i++) {
        new_dcro_codes[i] = sr_calib_v1_find_new_dcro_code(freq_table, i, offset);
    }
}

static uint8_t sr_calib_v2_clean_up_freq_table(uint16_t *freq_table, uint8_t *new_dcro_codes)
{
    uint16_t temp_freq_table[DCRO_CODE_COUNT_MAX];
    uint8_t temp_dcro_code[DCRO_CODE_COUNT_MAX];
    uint8_t size;
    uint8_t i = 0;
    int8_t code;

    /* Keep last code */
    temp_freq_table[i] = freq_table[DCRO_CODE_COUNT_MAX - 1];
    temp_dcro_code[i]  = DCRO_CODE_COUNT_MAX - 1;

    /* Start iterating from second to last code to first code to remove invalid codes */
    for (code = DCRO_CODE_COUNT_MAX - 2; code >= 0; code--) {
        /* Remove code if its frequency is not at least 80 MHz lower than code 15 (0xf) frequency */
        if (code > 15 && (freq_table[code] + 2 > freq_table[15])) {
            continue;
        }

        /* Keep code if its frequency is at least 80 MHz higher than the previous valid code */
        if (freq_table[code] >= (temp_freq_table[i] + 2)) {
            i++;
            temp_freq_table[i] = freq_table[code];
            temp_dcro_code[i]  = code;
        }
    }

    size = i + 1;

    /* Reverse tables to have the lowest code first */
    for (uint8_t j = 0; j < size; j++, i--) {
        freq_table[j]     = temp_freq_table[i];
        new_dcro_codes[j] = temp_dcro_code[i];
    }

    /* Fill the invalid DCRO codes and frequencies with invalid values */
    for (uint8_t j = size; j < DCRO_CODE_COUNT_MAX; j++) {
        new_dcro_codes[j] = 0xff;
        freq_table[j]     = 0x00;
    }

    return size;
}

static void sr_calib_v2_apply_vcro_offsets(calib_vars_t *calib_vars, uint16_t *freq_table, bool rx)
{
    uint8_t high_codes_bits;
    uint8_t low_codes_bits;
    int8_t offset;

    /*
     * The offset is different for TX and RX, but also for DCRO codes from
     * 0x0 to 0xf and from 0x10 to 0x1f.
     */
    if (rx) {
        high_codes_bits = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_RX_1) >> POS_FREQ_SHIFT_RX_1;
        low_codes_bits  = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_RX_0) >> POS_FREQ_SHIFT_RX_0;
    } else { /* tx */
        high_codes_bits = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_TX_1) >> POS_FREQ_SHIFT_TX_1;
        low_codes_bits  = (calib_vars->nvm_vcro_shift & BITS_FREQ_SHIFT_TX_0) >> POS_FREQ_SHIFT_TX_0;
    }

    /* Apply the offset to lower codes */
    offset = offset_from_bits(low_codes_bits);
    for (uint8_t i = 0x0; i < DCRO_CODE_COUNT_MAX / 2; i++) {
        /* Frequency value is on 8 bits, make sure it overflows
         * the way the calibration block would do it.
         */
        freq_table[i] = (uint8_t)(freq_table[i] - offset);
    }

    /* Apply the offset to higher codes */
    offset = offset_from_bits(high_codes_bits);
    for (uint8_t i = DCRO_CODE_COUNT_MAX / 2; i < DCRO_CODE_COUNT_MAX; i++) {
        /* Frequency value is on 8 bits, make sure it overflows
         * the way the calibration block would do it.
         */
        freq_table[i] = (uint8_t)(freq_table[i] - offset);
    }
}

static uint8_t sr_calib_v2_find_next_dcro_code(uint8_t code, uint8_t *new_dcro_codes, uint8_t new_dcro_codes_count)
{
    uint8_t i;
    uint8_t next_code = 255; /* Invalid */

    /* Find the initial code from the table of valid codes.
     * The next one is what we are looking for.
     */
    for (i = 0; i < new_dcro_codes_count; i++) {
        if (code == new_dcro_codes[i] && (i != (new_dcro_codes_count - 1))) {
            next_code = new_dcro_codes[i + 1];
            break;
        }
    }
    /* The initial code does not exist or is the last one,
     * so there is no "next" code for it.
     */
    if (i == new_dcro_codes_count) {
        next_code = 255; /* Invalid */
    }

    return next_code;
}

static int8_t offset_from_bits(uint8_t bits)
{
    int32_t offset = bits;

    if (0x8 & bits) {               /* Check if negative */
        offset = 0xfffffff0 | bits; /* Extend the sign on 32 bits */
    }

    return offset;
}

static bool use_calib_v1(calib_vars_t *calib_vars)
{
    if (BINNING_WAS_DONE_WITH_ATE(calib_vars->binning_setup_code)) {
        return false;
    }

    return ((IS_SR1020(calib_vars->phy_model) && IS_QFN28(calib_vars->phy_package) && CHIP_ID_IS_BELOW(573, calib_vars->chip_id)) ||
            (IS_SR1010(calib_vars->phy_model) && IS_QFN28(calib_vars->phy_package) && CHIP_ID_IS_BELOW(190, calib_vars->chip_id)) ||
            (IS_SR1020(calib_vars->phy_model) && IS_QFN48(calib_vars->phy_package) && CHIP_ID_IS_BELOW(28, calib_vars->chip_id)) ||
            (IS_SR1010(calib_vars->phy_model) && IS_QFN48(calib_vars->phy_package) && CHIP_ID_IS_BELOW(24, calib_vars->chip_id)));
}
