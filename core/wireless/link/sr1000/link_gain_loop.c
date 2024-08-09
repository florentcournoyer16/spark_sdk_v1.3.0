/** @file link_gain_loop.c
 *  @brief Gain loop module.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "link_gain_loop.h"
#include "sr_def.h"

/* CONSTANTS ******************************************************************/
#define LOWER_BOUND_MARGIN_TENTH_DB  120
#define HIGHER_BOUND_MARGIN_TENTH_DB 40
#define GAIN_ENTRY_COUNT             17

/* PRIVATE GLOBALS ************************************************************/
static const gain_entry_t gain_lookup_table[GAIN_ENTRY_COUNT] = {
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(0, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 0, 235, 74},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(1, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 12, 247, 67},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(2, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 23, 258, 61},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(3, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 42, 277, 54},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(4, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 60, 295, 48},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(5, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 85, 320, 44},
    {MOV2MASK(1, BITS_RFGAIN) | MOV2MASK(5, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 98, 333, 44},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(6, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 130, 365, 42},
    {MOV2MASK(1, BITS_RFGAIN) | MOV2MASK(6, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 142, 377, 42},
    {MOV2MASK(3, BITS_RFGAIN) | MOV2MASK(6, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 173, 408, 42},
    {MOV2MASK(3, BITS_RFGAIN) | MOV2MASK(6, BITS_IFOAGAIN) | MOV2MASK(3, BITS_IFGAIN3), 187, 422, 35},
    {MOV2MASK(0, BITS_RFGAIN) | MOV2MASK(7, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 221, 456, 42},
    {MOV2MASK(1, BITS_RFGAIN) | MOV2MASK(7, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 241, 476, 42},
    {MOV2MASK(2, BITS_RFGAIN) | MOV2MASK(7, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 258, 493, 41},
    {MOV2MASK(3, BITS_RFGAIN) | MOV2MASK(7, BITS_IFOAGAIN) | MOV2MASK(4, BITS_IFGAIN3), 271, 506, 41},
    {MOV2MASK(3, BITS_RFGAIN) | MOV2MASK(7, BITS_IFOAGAIN) | MOV2MASK(3, BITS_IFGAIN3), 292, 527, 35},
    {MOV2MASK(3, BITS_RFGAIN) | MOV2MASK(7, BITS_IFOAGAIN) | MOV2MASK(2, BITS_IFGAIN3), 318, 553, 31},
};

/* PUBLIC FUNCTIONS ***********************************************************/
void link_gain_loop_init(gain_loop_t *gain_loop, bool fixed_gain_enable, uint8_t rx_gain)
{
    gain_loop->gain_index        = 0;
    gain_loop->fixed_gain_enable = fixed_gain_enable;
    gain_loop->rx_gain           = rx_gain;
}

void link_gain_loop_update(gain_loop_t *gain_loop, frame_outcome_t frame_outcome, uint8_t rssi)
{
    uint16_t normalized_gain;

    switch (frame_outcome) {
    case FRAME_RECEIVED:
    case FRAME_SENT_ACK:
        normalized_gain = calculate_normalized_gain(gain_lookup_table[gain_loop->gain_index].min_tenth_db, rssi);
        if (normalized_gain < (gain_lookup_table[gain_loop->gain_index].min_tenth_db + LOWER_BOUND_MARGIN_TENTH_DB) &&
            (gain_loop->gain_index != 0)) {
            gain_loop->gain_index--;
        } else if (normalized_gain > (gain_lookup_table[gain_loop->gain_index].max_tenth_db - HIGHER_BOUND_MARGIN_TENTH_DB) &&
                   (gain_loop->gain_index != (GAIN_ENTRY_COUNT - 1))) {
            gain_loop->gain_index++;
        }
        break;
    case FRAME_REJECTED:
    case FRAME_SENT_ACK_REJECTED:
    case FRAME_LOST:
    case FRAME_SENT_ACK_LOST:
        break;
    default:
        break;
    }
}

uint8_t link_gain_loop_get_gain_value(gain_loop_t *gain_loop)
{
    if (gain_loop->fixed_gain_enable) {
        return gain_loop->rx_gain;
    } else {
        return gain_lookup_table[gain_loop->gain_index].gain_value;
    }
}

uint16_t link_gain_loop_get_min_tenth_db(uint8_t gain_index)
{
    return gain_lookup_table[gain_index].min_tenth_db;
}

uint16_t link_gain_loop_get_rnsi_tenth_db(uint8_t gain_index)
{
    return gain_lookup_table[gain_index].rnsi_tenth_db;
}
