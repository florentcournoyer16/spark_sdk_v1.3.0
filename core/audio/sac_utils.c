/** @file  sac_utils.c
 *  @brief Utility functions for the SPARK Audio Core.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_utils.h"

/* PUBLIC FUNCTIONS ***********************************************************/
sac_word_size_t sac_get_word_size_from_bit_depth(sac_bit_depth_t bit_depth)
{
    switch (bit_depth) {
    case SAC_16BITS:
        return SAC_16BITS_BYTE;
    case SAC_18BITS:
        return SAC_18BITS_BYTE;
    case SAC_20BITS:
        return SAC_20BITS_BYTE;
    case SAC_24BITS:
        return SAC_24BITS_BYTE;
    case SAC_32BITS:
        return SAC_32BITS_BYTE;
    default:
        return SAC_16BITS_BYTE;
    }
}

uint16_t sac_get_nb_packets_in_x_ms(uint16_t ms, uint16_t audio_payload_size, uint8_t nb_channel,
                                    sac_bit_depth_t sample_bit_depth, uint32_t sampling_rate)
{
    sac_word_size_t word_size = sac_get_word_size_from_bit_depth(sample_bit_depth);

    return ((ms / 1000.0) / ((audio_payload_size / (nb_channel * word_size)) / (float)sampling_rate));
}

