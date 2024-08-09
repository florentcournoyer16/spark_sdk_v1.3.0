/** @file  wps_utils.c
 *  @brief WPS utility function.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps_utils.h"

/* TYPES **********************************************************************/
#define IOOK_FLIP_BITS        4
#define FEC_DIVIDER           3
#define TWO_BIT_PPM_MODIFIER  2
#define OTHER_MOD_MODIFIER    1
#define SIZE_BITS             8
#define FEC_TRAIL_BITS        3
#define ACK_TURNAROUND_BITS   16
const uint8_t fec_multiplier[] = {3, 4, 5, 6, 7, 8, 9, 10};

/* PUBLIC FUNCTIONS ***********************************************************/
int wps_utils_gcd(int number1, int number2)
{
    int latest_remainder = number2; /* Remainder k - 1 */
    int predecesor       = number1; /* Remainder k - 2 */
    int temp_remainder;

    while (latest_remainder) {
        temp_remainder   = latest_remainder;
        latest_remainder = predecesor % temp_remainder;
        predecesor       = temp_remainder;
    }

    return predecesor;
}

uint32_t wps_utils_get_delayed_wakeup_event(uint32_t preamble_bits, uint32_t syncword_bits, bool iook, uint8_t fec,
                                            bool mod_2bitppm, uint8_t address_bits, uint32_t total_frame_size,
                                            uint32_t crc_bits, uint32_t cca_delay_pll, uint32_t cca_retry, bool ack,
                                            uint8_t ack_payload_size)
{
    uint32_t main_frame_time = (address_bits + SIZE_BITS + (total_frame_size * 8) + crc_bits + FEC_TRAIL_BITS) *
                               fec_multiplier[fec] * ((mod_2bitppm) ? TWO_BIT_PPM_MODIFIER : OTHER_MOD_MODIFIER) /
                               FEC_DIVIDER;
    uint32_t ack_frame_time = (address_bits + SIZE_BITS + (ack_payload_size * 8) + crc_bits + FEC_TRAIL_BITS) *
                              fec_multiplier[fec] * ((mod_2bitppm) ? TWO_BIT_PPM_MODIFIER : OTHER_MOD_MODIFIER) /
                              FEC_DIVIDER;
    if (ack) {
        return (preamble_bits + syncword_bits + ((iook) ? IOOK_FLIP_BITS : 0) + main_frame_time) +
               (cca_delay_pll * (cca_retry - 1)) + ACK_TURNAROUND_BITS +
               (preamble_bits + syncword_bits + ((iook) ? IOOK_FLIP_BITS : 0) + ack_frame_time);
    } else {
        return (preamble_bits + syncword_bits + ((iook) ? IOOK_FLIP_BITS : 0) + main_frame_time) +
               (cca_delay_pll * (cca_retry - 1));
    }
}
