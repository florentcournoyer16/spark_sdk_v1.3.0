/** @file sr_nvm_private.c
 *   @brief SR1000 non-volatile memory private module.
 *
 *   Functions related to writing the NVM and to its protocol.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *   @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sr_nvm_private.h"
#include "sr_access.h"
#include "sr_api.h"
#include "sr_def.h"

/* CONSTANTS ******************************************************************/
#define NVM_POST_WRITE_DELAY_MS 150

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void sr_nvm_set_byte(radio_hal_t *radio, nvm_vdd_hal_t *vdd, uint8_t addr, uint8_t byte);

/* PUBLIC FUNCTIONS ***********************************************************/
void sr_nvm_write(radio_t *radio, nvm_vdd_hal_t *vdd, uint8_t *buf, uint8_t addr_start, uint8_t addr_end)
{
    uint8_t idx          = 0;
    uint8_t addr_current = addr_start;
    uint8_t *read_reg;

    uwb_write_register_8(radio, REG_ACTIONS, 0);

    /* Wait until radio is awake. */
    do {
        read_reg = uwb_read_register_8(radio, REG_PWRSTATUS);
        uwb_transfer_blocking(radio);
    } while (!(*read_reg & BIT_AWAKE));

    sr_nvm_power_up(radio);

    while (addr_current <= addr_end) {
        sr_nvm_set_byte(&radio->radio_hal, vdd, addr_current++, buf[idx++]);
    }

    sr_nvm_power_down(radio);

    /* Wait to ensure subsequent read works. Value found by experiment on V8B dies */
    radio->radio_hal.delay_ms(NVM_POST_WRITE_DELAY_MS);
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Burn a byte into a NVM location. Assumes that the radio and NVM are powered up.
 *
 *  @param[in] radio  Radio HAL instance.
 *  @param[in] vdd    VDD control HAL instance.
 *  @param[in] addr   Address of the memory location.
 *  @param[in] byte   Byte value.
 *  @return None.
 */
static void sr_nvm_set_byte(radio_hal_t *radio, nvm_vdd_hal_t *vdd, uint8_t addr, uint8_t byte)
{
    uint8_t bit_array[16];
    uint8_t rx_array[16];
    uint8_t array_index = 0;

    /* Fill up array with bit/addr pairs for each 1 bit */
    for (uint8_t i = 0; i <= NVM_LAST_BIT_POS; i++) {
        if (byte & BIT(i)) {
            bit_array[array_index++] = i;
            bit_array[array_index++] = addr;
        }
    }
    /* Write the bit array to the NVM */
    if (array_index > 0) {
        radio->reset_cs();
        vdd->enable_vdd();
        radio->transfer_full_duplex_blocking(bit_array, rx_array, array_index);
        vdd->disable_vdd();
        radio->set_cs();
    }
}
