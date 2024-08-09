/** @file sr_nvm_private.h
 *  @brief SR1000 non-volatile memory private module.
 *
 *  Functions related to writing the NVM and to its protocol.
 *
 *  @copyright Copyright (C) 2020-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_NVM_PRIVATE_H_
#define SR_NVM_PRIVATE_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include "sr_nvm.h"

/* TYPES **********************************************************************/
/** @brief NVM structure.
 *
 */
typedef struct {
    void (*enable_vdd)(void);  /**< Enable NVM VDD power supply  */
    void (*disable_vdd)(void); /**< Disable NVM VDD power supply */
} nvm_vdd_hal_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Write data to the NVM.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] vdd         Vdd instance.
 *  @param[in] buf         Data to write.
 *  @param[in] addr_start  Start address of NVM.
 *  @param[in] addr_end    End address of NVM.
 */
void sr_nvm_write(radio_t *radio, nvm_vdd_hal_t *vdd, uint8_t *buf, uint8_t addr_start, uint8_t addr_end);

#endif /* SR_NVM_PRIVATE_H_ */
