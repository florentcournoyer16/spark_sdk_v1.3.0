/** @file  sr_api_error.h
 *  @brief SPARK Radio API error codes.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_API_ERROR_H_
#define SR_API_ERROR_H_

/* TYPES **********************************************************************/
/** @brief SR API error.
 */
typedef enum sr_api_error {
    API_ERR_NONE = 0,                      /*!< No error */
    API_ERR_RADIO_NOT_INITIALIZED,         /*!< API radio's instance has not been initialized
                                            *   prior to the API initialization
                                            */
    API_ERR_BUFFER_LOAD_THRESHOLD_TOO_BIG, /*!< Buffer load threshold is higher than
                                            *   what the register can store
                                            */
    API_ERR_DIVISION_BY_ZERO,              /*!< Division by zero */
} sr_api_error_t;

#endif /* SR_API_ERROR_H_ */
