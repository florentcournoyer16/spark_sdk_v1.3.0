/** @file  srac_error.h
 *  @brief SPARK Ranging Core error codes.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SRAC_ERROR_H_
#define SRAC_ERROR_H_

/* TYPES **********************************************************************/
/** @brief Ranging API error structure.
 */
typedef enum srac_error {
    /*! No error occurred */
    SRAC_ERR_NONE = 0,
    /*! A NULL pointer is passed as argument */
    SRAC_ERR_NULL_PTR,
    /*! There is an internal Ranging Core error */
    SRAC_ERR_INTERNAL,
    /*! There is an internal calibration error */
    SRAC_ERR_CALIBRATION,
    /*! An unsupported sample count was provided to the Ranging Core */
    SRAC_ERR_UNSUPPORTED_SAMPLE_COUNT,
} srac_error_t;

#endif /* SRAC_ERROR_H_ */
