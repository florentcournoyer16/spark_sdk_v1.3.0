/** @file  sac_error.h
 *  @brief SPARK Audio Core error codes.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_ERROR_H_
#define SAC_ERROR_H_

/* TYPES **********************************************************************/
/** @brief SAC Errors.
 */
typedef enum sac_error {
    /*! No error occurred */
    SAC_ERR_NONE = 0,
    /*! Not enough memory is allocated by the application for a full audio core initialization */
    SAC_ERR_NOT_ENOUGH_MEMORY,
    /*! Maximum number of processing stages for a given SAC pipeline is already reached when trying to add another one */
    SAC_ERR_PROC_STAGE_LIMIT_REACHED,
    /*! Producer's queue is full when trying to produce */
    SAC_ERR_PRODUCER_Q_FULL,
    /*! Consumer's queue is empty when trying to consume */
    SAC_ERR_CONSUMER_Q_EMPTY,
    /*! Initial buffering is not completed when trying to consume */
    SAC_ERR_BUFFERING_NOT_COMPLETE,
    /*! Producer's queue is empty when trying to process */
    SAC_ERR_NO_SAMPLES_TO_PROCESS,
    /*! An error occurred during the processing stage initialization */
    SAC_ERR_PROCESSING_STAGE_INIT,
    /*! Pipeline configuration is invalid */
    SAC_ERR_PIPELINE_CFG_INVALID,
    /*! A pointer is NULL while it should have been initialized */
    SAC_ERR_NULL_PTR,
    /*! An error occurred during the mixer module initialization */
    SAC_ERR_MIXER_INIT_FAILURE,
    /*! The maximum number of elements allowed have been reached */
    SAC_ERR_MAXIMUM_REACHED,
    /*! A processing stage's control function has been called with an invalid command */
    SAC_ERR_INVALID_CMD,
    /*! An error occurred during the fallback module initialization */
    SAC_ERR_FALLBACK_INIT_FAILURE,
    /*! The configured bit depth is invalid */
    SAC_ERR_BIT_DEPTH,
    /*! The configured channel count is invalid */
    SAC_ERR_CHANNEL_COUNT,
    /*! The configured mixer option is invalid */
    SAC_ERR_MIXER_OPTION,
    /*! A processing stage's control function has been called with an invalid argument */
    SAC_ERR_INVALID_ARG,
} sac_error_t;

#endif /* SAC_ERROR_H_ */
