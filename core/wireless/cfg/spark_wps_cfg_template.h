/** @file  spark_wps_cfg_template.h
 *  @brief SPARK WPS configuration template
 *
 *  @note Place this in your project to enable/disable feature in
 *        the WPS.
 *
 *  @note If one uses this file, please add SPARK SPARK_WPS_CFG_FILE_EXISTS
 *        to the compile definition.
 *
 *  @note By default, PHY stats and used timeslot stats are enabled.
 *        Link stats and PHY bands stats are disabled.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is confidential and proprietary.
 *  @author    SPARK FW Team.
 */
#ifndef SPARK_WPS_CFG_TEMPLATE_H_
#define SPARK_WPS_CFG_TEMPLATE_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS *********************************************************************/
/** @brief Disable the gathering of PHY statistics.
 *
 *  @note  PHY stats are the one located in the Link Quality Indicator
 *         module. Disabling these stats will also completely disable the
 *         per bands PHY stats and the used timeslots stats.
 */
#define WPS_DISABLE_PHY_STATS

/** @brief Disable the gathering of used timeslots statistics.
 *
 *  @note  When using an auto-sync connection, the Coordinator device will
 *         try to send empty frame when no frame from APP is available in
 *         the Xlayer. This macro here disable stats excluding
 *         these empty frame.
 */
#define WPS_DISABLE_STATS_USED_TIMESLOTS

/** @brief Enable the gathering of PHY statistics per bands.
 *
 *  @note  Do not enable these stats if PHY stats are
 *         disable.
 */
#ifndef WPS_DISABLE_PHY_STATS
#define WPS_ENABLE_PHY_STATS_PER_BANDS
#endif

/** @brief Enable the gathering of Links statistics.
 *
 *  @note This will Enable the following statistics :
 *          - tx_sent         : Number of payload sent
 *          - tx_byte_sent    : Number of byte sent
 *          - tx_drop         : Number of payload dropped
 *          - rx_received     : Number of payload received
 *          - rx_byte_received: Number of byte received
 *          - rx_overrun      : Number of payload dropped because of an RX buffer overrun
 *          - cca_pass        : Number of CCA TX abort
 *          - cca_fail        : Number of CCA TX anyway
 *
 */
#define WPS_DISABLE_LINK_STATS

#ifdef __cplusplus
}
#endif
#endif /* SPARK_WPS_CFG_H_ */
