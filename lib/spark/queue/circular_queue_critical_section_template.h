/** @file  circular_queue_critical_section_template.h
 *  @brief SPARK circular queue critical section macros definition template.
 *
 *  @note Copy this file in your application folder and rename it by removing the _template suffix.
 *        Then, implement the macros for your hardware platform.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef CIRCULAR_QUEUE_CRITICAL_SECTION_H_
#define CIRCULAR_QUEUE_CRITICAL_SECTION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS *********************************************************************/
/**@brief Macro for entering a critical region.
 */
#define CRITICAL_SECTION_ENTER()

/**@brief Macro for leaving a critical region.
 */
#define CRITICAL_SECTION_EXIT()

#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_QUEUE_CRITICAL_SECTION_H_ */
