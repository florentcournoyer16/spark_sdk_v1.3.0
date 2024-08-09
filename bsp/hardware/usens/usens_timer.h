#ifndef USENS_TIMER_H_
#define USENS_TIMER_H_

/* INCLUDES *******************************************************************/
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/types.h>

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the free running millisecond timer.
 *
 *  @param[in] irq_priority  Free running timer millisecond interrupt priority.
 */
extern void timer_free_running_ms_init(uint8_t irq_priority);

/** @brief Initialize the free running quarter millisecond timer.
 *
 *  @param[in] irq_priority  Timer interrupt priority.
 */
extern void timer_free_running_quarter_ms_init(uint8_t irq_priority);

/** @brief Blocking delay with a 1 millisecond resolution.
 *
 *  @param[in] delay_ms  Delay in milliseconds to wait.
 */
extern void timer_delay_ms(uint32_t delay_ms);

/** @brief Get timebase tick value.
 *
 *  @return Tick value.
 */
extern uint32_t timer_get_ms_tick(void);

/** @brief Get the free running timer tick count with a 1 millisecond resolution.
 *
 *  @return Tick count.
 */
extern uint64_t timer_free_running_ms_get_tick_count(void);

/** @brief Get the free running timer tick count with a 250 microseconds resolution.
 *
 *  @return Tick count.
 */
extern uint64_t timer_free_running_quarter_ms_get_tick_count(void);

#endif /* USENS_TIMER_H_ */

