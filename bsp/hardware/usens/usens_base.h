#ifndef USENS_BASE_H__
#define USENS_BASE_H__

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the board's peripherals.
 */
void usens_init(void);

#endif // USENS_BASE_H__