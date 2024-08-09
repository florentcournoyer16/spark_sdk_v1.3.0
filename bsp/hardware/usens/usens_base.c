/* INCLUDES *******************************************************************/

#include "usens_base.h"
#include "usens_radio.h"
#include "usens_timer.h"
#include "usens_it.h"
#include "hw_cfg.h"
/* PUBLIC FUNCTION ************************************************************/
void usens_init(void)
{
    radio_peripherals_init();
    usens_it_init();
}