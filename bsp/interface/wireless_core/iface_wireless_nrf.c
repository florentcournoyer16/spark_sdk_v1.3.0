#include "iface_wireless.h"
#include "swc_api.h"
#include "usens_radio.h"
#include "usens_it.h"
#include "usens_timer.h"

/* PRIVATE FUNCTIONS ***********************************************************/
void iface_swc_hal_init(swc_hal_t *hal)
{
    hal->radio_hal[0].set_shutdown_pin   = radio_set_shutdown_pin;
    hal->radio_hal[0].reset_shutdown_pin = radio_reset_shutdown_pin;
    hal->radio_hal[0].set_reset_pin      = radio_set_reset_pin;
    hal->radio_hal[0].reset_reset_pin    = radio_reset_reset_pin;
    hal->radio_hal[0].read_irq_pin       = radio_read_irq_pin;
    hal->radio_hal[0].set_cs             = radio_spi_set_cs;
    hal->radio_hal[0].reset_cs           = radio_spi_reset_cs;
    hal->radio_hal[0].delay_ms           = timer_delay_ms;
    hal->radio_hal[0].get_tick           = timer_free_running_ms_get_tick_count;
    hal->radio_hal[0].tick_frequency_hz  = 1000;

    hal->radio_hal[0].transfer_full_duplex_blocking     = radio_spi_transfer_full_duplex_blocking;
    hal->radio_hal[0].transfer_full_duplex_non_blocking = radio_spi_transfer_full_duplex_non_blocking;
    hal->radio_hal[0].is_spi_busy                       = radio_is_spi_busy;
    hal->radio_hal[0].context_switch                    = radio_context_switch;
    hal->radio_hal[0].disable_radio_irq                 = radio_disable_irq_it;
    hal->radio_hal[0].enable_radio_irq                  = radio_enable_irq_it;
    hal->radio_hal[0].disable_radio_dma_irq             = radio_disable_dma_irq_it;
    hal->radio_hal[0].enable_radio_dma_irq              = radio_enable_dma_irq_it;

    hal->context_switch = radio_callback_context_switch;
    timer_free_running_ms_init(0);
}

void iface_swc_handlers_init(void)
{
    usens_set_radio_irq_callback(swc_radio_irq_handler);
    usens_set_radio_dma_rx_callback(swc_radio_spi_receive_complete_handler);
    usens_set_pendsv_callback(swc_connection_callbacks_processing_handler);
    // DMA Rx Callback : No DMA usage implemented
    // PendSV Callback : Not implemented
}