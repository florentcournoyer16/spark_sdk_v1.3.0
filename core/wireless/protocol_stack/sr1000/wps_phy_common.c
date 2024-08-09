/** @file  wps_phy_common.c
 *  @brief Wireless protocol stack PHY control.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps.h"
#include "wps_phy.h"

/* CONSTANTS ******************************************************************/
#define MAX_CUT_THROUGH_FRAME_SIZE 255
#define EMPTY_BYTE                 1
#define PARTIAL_FRAME_COUNT        3
#define PARTIAL_FRAME_BASE_INDEX   0
#define SIZE_HDR_SIZE              1
#define HDR_SIZE_HDR_SIZE          1
#define DEFAULT_RX_IDLE_PWR        RX_IDLE_PWR_HIGH
#define DISABLE_CCA_THRSH_VALUE    0xff
#define FAST_SYNC_TIMER_VALUE      32000
#define MAX_SLEEP_TIME_VALUE       0xFFFF
#define MAX_RX_TIMEOUT_VALUE       0x0FFF

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void prepare_phy(wps_phy_t *phy);
static void prepare_radio(wps_phy_t *phy);
static void set_config(wps_phy_t *phy);
static void set_channel_config(wps_phy_t *phy);
static void enable_radio_irq(wps_phy_t *phy);
static void check_radio_irq(wps_phy_t *phy);
static void set_header(wps_phy_t *phy);
static void set_payload(wps_phy_t *phy);
static void read_events(wps_phy_t *phy);
static void read_events_syncing(wps_phy_t *phy);
static void process_event_tx(wps_phy_t *phy);
static void process_event_rx(wps_phy_t *phy);
static void handle_good_frame(wps_phy_t *wps_phy);
static void handle_good_auto_reply(wps_phy_t *wps_phy);
static void get_frame_header(wps_phy_t *phy);
static void get_auto_reply_header(wps_phy_t *phy);
static void get_payload(wps_phy_t *phy);
static void handle_missed_frame(wps_phy_t *wps_phy);
static void handle_missed_auto_reply(wps_phy_t *wps_phy);
static void handle_cca_fail(wps_phy_t *wps_phy);
static void end_flush(wps_phy_t *phy);
static void close_spi(wps_phy_t *phy);
static void end(wps_phy_t *phy);
static void none(wps_phy_t *phy);
static void clear_err(wps_phy_t *wps_phy);
static void prepare_syncing(wps_phy_t *phy);
static void transfer_register(wps_phy_t *phy);

static bool main_is_tx(wps_phy_t *phy);
static bool auto_is_tx(wps_phy_t *phy);
static bool tx_complete(radio_events_t radio_events);
static bool tx_complete_auto_reply(radio_events_t radio_events);
static bool rx_good(radio_events_t radio_events);
static bool rx_rejected(radio_events_t radio_events);
static bool rx_lost(radio_events_t radio_events);
static int_flag_cfg_t set_events_for_tx_with_ack(void);
static int_flag_cfg_t set_events_for_tx_without_ack(void);
static int_flag_cfg_t set_events_for_rx_with_ack(void);
static int_flag_cfg_t set_events_for_rx_without_ack(void);
static int_flag_cfg_t set_events_for_rx_with_auto_payload(void);
static void enqueue_states(wps_phy_t *wps_phy, wps_phy_state_t *state);

#ifdef WPS_ENABLE_CUT_THROUGH_MODE
static void set_payload_cut_through(wps_phy_t *phy);
static void setup_cut_through(wps_phy_t *phy);
static void get_payload_cut_through(wps_phy_t *phy);
static void end_tx_cut_through(wps_phy_t *phy);
static void signal_yield(wps_phy_t *phy);

static bool frame_fits_in_radio_fifo(uint8_t payload_size, uint8_t header_size);
static uint8_t get_bufload_thresh_tx(fec_level_t fec_level, uint8_t partial_frame_size);
static uint8_t get_bufload_thresh_rx(fec_level_t fec_level, uint8_t partial_frame_size);
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */

/* TYPES **********************************************************************/
static wps_phy_state_t prepare_phy_states[]           = {prepare_phy, end};
static wps_phy_state_t set_config_states[]            = {set_config, close_spi, set_channel_config, end};
static wps_phy_state_t set_header_states[]            = {close_spi, set_header, end};
static wps_phy_state_t set_payload_states[]           = {set_payload, end};
static wps_phy_state_t wait_radio_states_tx[]         = {close_spi, enable_radio_irq, read_events, close_spi, process_event_tx, end};
static wps_phy_state_t wait_radio_states_rx[]         = {close_spi, enable_radio_irq, read_events, close_spi, process_event_rx, end};
static wps_phy_state_t get_event_states[]             = {read_events, close_spi, process_event_tx, end};
static wps_phy_state_t get_frame_header_states[]      = {close_spi, get_frame_header, end};
static wps_phy_state_t get_auto_reply_header_states[] = {close_spi, get_auto_reply_header, end};
static wps_phy_state_t get_payload_states[]           = {get_payload, end};
static wps_phy_state_t new_frame_states[]             = {close_spi, end};
static wps_phy_state_t flush_fifo_states[]            = {close_spi, end_flush, end};
static wps_phy_state_t syncing_states[]           = {close_spi, enable_radio_irq, read_events_syncing, close_spi, process_event_rx, end};
static wps_phy_state_t wait_to_send_auto_reply[]  = {check_radio_irq, end};
static wps_phy_state_t transfer_register_states[] = {transfer_register, end};
static wps_phy_state_t end_states[]               = {none};

#ifdef WPS_ENABLE_CUT_THROUGH_MODE
static wps_phy_state_t prepare_radio_cut_through_states[] = {
    set_header, set_payload_cut_through, prepare_radio, signal_yield, close_spi, set_config, close_spi, enable_radio_irq, end};
static wps_phy_state_t set_payload_cut_through_states[] = {set_payload_cut_through, close_spi, enable_radio_irq, end};
static wps_phy_state_t get_payload_cut_through_states[] = {close_spi, setup_cut_through, get_payload_cut_through, end};
static wps_phy_state_t end_tx_cut_through_states[]      = {close_spi, end_tx_cut_through, end};
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */

/* PRIVATE GLOBALS ************************************************************/
static phy_output_signal_t unused_signal;
static xlayer_frame_t unused_frame;

/* PUBLIC FUNCTIONS ***********************************************************/
void phy_init(wps_phy_t *wps_phy, wps_phy_cfg_t *cfg)
{
    wps_phy->wait_for_ack_tx = true;
    wps_phy->state_step      = 0;
    wps_phy->radio           = cfg->radio;
    wps_phy->source_address  = cfg->local_address;
    wps_phy->current_state   = prepare_phy_states;
    wps_phy->end_state       = end;
    circular_queue_init(&wps_phy->next_states, wps_phy->next_state_pool, PHY_STATE_Q_SIZE, sizeof(wps_phy_state_t **));

    for (uint8_t i = 0; i < PHY_MAX_CERTIF_HEADER_SIZE; i++) {
        wps_phy->certification_header[i] = (i % 2 == 0) ? PHY_CERTIF_BYTE0 : PHY_CERTIF_BYTE1;
    }

    /* Disable IRQ during INIT */
    uwb_disable_irq(wps_phy->radio);

    uwb_set_syncword_config(wps_phy->radio, &cfg->syncword_cfg);

    /* Configure CRC */
    uwb_set_crc(wps_phy->radio, cfg->crc_polynomial);

    /* Preamble */
    uwb_set_preamble_length(wps_phy->radio, cfg->preamble_len);

    /* Set TX/RX len */
    uwb_set_rx_packet_size(wps_phy->radio, MAX_FRAMESIZE);
    uwb_set_tx_packet_size(wps_phy->radio, MAX_FRAMESIZE);

    /* Config local address */
    uwb_set_local_address(wps_phy->radio, cfg->local_address, ADDRESS_LENGTH_16);

    /* Set sleep level */
    sleep_events_t sleep_events = (sleep_events_t)(SLEEP_RX_TIMEOUT | SLEEP_TX_END | SLEEP_RX_END);

    uwb_set_sleep_config(wps_phy->radio, cfg->sleep_lvl, sleep_events);

    /* Flush RX/TX FIFO */
    uwb_set_radio_actions(wps_phy->radio,
                          SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, GO_TO_SLEEP, FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER));

    /* Enable IRQ on NEW_PACKET_IT */
    uwb_set_int_flag(wps_phy->radio,
                     SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, wps_phy->radio->irq_polarity, NEW_PACKET_IT, RX_TIMEOUT_IT, TX_END_IT));

    /* Set RX waited source to use with address filtering */
    uwb_set_rx_waited_src(wps_phy->radio, RX_WAIT_SOURCE_REGISTER);

    if ((cfg->sleep_lvl != SLEEP_IDLE) && (wps_phy->radio->phy_version == PHY_VERSION_8_3)) {
        uwb_set_dll_tuning(wps_phy->radio, INTEGLEN_SHORT);
    }

    /* Enable phase tracking */
    uwb_enable_phase_tracking(wps_phy->radio);

    uwb_set_packet_config(wps_phy->radio, DEFAULT_PACKET_CONFIGURATION);

    (void)uwb_get_irq_flags(wps_phy->radio);

    /* Transfer */
    uwb_transfer_blocking(wps_phy->radio);
}

void phy_connect(wps_phy_t *wps_phy)
{
    uwb_apply_saved_calibration(wps_phy->radio);
    uwb_set_rx_timeout_raw(wps_phy->radio, 0xFFFF, 0xFF);

    /* Clear Status */
    (void)uwb_get_irq_flags(wps_phy->radio);

    uwb_set_radio_actions(wps_phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER,
                                                            INIT_TIMER_RESET_BOTH_WAKE_UP_TIMER, GO_TO_SLEEP));

    uwb_transfer_blocking(wps_phy->radio);

    sr_access_enable_radio_irq(&wps_phy->radio->radio_hal);
    sr_access_enable_dma_irq(&wps_phy->radio->radio_hal);

    wps_phy->state_step    = 0;
    wps_phy->current_state = prepare_phy_states;
    circular_queue_init(&wps_phy->next_states, wps_phy->next_state_pool, PHY_STATE_Q_SIZE, sizeof(wps_phy_state_t **));
    wps_phy->signal_main = PHY_SIGNAL_CONNECT;
}

void phy_connect_single(wps_phy_t *wps_phy)
{
    uwb_set_radio_actions(wps_phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR));
    uwb_transfer_blocking(wps_phy->radio);

    do {
        wps_phy->pwr_status_cmd = uwb_read_register_8(wps_phy->radio, REG_PWRSTATUS);
        uwb_transfer_blocking(wps_phy->radio);
    } while (!(*wps_phy->pwr_status_cmd & BIT_AWAKE));

    uwb_set_timer_config(wps_phy->radio, SET_TIMER_CFG(TIMER_CFG_CLEAR, AUTOWAKE_UP_ENABLE, WAKE_UP_ONCE_ENABLE));
    phy_connect(wps_phy);
    wps_phy->radio->radio_hal.context_switch();
}

void phy_connect_multi(wps_phy_t *wps_phy)
{
    uwb_set_timer_config(wps_phy->radio, SET_TIMER_CFG(TIMER_CFG_CLEAR, AUTOWAKE_UP_DISABLE, WAKE_UP_ONCE_ENABLE));
    uwb_set_radio_actions(wps_phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, GO_TO_SLEEP, INIT_TIMER_RESET_BOTH_WAKE_UP_TIMER));
}

void phy_wakeup_multi(wps_phy_t *wps_phy)
{
    wps_phy->cfg.radio_actions |= SET_RADIO_ACTIONS(INIT_TIMER_RESET_BOTH_WAKE_UP_TIMER);
    uwb_set_radio_actions(wps_phy->radio, wps_phy->cfg.radio_actions);
}

void phy_disconnect(wps_phy_t *wps_phy)
{
    while (wps_phy->radio->radio_hal.is_spi_busy()) {
        /* wait for any SPI transfer to complete */
    }

    sr_access_close(&wps_phy->radio->radio_hal);

    /* Disable peripherals interrupts */
    wps_phy->radio->radio_hal.disable_radio_dma_irq();
    wps_phy->radio->radio_hal.disable_radio_irq();

    /* Disable radio interrupts */
    uwb_disable_irq(wps_phy->radio);

    /* Reset timer configuration for proper sleep */
    uwb_set_timer_config(wps_phy->radio, SET_TIMER_CFG(TIMER_CFG_CLEAR, AUTOWAKE_UP_DISABLE, WAKE_UP_ONCE_DISABLE));

    /* Set sleep level to deep sleep when disconnecting */
    sleep_events_t sleep_events = (sleep_events_t)(SLEEP_RX_TIMEOUT | SLEEP_TX_END | SLEEP_RX_END);
    sleep_lvl_t sleep_lvl       = SLEEP_DEEP;

    uwb_set_sleep_config(wps_phy->radio, sleep_lvl, sleep_events);

    /* Clear Status */
    (void)uwb_get_irq_flags(wps_phy->radio);

    /* Reset CCA & go to sleep */
    uint8_t empty_tx_pattern[NB_PULSES] = {0};

    uwb_set_cac(wps_phy->radio, DEFAULT_RX_IDLE_PWR, DISABLE_CCA_THRSH_VALUE);
    uwb_set_radio_actions(wps_phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, FLUSH_TX_RESET_TX_BUFFER, GO_TO_SLEEP));
    uwb_select_channel(wps_phy->radio, empty_tx_pattern, wps_phy->config->channel->pulse_size);

    uwb_transfer_blocking(wps_phy->radio);

    /* Set timer to max value */
    uwb_set_radio_actions(wps_phy->radio,
                          SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR,
                                            INIT_TIMER_RESET_BOTH_WAKE_UP_TIMER, GO_TO_SLEEP));
    uwb_set_timer_cfg_burst(wps_phy->radio, MAX_SLEEP_TIME_VALUE, MAX_RX_TIMEOUT_VALUE, 3, 0);
    uwb_transfer_blocking(wps_phy->radio);

    /* Wait until we are in deep sleep */
    do {
        wps_phy->pwr_status_cmd = uwb_read_register_8(wps_phy->radio, REG_PWRSTATUS);
        uwb_transfer_blocking(wps_phy->radio);
    } while (*wps_phy->pwr_status_cmd != 0);
}

phy_output_signal_t phy_get_main_signal(wps_phy_t *wps_phy)
{
    return wps_phy->signal_main;
}

phy_output_signal_t phy_get_auto_signal(wps_phy_t *wps_phy)
{
    return wps_phy->signal_auto;
}

void phy_set_main_xlayer(wps_phy_t *wps_phy, xlayer_t *xlayer, xlayer_cfg_internal_t *xlayer_cfg)
{
    wps_phy->config = xlayer_cfg;
    wps_phy->xlayer_main = xlayer;
}

void phy_set_auto_xlayer(wps_phy_t *wps_phy, xlayer_t *xlayer)
{
    wps_phy->xlayer_auto = xlayer;
}

void phy_write_register(wps_phy_t *wps_phy, uint8_t starting_reg, uint8_t data)
{
    wps_phy->write_request_info.target_register = starting_reg;
    wps_phy->write_request_info.data            = data;
    wps_phy->write_request_info.pending_request = true;
    enqueue_states(wps_phy, transfer_register_states);
}

void phy_read_register(wps_phy_t *wps_phy, uint8_t target_register, uint8_t *rx_buffer, bool *xfer_cmplt)
{
    wps_phy->read_request_info.rx_buffer       = rx_buffer;
    wps_phy->read_request_info.xfer_cmplt      = xfer_cmplt;
    wps_phy->read_request_info.target_register = target_register;
    wps_phy->read_request_info.pending_request = true;
    enqueue_states(wps_phy, transfer_register_states);
}

void phy_enable_debug_feature(wps_phy_t *wps_phy, phy_debug_cfg_t *phy_debug)
{
    (void)wps_phy;
    (void)phy_debug;
}

void phy_enqueue_prepare(wps_phy_t *wps_phy)
{
    wps_phy->next_states.enqueue_it   = wps_phy->next_states.buffer_begin;
    wps_phy->next_states.dequeue_it   = wps_phy->next_states.buffer_begin;
    wps_phy->state_step    = 0;
    wps_phy->current_state = prepare_phy_states;
}

void phy_enqueue_none(wps_phy_t *wps_phy)
{
    wps_phy->next_states.enqueue_it   = wps_phy->next_states.buffer_begin;
    wps_phy->next_states.dequeue_it   = wps_phy->next_states.buffer_begin;
    wps_phy->state_step    = 0;
    wps_phy->current_state = end_states;
}

void phy_clear_access(wps_phy_t *wps_phy)
{
    wps_phy->radio->access_sequence.index = 0;
}

/* PRIVATE FUNCTION ***********************************************************/
/** @brief Write to target register.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void transfer_register(wps_phy_t *phy)
{
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    if (phy->write_request_info.pending_request) {
        /* Write register request */
        tx_buffer[0] = phy->write_request_info.target_register | REG_WRITE;
        tx_buffer[1] = phy->write_request_info.data;
        sr_access_open(&phy->radio->radio_hal);
        phy->radio->radio_hal.transfer_full_duplex_blocking(tx_buffer, rx_buffer, 2);
        sr_access_close(&phy->radio->radio_hal);
        phy->write_request_info.pending_request = false;
    } else if (phy->read_request_info.pending_request) {
        /* Read register request */
        tx_buffer[0] = phy->read_request_info.target_register;
        while (phy->radio->radio_hal.is_spi_busy())
            ;
        sr_access_open(&phy->radio->radio_hal);
        phy->radio->radio_hal.transfer_full_duplex_blocking(tx_buffer, rx_buffer, 2);
        sr_access_close(&phy->radio->radio_hal);
        *phy->read_request_info.rx_buffer = rx_buffer[1];
        /* Thread safety */
        do {
            *phy->read_request_info.xfer_cmplt = true;
        } while (!(*phy->read_request_info.xfer_cmplt));
        /* Read register request */
        phy->read_request_info.pending_request = false;
    }
}

/** @brief Enqueue a new state to the state machine.
 *
 *   @param[in] wps_phy  PHY instance struct.
 *   @param[in] state   New state to enqueue.
 */
static void enqueue_states(wps_phy_t *wps_phy, wps_phy_state_t *state)
{
    wps_phy_state_t **enqueue_states;

    enqueue_states  = (wps_phy_state_t **)circular_queue_get_free_slot_raw(&wps_phy->next_states);
    *enqueue_states = state;
    circular_queue_enqueue_raw(&wps_phy->next_states);
}

/** @brief Setup the state machine to send the frame payload to the radio.
 *
 *  @param[in] wps_phy        PHY instance struct.
 *  @param[in] payload_size  Frame payload size.
 */
static void enqueue_tx_prepare_frame_states(wps_phy_t *wps_phy, uint8_t header_size, uint8_t payload_size)
{
    if (header_size + payload_size != 0) {
        enqueue_states(wps_phy, set_header_states);
    }

    if (payload_size != 0) {
        enqueue_states(wps_phy, set_payload_states);
    }
}

/** @brief Setup the state machine to receive payload from the radio.
 *
 *  @param[in] wps_phy        PHY instance struct.
 */
static void enqueue_rx_prepare_frame_states(wps_phy_t *wps_phy)
{
    enqueue_states(wps_phy, wait_radio_states_rx);
}

#ifndef WPS_ENABLE_CUT_THROUGH_MODE
/** @brief Setup the PHY state machine.
 *
 *  @param[in] wps_phy        PHY instance struct.
 *  @param[in] payload_size  Frame payload size.
 */
static void prepare_phy(wps_phy_t *phy)
{
    if (phy->input_signal == PHY_SIGNAL_SYNCING) {
        enqueue_states(phy, syncing_states);
        prepare_syncing(phy);
    } else {
        if (main_is_tx(phy)) {
            phy->tx.frame        = &phy->xlayer_main->frame;
            phy->tx.payload_size = phy->tx.frame->payload_end_it - phy->tx.frame->payload_begin_it;
        }
        enqueue_states(phy, set_config_states);
        prepare_radio(phy);
    }
}
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */

/** @brief Sub function to prepare a transmit frame.
 *
 *  @param[in] signal_data   Data required to process the state. The type shall be wps_phy_t.
 *  @param[in] radio_events  Radio's interrupt events enumeration.
 *  @param[in] radio_actions   Radio actions instance.
 */
static void prepare_radio_tx(wps_phy_t *phy, int_flag_cfg_t *radio_events)
{
    uint8_t header_size;
    uint8_t rx_packet_size;

    phy->cfg.channel                   = phy->config->channel;
    phy->cfg.destination_address       = &phy->xlayer_main->frame.destination_address;
    phy->cfg.source_address            = &phy->xlayer_main->frame.source_address;
    phy->cfg.power_up_delay            = &phy->config->power_up_delay;
    phy->cfg.rx_timeout                = &phy->config->rx_timeout;
    phy->cfg.sleep_time                = &phy->config->sleep_time;
    phy->cfg.rx_wait_time              = &phy->config->rx_wait_time;
    phy->cfg.sleep_level               = phy->config->sleep_level;
    phy->tx.modulation                 = &phy->config->modulation;
    phy->tx.fec                        = &phy->config->fec;
    phy->tx.cca_threshold              = &phy->config->cca_threshold;
    phy->tx.cca_retry_time             = phy->config->cca_retry_time;
    phy->tx.cca_max_try_count          = phy->config->cca_max_try_count;
    phy->tx.cca_fail_action            = phy->config->cca_fail_action;
    phy->tx.cca_try_count              = &phy->config->cca_try_count;
    phy->tx.frame                      = &phy->xlayer_main->frame;
    phy->tx.signal                     = &phy->signal_main;
    phy->rx.rx_constgain               = &phy->config->rx_constgain;
    phy->rx.rssi_raw                   = &phy->config->rssi_raw;
    phy->rx.rnsi_raw                   = &phy->config->rnsi_raw;
    header_size                        = phy->tx.frame->header_end_it - phy->tx.frame->header_begin_it;

    /* Autoreply mode */
    if (phy->xlayer_auto != NULL) {
        phy->auto_reply_mode = AUTO_REPLY_ENABLE;
        phy->rx.frame        = &phy->xlayer_auto->frame;
        phy->rx.signal       = &phy->signal_auto;
        phy->cfg.phase_info  = phy->config->phases_info;
        *radio_events        = set_events_for_tx_with_ack();
        rx_packet_size       = phy->xlayer_auto->frame.payload_memory_size  + phy->rx.frame->header_memory_size + HDR_SIZE_HDR_SIZE;
        /* Ack mode */
    } else if (phy->config->expect_ack) {
        phy->auto_reply_mode = AUTO_REPLY_ENABLE;
        phy->cfg.phase_info  = NULL;
        phy->rx.frame        = &unused_frame;
        phy->rx.signal       = &unused_signal;
        phy->rx.payload_size = 0;
        *radio_events        = set_events_for_tx_with_ack();
        rx_packet_size       = 1;
        /* Nack mode */
    } else {
        phy->auto_reply_mode = AUTO_REPLY_DISABLE;
        phy->cfg.phase_info  = NULL;
        phy->rx.frame        = &unused_frame;
        phy->rx.signal       = &unused_signal;
        *radio_events        = set_events_for_tx_without_ack();
        rx_packet_size       = 1;
    }

    uwb_set_const_gains(phy->radio, *phy->rx.rx_constgain);

    phy->cfg.radio_actions |= SET_RADIO_ACTIONS(RX_MODE_ENABLE_TRANSMISSION);
    if ((header_size == 0) && !phy->config->certification_header_en) {
        uwb_set_cac(phy->radio, DEFAULT_RX_IDLE_PWR, DISABLE_CCA_THRSH_VALUE);
        *radio_events  = SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, WAKEUP_IT_ENABLE);
        rx_packet_size = 1;
    } else {
        uwb_set_cac(phy->radio, RX_IDLE_PWR_HIGH, *phy->tx.cca_threshold);
        phy->cfg.radio_actions |= SET_RADIO_ACTIONS(START_TX);
    }

    uwb_set_rx_packet_size(phy->radio, rx_packet_size);
    uwb_set_rx_waited_src(phy->radio, RX_WAIT_SOURCE_DEFAULT);
    if ((header_size + phy->tx.payload_size) == 0) {
        uwb_set_tx_packet_size(phy->radio, 0);
    } else {
        uwb_set_tx_packet_size(phy->radio,
                               (header_size + phy->tx.payload_size + HDR_SIZE_HDR_SIZE));
    }
    uwb_set_rx_pause_time(phy->radio, phy->tx.cca_retry_time);

#ifdef WPS_ENABLE_CUT_THROUGH_MODE
    uwb_set_packet_config(phy->radio, phy->config->packet_cfg);
    if (frame_fits_in_radio_fifo(phy->tx.payload_size, header_size)) {
        enqueue_tx_prepare_frame_states(phy, header_size, phy->tx.payload_size);
        enqueue_states(phy, wait_radio_states_tx);
    } else {
        uint8_t bufload_thresh = get_bufload_thresh_tx(phy->config->fec, phy->tx.payload_size / phy->partial_frame_count);

        uwb_set_tx_buffer_load_irq_threshold(phy->radio, bufload_thresh);

        for (int i = 0; i < (phy->partial_frame_count - 1); i++) {
            enqueue_states(phy, set_payload_cut_through_states);
        }
        enqueue_states(phy, get_event_states);
    }
#else
    enqueue_tx_prepare_frame_states(phy, header_size, phy->tx.payload_size);
    enqueue_states(phy, wait_radio_states_tx);
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */
}

/** @brief Sub function to prepare a receive frame.
 *
 *  @param[in] signal_data   Data required to process the state. The type shall be wps_phy_t.
 *  @param[in] radio_events  Radio's interrupt events enumeration.
 *  @param[in] radio_actions   Radio actions instance.
 */
static void prepare_radio_rx(wps_phy_t *phy, int_flag_cfg_t *radio_events)
{
    uint8_t payload_size;
    uint8_t header_size;
    uint8_t packet_size;

    phy->cfg.channel                   = phy->config->channel;
    phy->cfg.power_up_delay            = &phy->config->power_up_delay;
    phy->cfg.rx_timeout                = &phy->config->rx_timeout;
    phy->cfg.sleep_time                = &phy->config->sleep_time;
    phy->cfg.rx_wait_time              = &phy->config->rx_wait_time;
    phy->cfg.sleep_level               = phy->config->sleep_level;
    phy->rx.rx_constgain               = &phy->config->rx_constgain;
    phy->rx.rssi_raw                   = &phy->config->rssi_raw;
    phy->rx.rnsi_raw                   = &phy->config->rnsi_raw;
    phy->rx.frame                      = &phy->xlayer_main->frame;
    phy->rx.signal                     = &phy->signal_main;
    phy->rx.payload_size               = phy->xlayer_main->frame.payload_memory_size - phy->xlayer_main->frame.header_memory_size - SIZE_HDR_SIZE;
    phy->rx.header_size                = phy->xlayer_main->frame.header_memory_size;
    phy->tx.modulation                 = &phy->config->modulation;
    phy->tx.fec                        = &phy->config->fec;
    phy->tx.cca_threshold              = &phy->config->cca_threshold;

    /* Autoreply mode */
    if (phy->xlayer_auto != NULL) {
        phy->auto_reply_mode         = AUTO_REPLY_ENABLE;
        *radio_events                = set_events_for_rx_with_ack();
        phy->cfg.destination_address = &phy->xlayer_auto->frame.destination_address;
        phy->cfg.source_address      = &phy->xlayer_auto->frame.source_address;
        phy->tx.frame                = &phy->xlayer_auto->frame;
        phy->tx.signal               = &phy->signal_auto;
        phy->cfg.phase_info          = phy->config->phases_info;

        payload_size = phy->tx.frame->payload_end_it - phy->tx.frame->payload_begin_it;
        header_size  = phy->tx.frame->header_end_it - phy->tx.frame->header_begin_it;
        packet_size = (header_size + payload_size == 0) ? 0 : (header_size + payload_size + HDR_SIZE_HDR_SIZE);
        uwb_set_tx_packet_size(phy->radio, packet_size);
        enqueue_tx_prepare_frame_states(phy, header_size, payload_size);
        enqueue_states(phy, wait_radio_states_rx);
        /* Ack mode */
    } else if (phy->config->expect_ack) {
        phy->auto_reply_mode         = AUTO_REPLY_ENABLE;
        *radio_events                = set_events_for_rx_with_ack();
        phy->cfg.destination_address = &phy->xlayer_main->frame.source_address;
        phy->cfg.source_address      = &phy->xlayer_main->frame.destination_address;
        phy->cfg.phase_info          = NULL;
        phy->tx.frame                = &unused_frame;
        phy->tx.signal               = &unused_signal;
        enqueue_rx_prepare_frame_states(phy);

        payload_size = phy->tx.frame->payload_end_it - phy->tx.frame->payload_begin_it;
        header_size  = phy->tx.frame->header_end_it - phy->tx.frame->header_begin_it;
        uwb_set_tx_packet_size(phy->radio, (header_size + payload_size));
        /* Nack mode */
    } else {
        phy->auto_reply_mode         = AUTO_REPLY_DISABLE;
        *radio_events                = set_events_for_rx_without_ack();
        phy->cfg.destination_address = &phy->xlayer_main->frame.source_address;
        phy->cfg.source_address      = &phy->xlayer_main->frame.destination_address;
        phy->cfg.phase_info          = NULL;
        phy->tx.frame                = &unused_frame;
        phy->tx.signal               = &unused_signal;
        enqueue_rx_prepare_frame_states(phy);
    }

#ifdef WPS_ENABLE_CUT_THROUGH_MODE
    if (!frame_fits_in_radio_fifo(phy->rx.payload_size, phy->rx.header_size)) {
        uint8_t bufload_thresh = get_bufload_thresh_rx(phy->config->fec, phy->rx.payload_size / phy->partial_frame_count);

        uwb_set_rx_buffer_load_irq_threshold(phy->radio, bufload_thresh);
    }
    uwb_set_packet_config(phy->radio, phy->config->packet_cfg);
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */

    uwb_set_rx_packet_size(phy->radio, phy->xlayer_main->frame.payload_memory_size + phy->rx.header_size + HDR_SIZE_HDR_SIZE);
    uwb_set_const_gains(phy->radio, *phy->rx.rx_constgain);
    uwb_set_rx_waited_src(phy->radio, RX_WAIT_SOURCE_REGISTER);
    /* Disable CCA */
    uwb_set_cac(phy->radio, RX_IDLE_PWR_HIGH, DISABLE_CCA_THRSH_VALUE);
    uwb_set_rx_pause_time(phy->radio, 0);
    phy->cfg.radio_actions |= SET_RADIO_ACTIONS(RX_MODE_ENABLE_RECEPTION);
}

/** @brief State : prepare the radio to send or receive a frame.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void prepare_radio(wps_phy_t *phy)
{
    int_flag_cfg_t radio_events = INT_FLAG_ENABLE_CLEAR;
    main_modem_feat_t main_modem_feat;

    phy->signal_auto       = PHY_SIGNAL_NONE;
    phy->cfg.radio_actions = RADIO_ACTIONS_CLEAR;

    if (main_is_tx(phy)) {
        prepare_radio_tx(phy, &radio_events);
    } else {
        prepare_radio_rx(phy, &radio_events);
    }

    /* Clear radio flags */
    (void)uwb_get_irq_flags(phy->radio);

    uwb_set_destination_address(phy->radio, *phy->cfg.destination_address, ADDRESS_LENGTH_16);
    /* Setup auto-reply in radio */
    main_modem_feat = SET_MAIN_MODEM_FEAT(MAIN_MODEM_FEAT_CLEAR, phy->auto_reply_mode, *phy->tx.modulation, *phy->tx.fec, AUTO_TX_DISABLE,
                                          ISI_MITIG_0);
    uwb_set_main_modem_features(phy->radio, main_modem_feat);

    uwb_set_int_flag(phy->radio, radio_events | phy->radio->irq_polarity);

#if WPS_RADIO_COUNT == 1
#ifdef WPS_ENABLE_CUT_THROUGH_MODE
    uwb_set_radio_actions(phy->radio, SET_RADIO_ACTIONS(phy->cfg.radio_actions,
                                                        GO_TO_SLEEP,
                                                        FLUSH_RX_RESET_RX_BUFFER,
                                                        FLUSH_TX_RESET_TX_BUFFER));
#else
    uwb_set_radio_actions(phy->radio,
                          SET_RADIO_ACTIONS(phy->cfg.radio_actions, GO_TO_SLEEP, FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER));
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */
#else
    uwb_set_radio_actions(phy->radio,
                          SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, GO_TO_SLEEP, FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER));
#endif

    uwb_set_integgain(phy->radio, phy->config->channel->integgain);
    uwb_set_timer_cfg_burst(phy->radio, *phy->cfg.sleep_time, *phy->cfg.rx_timeout, 3, *phy->cfg.power_up_delay);
}

/** @brief State : Send the Radio config through the SPI.
 *
 *  @param[in] signal_data Data required to process the state. The type shall be wps_phy_t.
 */
static void set_config(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_PREPARE_DONE;
    uwb_transfer_non_blocking(phy->radio);
}

static void set_channel_config(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_YIELD;
    if ((phy->cfg.sleep_level == SLEEP_IDLE) && (phy->radio->phy_version == PHY_VERSION_8_3)) {
        /* #1: Bit INTEGLEN of register DLLTUNING (0x1D) needs to be set then reset every time the radio is put to sleep,
         *     after the sleep time compare value is configured, to ensure proper operation.
         *     It is located here since set_channel_config is supposed to do the last SPI transfer before the transceiver
         *     is sent to sleep.
         */
        uwb_set_dll_tuning(phy->radio, SET_DLLTUNING(INTEGLEN_SHORT));
        uwb_set_dll_tuning(phy->radio, SET_DLLTUNING(INTEGLEN_FULL));
    }
    uwb_set_rx_filters_raw(phy->radio, phy->config->channel->channel.rx_filter);
    uwb_select_channel(phy->radio, (uint8_t *)phy->config->channel->channel.tx_pattern,
                       phy->config->channel->pulse_size);
    uwb_transfer_non_blocking(phy->radio);
}

/** @brief State : Fill the header of the frame in the radio tx fifo.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void set_header(wps_phy_t *phy)
{
    xlayer_frame_t *frame = phy->tx.frame;

    sr_access_disable_radio_irq(&phy->radio->radio_hal);

    phy->signal_main = PHY_SIGNAL_YIELD;

    if (phy->config->certification_header_en) {
        uwb_write_register_8(phy->radio, REG_TXFIFO, PHY_CERTIF_BYTE1);
        uwb_transfer_blocking(phy->radio);
        uwb_fill_header_non_blocking(phy->radio, phy->certification_header,
                                     (frame->header_end_it - frame->header_begin_it));
    } else {
        uwb_write_register_8(phy->radio, REG_TXFIFO,
                             (frame->header_end_it - frame->header_begin_it));
        uwb_transfer_blocking(phy->radio);
        uwb_fill_header_non_blocking(phy->radio, frame->header_begin_it,
                                     (frame->header_end_it - frame->header_begin_it));
    }
}

/** @brief State : Fill the payload of the frame in the radio tx fifo.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void set_payload(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    xlayer_frame_t *frame = phy->tx.frame;

    phy->signal_main = PHY_SIGNAL_YIELD;

    uwb_fill_data_non_blocking(phy->radio, frame->payload_begin_it, (frame->payload_end_it - frame->payload_begin_it));
}

/** @brief State : Re-enable the radio when the data have been written on the radio.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void enable_radio_irq(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }
    phy->signal_main = PHY_SIGNAL_CONFIG_COMPLETE;
    sr_access_enable_radio_irq(&phy->radio->radio_hal);

    /* If we missed the rising edge, do a context switch */
    if (phy->radio->radio_hal.read_irq_pin()) {
        phy->radio->radio_hal.context_switch();
    }
}

/** @brief State : Re-enable the radio when the data have been written on the radio.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void check_radio_irq(wps_phy_t *phy)
{
    /* irq pin is low, auto-reply has not finish to send */
    sr_access_enable_radio_irq(&phy->radio->radio_hal);
    if (!phy->radio->radio_hal.read_irq_pin()) {
        phy->signal_main = PHY_SIGNAL_YIELD;
    }
}

/** @brief State : Ask the radio for the irq flags after a radio interrupt.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void read_events(wps_phy_t *phy)
{
    read_reg_16_t read_irq_status;

    if (phy->input_signal != PHY_SIGNAL_RADIO_IRQ) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    read_irq_status     = uwb_get_irq_flags(phy->radio);
    phy->irq_status_1   = read_irq_status.msb;
    phy->irq_status_2   = read_irq_status.lsb;
    phy->pwr_status_cmd = uwb_read_register_8(phy->radio, REG_PWRSTATUS);

    phy->signal_main = PHY_SIGNAL_YIELD;
    uwb_transfer_non_blocking(phy->radio);
}

/** @brief State : Ask the radio for the IRQ flags after a radio interrupt when syncing.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void read_events_syncing(wps_phy_t *phy)
{
    read_reg_16_t read_irq_status;

    if (phy->input_signal != PHY_SIGNAL_RADIO_IRQ) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    uwb_set_timer_config(phy->radio, SET_TIMER_CFG(TIMER_CFG_CLEAR, AUTOWAKE_UP_ENABLE, WAKE_UP_ONCE_ENABLE));

    read_irq_status     = uwb_get_irq_flags(phy->radio);
    phy->irq_status_1   = read_irq_status.msb;
    phy->irq_status_2   = read_irq_status.lsb;
    phy->pwr_status_cmd = uwb_read_register_8(phy->radio, REG_PWRSTATUS);

    phy->signal_main = PHY_SIGNAL_YIELD;
    uwb_transfer_non_blocking(phy->radio);
}

/** @brief State : Read the IRQ flags and take action regarding of the outcome for TX.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void process_event_tx(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    radio_events_t radio_events;

    radio_events = (*phy->irq_status_2 << 8) | (*phy->irq_status_1);

    /* Handle CCA fail */
    if (radio_events & CSC_FAIL_IT) {
        handle_cca_fail(phy);
        /* Handle autoreply reception */
    } else if (rx_good(radio_events)) {
        phy->tx.frame->frame_outcome = FRAME_SENT_ACK;
        phy->rx.frame->frame_outcome = FRAME_RECEIVED;
        handle_good_auto_reply(phy);
    } else if (rx_lost(radio_events)) {
        /* #2: When a timeout occurs, check if the RXEN bit of the transceiver is set and if so, clear
         *     any pending IRQs and disable the transceiver interrupts to ensure proper operation.
         *     Interrupts will be automatically re-enabled in a subsequent stage of the state machine.
         */
        if (BIT_RXEN & *phy->pwr_status_cmd) {
            sr_access_disable_radio_irq(&phy->radio->radio_hal);
            phy->tx.frame->frame_outcome = FRAME_SENT_ACK_LOST;
            phy->rx.frame->frame_outcome = FRAME_LOST;
            handle_missed_auto_reply(phy);
            enqueue_states(phy, prepare_phy_states);
        } else {
            phy->tx.frame->frame_outcome = FRAME_SENT_ACK_LOST;
            phy->rx.frame->frame_outcome = FRAME_LOST;
            handle_missed_auto_reply(phy);
            enqueue_states(phy, prepare_phy_states);
        }
    } else if (rx_rejected(radio_events)) {
        phy->tx.frame->frame_outcome = FRAME_SENT_ACK_REJECTED;
        phy->rx.frame->frame_outcome = FRAME_REJECTED;
        handle_missed_auto_reply(phy);
        enqueue_states(phy, prepare_phy_states);
        /* Handle TX */
    } else if (tx_complete(radio_events)) {
        *phy->tx.signal              = PHY_SIGNAL_FRAME_SENT_NACK;
        *phy->rx.signal              = PHY_SIGNAL_FRAME_MISSED;
        phy->tx.frame->frame_outcome = FRAME_SENT_ACK_LOST;
        phy->rx.frame->frame_outcome = FRAME_LOST;
        enqueue_states(phy, prepare_phy_states);
    } else if (radio_events & WAKEUP_IT) {
        *phy->tx.signal              = PHY_SIGNAL_FRAME_SENT_NACK;
        *phy->rx.signal              = PHY_SIGNAL_FRAME_MISSED;
        phy->tx.frame->frame_outcome = FRAME_WAIT;
        phy->rx.frame->frame_outcome = FRAME_LOST;
        enqueue_states(phy, prepare_phy_states);
    }
}

/** @brief State : Read the IRQ flags and take action regarding of the outcome for RX.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void process_event_rx(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    radio_events_t radio_events;

    radio_events = (*phy->irq_status_2 << 8) | (*phy->irq_status_1);

    /* Handle RX frame */
    if (rx_good(radio_events)) {
        phy->tx.frame->frame_outcome = FRAME_SENT_ACK;
        phy->rx.frame->frame_outcome = FRAME_RECEIVED;
        handle_good_frame(phy);
    } else if (rx_lost(radio_events)) {
        /* #3: When a timeout occurs, check if the RXEN bit of the transceiver is set and if so, clear
         *     any pending IRQs and disable the transceiver interrupts to ensure proper operation.
         *     Interrupts will be automatically re-enabled in a subsequent stage of the state machine.
         */
        if (BIT_RXEN & *phy->pwr_status_cmd) {
            sr_access_disable_radio_irq(&phy->radio->radio_hal);
            phy->tx.frame->frame_outcome = FRAME_SENT_ACK_LOST;
            phy->rx.frame->frame_outcome = FRAME_LOST;
            handle_missed_frame(phy);
            enqueue_states(phy, prepare_phy_states);
        } else {
            phy->tx.frame->frame_outcome = FRAME_SENT_ACK_LOST;
            phy->rx.frame->frame_outcome = FRAME_LOST;
            handle_missed_frame(phy);
            enqueue_states(phy, prepare_phy_states);
        }
    } else if (rx_rejected(radio_events)) {
        phy->tx.frame->frame_outcome = FRAME_SENT_ACK_REJECTED;
        phy->rx.frame->frame_outcome = FRAME_REJECTED;
        handle_missed_frame(phy);
        enqueue_states(phy, prepare_phy_states);
#ifdef WPS_ENABLE_CUT_THROUGH_MODE
    } else if (radio_events & BUF_LOAD_TH_IT) {
        enqueue_states(phy, get_payload_cut_through_states);
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */
    }
}

#ifndef WPS_ENABLE_CUT_THROUGH_MODE
/** @brief Handle a good frame received by the radio.
 *
 *  Ask the radio for RX wait time, RSSI, RNSI and payload size.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_good_frame(wps_phy_t *phy)
{
    int_flag_cfg_t radio_events = INT_FLAG_ENABLE_CLEAR;
    radio_events_t last_events  = (*phy->irq_status_2 << 8) | (*phy->irq_status_1);

    phy->signal_main = PHY_SIGNAL_YIELD;
    phy->rx_wait     = uwb_get_rx_wait_time(phy->radio);
    phy->rssi        = uwb_get_rssi(phy->radio);
    phy->rnsi        = uwb_get_rnsi(phy->radio);

    if (phy->xlayer_auto != NULL) {
        if (auto_is_tx(phy) && !(last_events & BROADCAST_IT)) {
            phy->wait_for_ack_tx = true;
            if (!tx_complete_auto_reply(last_events)) {
                /*Tx end is enable to wait the transmission of the autoreply.*/
                radio_events = set_events_for_rx_with_auto_payload();
                uwb_set_int_flag(phy->radio, radio_events | phy->radio->irq_polarity);
                sr_access_disable_radio_irq(&phy->radio->radio_hal);
            } else {
                phy->wait_for_ack_tx = false;
            }

            phy->signal_auto = PHY_SIGNAL_FRAME_SENT_NACK;
        }
    }
    phy->rx_frame_size = uwb_read_frame_size(phy->radio);
    if (phy->cfg.phase_info != NULL) {
        phy->phase_info = uwb_get_phase_info(phy->radio);
    }
    uwb_transfer_non_blocking(phy->radio);
    enqueue_states(phy, get_frame_header_states);
}

/** @brief Handle a good auto reply received by the radio.
 *
 *  Ask the radio for RX wait time, RSSI, RNSI and payload size.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_good_auto_reply(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_YIELD;
    phy->rx_wait     = uwb_get_rx_wait_time(phy->radio);
    phy->rssi        = uwb_get_rssi(phy->radio);
    phy->rnsi        = uwb_get_rnsi(phy->radio);

    phy->rx_frame_size = uwb_read_frame_size(phy->radio);
    if (phy->cfg.phase_info != NULL) {
        phy->phase_info = uwb_get_phase_info(phy->radio);
    }
    uwb_transfer_non_blocking(phy->radio);
    enqueue_states(phy, get_auto_reply_header_states);
}
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */

/** @brief Handle a missed frame.
 *
 *  Set the signals to notify the user and flush the TX fifo in RX mode.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_missed_frame(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_YIELD;

    uwb_set_radio_actions(phy->radio,
                          SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, GO_TO_SLEEP, FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER));

    uwb_transfer_non_blocking(phy->radio);
    enqueue_states(phy, flush_fifo_states);
}

/** @brief Handle a missed auto reply.
 *
 *  Set the signals to notify the user and flush the TX fifo in RX mode.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_missed_auto_reply(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_YIELD;

    *phy->tx.signal = PHY_SIGNAL_FRAME_SENT_NACK;
    *phy->rx.signal = PHY_SIGNAL_FRAME_MISSED;
}

/** @brief Handle a Clear Channel Assessment (CCA) fail.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_cca_fail(wps_phy_t *phy)
{
    phy->signal_main                    = PHY_SIGNAL_YIELD;
    uint8_t empty_tx_pattern[NB_PULSES] = {0};

    (*phy->tx.cca_try_count)++;
    if (*phy->tx.cca_try_count >= phy->tx.cca_max_try_count) {
        /* Disable CCA */
        uwb_set_cac(phy->radio, DEFAULT_RX_IDLE_PWR, DISABLE_CCA_THRSH_VALUE);
        if (phy->tx.cca_fail_action == CCA_FAIL_ACTION_ABORT_TX) {
            uwb_set_radio_actions(phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, FLUSH_TX_RESET_TX_BUFFER));
            uwb_select_channel(phy->radio, empty_tx_pattern, phy->config->channel->pulse_size);
        }
        uwb_transfer_blocking(phy->radio);
    }

    enqueue_states(phy, get_event_states);
}

/** @brief State : Ask the radio to get the frame header.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void get_frame_header(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    *phy->cfg.rx_wait_time = (MOV2MASK(*phy->rx_wait.rx_wait_time1, BITS_RXWAITED8) << 8) | (*phy->rx_wait.rx_wait_time0);
    *phy->rx.rssi_raw      = *phy->rssi;
    *phy->rx.rnsi_raw      = *phy->rnsi;
    if (phy->cfg.phase_info != NULL) {
        phy->cfg.phase_info->phase1 = phy->phase_info[0];
        phy->cfg.phase_info->phase2 = phy->phase_info[1];
        phy->cfg.phase_info->phase3 = phy->phase_info[2];
        phy->cfg.phase_info->phase4 = phy->phase_info[3];
    }

    if (*phy->rx_frame_size == 0) {
        phy->rx.frame->payload_end_it = phy->rx.frame->header_begin_it;
        *phy->tx.signal               = PHY_SIGNAL_FRAME_SENT_NACK;
        *phy->rx.signal               = PHY_SIGNAL_FRAME_MISSED;
        enqueue_states(phy, prepare_phy_states);
    } else {
        *phy->rx_frame_size -= HDR_SIZE_HDR_SIZE;
        phy->cfg.header_size = uwb_read_frame_size(phy->radio);
        uwb_transfer_blocking(phy->radio);
        phy->signal_main = PHY_SIGNAL_YIELD;

        phy->rx.frame->header_begin_it  = phy->rx.frame->header_memory;
        phy->rx.frame->payload_begin_it = phy->rx.frame->header_memory;
        phy->rx.frame->payload_end_it   = phy->rx.frame->header_memory + *phy->cfg.header_size + EMPTY_BYTE;

        phy->radio->access_sequence.tx_buffer[0] = REG_READ_BURST | REG_RXFIFO;
        sr_access_spi_transfer_non_blocking(&phy->radio->radio_hal, phy->radio->access_sequence.tx_buffer, phy->rx.frame->header_memory,
                                            *phy->cfg.header_size + EMPTY_BYTE);
        enqueue_states(phy, get_payload_states);
        enqueue_states(phy, prepare_phy_states);
    }
}

/** @brief State : Ask the radio to get the auto reply header.
 *
 *  If the payload is empty, the user is notified.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void get_auto_reply_header(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    *phy->cfg.rx_wait_time = (MOV2MASK(*phy->rx_wait.rx_wait_time1, BITS_RXWAITED8) << 8) | (*phy->rx_wait.rx_wait_time0);
    *phy->rx.rssi_raw      = *phy->rssi;
    *phy->rx.rnsi_raw      = *phy->rnsi;
    if (phy->cfg.phase_info != NULL) {
        phy->cfg.phase_info->phase1 = phy->phase_info[0];
        phy->cfg.phase_info->phase2 = phy->phase_info[1];
        phy->cfg.phase_info->phase3 = phy->phase_info[2];
        phy->cfg.phase_info->phase4 = phy->phase_info[3];
    }

    if ((*phy->rx_frame_size == 0) || (phy->rx.frame == &unused_frame)) {
        phy->rx.frame->payload_end_it = phy->rx.frame->header_begin_it;
        *phy->tx.signal               = PHY_SIGNAL_FRAME_SENT_ACK;
        *phy->rx.signal               = PHY_SIGNAL_FRAME_MISSED;
        enqueue_states(phy, prepare_phy_states);
    } else {
        *phy->rx_frame_size -= HDR_SIZE_HDR_SIZE;
        phy->cfg.header_size = uwb_read_frame_size(phy->radio);
        uwb_transfer_blocking(phy->radio);
        phy->signal_main = PHY_SIGNAL_YIELD;

        phy->rx.frame->header_begin_it  = phy->rx.frame->header_memory;
        phy->rx.frame->payload_begin_it = phy->rx.frame->header_memory;
        phy->rx.frame->payload_end_it   = phy->rx.frame->header_memory + *phy->cfg.header_size + EMPTY_BYTE;

        phy->radio->access_sequence.tx_buffer[0] = REG_READ_BURST | REG_RXFIFO;
        sr_access_spi_transfer_non_blocking(&phy->radio->radio_hal, phy->radio->access_sequence.tx_buffer, phy->rx.frame->header_memory,
                                            *phy->cfg.header_size + EMPTY_BYTE);
        enqueue_states(phy, get_payload_states);
        enqueue_states(phy, prepare_phy_states);
    }
}

/** @brief State : Ask the radio to get the payload.
 *
 *  If the payload is empty, the user is notified.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void get_payload(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    uint8_t payload_size = *phy->rx_frame_size - *phy->cfg.header_size;

    *phy->rx.signal = PHY_SIGNAL_FRAME_RECEIVED;

    if (payload_size == 0) {
        sr_access_close(&phy->radio->radio_hal);
    } else {
        sr_access_spi_transfer_non_blocking(&phy->radio->radio_hal, phy->radio->access_sequence.tx_buffer, phy->rx.frame->payload_end_it,
                                            payload_size);
        phy->rx.frame->payload_end_it += payload_size;
        enqueue_states(phy, new_frame_states);
    }
    if ((phy->xlayer_auto != NULL) && phy->wait_for_ack_tx) {
        if (auto_is_tx(phy)) {
            enqueue_states(phy, wait_to_send_auto_reply);
        } else {
            *phy->tx.signal = PHY_SIGNAL_FRAME_SENT_ACK;
        }
    }
}

/** @brief State : handle the end of the tx fifo flush. Notify the user for a frame lost.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void end_flush(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    *phy->tx.signal = (phy->xlayer_auto != NULL) ? PHY_SIGNAL_FRAME_NOT_SENT : PHY_SIGNAL_FRAME_SENT_NACK;
    *phy->rx.signal = PHY_SIGNAL_FRAME_MISSED;
}

/** @brief State : Close the spi after transmission or reception.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void close_spi(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        phy->state_step--;
        phy->signal_main = PHY_SIGNAL_YIELD;
        return;
    }

    sr_access_close(&phy->radio->radio_hal);
}

/** @brief State : End of a state machine sequence.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void end(wps_phy_t *phy)
{
    wps_phy_state_t **dequeue_state;

    phy->state_step    = 0;
    dequeue_state      = (wps_phy_state_t **)circular_queue_front_raw(&phy->next_states);
    phy->current_state = *dequeue_state;
    circular_queue_dequeue_raw(&phy->next_states);
}

static void none(wps_phy_t *phy)
{
    (void)phy;
}

/** @brief Clear radio irq pin on error.
 *
 *  @param wps_phy WPS PHY instance.
 */
static void clear_err(wps_phy_t *wps_phy)
{
    /* Disable radio interrupts */
    uwb_disable_irq(wps_phy->radio);

    /* Clear Status */
    (void)uwb_get_irq_flags(wps_phy->radio);

    uwb_set_timer_config(wps_phy->radio, SET_TIMER_CFG(TIMER_CFG_CLEAR, AUTOWAKE_UP_ENABLE, WAKE_UP_ONCE_ENABLE));

    uwb_set_rx_timeout_raw(wps_phy->radio, 0xFFFF, 0xFF);

    uwb_set_radio_actions(wps_phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, INIT_TIMER_RESET_BOTH_WAKE_UP_TIMER,
                                                            FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER, GO_TO_SLEEP));

    sr_access_enable_radio_irq(&wps_phy->radio->radio_hal);
    uwb_transfer_blocking(wps_phy->radio);
}

static void prepare_syncing(wps_phy_t *phy)
{
    main_modem_feat_t main_modem_feat;
    radio_timer_config_t timer_config;

    phy->signal_main = PHY_SIGNAL_YIELD;

    phy->cfg.destination_address       = &phy->xlayer_main->frame.source_address;
    phy->cfg.channel                   = phy->config->channel;
    phy->cfg.power_up_delay            = &phy->config->power_up_delay;
    phy->cfg.rx_timeout                = &phy->config->rx_timeout;
    phy->cfg.sleep_time                = &phy->config->sleep_time;
    phy->cfg.rx_wait_time              = &phy->config->rx_wait_time;
    phy->cfg.sleep_level               = phy->config->sleep_level;
    phy->rx.rx_constgain               = &phy->config->rx_constgain;
    phy->rx.rssi_raw                   = &phy->config->rssi_raw;
    phy->rx.rnsi_raw                   = &phy->config->rnsi_raw;
    phy->rx.frame                      = &phy->xlayer_main->frame;
    phy->rx.signal                     = &phy->signal_main;
    phy->rx.payload_size = phy->xlayer_main->frame.payload_memory_size - phy->xlayer_main->frame.header_memory_size - SIZE_HDR_SIZE;
    phy->tx.frame        = &unused_frame;
    phy->tx.signal       = &unused_signal;
    phy->tx.modulation   = &phy->config->modulation;
    phy->tx.fec          = &phy->config->fec;

    if (phy->config->expect_ack) {
        uwb_set_tx_packet_size(phy->radio, 0);
    }
    uwb_set_rx_packet_size(phy->radio, MAX_FRAMESIZE);

    uwb_set_destination_address(phy->radio, *phy->cfg.destination_address, ADDRESS_LENGTH_16);

    if (phy->config->expect_ack) {
        main_modem_feat = SET_MAIN_MODEM_FEAT(MAIN_MODEM_FEAT_CLEAR, AUTO_REPLY_ENABLE);
    } else {
        main_modem_feat = SET_MAIN_MODEM_FEAT(MAIN_MODEM_FEAT_CLEAR, AUTO_REPLY_DISABLE);
    }
    main_modem_feat |= SET_MAIN_MODEM_FEAT(*phy->tx.modulation, *phy->tx.fec, AUTO_TX_DISABLE, ISI_MITIG_0);
    uwb_set_main_modem_features(phy->radio, main_modem_feat);

    uwb_set_integgain(phy->radio, phy->cfg.channel->integgain);
    uwb_set_packet_config(phy->radio, phy->config->packet_cfg);
    uwb_set_const_gains(phy->radio, *phy->rx.rx_constgain);
    uwb_set_int_flag(phy->radio, SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, phy->radio->irq_polarity, NEWPKT_IT_ENABLE));

    timer_config = SET_TIMER_CFG(TIMER_CFG_CLEAR, AUTOWAKE_UP_ENABLE, SYNC_AT_END_DISABLE, SYNC_RX_BEG_ENABLE, WAKE_UP_ONCE_DISABLE);
    uwb_set_wake_sleep_raw(phy->radio, FAST_SYNC_TIMER_VALUE);

    /* Disable CCA */
    uwb_set_cac(phy->radio, DEFAULT_RX_IDLE_PWR, DISABLE_CCA_THRSH_VALUE);
    uwb_set_rx_pause_time(phy->radio, 0);
    uwb_set_timer_config(phy->radio, timer_config);
    uwb_set_rx_timeout_raw(phy->radio, FAST_SYNC_TIMER_VALUE - 1, 3);
    uwb_set_pwr_up_delay_raw(phy->radio, *phy->cfg.power_up_delay);

    uwb_set_radio_actions(phy->radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, FLUSH_RX_RESET_RX_BUFFER, FLUSH_TX_RESET_TX_BUFFER,
                                                        GO_TO_SLEEP, RX_MODE_ENABLE_RECEPTION));

    uwb_set_rx_filters_raw(phy->radio, phy->config->channel->channel.rx_filter);
    uwb_select_channel(phy->radio, (uint8_t *)phy->cfg.channel->channel.tx_pattern, phy->cfg.channel->pulse_size);

    sr_access_disable_radio_irq(&phy->radio->radio_hal);

    uwb_transfer_non_blocking(phy->radio);
}

/** @brief Get if the main frame is in transmit mode.
 *
 *  @param[in] phy Layer one instance.
 *  @retval true
 *  @retval false
 */
static bool main_is_tx(wps_phy_t *phy)
{
    return (phy->xlayer_main->frame.destination_address != phy->source_address);
}

/** @brief Get if the main frame is in transmit mode.
 *
 *  @param[in] phy Layer one instance.
 *  @retval true
 *  @retval false
 */
static bool auto_is_tx(wps_phy_t *phy)
{
    return (phy->xlayer_auto->frame.destination_address != phy->source_address);
}

/** @brief Get the TX complete status.
 *
 *  @param[in] radio_events
 *  @retval true
 *  @retval false
 */
static bool tx_complete(radio_events_t radio_events)
{
    return (((radio_events & TX_END_IT) && !(radio_events & NEW_PACKET_IT) && !(radio_events & RX_TIMEOUT_IT))) ||
           (radio_events & TX_UNDERFLOW_IT);
}

/** @brief Get the TX complete of auto-reply frame status.
 *
 *  @note This is slightly different then tx_complete, since
 *        NEW_PACKET_IT is trigger during reception of frame, so
 *        only event to check is TX_END_IT
 *
 *  @param[in] radio_events
 *  @retval true
 *  @retval false
 */
static bool tx_complete_auto_reply(radio_events_t radio_events)
{
    return (((radio_events & TX_END_IT) && !(radio_events & RX_TIMEOUT_IT))) || (radio_events & TX_UNDERFLOW_IT);
}

/** @brief Get the RX good status.
 *
 *  @param[in] radio_events
 *  @retval true
 *  @retval false
 */
static bool rx_good(radio_events_t radio_events)
{
    return ((radio_events & NEW_PACKET_IT) && (radio_events & CRC_PASS_IT) &&
            ((radio_events & ADDR_MATCH_IT) || (radio_events & BROADCAST_IT)));
}

/** @brief Get the RX rejected status.
 *
 *  @param[in] radio_events
 *  @retval true
 *  @retval false
 */
static bool rx_rejected(radio_events_t radio_events)
{
    return ((radio_events & NEW_PACKET_IT) &&
            (!(radio_events & CRC_PASS_IT) || !((radio_events & ADDR_MATCH_IT) || (radio_events & BROADCAST_IT))));
}

/** @brief Get the RX lost status.
 *
 *  @param[in] radio_events
 *  @retval true
 *  @retval false
 */
static bool rx_lost(radio_events_t radio_events)
{
    return (radio_events & RX_TIMEOUT_IT);
}

/** @brief Set the events for tx with ack.
 *
 *  @return radio_events_t
 */
static int_flag_cfg_t set_events_for_tx_with_ack(void)
{
    return SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, NEWPKT_IT_ENABLE, RXTIMEO_IT_ENABLE, BUFLOAD_IT_ENABLE, CSC_FAIL_IT_ENABLE,
                            TXUDRFL_IT_ENABLE);
}

/** @brief Set the events for tx without ack.
 *
 *  @return radio_events_t
 */
static int_flag_cfg_t set_events_for_tx_without_ack(void)
{
    return SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, TXEND_IT_ENABLE, BUFLOAD_IT_ENABLE, CSC_FAIL_IT_ENABLE, TXUDRFL_IT_ENABLE);
}

/** @brief Set the events for rx with ack.
 *
 *  @return radio_events_t
 */
static int_flag_cfg_t set_events_for_rx_with_ack(void)
{
    return SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, NEWPKT_IT_ENABLE, RXTIMEO_IT_ENABLE, BUFLOAD_IT_ENABLE);
}

/** @brief Set the events for rx without ack.
 *
 *  @return radio_events_t
 */
static int_flag_cfg_t set_events_for_rx_without_ack(void)
{
    return SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, NEWPKT_IT_ENABLE, RXTIMEO_IT_ENABLE, BUFLOAD_IT_ENABLE);
}

/** @brief Set the events for rx with payload.
 *
 *  @return radio_events_t
 */
static int_flag_cfg_t set_events_for_rx_with_auto_payload(void)
{
    return SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, TXEND_IT_ENABLE, RXTIMEO_IT_ENABLE);
}

#ifdef WPS_ENABLE_CUT_THROUGH_MODE
/** @brief Setup the PHY state machine.
 *
 *  @param[in] wps_phy        PHY instance struct.
 *  @param[in] payload_size  Frame payload size.
 */
static void prepare_phy(wps_phy_t *phy)
{
    if (phy->input_signal == PHY_SIGNAL_SYNCING) {
        enqueue_states(phy, syncing_states);
        prepare_syncing(phy);
    } else {
        phy->partial_frame_index = PARTIAL_FRAME_BASE_INDEX;
        phy->partial_frame_count = PARTIAL_FRAME_COUNT;

        if (main_is_tx(phy)) {
            phy->tx.frame        = &phy->xlayer_main->frame;
            phy->tx.payload_size = phy->tx.frame->payload_end_it - phy->tx.frame->payload_begin_it;
            if (!frame_fits_in_radio_fifo(phy->tx.payload_size, phy->tx.frame->header_memory_size)) {
                enqueue_states(phy, prepare_radio_cut_through_states);

                wps_phy_state_t **next_state = (wps_phy_state_t **)circular_queue_front_raw(&phy->next_states);

                if (*next_state == new_frame_states) {
                    phy->signal_main = PHY_SIGNAL_YIELD;
                }
            } else {
                enqueue_states(phy, set_config_states);
                prepare_radio(phy);
            }
        } else {
            enqueue_states(phy, set_config_states);
            prepare_radio(phy);
        }
        if (main_is_tx(phy)) {
            phy->tx.frame        = &phy->xlayer_main->frame;
            phy->tx.payload_size = phy->tx.frame->payload_end_it - phy->tx.frame->payload_begin_it;
        }
        enqueue_states(phy, set_config_states);
        prepare_radio(phy);
    }
}

/** @brief State : Setup get payload in cut-through mode.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void setup_cut_through(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    if (phy->partial_frame_index >= (phy->partial_frame_count - 1)) {
        *phy->cfg.rx_wait_time = (MOV2MASK(*phy->rx_wait.rx_wait_time1, BITS_RXWAITED8) << 8) | (*phy->rx_wait.rx_wait_time0);
        *phy->rx.rssi_raw      = *phy->rssi;
        *phy->rx.rnsi_raw      = *phy->rnsi;
    }

    if (phy->partial_frame_index >= (phy->partial_frame_count - 2)) {
        uwb_disable_rx_buffer_load_irq(phy->radio);
    }

    phy->signal_main = PHY_SIGNAL_YIELD;

    phy->radio->access_sequence.tx_buffer[phy->radio->access_sequence.index++] = REG_READ_BURST | REG_RXFIFO;
    uwb_transfer_non_blocking(phy->radio);
}

/** @brief State : Ask the radio to get the payload of a partial frame in cut-through mode.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void get_payload_cut_through(wps_phy_t *phy)
{
    uint8_t partial_frame_len;

    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    phy->signal_main = PHY_SIGNAL_YIELD;

    /* First partial frame */
    if (phy->partial_frame_index == 0) {
        partial_frame_len = phy->rx.payload_size / phy->partial_frame_count + phy->rx.payload_size % phy->partial_frame_count +
                            phy->rx.frame->header_memory_size;

        phy->rx.frame->header_begin_it  = phy->rx.frame->header_memory;
        phy->rx.frame->payload_begin_it = phy->rx.frame->header_memory;
        phy->rx.frame->payload_end_it   = phy->rx.frame->header_memory + EMPTY_BYTE;
        /* Subsequent partial frames */
    } else if (phy->partial_frame_index < (phy->partial_frame_count - 1)) {
        partial_frame_len = phy->rx.payload_size / phy->partial_frame_count;
        /* Last partial frame */
    } else {
        partial_frame_len = phy->rx.payload_size / phy->partial_frame_count;

        *phy->tx.signal = main_is_tx(phy) ? PHY_SIGNAL_FRAME_SENT_ACK : PHY_SIGNAL_FRAME_SENT_NACK;
        *phy->rx.signal = PHY_SIGNAL_FRAME_RECEIVED;
    }

    sr_access_spi_transfer_non_blocking(&phy->radio->radio_hal, phy->radio->access_sequence.tx_buffer, phy->rx.frame->payload_end_it,
                                        partial_frame_len);

    phy->rx.frame->payload_end_it += partial_frame_len;
    phy->partial_frame_index++;

    if (phy->partial_frame_index < phy->partial_frame_count) {
        sr_access_disable_radio_irq(&phy->radio->radio_hal);
        enqueue_states(phy, wait_radio_states_rx);
    } else {
        enqueue_states(phy, prepare_phy_states);
        enqueue_states(phy, new_frame_states);
    }
}

/** @brief State : handle the end of the tx fifo flush. Notify the user for a frame lost.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void end_tx_cut_through(wps_phy_t *phy)
{
    if (phy->input_signal != PHY_SIGNAL_DMA_CMPLT) {
        clear_err(phy);
        phy->signal_main = PHY_SIGNAL_ERROR;
        return;
    }

    *phy->rx.rssi_raw = *phy->rssi;
    *phy->rx.rnsi_raw = *phy->rnsi;

    *phy->tx.signal = PHY_SIGNAL_FRAME_SENT_ACK;
}

/** @brief Handle a good frame received by the radio.
 *
 *  Ask the radio for RX wait time, RSSI, RNSI and payload size.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_good_frame(wps_phy_t *phy)
{
    int_flag_cfg_t radio_events = INT_FLAG_ENABLE_CLEAR;
    radio_events_t last_events  = (*phy->irq_status_2 << 8) | (*phy->irq_status_1);

    phy->signal_main = PHY_SIGNAL_YIELD;
    phy->rx_wait     = uwb_get_rx_wait_time(phy->radio);
    phy->rssi        = uwb_get_rssi(phy->radio);
    phy->rnsi        = uwb_get_rnsi(phy->radio);

    if ((phy->xlayer_auto != NULL) {
        if (auto_is_tx(phy)) {
            phy->wait_for_ack_tx = true;
            if (!tx_complete_auto_reply(last_events)) {
                /*Tx end is enable to wait the transmission of the autoreply.*/
                radio_events = set_events_for_rx_with_auto_payload();
                uwb_set_int_flag(phy->radio, radio_events | phy->radio->irq_polarity);
                sr_access_disable_radio_irq(&phy->radio->radio_hal);
            } else {
                phy->wait_for_ack_tx = false;
            }

            phy->signal_auto = PHY_SIGNAL_FRAME_SENT_NACK;
        }
    }
    phy->rx_frame_size = uwb_read_frame_size(phy->radio);
    if (phy->cfg.phase_info != NULL) {
        phy->phase_info = uwb_get_phase_info(phy->radio);
    }
    uwb_transfer_non_blocking(phy->radio);
    enqueue_states(phy, get_frame_header_states);
}

/** @brief Handle a good auto reply received by the radio.
 *
 *  Ask the radio for RX wait time, RSSI, RNSI and payload size.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void handle_good_auto_reply(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_YIELD;
    phy->rx_wait     = uwb_get_rx_wait_time(phy->radio);
    phy->rssi        = uwb_get_rssi(phy->radio);
    phy->rnsi        = uwb_get_rnsi(phy->radio);

    phy->rx_frame_size = uwb_read_frame_size(phy->radio);
    if (phy->cfg.phase_info != NULL) {
        phy->phase_info = uwb_get_phase_info(phy->radio);
    }
    uwb_transfer_non_blocking(phy->radio);
    enqueue_states(phy, get_auto_reply_header_states);
}

bool frame_fits_in_radio_fifo(uint8_t payload_size, uint8_t header_size)
{
    return ((payload_size + header_size) <= MAX_FRAMESIZE);
}

/** @brief Get buffer load threshold for TX.
 *
 *  @param[in] fec_level
 *  @param[in] partial_frame_size
 *  @return buff_load_thresh
 */
uint8_t get_bufload_thresh_tx(fec_level_t fec_level, uint8_t partial_frame_size)
{
    uint8_t bufload_thresh = 0;

    switch (fec_level) {
    case FEC_LVL_0:
    case FEC_LVL_1:
        bufload_thresh = partial_frame_size - 1;
        break;
    case FEC_LVL_2:
        bufload_thresh = partial_frame_size - partial_frame_size / 4;
        break;
    case FEC_LVL_3:
        bufload_thresh = partial_frame_size - partial_frame_size / 2;
        break;
    }
    return bufload_thresh;
}

/** @brief Get buffer load threshold for RX.
 *
 *  @param[in] fec_level
 *  @param[in] partial_frame_size
 *  @return buff_load_thresh
 */
uint8_t get_bufload_thresh_rx(fec_level_t fec_level, uint8_t partial_frame_size)
{
    uint8_t bufload_thresh = 0;

    switch (fec_level) {
    case FEC_LVL_0:
    case FEC_LVL_1:
        bufload_thresh = 1 + partial_frame_size / 2;
        break;
    case FEC_LVL_2:
        bufload_thresh = partial_frame_size / 2 + partial_frame_size / 4;
        break;
    case FEC_LVL_3:
        bufload_thresh = partial_frame_size;
        break;
    }
    return bufload_thresh;
}

/** @brief State : Fill the payload of a partial frame in the radio tx fifo for cut through mode.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void set_payload_cut_through(wps_phy_t *phy)
{
    uint8_t partial_frame_len;

    sr_access_disable_radio_irq(&phy->radio->radio_hal);
    if (phy->partial_frame_index == 0) {
        phy->tx.frame->payload_end_it = phy->tx.frame->payload_begin_it;

        partial_frame_len = phy->tx.payload_size / phy->partial_frame_count + phy->tx.payload_size % phy->partial_frame_count;
        uwb_fill_data_non_blocking(phy->radio, phy->tx.frame->payload_end_it, partial_frame_len);
        phy->tx.frame->payload_end_it += partial_frame_len;
    } else {
        if (!(phy->partial_frame_index + 1 < phy->partial_frame_count)) {
            uwb_disable_tx_buffer_load_irq(phy->radio);
        }
        partial_frame_len = phy->tx.payload_size / phy->partial_frame_count;
        uwb_fill_cut_through_data(phy->radio, phy->tx.frame->payload_end_it, partial_frame_len);
        uwb_transfer_non_blocking(phy->radio);
        phy->tx.frame->payload_end_it += partial_frame_len;
        phy->signal_main = PHY_SIGNAL_YIELD;
    }
    phy->partial_frame_index++;
}

/** @brief Release the CPU.
 *
 *  @param[in] signal_data  Data required to process the state. The type shall be wps_phy_t.
 */
static void signal_yield(wps_phy_t *phy)
{
    phy->signal_main = PHY_SIGNAL_YIELD;
}
#endif /* WPS_ENABLE_CUT_THROUGH_MODE */
