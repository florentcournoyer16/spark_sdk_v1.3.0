/** @file  swc_api.c
 *  @brief SPARK Wireless Core Application Programming Interface.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include <zephyr/sys/printk.h>

#include "swc_api.h"
#include <stdio.h>
#include "mem_pool.h"
#include "swc_error.h"
#include "swc_utils.h"
#include "wps.h"
#include "wps_frag.h"
#include "wps_stats.h"

/* CONSTANTS ******************************************************************/
/*! Default preamble length. */
#ifndef WPS_DEFAULT_PREAMBLE_LEN
#define WPS_DEFAULT_PREAMBLE_LEN        94
#endif

/*! Default CRC. */
#ifndef WPS_DEFAULT_CRC
#define WPS_DEFAULT_CRC                 0xBAAD
#endif

/*! Default sync word length. */
#ifndef WPS_DEFAULT_SYNC_WORD_LEN
#define WPS_DEFAULT_SYNC_WORD_LEN       SYNCWORD_LENGTH_32
#endif

/*! Default callback queue size. */
#ifndef WPS_DEFAULT_CALLBACK_QUEUE_SIZE
#define WPS_DEFAULT_CALLBACK_QUEUE_SIZE 20
#endif

/*! Default frequency shift setting. */
#ifndef WPS_DEFAULT_FREQ_SHIFT
#define WPS_DEFAULT_FREQ_SHIFT          true
#endif

/*! Default pulse start position. */
#ifndef WPS_DEFAULT_PULSE_START_POS
#define WPS_DEFAULT_PULSE_START_POS     2
#endif

/*! Default randomize pulse phase setting. */
#ifndef WPS_DEFAULT_RND_PHASE
#define WPS_DEFAULT_RND_PHASE           RND_PHASE_ENABLE
#endif

/*! Default pulse spacing. */
#ifndef WPS_DEFAULT_PULSE_SPACING
#define WPS_DEFAULT_PULSE_SPACING       1
#endif

/*! Default random data rate offset rollover value. */
#ifndef WPS_DEFAULT_RDO_ROLLOVER_VAL
#define WPS_DEFAULT_RDO_ROLLOVER_VAL    15
#endif

/*! Default random data rate offset increment interval. */
#ifndef WPS_DEFAULT_RDO_STEP_MS
#define WPS_DEFAULT_RDO_STEP_MS         10
#endif

/*! Default random data rate offset increment step. */
#ifndef WPS_DEFAULT_RDO_STEP_VALUE
#define WPS_DEFAULT_RDO_STEP_VALUE      1
#endif

/*! Default Inter-symbol interference mitigation. */
#ifndef WPS_DEFAULT_ISI_MITIG_LEVEL
#define WPS_DEFAULT_ISI_MITIG_LEVEL     ISI_MITIG_0 /* not used with SR1000 series */
#endif

/*! Default reception gain. */
#ifndef WPS_DEFAULT_RX_GAIN
#define WPS_DEFAULT_RX_GAIN             0x00        /* used only when gain loop is disabled */
#endif

/*! Default transmission jitter. */
#ifndef WPS_DEFAULT_TX_JITTER
#define WPS_DEFAULT_TX_JITTER           false
#endif

/*! Default multi radio average count for radio selection at runtime. */
#ifndef WPS_DEFAULT_MULTI_AVG_COUNT
#define WPS_DEFAULT_MULTI_AVG_COUNT     4
#endif

/*! Default multi radio mode state. */
#ifndef WPS_DEFAULT_MULTI_MODE
#define WPS_DEFAULT_MULTI_MODE          MULTI_RADIO_MODE_0
#endif

/*! Default multi radio RSSI threshold. */
#ifndef WPS_DEFAULT_MULTI_RSSI_THRESH
#define WPS_DEFAULT_MULTI_RSSI_THRESH   25
#endif

/*! Default consecutive successful received packet for connect state. */
#ifndef WPS_DEFAULT_CONNECT_STATUS_COUNT
#define WPS_DEFAULT_CONNECT_STATUS_COUNT    1
#endif

/*! Default consecutive missing received packet for disconnect state. */
#ifndef WPS_DEFAULT_DISCONNECT_STATUS_COUNT
#define WPS_DEFAULT_DISCONNECT_STATUS_COUNT 100
#endif

/*! Default maximal timeslot offset. */
#ifndef WPS_DEFAULT_MAX_TIMESLOT_OFFSET
#define WPS_DEFAULT_MAX_TIMESLOT_OFFSET     48
#endif

/*! Default synchronise frame lost maximal duration. */
#ifndef WPS_DEFAULT_SYNC_FRAME_LOST_MAX_DURATION
#define WPS_DEFAULT_SYNC_FRAME_LOST_MAX_DURATION ((uint32_t)409600) /* 409600 PLL cycles = 20 ms */
#endif

#ifndef WPS_DEFAULT_REQUEST_MEMORY_SIZE
#define WPS_DEFAULT_REQUEST_MEMORY_SIZE     5
#endif

/*! Receiver integrator gain for one pulse. */
#ifndef WPS_INTEGGAIN_ONE_PULSE_VAL
#define WPS_INTEGGAIN_ONE_PULSE_VAL   1
#endif

/*! Receiver integrator gain for multiple pulse. */
#ifndef WPS_INTEGGAIN_MANY_PULSES_VAL
#define WPS_INTEGGAIN_MANY_PULSES_VAL 0
#endif

/*! Sync word bit cost maximal value. */
#ifndef WPS_SYNC_WORD_BIT_COST_MAX_VAL
#define WPS_SYNC_WORD_BIT_COST_MAX_VAL   3
#endif

/*! Sync word tolerance maximal value. */
#ifndef WPS_SYNC_WORD_TOLERANCE_MAX_VAL
#define WPS_SYNC_WORD_TOLERANCE_MAX_VAL  31
#endif

/*! Preamble length maximal value. */
#ifndef WPS_PREAMBLE_LEN_MAX_VAL
#define WPS_PREAMBLE_LEN_MAX_VAL  524
#endif

/*! Minimal pulse count. */
#define PULSE_COUNT_MIN 1
/*! Maximal pulse count. */
#define PULSE_COUNT_MAX 3
/*! Maximal pulse width. */
#define PULSE_WIDTH_MAX 7
/*! Maximal pulse gain. */
#define PULSE_GAIN_MAX  3
/*! Maximal clear channel assessment threshold. */
#define CCA_THRESH_MAX  47
/*! The radio's maximum payload size is 128, one byte must be reserved for the header size. */
#define FRAME_SIZE_MAX  127

/* MACROS *********************************************************************/
#define HW_ADDR(net_id, node_id)        SWC_CONCAT_8B_TO_16B((net_id), (node_id))
#define NET_ID_FROM_PAN_ID(pan_id)      ((pan_id) & 0x0ff)
#define SYNCWORD_ID_FROM_PAN_ID(pan_id) (((pan_id) & 0xf00) >> 8)

#define CHECK_ERROR(cond, err_ptr, err_code, ret) \
    do {                                          \
        if (cond) {                               \
            *(err_ptr) = (err_code);              \
            ret;                                  \
        }                                         \
    } while (0)

/* PUBLIC GLOBALS *************************************************************/
wps_t wps;

/* PRIVATE GLOBALS ************************************************************/
static bool is_started;
static mem_pool_t mem_pool;
static nvm_t saved_nvm[WPS_RADIO_COUNT];
static calib_vars_t saved_calib_vars[WPS_RADIO_COUNT];
/*! This variable is used to lock/unlock reserved address in the SWC. */

static bool reserved_address_lock = true;

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static bool has_main_timeslot(int32_t *timeslot_id, uint32_t timeslot_count);
static bool is_rx_connection(uint8_t local_address, uint8_t source_address);
static uint16_t get_rdo_increment_step(uint32_t *timeslot_sequence, uint32_t timeslot_sequence_length, uint32_t rdo_step_ms);
static bool network_role_supported(swc_role_t role);
static wps_role_t network_role_swc_to_wps(swc_role_t role);
static bool sleep_level_supported(swc_sleep_level_t level, schedule_t *schedule);
static sleep_lvl_t sleep_level_swc_to_wps(swc_sleep_level_t level);
static bool irq_polarity_supported(swc_irq_polarity_t pol);
static irq_polarity_t irq_polarity_swc_to_wps(swc_irq_polarity_t pol);
static bool spi_mode_supported(swc_spi_mode_t mode);
static std_spi_t spi_mode_swc_to_wps(swc_spi_mode_t mode);
static bool modulation_supported(swc_modulation_t modulation);
static modulation_t modulation_swc_to_wps(swc_modulation_t modulation);
static bool fec_level_supported(swc_fec_level_t level);
static fec_level_t fec_level_swc_to_wps(swc_fec_level_t level);
static bool cca_fail_action_supported(swc_cca_fail_action_t action);
static cca_fail_action_t cca_fail_action_swc_to_wps(swc_cca_fail_action_t action);
static void radio_hal_swc_to_wps(radio_hal_t *wps_hal, const swc_radio_hal_t *swc_hal,
                                 swc_error_t *err);
static void save_radio_configuration(uint8_t radio_id, nvm_t *nvm, calib_vars_t *calib_vars);
static void get_saved_radio_configuration(uint8_t radio_id, nvm_t *nvm, calib_vars_t *calib_vars, radio_t *radio);
static void check_main_connection_priority_errors(swc_node_t *node, timeslot_t timeslot,
                                                  swc_error_t *err);
static void check_auto_connection_priority_errors(swc_node_t *node, timeslot_t timeslot,
                                                  swc_error_t *err);
static int format_radio_nvm(wps_radio_t *wps_radio, char *buffer, uint16_t size);

/* PUBLIC FUNCTIONS ***********************************************************/
void swc_init(swc_cfg_t cfg, swc_hal_t *hal, swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR(hal == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR((WPS_RADIO_COUNT == 2) && cfg.fast_sync_enabled, err, SWC_ERR_FAST_SYNC_WITH_DUAL_RADIO, return);
    CHECK_ERROR(cfg.timeslot_sequence == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(cfg.channel_sequence == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(cfg.timeslot_sequence_length == 0, err, SWC_ERR_ZERO_TIMESLOT_SEQ_LEN, return);
    CHECK_ERROR(cfg.channel_sequence_length == 0, err, SWC_ERR_ZERO_CHAN_SEQ_LEN, return);

    uint32_t timeslot_sequence_pll_cycle[cfg.timeslot_sequence_length];
    timeslot_t *timeslots;
    wps_callback_inst_t *callback_queue;
    wps_request_info_t *request;
    wps_request_config_info_t request_config;

    mem_pool_init(&mem_pool, cfg.memory_pool, (size_t)cfg.memory_pool_size);

    /* Allocate memory */
    request_config.schedule_ratio_buffer = mem_pool_malloc(&mem_pool, sizeof(wps_schedule_ratio_cfg_t) * WPS_DEFAULT_REQUEST_MEMORY_SIZE);
    CHECK_ERROR(request_config.schedule_ratio_buffer == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);
    request_config.schedule_ratio_size = WPS_DEFAULT_REQUEST_MEMORY_SIZE;

    request_config.write_request_buffer = mem_pool_malloc(&mem_pool, sizeof(wps_write_request_info_t) * WPS_DEFAULT_REQUEST_MEMORY_SIZE);
    CHECK_ERROR(request_config.write_request_buffer == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);
    request_config.write_request_size = WPS_DEFAULT_REQUEST_MEMORY_SIZE;

    request_config.read_request_buffer = mem_pool_malloc(&mem_pool, sizeof(wps_read_request_info_t) * WPS_DEFAULT_REQUEST_MEMORY_SIZE);
    CHECK_ERROR(request_config.read_request_buffer == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);
    request_config.read_request_size = WPS_DEFAULT_REQUEST_MEMORY_SIZE;

    timeslots = mem_pool_malloc(&mem_pool, sizeof(timeslot_t) * cfg.timeslot_sequence_length);
    CHECK_ERROR(timeslots == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);
    callback_queue = mem_pool_malloc(&mem_pool, sizeof(wps_callback_inst_t) * WPS_DEFAULT_CALLBACK_QUEUE_SIZE);
    CHECK_ERROR(callback_queue == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);
    request = mem_pool_malloc(&mem_pool, sizeof(wps_request_info_t) * WPS_DEFAULT_REQUEST_MEMORY_SIZE);
    CHECK_ERROR(request == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);

    /* Initialize the callback queue which will be used to accumulate and run the WPS callbacks asynchronously */
    wps_init_callback_queue(&wps, callback_queue, WPS_DEFAULT_CALLBACK_QUEUE_SIZE, hal->context_switch);

    /* Initialize the request queue which will be used to accumulate request from the application to the WPS */
    wps_init_request_queue(&wps, request, WPS_DEFAULT_REQUEST_MEMORY_SIZE, &request_config);

#if WPS_RADIO_COUNT == 2
    CHECK_ERROR(hal->timer_start == NULL, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    CHECK_ERROR(hal->timer_stop == NULL, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    CHECK_ERROR(hal->timer_set_period == NULL, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    CHECK_ERROR(hal->timer_set_max_period == NULL, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    CHECK_ERROR(hal->disable_timer_irq == NULL, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    CHECK_ERROR(hal->enable_timer_irq == NULL, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    CHECK_ERROR(hal->timer_frequency_hz == 0, err, SWC_ERR_MULTI_RADIO_HAL_INVALID, return);
    /* Initialize MCU timer functions used for timing when in dual radio configuration */
    wps_multi_cfg_t multi_cfg = {
        .timer_start          = hal->timer_start,
        .timer_stop           = hal->timer_stop,
        .timer_set_period     = hal->timer_set_period,
        .timer_set_max_period = hal->timer_set_max_period,
        .disable_timer_irq    = hal->disable_timer_irq,
        .enable_timer_irq     = hal->enable_timer_irq,
        .timer_frequency_hz   = hal->timer_frequency_hz,
        .avg_sample_count     = WPS_DEFAULT_MULTI_AVG_COUNT,
        .mode                 = WPS_DEFAULT_MULTI_MODE,
        .rssi_threshold       = WPS_DEFAULT_MULTI_RSSI_THRESH
    };
    wps_multi_init(multi_cfg, &wps_err);
#endif

    for (uint32_t i = 0; i < cfg.timeslot_sequence_length; i++) {
        timeslot_sequence_pll_cycle[i] = wps_us_to_pll_cycle(cfg.timeslot_sequence[i], CHIP_RATE_20_48_MHZ);
    }

    wps_config_network_schedule(&wps, timeslot_sequence_pll_cycle, timeslots, cfg.timeslot_sequence_length, &wps_err);
    wps_config_network_channel_sequence(&wps, cfg.channel_sequence, cfg.channel_sequence_length, &wps_err);

    /* Enable/disable global miscellaneous WPS features */
#if (WPS_RADIO_COUNT == 1)
    if (cfg.fast_sync_enabled) {
        wps_enable_fast_sync(&wps, &wps_err);
    } else {
        wps_disable_fast_sync(&wps, &wps_err);
    }
#endif
    if (cfg.random_channel_sequence_enabled) {
        wps_enable_random_channel_sequence(&wps, &wps_err);
    } else {
        wps_disable_random_channel_sequence(&wps, &wps_err);
    }

    if (cfg.certification_mode_enabled) {
        wps_enable_certification_mode(&wps, &wps_err);
    } else {
        wps_disable_certification_mode(&wps, &wps_err);
    }

    if (cfg.rdo_enabled) {
        uint16_t increment_step = get_rdo_increment_step(cfg.timeslot_sequence, cfg.timeslot_sequence_length,
                                                         WPS_DEFAULT_RDO_STEP_MS);
        wps_enable_rdo(&wps, WPS_DEFAULT_RDO_ROLLOVER_VAL, increment_step, &wps_err);
    } else {
        wps_disable_rdo(&wps, &wps_err);
    }

    if (cfg.ddcm_enabled) {
        wps_enable_ddcm(&wps, WPS_DEFAULT_MAX_TIMESLOT_OFFSET, WPS_DEFAULT_SYNC_FRAME_LOST_MAX_DURATION, &wps_err);
    } else {
        wps_disable_ddcm(&wps, &wps_err);
    }

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);
}

swc_node_t *swc_node_init(swc_node_cfg_t cfg, swc_error_t *err)
{
    wps_error_t wps_err;
    wps_node_cfg_t wps_node_cfg;
    swc_node_t *node;

    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return NULL);
    CHECK_ERROR(cfg.local_address == SWC_BROADCAST_ADDRESS, err, SWC_ERR_LOCAL_ADDRESS, return NULL);
    CHECK_ERROR(cfg.pan_id & 0xf000, err, SWC_ERR_PAN_ID, return NULL);
    CHECK_ERROR(((cfg.pan_id == 0x0000) && reserved_address_lock), err, SWC_ERR_PAN_ID, return NULL);
    CHECK_ERROR(!network_role_supported(cfg.role), err, SWC_ERR_NETWORK_ROLE, return NULL);
    CHECK_ERROR(!sleep_level_supported(cfg.sleep_level, &wps.mac.scheduler.schedule), err, SWC_ERR_SLEEP_LEVEL, return NULL);

    /* Allocate memory */
    node = mem_pool_malloc(&mem_pool, sizeof(swc_node_t));
    CHECK_ERROR(node == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
    node->wps_node_handle = mem_pool_malloc(&mem_pool, sizeof(wps_node_t));
    CHECK_ERROR(node->wps_node_handle == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
    node->wps_radio_handle = mem_pool_malloc(&mem_pool, sizeof(wps_radio_t) * WPS_RADIO_COUNT);
    CHECK_ERROR(node->wps_radio_handle == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    node->radio_count = 0;
    node->cfg         = cfg;

    wps_node_cfg.role                         = network_role_swc_to_wps(cfg.role);
    wps_node_cfg.preamble_len                 = WPS_DEFAULT_PREAMBLE_LEN;
    wps_node_cfg.sleep_lvl                    = sleep_level_swc_to_wps(cfg.sleep_level);
    wps_node_cfg.crc_polynomial               = WPS_DEFAULT_CRC;
    wps_node_cfg.local_address                = HW_ADDR(NET_ID_FROM_PAN_ID(cfg.pan_id), cfg.local_address);
    wps_node_cfg.syncword_cfg.syncword        = sync_word_table[SYNCWORD_ID_FROM_PAN_ID(cfg.pan_id)];
    wps_node_cfg.syncword_cfg.syncword_length = WPS_DEFAULT_SYNC_WORD_LEN;
    wps_node_cfg.isi_mitig                    = WPS_DEFAULT_ISI_MITIG_LEVEL;
    wps_node_cfg.rx_gain                      = WPS_DEFAULT_RX_GAIN;
    wps_node_cfg.tx_jitter_enabled            = WPS_DEFAULT_TX_JITTER;
    wps_node_cfg.chip_rate                    = CHIP_RATE_20_48_MHZ;
    wps_node_cfg.frame_lost_max_duration      = WPS_DEFAULT_SYNC_FRAME_LOST_MAX_DURATION;
    
    wps_set_network_id(&wps, NET_ID_FROM_PAN_ID(cfg.pan_id), &wps_err);
    wps_set_syncing_address(&wps, HW_ADDR(NET_ID_FROM_PAN_ID(cfg.pan_id), cfg.coordinator_address), &wps_err);
    wps_config_node(node->wps_node_handle, node->wps_radio_handle, &wps_node_cfg, &wps_err);

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return NULL);

    return node;
}

void swc_node_add_radio(swc_node_t *node, swc_radio_cfg_t cfg, swc_hal_t *hal, swc_error_t *err)
{
    wps_error_t wps_err;
    uint8_t radio_id;

    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR((node == NULL) || (hal == NULL), err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(!irq_polarity_supported(cfg.irq_polarity), err, SWC_ERR_IRQ_POLARITY, return);
    CHECK_ERROR(!spi_mode_supported(cfg.spi_mode), err, SWC_ERR_SPI_MODE, return);

    radio_id = node->radio_count;
    radio_hal_swc_to_wps(&node->wps_radio_handle[radio_id].radio.radio_hal,
                         &hal->radio_hal[radio_id], err);
    CHECK_ERROR(*err != SWC_ERR_NONE, err, *err, return);

    node->wps_radio_handle[radio_id].radio.irq_polarity                 = irq_polarity_swc_to_wps(cfg.irq_polarity);
    node->wps_radio_handle[radio_id].radio.std_spi                      = spi_mode_swc_to_wps(cfg.spi_mode);
    node->wps_radio_handle[radio_id].radio.clock_source.pll_dis         = PLL_ENABLE;
    node->wps_radio_handle[radio_id].radio.clock_source.pll_clk_source  = SYMBSRC_INTERNAL;
    node->wps_radio_handle[radio_id].radio.clock_source.xtal_clk_source = XTALSRC_INTERNAL;

    /* Allocate memory */
    node->wps_radio_handle[radio_id].nvm = mem_pool_malloc(&mem_pool, sizeof(nvm_t));
    CHECK_ERROR(node->wps_radio_handle[radio_id].nvm == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);
    node->wps_radio_handle[radio_id].spectral_calib_vars = mem_pool_malloc(&mem_pool, sizeof(calib_vars_t));
    CHECK_ERROR(node->wps_radio_handle[radio_id].spectral_calib_vars == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);

    /* Disable MCU external interrupt servicing the radio IRQ before initializing the WPS.
     * It will be later re-activated with a call to the swc_connect() function.
     */
    hal->radio_hal[radio_id].disable_radio_irq();

    wps_radio_init(&node->wps_radio_handle[radio_id], cfg.no_reset, &wps_err);

    if (cfg.no_calibration) {
        get_saved_radio_configuration(radio_id,
                                      node->wps_radio_handle[radio_id].nvm,
                                      node->wps_radio_handle[radio_id].spectral_calib_vars,
                                      &node->wps_radio_handle[radio_id].radio);
    } else {
        wps_radio_calibration(&node->wps_radio_handle[radio_id]);
    }
    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);

    node->radio_count++;

    save_radio_configuration(radio_id, node->wps_radio_handle[radio_id].nvm, node->wps_radio_handle[radio_id].spectral_calib_vars);
}

uint64_t swc_node_get_radio_serial_number(swc_node_t *node, swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(node == NULL, err, SWC_ERR_NULL_PTR, return 0);
    CHECK_ERROR(node->wps_radio_handle == NULL, err, SWC_ERR_NULL_PTR, return 0);

    return wps_radio_get_serial_number(&node->wps_radio_handle[0]);
}

int swc_format_radio_nvm(swc_node_t *node, char *buffer, uint16_t size)
{
    return format_radio_nvm(&node->wps_radio_handle[0], buffer, size);
}

#if (WPS_RADIO_COUNT == 2)
uint64_t swc_node_get_radio2_serial_number(swc_node_t *node, swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(node == NULL, err, SWC_ERR_NULL_PTR, return 0);
    CHECK_ERROR(node->wps_radio_handle == NULL, err, SWC_ERR_NULL_PTR, return 0);
    CHECK_ERROR(node->radio_count != 2, err, SWC_ERR_SECOND_RADIO_NOT_INIT, return 0);

    return wps_radio_get_serial_number(&node->wps_radio_handle[1]);
}

int swc_format_radio2_nvm(swc_node_t *node, char *buffer, uint16_t size)
{
    return format_radio_nvm(&node->wps_radio_handle[1], buffer, size);
}
#endif

swc_connection_t *swc_connection_init(swc_node_t *node, swc_connection_cfg_t cfg, swc_hal_t *hal, swc_error_t *err)
{
    wps_error_t wps_err;
    wps_connection_cfg_t wps_conn_cfg;
    wps_header_cfg_t wps_header_cfg = {0};
    uint16_t *frag_tx_meta_buffer;
    uint8_t header_size;
    uint8_t conn_frame_length;
    uint8_t cca_try_count_max_value = 0;
    uint8_t threshold_count         = 0;
    uint8_t *fallback_threshold     = NULL;
    uint8_t *fallback_cca_try_count = NULL;
    bool is_rx_conn;
    bool has_main_ts;
    swc_connection_t *conn;
    phase_infos_t *phase_infos_buffer;
    rf_channel_t(*channel_buffer)[WPS_NB_RF_CHANNEL][WPS_RADIO_COUNT];

    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return NULL);
    CHECK_ERROR((node == NULL) || (hal == NULL), err, SWC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR((cfg.timeslot_id == NULL), err, SWC_ERR_NULL_PTR, return NULL);
    CHECK_ERROR((cfg.timeslot_count == 0), err, SWC_ERR_ZERO_TIMESLOT_COUNT, return NULL);
    CHECK_ERROR((cfg.queue_size < 2), err, SWC_ERR_MIN_QUEUE_SIZE, return NULL);

    is_rx_conn  = is_rx_connection(node->cfg.local_address, cfg.source_address);
    has_main_ts = has_main_timeslot(cfg.timeslot_id, cfg.timeslot_count);

    CHECK_ERROR(cfg.source_address == SWC_BROADCAST_ADDRESS, err, SWC_ERR_SOURCE_ADDRESS, return NULL);
    CHECK_ERROR(is_rx_conn && (cfg.destination_address == SWC_BROADCAST_ADDRESS), err, SWC_ERR_DESTINATION_ADDRESS, return NULL);

    /* Certain settings need not to be initialized when a connection uses only auto-reply timeslots,
     * so skip some error checks if that is the case.
     */
    if (has_main_ts) {
        CHECK_ERROR(cfg.arq_enabled && !cfg.ack_enabled, err, SWC_ERR_ARQ_WITH_ACK_DISABLED, return NULL);
        CHECK_ERROR(!modulation_supported(cfg.modulation), err, SWC_ERR_MODULATION, return NULL);
        CHECK_ERROR(!fec_level_supported(cfg.fec), err, SWC_ERR_FEC_LEVEL, return NULL);
        CHECK_ERROR(cfg.cca_enabled && !cca_fail_action_supported(cfg.cca_settings.fail_action), err,
                    SWC_ERR_CCA_FAIL_ACTION, return NULL);
        CHECK_ERROR(cfg.cca_enabled && cfg.cca_settings.threshold > CCA_THRESH_MAX, err, SWC_ERR_CCA_THRESHOLD,
                    return NULL);
    }

    wps_header_cfg.main_connection = has_main_ts;
    wps_header_cfg.rdo_enabled     = cfg.rdo_enabled;

    if (cfg.ranging_enabled) {
        if (!has_main_ts) {
            wps_header_cfg.ranging_responder = true;
            wps_header_cfg.ranging_initiator = false;
        } else {
            wps_header_cfg.ranging_initiator = true;
            wps_header_cfg.ranging_responder = false;
        }
    }

    wps_header_cfg.connection_id             = cfg.connection_id_enabled;

    header_size = wps_get_connection_header_size(&wps, wps_header_cfg);
    CHECK_ERROR(((cfg.max_payload_size + header_size + WPS_PAYLOAD_SIZE_BYTE_SIZE) > FRAME_SIZE_MAX),
                err, SWC_ERR_PAYLOAD_TOO_BIG, return NULL);
    conn_frame_length = cfg.max_payload_size + header_size + WPS_PAYLOAD_SIZE_BYTE_SIZE;

    /* Allocate memory */
    conn = mem_pool_malloc(&mem_pool, sizeof(swc_connection_t));
    CHECK_ERROR(conn == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
    conn->wps_conn_handle = mem_pool_malloc(&mem_pool, sizeof(wps_connection_t));
    CHECK_ERROR(conn->wps_conn_handle == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    if (cfg.fallback_enabled) {
        threshold_count = 1; /* Only one level of fallback is supported */

        fallback_threshold = mem_pool_malloc(&mem_pool, sizeof(uint8_t) * threshold_count);
        CHECK_ERROR(fallback_threshold == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
        memcpy(fallback_threshold, &cfg.fallback_settings.threshold, threshold_count);
    }
    if (cfg.cca_enabled) {
        cca_try_count_max_value = cfg.cca_settings.try_count;

        /* Fallback mode has its own CCA try count */
        if (cfg.fallback_enabled) {
            fallback_cca_try_count = mem_pool_malloc(&mem_pool, sizeof(uint8_t) * (1 + threshold_count));
            CHECK_ERROR(fallback_cca_try_count == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
            fallback_cca_try_count[0] = cfg.cca_settings.try_count;          /* CCA try count when in normal mode */
            fallback_cca_try_count[1] = cfg.fallback_settings.cca_try_count; /* CCA try count when in fallback mode */
            cca_try_count_max_value   = cfg.cca_settings.try_count > cfg.fallback_settings.cca_try_count
                                            ? cfg.cca_settings.try_count
                                            : cfg.fallback_settings.cca_try_count;
        }
    }
    if (cfg.throttling_enabled) {
        conn->wps_conn_handle->pattern = mem_pool_malloc(&mem_pool, sizeof(bool) * WPS_PATTERN_THROTTLE_GRANULARITY);
        CHECK_ERROR(conn->wps_conn_handle->pattern == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
    }
    channel_buffer = mem_pool_malloc(&mem_pool,
                                     sizeof(rf_channel_t[WPS_NB_RF_CHANNEL][WPS_RADIO_COUNT]) * (threshold_count + 1));
    CHECK_ERROR(channel_buffer == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);

    conn->channel_count = 0;
    conn->cfg           = cfg;

    wps_conn_cfg.source_address      = HW_ADDR(NET_ID_FROM_PAN_ID(node->cfg.pan_id), cfg.source_address);
    wps_conn_cfg.destination_address = HW_ADDR(NET_ID_FROM_PAN_ID(node->cfg.pan_id), cfg.destination_address);
    wps_conn_cfg.header_length       = header_size;
    wps_conn_cfg.frame_length        = conn_frame_length;
    wps_conn_cfg.get_tick            = hal->radio_hal->get_tick;
    wps_conn_cfg.fifo_buffer_size    = cfg.queue_size;
    wps_conn_cfg.fallback_count      = threshold_count;
    wps_conn_cfg.fallback_threshold  = fallback_threshold;
    wps_conn_cfg.channel_buffer      = channel_buffer;
    wps_conn_cfg.priority            = cfg.priority;
    CHECK_ERROR((wps_conn_cfg.priority > 0) && (wps_header_cfg.connection_id == false), err, SWC_ERR_NON_ZERO_PRIORITY_WITHOUT_CONN_ID, return NULL);
    CHECK_ERROR((wps_conn_cfg.priority > WPS_MAX_CONN_PRIORITY), err, SWC_ERR_MAX_CONN_PRIORITY, return NULL);

    wps_create_connection(conn->wps_conn_handle, node->wps_node_handle, &wps_conn_cfg, &wps_err);

    wps_connection_config_frame(conn->wps_conn_handle, modulation_swc_to_wps(cfg.modulation),
                                fec_level_swc_to_wps(cfg.fec), &wps_err);

    wps_connection_set_timeslot(conn->wps_conn_handle, &wps, cfg.timeslot_id, cfg.timeslot_count, &wps_err);
    CHECK_ERROR(wps_err == WPS_TIMESLOT_CONN_LIMIT_REACHED_ERROR, err, SWC_ERR_TIMESLOT_CONN_LIMIT_REACHED, return NULL);

    connect_status_cfg_t status_cfg = {
        .connect_count    = WPS_DEFAULT_CONNECT_STATUS_COUNT,
        .disconnect_count = WPS_DEFAULT_DISCONNECT_STATUS_COUNT,
    };

    wps_connection_config_status(conn->wps_conn_handle, &status_cfg, &wps_err);

    if (cfg.ack_enabled) {
        wps_connection_enable_ack(conn->wps_conn_handle, &wps_err);
    } else {
        wps_connection_disable_ack(conn->wps_conn_handle, &wps_err);
    }

    if (cfg.arq_enabled) {
        wps_connection_enable_stop_and_wait_arq(conn->wps_conn_handle, node->wps_node_handle->cfg.local_address,
                                                cfg.arq_settings.try_count, cfg.arq_settings.time_deadline, &wps_err);
    } else {
        wps_connection_disable_stop_and_wait_arq(conn->wps_conn_handle, &wps_err);
    }

    if (cfg.auto_sync_enabled) {
        wps_connection_enable_auto_sync(conn->wps_conn_handle, &wps_err);
    } else {
        wps_connection_disable_auto_sync(conn->wps_conn_handle, &wps_err);
    }

    if (cfg.cca_enabled) {
        wps_connection_enable_cca(conn->wps_conn_handle, conn->cfg.cca_settings.threshold,
                                  conn->cfg.cca_settings.retry_time, cca_try_count_max_value, fallback_cca_try_count,
                                  cca_fail_action_swc_to_wps(conn->cfg.cca_settings.fail_action), &wps_err);
    } else {
        wps_connection_disable_cca(conn->wps_conn_handle, &wps_err);
    }

    if (cfg.throttling_enabled) {
        wps_init_connection_throttle(conn->wps_conn_handle, &wps_err);
    }

    wps_configure_header_connection(&wps, conn->wps_conn_handle, wps_header_cfg, &wps_err);

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return NULL);

    if (cfg.fragmentation_enabled) {
        frag_tx_meta_buffer = mem_pool_malloc(&mem_pool, sizeof(uint16_t) * cfg.queue_size);
        CHECK_ERROR(frag_tx_meta_buffer == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
        wps_frag_init(conn->wps_conn_handle, (void *)frag_tx_meta_buffer, cfg.queue_size);
    } else {
        conn->wps_conn_handle->frag.enabled = false;
    }

    if (cfg.latency_opt_enabled) {
        wps_connection_optimize_latency(conn->wps_conn_handle, cfg.rx_ack_payload_size, node->wps_node_handle, true, false, &wps_err);
    }

    for (uint32_t i = 0; i < cfg.timeslot_count; ++i) {
        uint32_t id = cfg.timeslot_id[i];

        if ((id & BIT_AUTO_REPLY_TIMESLOT)) {
            id = id & TIMESLOT_VALUE_MASK;
            check_auto_connection_priority_errors(node, wps.mac.scheduler.schedule.timeslot[id], err);
        } else {
            check_main_connection_priority_errors(node, wps.mac.scheduler.schedule.timeslot[id], err);
        }
    }

    if (cfg.ranging_enabled && is_rx_conn && !has_main_ts) {
        phase_infos_buffer = mem_pool_malloc(&mem_pool, sizeof(phase_infos_t) * cfg.ranging_settings.sample_count);
        CHECK_ERROR(phase_infos_buffer == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return NULL);
        wps_connection_enable_phases_aquisition(conn->wps_conn_handle, phase_infos_buffer, cfg.ranging_settings.sample_count, &wps_err);
    }
    return conn;
}

swc_connection_cfg_t swc_get_beacon_connection_config(swc_node_t *node, uint8_t source_address, int32_t *timeslot_id, uint8_t timeslot_count)
{
    uint8_t destination_address;

    if (is_rx_connection(node->cfg.local_address, source_address)) {
        destination_address = (uint8_t)node->wps_node_handle->cfg.local_address;
    } else {
        destination_address = SWC_BROADCAST_ADDRESS;
    }

    swc_connection_cfg_t beacon_conn_cfg = {
        .name = "Beacon Connection",
        .source_address = source_address,
        .destination_address = destination_address,
        .max_payload_size = 0,
        .queue_size = 2,
        .modulation = SWC_MOD_2BITPPM,
        .fec = SWC_FEC_2,
        .timeslot_id = timeslot_id,
        .timeslot_count = timeslot_count,
        .auto_sync_enabled = true,
    };
    return beacon_conn_cfg;
}

void swc_connection_add_channel(swc_connection_t *conn, swc_node_t *node, swc_channel_cfg_t cfg, swc_error_t *err)
{
    wps_error_t wps_err;
    channel_cfg_t wps_chann_cfg;
    bool is_rx_conn;
    bool is_tx_conn;
    bool has_main_ts;

    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR((conn == NULL) || (node == NULL), err, SWC_ERR_NULL_PTR, return);

    is_rx_conn  = is_rx_connection(node->cfg.local_address, conn->cfg.source_address);
    is_tx_conn  = !is_rx_conn;
    has_main_ts = has_main_timeslot(conn->cfg.timeslot_id, conn->cfg.timeslot_count);

    CHECK_ERROR(has_main_ts == false, err, SWC_ERR_ADD_CHANNEL_ON_INVALID_CONNECTION, return);

    if (is_tx_conn || (is_rx_conn && conn->cfg.ack_enabled)) {
        CHECK_ERROR((cfg.tx_pulse_count < PULSE_COUNT_MIN) || (cfg.tx_pulse_count > PULSE_COUNT_MAX), err,
                    SWC_ERR_TX_PULSE_COUNT, return);
        CHECK_ERROR(cfg.tx_pulse_width > PULSE_WIDTH_MAX, err, SWC_ERR_TX_PULSE_WIDTH, return);
        CHECK_ERROR(cfg.tx_pulse_gain > PULSE_GAIN_MAX, err, SWC_ERR_TX_PULSE_GAIN, return);
    }

    if ((is_tx_conn && conn->cfg.ack_enabled) || is_rx_conn) {
        CHECK_ERROR((cfg.rx_pulse_count < PULSE_COUNT_MIN) || (cfg.rx_pulse_count > PULSE_COUNT_MAX), err,
                    SWC_ERR_RX_PULSE_COUNT, return);
    }

    wps_chann_cfg.power = mem_pool_malloc(&mem_pool, sizeof(tx_power_settings_t));
    CHECK_ERROR(wps_chann_cfg.power == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);

    /* Configure RF channels the connection will use */
    wps_chann_cfg.frequency          = cfg.frequency;
    wps_chann_cfg.power->pulse_count = cfg.tx_pulse_count;
    wps_chann_cfg.power->pulse_width = cfg.tx_pulse_width;
    wps_chann_cfg.power->tx_gain     = cfg.tx_pulse_gain;
    wps_chann_cfg.freq_shift_enable  = WPS_DEFAULT_FREQ_SHIFT;
    wps_chann_cfg.pulse_spacing      = WPS_DEFAULT_PULSE_SPACING;
    wps_chann_cfg.pulse_start_pos    = WPS_DEFAULT_PULSE_START_POS;
    wps_chann_cfg.rdn_phase_enable   = WPS_DEFAULT_RND_PHASE;
    wps_chann_cfg.integgain = (cfg.rx_pulse_count == 1) ? WPS_INTEGGAIN_ONE_PULSE_VAL : WPS_INTEGGAIN_MANY_PULSES_VAL;

    wps_connection_config_channel(conn->wps_conn_handle, node->wps_node_handle, conn->channel_count, 0, &wps_chann_cfg,
                                  &wps_err);

    if (is_tx_conn && conn->cfg.fallback_enabled) {
        wps_chann_cfg.power->pulse_count += conn->cfg.fallback_settings.tx_pulse_count_offset;
        wps_chann_cfg.power->pulse_width += conn->cfg.fallback_settings.tx_pulse_width_offset;
        wps_chann_cfg.power->tx_gain += conn->cfg.fallback_settings.tx_pulse_gain_offset;

        CHECK_ERROR((wps_chann_cfg.power->pulse_count < PULSE_COUNT_MIN) ||
                        (wps_chann_cfg.power->pulse_count > PULSE_COUNT_MAX),
                    err, SWC_ERR_TX_PULSE_COUNT_OFFSET, return);
        CHECK_ERROR(wps_chann_cfg.power->pulse_width > PULSE_WIDTH_MAX, err, SWC_ERR_TX_PULSE_WIDTH_OFFSET, return);
        CHECK_ERROR(wps_chann_cfg.power->tx_gain > PULSE_GAIN_MAX, err, SWC_ERR_TX_GAIN_OFFSET, return);

        wps_connection_config_channel(conn->wps_conn_handle, node->wps_node_handle, conn->channel_count, 1,
                                      &wps_chann_cfg, &wps_err);
    }

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);

    conn->channel_count++;
}

void swc_connection_set_tx_success_callback(swc_connection_t *conn, void (*cb)(void *conn), swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);

    if (conn->cfg.fragmentation_enabled) {
        wps_frag_set_tx_success_callback(conn->wps_conn_handle, cb, conn);
    } else {
        wps_set_tx_success_callback(conn->wps_conn_handle, cb, conn);
    }
}

void swc_connection_set_tx_fail_callback(swc_connection_t *conn, void (*cb)(void *conn), swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);

    wps_set_tx_fail_callback(conn->wps_conn_handle, cb, conn);
}

void swc_connection_set_tx_dropped_callback(swc_connection_t *conn, void (*cb)(void *conn), swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);

    wps_set_tx_drop_callback(conn->wps_conn_handle, cb, conn);
}

void swc_connection_set_rx_success_callback(swc_connection_t *conn, void (*cb)(void *conn), swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);
    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);

    if (conn->cfg.fragmentation_enabled) {
        wps_frag_set_rx_success_callback(conn->wps_conn_handle, cb, conn);
    } else {
        wps_set_rx_success_callback(conn->wps_conn_handle, cb, conn);
    }
}

void swc_connection_set_event_callback(swc_connection_t *conn, void (*cb)(void *conn), swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);
    if (conn->cfg.fragmentation_enabled) {
        wps_frag_set_event_callback(conn->wps_conn_handle, cb, conn);
    } else {
        wps_set_event_callback(conn->wps_conn_handle, cb, conn);
    }
}

void swc_connection_set_ranging_data_ready_callback(swc_connection_t *conn, void (*cb)(void *conn), swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);

    wps_set_ranging_data_ready_callback(conn->wps_conn_handle, cb, conn);
}

void swc_connection_set_throttling_active_ratio(swc_connection_t *conn, uint8_t active_ratio, swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(conn->cfg.throttling_enabled == false, err, SWC_ERR_THROTTLING_NOT_SUPPORTED, return);

    wps_set_active_ratio(&wps, conn->wps_conn_handle, active_ratio, &wps_err);

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);
}

void swc_connection_get_payload_buffer(swc_connection_t *conn, uint8_t **payload_buffer, swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    CHECK_ERROR((conn == NULL) || (payload_buffer == NULL), err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR((conn->cfg.fragmentation_enabled), err, SWC_ERR_FRAGMENTATION_NOT_SUPPORTED, return);

    wps_get_free_slot(conn->wps_conn_handle, payload_buffer, &wps_err);
    if (wps_err != WPS_NO_ERROR) {
        *err            = SWC_ERR_NO_BUFFER_AVAILABLE;
        *payload_buffer = NULL;
        return;
    }
}

void swc_connection_send(swc_connection_t *conn, uint8_t *payload_buffer, uint16_t size, swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    CHECK_ERROR((conn == NULL) || (payload_buffer == NULL), err, SWC_ERR_NULL_PTR, return);
    if (is_rx_connection(wps.node->cfg.local_address, conn->cfg.source_address)) {
        *err = SWC_ERR_SEND_ON_RX_CONN;
        return;
    }
    if (!conn->cfg.fragmentation_enabled) {
        wps_send(conn->wps_conn_handle, payload_buffer, size, &wps_err);
    } else {
        wps_frag_send(conn->wps_conn_handle, payload_buffer, size, &wps_err);
    }
    if (wps_err == WPS_WRONG_TX_SIZE_ERROR) {
        *err = SWC_ERR_SIZE_TOO_BIG;
        return;
    } else if (wps_err == WPS_QUEUE_FULL_ERROR) {
        *err = SWC_ERR_SEND_QUEUE_FULL;
        return;
    }
}

uint16_t swc_connection_receive(swc_connection_t *conn, uint8_t **payload, swc_error_t *err)
{
    wps_error_t wps_err;
    wps_rx_frame frame;

    *err = SWC_ERR_NONE;

    CHECK_ERROR((conn == NULL) || (payload == NULL), err, SWC_ERR_NULL_PTR, return 0);
    CHECK_ERROR((conn->cfg.fragmentation_enabled), err, SWC_ERR_FRAGMENTATION_NOT_SUPPORTED, return 0);

    frame = wps_read(conn->wps_conn_handle, &wps_err);
    if (wps_err != WPS_NO_ERROR) {
        *err     = SWC_ERR_RECEIVE_QUEUE_EMPTY;
        *payload = NULL;
        return 0;
    }
    *payload = frame.payload;

    return frame.size;
}

void swc_connection_receive_complete(swc_connection_t *conn, swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    CHECK_ERROR(conn == NULL, err, SWC_ERR_NULL_PTR, return);

    wps_read_done(conn->wps_conn_handle, &wps_err);

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_RECEIVE_QUEUE_EMPTY, return);
}

void swc_reserved_address_unlock(void)
{
    reserved_address_lock = false;
}

void swc_reserved_address_lock(void)
{
    reserved_address_lock = true;
}

uint16_t swc_get_ranging_data(swc_connection_t *conn, void **ranging_data, swc_error_t *err)
{
    wps_error_t wps_err;
    wps_phase_frame phase_frame;

    *err = SWC_ERR_NONE;

    phase_frame = wps_read_phase(conn->wps_conn_handle, &wps_err);
    CHECK_ERROR((phase_frame.payload == NULL) || (wps_err == WPS_QUEUE_EMPTY_ERROR), err, SWC_ERR_NULL_PTR, return 0);

    *ranging_data = phase_frame.payload;
    return phase_frame.size;
}

void swc_ranging_data_complete(swc_connection_t *conn, swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    wps_read_phase_done(conn->wps_conn_handle, &wps_err);
    CHECK_ERROR(wps_err == WPS_QUEUE_EMPTY_ERROR, err, SWC_ERR_NULL_PTR, return);
}

uint16_t swc_connection_receive_to_buffer(swc_connection_t *conn, uint8_t *payload, uint16_t size, swc_error_t *err)
{
    wps_error_t wps_err;
    wps_rx_frame frame;

    *err = SWC_ERR_NONE;

    CHECK_ERROR((conn == NULL) || (payload == NULL), err, SWC_ERR_NULL_PTR, return 0);

    if (conn->cfg.fragmentation_enabled) {
        frame = wps_frag_read(conn->wps_conn_handle, payload, size, &wps_err);
    } else {
        frame = wps_read_to_buffer(conn->wps_conn_handle, payload, size, &wps_err);
    }
    if (wps_err != WPS_NO_ERROR) {
        *err = SWC_ERR_RECEIVE_QUEUE_EMPTY;
        return 0;
    }
    return frame.size;
}

uint16_t swc_connection_get_enqueued_count(swc_connection_t *conn, swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    if (conn->cfg.fragmentation_enabled) {
        return wps_frag_get_fifo_size(conn->wps_conn_handle);
    } else {
        return wps_get_fifo_size(conn->wps_conn_handle);
    }
}

bool swc_connection_get_connect_status(swc_connection_t *conn, swc_error_t *err)
{
    *err = SWC_ERR_NONE;

    return wps_get_connect_status(conn->wps_conn_handle);
}

void swc_setup(swc_node_t *node, swc_error_t *err)
{
    wps_error_t wps_err;
    uint8_t *xlayer_pool;

    CHECK_ERROR(is_started == true, err, SWC_ERR_CHANGING_CONFIG_WHILE_RUNNING, return);

    *err = SWC_ERR_NONE;

    CHECK_ERROR(node == NULL, err, SWC_ERR_NULL_PTR, return);

#if (WPS_RADIO_COUNT == 2)
    CHECK_ERROR(node->radio_count != 2, err, SWC_ERR_SECOND_RADIO_NOT_INIT, return);
#endif

    xlayer_pool = (uint8_t *)mem_pool_malloc(&mem_pool, wps_get_xlayer_queue_nb_bytes_needed(node->wps_node_handle, &wps_err));
    CHECK_ERROR(xlayer_pool == NULL, err, SWC_ERR_NOT_ENOUGH_MEMORY, return);

    wps_init_xlayer(node->wps_node_handle, xlayer_pool, &wps_err);

    wps_init(&wps, node->wps_node_handle, &wps_err);

    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);
}

swc_status_t swc_get_status(void)
{
    if (is_started) {
        return SWC_STATUS_RUNNING;
    } else {
        return SWC_STATUS_STOPPED;
    }
}

void swc_connect(swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    wps_connect(&wps, &wps_err);

    CHECK_ERROR(wps_err == WPS_NOT_INIT_ERROR, err, SWC_ERR_NOT_INITIALIZED, return);
    CHECK_ERROR(wps_err == WPS_ALREADY_CONNECTED_ERROR, err, SWC_ERR_ALREADY_CONNECTED, return);
    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);
    is_started = true;
}

void swc_disconnect(swc_error_t *err)
{
    wps_error_t wps_err;

    *err = SWC_ERR_NONE;

    wps_disconnect(&wps, &wps_err);

    CHECK_ERROR(wps_err == WPS_ALREADY_DISCONNECTED_ERROR, err, SWC_ERR_NOT_CONNECTED, return);
    CHECK_ERROR(wps_err == WPS_DISCONNECT_TIMEOUT_ERROR, err, SWC_ERR_DISCONNECT_TIMEOUT, return);
    CHECK_ERROR(wps_err == WPS_NOT_INIT_ERROR, err, SWC_ERR_NOT_INITIALIZED, return);
    CHECK_ERROR(wps_err != WPS_NO_ERROR, err, SWC_ERR_INTERNAL, return);
    is_started = false;
}

swc_event_t swc_get_event(swc_connection_t *conn)
{
    switch (wps_get_event(conn->wps_conn_handle)) {
    case WPS_EVENT_CONNECT:
        return SWC_EVENT_CONNECT;
    case WPS_EVENT_DISCONNECT:
        return SWC_EVENT_DISCONNECT;
    case WPS_EVENT_ERROR:
        return SWC_EVENT_ERROR;
    default:
        return SWC_EVENT_NONE;
    }
}

swc_error_t swc_get_event_error(swc_connection_t *conn)
{
    switch (wps_get_error(conn->wps_conn_handle)) {
    case WPS_RX_OVERRUN_ERROR:
        return SWC_ERR_RX_OVERRUN;
    default:
        return SWC_ERR_INTERNAL;
    }
}

swc_fallback_info_t swc_connection_get_fallback_info(swc_connection_t *conn, swc_error_t *err)
{
    swc_fallback_info_t info = {0};

    *err = SWC_ERR_NONE;

    if (conn == NULL) {
        *err = SWC_ERR_NULL_PTR;
        return info;
    }

    info.link_margin       = wps_stats_get_inst_phy_margin_fast(conn->wps_conn_handle);
    info.cca_fail_count    = wps_get_phy_total_cca_fail_count(conn->wps_conn_handle);
    info.cca_tx_fail_count = wps_get_phy_total_cca_tx_fail_count(conn->wps_conn_handle);
    info.tx_pkt_dropped    = wps_get_phy_total_pkt_dropped(conn->wps_conn_handle);

    return info;
}

uint32_t swc_get_allocated_bytes(void)
{
    return mem_pool_get_allocated_bytes(&mem_pool);
}

void swc_free_memory(void)
{
    is_started = false;
    mem_pool_free(&mem_pool);
}

void swc_connection_callbacks_processing_handler(void)
{
    wps_process_callback(&wps);
}

void swc_send_tx_flush_request(swc_connection_t *conn)
{
    conn->wps_conn_handle->tx_flush = true;
}

#if (WPS_RADIO_COUNT == 1)
void swc_radio_irq_handler(void)
{
    wps_radio_irq(&wps);
}

void swc_radio_spi_receive_complete_handler(void)
{
    wps_transfer_complete(&wps);
}
#elif (WPS_RADIO_COUNT == 2)
void swc_radio1_irq_handler(void)
{
    wps_set_irq_index(0);
    wps_radio_irq(&wps);
}

void swc_radio1_spi_receive_complete_handler(void)
{
    wps_set_irq_index(0);
    wps_transfer_complete(&wps);
}

void swc_radio2_irq_handler(void)
{
    wps_set_irq_index(1);
    wps_radio_irq(&wps);
}

void swc_radio2_spi_receive_complete_handler(void)
{
    wps_set_irq_index(1);
    wps_transfer_complete(&wps);
}

void swc_radio_synchronization_timer_callback(void)
{
    wps_multi_radio_timer_process(&wps);
}
#else
#error "Number of radios must be either 1 or 2"
#endif

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Check if a connection is using at least on main timeslot.
 *
 *  @param[in] timeslot_id     ID of timeslots used by the connection.
 *  @param[in] timeslot_count  Number of timeslots used by the connection.
 *  @retval true  Connection is using at least one main timeslot.
 *  @retval false Connection is using only auto-reply timeslots.
 */
static bool has_main_timeslot(int32_t *timeslot_id, uint32_t timeslot_count)
{
    bool main_timeslot = false;

    for (uint32_t i = 0; i < timeslot_count; i++) {
        main_timeslot = !(timeslot_id[i] & BIT_AUTO_REPLY_TIMESLOT);
        if (main_timeslot) {
            break;
        }
    }

    return main_timeslot;
}

/** @brief Calculate the increment step of the RDO from an increment time in miliseconds
 *
 *  @param[in] timeslot_sequence        Network schedule as an array of timeslot durations in microseconds.
 *  @param[in] timeslot_sequence_length Number of timeslots in the timeslot sequence.
 *  @param[in] rdo_step_ms              Time between RDO increment in ms.
 *  @return    RDO increment step.
 */
static uint16_t get_rdo_increment_step(uint32_t *timeslot_sequence, uint32_t timeslot_sequence_length,
                                       uint32_t rdo_step_ms)
{
    uint32_t average = 0;

    for (size_t i = 0; i < timeslot_sequence_length; i++) {
        average += timeslot_sequence[i];
    }
    average = average / timeslot_sequence_length;

    return rdo_step_ms * 1000 / average;
}

/** @brief Check if the connection is an RX one.
 *
 *  @param[in] local_address   Node's local address.
 *  @param[in] source_address  Connection's source address.
 *  @retval true  This is an RX connection.
 *  @retval false This is a TX connection.
 */
static bool is_rx_connection(uint8_t local_address, uint8_t source_address)
{
    return (local_address != source_address);
}

/** @brief Check if the network role is supported.
 *
 *  @param[in] role  Node's network role.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool network_role_supported(swc_role_t role)
{
    switch (role) {
    case SWC_ROLE_COORDINATOR:
    case SWC_ROLE_NODE:
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC network role to WPS's.
 *
 *  @param[in] role  Node's network role (SWC type).
 *  @return Node's network role (WPS type).
 */
static wps_role_t network_role_swc_to_wps(swc_role_t role)
{
    switch (role) {
    case SWC_ROLE_COORDINATOR:
        return NETWORK_COORDINATOR;
    case SWC_ROLE_NODE:
        return NETWORK_NODE;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Check if the sleep level is supported.
 *
 *  @param[in] level     Node's sleep level.
 *  @param[in] schedule  Pointer to WPS schedule.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool sleep_level_supported(swc_sleep_level_t level, schedule_t *schedule)
{
    switch (level) {
    case SWC_SLEEP_IDLE:
        /* Verify that timeslots are valid */
        for (uint32_t i = 0; i < schedule->size; i++) {
            if (schedule->timeslot[i].duration_pll_cycles > UINT16_MAX) {
                return false;
            }
        }
        return true;
    case SWC_SLEEP_SHALLOW:
    case SWC_SLEEP_DEEP:
        /* Verify that timeslots are valid */
        for (uint32_t i = 0; i < schedule->size; i++) {
            if ((schedule->timeslot[i].duration_pll_cycles / PLL_RATIO) >= UINT16_MAX) {
                return false;
            }
        }
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC sleep level to WPS's.
 *
 *  @param[in] level  Node's sleep level (SWC type).
 *  @return Node's sleep level (WPS type).
 */
static sleep_lvl_t sleep_level_swc_to_wps(swc_sleep_level_t level)
{
    switch (level) {
    case SWC_SLEEP_IDLE:
        return SLEEP_IDLE;
    case SWC_SLEEP_SHALLOW:
        return SLEEP_SHALLOW;
    case SWC_SLEEP_DEEP:
        return SLEEP_DEEP;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Check if the IRQ polarity is supported.
 *
 *  @param[in] pol  Radio's IRQ polarity.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool irq_polarity_supported(swc_irq_polarity_t pol)
{
    switch (pol) {
    case SWC_IRQ_ACTIVE_LOW:
    case SWC_IRQ_ACTIVE_HIGH:
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC IRQ polarity to WPS's.
 *
 *  @param[in] pol  Radio's IRQ polarity (SWC type).
 *  @return Radio's IRQ polarity (WPS type).
 */
static irq_polarity_t irq_polarity_swc_to_wps(swc_irq_polarity_t pol)
{
    switch (pol) {
    case SWC_IRQ_ACTIVE_LOW:
        return IRQ_ACTIVE_LOW;
    case SWC_IRQ_ACTIVE_HIGH:
        return IRQ_ACTIVE_HIGH;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Check if the SPI mode is supported.
 *
 *  @param[in] mode  Radio's SPI mode.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool spi_mode_supported(swc_spi_mode_t mode)
{
    switch (mode) {
    case SWC_SPI_STANDARD:
    case SWC_SPI_FAST:
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC SPI mode to WPS's.
 *
 *  @param[in] mode  Radio's SPI mode (SWC type).
 *  @return Radio's SPI mode (WPS type).
 */
static std_spi_t spi_mode_swc_to_wps(swc_spi_mode_t mode)
{
    switch (mode) {
    case SWC_SPI_STANDARD:
        return SPI_STANDARD;
    case SWC_SPI_FAST:
        return SPI_FAST;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Check if the modulation is supported.
 *
 *  @param[in] modulation  Connection's modulation.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool modulation_supported(swc_modulation_t modulation)
{
    switch (modulation) {
    case SWC_MOD_IOOK:
    case SWC_MOD_2BITPPM:
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC modulation to WPS's.
 *
 *  @param[in] modulation  Connection's modulation (SWC type).
 *  @return Connection's modulation (WPS type).
 */
static modulation_t modulation_swc_to_wps(swc_modulation_t modulation)
{
    switch (modulation) {
    case SWC_MOD_IOOK:
        return MODULATION_IOOK;
    case SWC_MOD_2BITPPM:
        return MODULATION_2BITPPM;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Check if the FEC level is supported.
 *
 *  @param[in] level  Connection's FEC level.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool fec_level_supported(swc_fec_level_t level)
{
    switch (level) {
    case SWC_FEC_0:
    case SWC_FEC_1:
    case SWC_FEC_2:
    case SWC_FEC_3:
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC FEC level to WPS's.
 *
 *  @param[in] level  Connection's FEC level (SWC type).
 *  @return Connection's FEC level (WPS type).
 */
static fec_level_t fec_level_swc_to_wps(swc_fec_level_t level)
{
    switch (level) {
    case SWC_FEC_0:
        return FEC_LVL_0;
    case SWC_FEC_1:
        return FEC_LVL_1;
    case SWC_FEC_2:
        return FEC_LVL_2;
    case SWC_FEC_3:
        return FEC_LVL_3;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Check if the CCA fail action is supported.
 *
 *  @param[in] action  Connection's CCA fail action.
 *  @retval true  Supported.
 *  @retval false Not supported.
 */
static bool cca_fail_action_supported(swc_cca_fail_action_t action)
{
    switch (action) {
    case SWC_CCA_FORCE_TX:
    case SWC_CCA_ABORT_TX:
        return true;
    default:
        return false;
    }
}

/** @brief Convert SWC CCA fail action to WPS's.
 *
 *  @param[in] action  Connection's CCA fail action (SWC type).
 *  @return Connection's CCA fail action (WPS type).
 */
static cca_fail_action_t cca_fail_action_swc_to_wps(swc_cca_fail_action_t action)
{
    switch (action) {
    case SWC_CCA_FORCE_TX:
        return CCA_FAIL_ACTION_TX;
    case SWC_CCA_ABORT_TX:
        return CCA_FAIL_ACTION_ABORT_TX;
    default: /* This should never happen */
        while (1);
    }
}

/** @brief Convert SWC radio HAL to WPS's.
 *
 *  @param[out] wps  WPS radio HAL.
 *  @param[in]  swc  SWC radio HAL.
 */
static void radio_hal_swc_to_wps(radio_hal_t *wps_hal, const swc_radio_hal_t *swc_hal,
                                 swc_error_t *err)
{
    CHECK_ERROR(swc_hal->set_shutdown_pin == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->reset_shutdown_pin == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->set_reset_pin == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->reset_reset_pin == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->read_irq_pin == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->set_cs == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->reset_cs == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->get_tick == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->transfer_full_duplex_blocking == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->transfer_full_duplex_non_blocking == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->is_spi_busy == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->context_switch == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->disable_radio_irq == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->enable_radio_irq == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->disable_radio_dma_irq == NULL, err, SWC_ERR_NULL_PTR, return);
    CHECK_ERROR(swc_hal->enable_radio_dma_irq == NULL, err, SWC_ERR_NULL_PTR, return);

    wps_hal->set_shutdown_pin                  = swc_hal->set_shutdown_pin;
    wps_hal->reset_shutdown_pin                = swc_hal->reset_shutdown_pin;
    wps_hal->set_reset_pin                     = swc_hal->set_reset_pin;
    wps_hal->reset_reset_pin                   = swc_hal->reset_reset_pin;
    wps_hal->read_irq_pin                      = swc_hal->read_irq_pin;
    wps_hal->set_cs                            = swc_hal->set_cs;
    wps_hal->reset_cs                          = swc_hal->reset_cs;
    wps_hal->delay_ms                          = swc_hal->delay_ms;
    wps_hal->get_tick                          = swc_hal->get_tick;
    wps_hal->tick_frequency_hz                 = swc_hal->tick_frequency_hz;
    wps_hal->transfer_full_duplex_blocking     = swc_hal->transfer_full_duplex_blocking;
    wps_hal->transfer_full_duplex_non_blocking = swc_hal->transfer_full_duplex_non_blocking;
    wps_hal->is_spi_busy                       = swc_hal->is_spi_busy;
    wps_hal->context_switch                    = swc_hal->context_switch;
    wps_hal->disable_radio_irq                 = swc_hal->disable_radio_irq;
    wps_hal->enable_radio_irq                  = swc_hal->enable_radio_irq;
    wps_hal->disable_radio_dma_irq             = swc_hal->disable_radio_dma_irq;
    wps_hal->enable_radio_dma_irq              = swc_hal->enable_radio_dma_irq;
}

/** @brief Save the current NVM and calibration.
 *
 *  @param[in] radio_id    Radio number.
 *  @param[in] nvm         NVM structure.
 *  @param[in] calib_vars  Calibration structure.
 */
static void save_radio_configuration(uint8_t radio_id, nvm_t *nvm, calib_vars_t *calib_vars)
{
    memcpy(&saved_nvm[radio_id], nvm, sizeof(nvm_t));
    memcpy(&saved_calib_vars[radio_id], calib_vars, sizeof(calib_vars_t));
}

/** @brief Get the previously saved calibration and NVM using save_radio_configuration.
 *
 * @param[in]  radio_id    Radio number.
 * @param[out] nvm         NVM structure.
 * @param[out] calib_vars  Calibration structure.
 * @param[in]  radio       Radio structure.
 */
static void get_saved_radio_configuration(uint8_t radio_id, nvm_t *nvm, calib_vars_t *calib_vars, radio_t *radio)
{
    memcpy(nvm, &saved_nvm[radio_id], sizeof(nvm_t));
    memcpy(calib_vars, &saved_calib_vars[radio_id], sizeof(calib_vars_t));
    radio->shadow_reg.reg_resistune = calib_vars->resistune;
    radio->phy_version = calib_vars->phy_version;
}

/** @brief Do main connection priority error checks.
 *
 * Some main connection fields must be identical in all connections assigned to the same timeslot,
 * we need to return an error to the SWC user if this is not the case for any of those fields.
 * This check needs to be done every time a connection is assigned to a time slot.
 *
 * @param[in]  timeslots  Timeslot.
 * @param[out] err  Wireless Core error code.
 */
static void check_main_connection_priority_errors(swc_node_t *node, timeslot_t timeslot,
                                                  swc_error_t *err)
{
    wps_connection_t *connection;
    wps_connection_t *first_connection = timeslot.connection_main[0];
    bool is_timeslot_rx = is_rx_connection(node->cfg.local_address,
                                           first_connection->source_address);

    for (uint8_t i = 1; i < timeslot.main_connection_count; i++) {
        connection = timeslot.connection_main[i];

        if (is_timeslot_rx) {
            /* Both local address should match. */
            CHECK_ERROR((first_connection->destination_address != connection->destination_address),
                        err, SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
            /* If first connection does not have ack, radio won't be configured with acks. */
            CHECK_ERROR((!first_connection->ack_enable && connection->ack_enable), err,
                        SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        } else {
            /* Both local address should match. */
            CHECK_ERROR((first_connection->source_address != connection->source_address), err,
                        SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        }
        CHECK_ERROR(memcmp(&first_connection->link_phase, &connection->link_phase, sizeof(link_phase_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR((first_connection->header_size != connection->header_size), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->link_protocol, &connection->link_protocol, sizeof(link_protocol_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->frag, &connection->frag, sizeof(frag_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->frame_cfg, &connection->frame_cfg, sizeof(frame_cfg_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->packet_cfg, &connection->packet_cfg, sizeof(packet_cfg_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
    }
}

/** @brief Do auto connection priority error checks.
 *
 * Some auto connection fields must be identical in all connections assigned to the same timeslot,
 * we need to return an error to the SWC user if this is not the case for any of those fields.
 * This check needs to be done every time a connection is assigned to a time slot.
 *
 * @param[in]  timeslots  Timeslot.
 * @param[out] err  Wireless Core error code.
 */
static void check_auto_connection_priority_errors(swc_node_t *node, timeslot_t timeslot,
                                                  swc_error_t *err)
{
    wps_connection_t *connection;
    wps_connection_t *first_connection = timeslot.connection_auto_reply[0];
    bool is_timeslot_rx = is_rx_connection(node->cfg.local_address,
                                           first_connection->source_address);

    for (uint8_t i = 1; i < timeslot.main_connection_count; i++) {
        connection = timeslot.connection_main[i];
        /* Main connection source address should match auto reply destination address. */
        CHECK_ERROR((first_connection->destination_address != connection->source_address),
                    err, SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        /* Main connection destination address should match auto reply source address. */
        CHECK_ERROR((first_connection->source_address != connection->destination_address), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
    }

    for (uint8_t i = 1; i < timeslot.auto_connection_count; i++) {
        connection = timeslot.connection_auto_reply[i];

        /* Both destination and source address should match. */
        CHECK_ERROR((first_connection->destination_address != connection->destination_address),
                    err, SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR((first_connection->source_address != connection->source_address), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        if (is_timeslot_rx) {
            /* If first connection does not have ack, radio won't be configured with acks. */
            CHECK_ERROR((!first_connection->ack_enable && connection->ack_enable), err,
                        SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        }
        CHECK_ERROR(memcmp(&first_connection->link_phase, &connection->link_phase, sizeof(link_phase_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR((first_connection->header_size != connection->header_size), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->link_protocol, &connection->link_protocol, sizeof(link_protocol_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->frag, &connection->frag, sizeof(frag_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->frame_cfg, &connection->frame_cfg, sizeof(frame_cfg_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
        CHECK_ERROR(memcmp(&first_connection->packet_cfg, &connection->packet_cfg, sizeof(packet_cfg_t)), err,
                    SWC_ERR_NON_MATCHING_SAME_TIMESLOT_CONN_FIELD, return);
    }
}

/** @brief Get a formatted string of the radio's nvm content.
 *
 *  @param[in] wps_radio  Radio handle.
 *  @param[in] buffer     Buffer where to put the formatted string.
 *  @param[in] size       Size of the buffer.
 *  @return The formated string length, excluding the NULL terminator.
 */
static int format_radio_nvm(wps_radio_t *wps_radio, char *buffer, uint16_t size)
{
    static const char *const phy_version_strings[] = {
        "v8",
        "v8.1",
        "v8.2",
        "v8.3"
    };
    static const char *const phy_model_strings[] = {
        "SR1010",
        "SR1020"
    };
    int string_length;
    uint8_t id_model = wps_radio_get_product_id_model(wps_radio);
    uint8_t id_version = wps_radio_get_product_id_version(wps_radio);
    uint64_t radio_serial = wps_radio_get_serial_number(wps_radio);
    uint8_t *serial_ptr = (uint8_t *)&radio_serial;
    char tmp[32];

    /* Format the serial string. */
    snprintf(tmp, sizeof(tmp), "%c%c%02x%02x%02x%02x%02x%02x",
                               serial_ptr[7],
                               serial_ptr[6],
                               serial_ptr[5],
                               serial_ptr[4],
                               serial_ptr[3],
                               serial_ptr[2],
                               serial_ptr[1],
                               serial_ptr[0]);

    /* Format the output string. */
    string_length = snprintf(buffer, size,
                             "<<  RADIO NVM  >>\r\n"
                             " Radio Serial: %s\r\n"
                             " Radio model: %s\r\n"
                             " Radio version: %s\r\n",
                             tmp,
                             phy_model_strings[id_model],
                             phy_version_strings[id_version]);

    return string_length;
}
