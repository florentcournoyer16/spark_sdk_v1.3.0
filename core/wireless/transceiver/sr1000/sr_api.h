/** @file sr_api.h
 *  @brief Radio level application programming interface.
 *
 *  @copyright Copyright (C) 2020-2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SR_API_H_
#define SR_API_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sr_access.h"
#include "sr_api_error.h"
#include "sr_def.h"
#include "sr_phy_error.h"
#include "sr_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
static const uint32_t sync_word_table[] = {
    0x08ecad3e, 0x09ae74e5, 0x0a2fb635, 0x0ade3365, 0x0b1ae937, 0x0cbad627, 0x0ce2a76d, 0x0e6ae45b,
    0xe129ab17, 0xe126eac6, 0xe1225779, 0xe620a5db, 0xe92e8c4e, 0xe5a0af32, 0x0daf91ac, 0x0ca2fb72,
};

#define CLOCK_FACTOR         1000  /*!< Symbol Clock factor for fixed point operation */
#define CRYSTAL_CLOCK_PERIOD 32768 /*!< Radio crystal clock frequency */
#define SYMBOL_CLOCK_PERIOD  20480 /*!< Symbol clock frequency */
#define POWER_UP_TIME        1000  /*!< Radio power up time in seconds */
#define MS_TO_S              1000  /*!< Milliseconds to seconds conversion factor */

/* MACRO **********************************************************************/
/* ALWAYS USE THESE MACRO WHEN SETTING THESE REGISTER */
#define SET_RADIO_ACTIONS(...)   OR(__VA_ARGS__) /*!< Register 1F(Radio actions) setter for the SR1000 API */
#define SET_INT_FLAG_CFG(...)    OR(__VA_ARGS__) /*!< Register 0(Radio flags) setter for the SR1000 API */
#define SET_TIMER_CFG(...)       OR(__VA_ARGS__) /*!< Register 04(Sleep configuration) setter for the SR1000 API */
#define SET_PACKET_CFG(...)      OR(__VA_ARGS__) /*!< Register 3F(Packet configuration) setter for the SR1000 API */
#define SET_MAIN_MODEM_FEAT(...) OR(__VA_ARGS__) /*!< Register 2C(Main modem features) setter for the SR1000 API */
#define SET_DLLTUNING(...)       OR(__VA_ARGS__) /*!< Register 1D(DLL tuning) setter for the SR1000 API */
#define SET_CTRL_PERIPH(...)     OR(__VA_ARGS__) /*!< Register 0E(Disable peripherals) setter for the SR1000 API */
#define SET_TX_PARAMS(...)       OR(__VA_ARGS__) /*!< Register 1C(TX parameters) setter for the SR1000 API */

/* TYPES **********************************************************************/
typedef uint32_t int_flag_cfg_t;       /*!< Register 00(Radio events) type for SR1000 API */
typedef uint16_t tmr_slp_config_t;     /*!< Register 04(Sleep configuration) type for SR1000 API */
typedef uint16_t radio_actions_t;      /*!< Register 1F(Radio actions) type for SR1000 API */
typedef uint16_t main_modem_feat_t;    /*!< Register 2C(Main modem features) type for SR1000 API */
typedef uint16_t peripherals_ctrl_t;   /*!< Register 0E(Disable peripherals) type for SR1000 API */
typedef uint16_t dll_tuning_t;         /*!< Register 1D(DLL tuning) type for SR1000 API */
typedef uint16_t clear_to_send_t;      /*!< Register 2D(Clear to send) type for SR1000 API */
typedef uint16_t packet_cfg_t;         /*!< Register 3F(Packet configuration) type for SR1000 API */
typedef uint16_t radio_timer_config_t; /*!< Register 05(Timer configuration) type for SR1000 API */
typedef uint16_t tx_params_t;          /*!< Register 1C(TX parameters) type for SR1000 API */

/** @brief API shadow register.
 *
 *  @note This is use by a couple of api function
 *        when a register has been changed but not written
 *        to yet.
 */
typedef struct shadow_register {
    reg_16_bit_t reg_int_flag_status; /*!< Current value in REG_IRQMASK1 & 2*/
    uint8_t reg_slp_cfg;              /*!< Current value in REG_SLEEPCONF */
    uint8_t radio_actions;            /*!< Current value in REG_ACTIONS */
    uint8_t reg_sync_word_cfg;        /*!< Current value in REG_SYNCWORDCFG */
    uint8_t reg_tx_params;            /*!< Current value in REG_TXPARAMS */
    uint8_t reg_packet_cfg;           /*!< Current value in REG_PACKETCFG */
    uint8_t reg_disables;             /*!< Current value in REG_DISABLES */
    uint8_t reg_dll_tuning;           /*!< Current value in REG_DLLTUNING */
    uint8_t reg_pll_startup;          /*!< Current value in REG_PLLSTARTUP */
    uint8_t reg_modem_gain;           /*!< Current value in REG_MODEMGAIN */
    uint8_t reg_main_modem;           /*!< Current value in REG_MAINMODEM */
    uint8_t reg_resistune;            /*!< Current value in REG_RESISTUNE */
    uint8_t reg_timer_cfg;            /*!< Current value in REG_TIMERCONF */
    uint8_t tx_pulse_size;            /*!< Current size of the TX pulse pattern */
} radio_shadow_reg_t;

/** @brief Radio internal or external clock source.
 */
typedef struct clock_source {
    pll_dis_t                pll_dis;        /*!< Disable internal PLL clock source */
    symbol_rate_clk_source_t pll_clk_source; /*!< Enable external PLL clock source */
    xtal_osc_clk_source_t xtal_clk_source;   /*!< Enable external XTAL clock source */
} clock_source_t;

/** @brief Radio instance.
 */
typedef struct radio {
    radio_hal_t radio_hal;                     /*!< SR1000 radio HAL function pointer structure */
    irq_polarity_t irq_polarity;                /*!< Interrupt polarity */
    access_sequence_instance_t access_sequence; /*!< Sequence to be written/read from the communication bus */
    radio_shadow_reg_t shadow_reg;              /*!< Shadow register instance */
    std_spi_t std_spi;                          /*!< Standard SPI operations */
    uint8_t phy_version;                        /*!< Radio version */
    clock_source_t clock_source;                /*!< Radio PLL and XTAL clock source (Internal or external) */
} radio_t;

/** @brief Receiver timeout and power-up duration instance.
 */
typedef struct rx_timeout {
    uint32_t rx_period_us;         /*!< Receiver timeout, in us */
    uint16_t rx_power_up_delay_ns; /*!< Receiver power-up delay, in ns */
} rx_timeout_t;

/** @brief The 16-bit read register split in two 8-bit registers.
 */
typedef struct read_reg_16 {
    uint8_t *msb; /*!< MSB for register value read */
    uint8_t *lsb; /*!< LSB for register value read */
} read_reg_16_t;

/** @brief Receiver wait time reception buffer instance.
 */
typedef struct rx_wait_time {
    uint8_t *rx_wait_time0; /*!< 8-bit LSB value of the 16-bit register */
    uint8_t *rx_wait_time1; /*!< 8-bit MSB value of the 16-bit register */
} rx_wait_time_t;

/** @brief Last received address instance.
 */
typedef struct last_addr {
    uint8_t *msb; /*!< 8-bit LSB value of the 16-bit register */
    uint8_t *lsb; /*!< 8-bit MSB value of the 16-bit register */
} last_addr_t;

/** @brief Synchronization word configuration.
 */
typedef struct syncword_cfg {
    uint32_t syncword;                 /*!< Synchronization word, 16 or 32 bits.*/
    uint8_t syncword_bit_cost;         /*!< Synchronization word detection bit mismatch's extra cost, 3 bits range value */
    uint8_t syncword_tolerance;        /*!< Synchronization word detection tolerance, 5 bits range value */
    syncword_length_t syncword_length; /*!< Synchronization word length, either SYNCWORD_LENGTH_16 or SYNCWORD_LENGTH_32 */
} syncword_cfg_t;

/** @brief Frame configuration structure.
 *
 *  @note Define the modulation, FEC and preamble len of the frame.
 */
typedef struct {
    modulation_t modulation; /*!< RF modulation */
    fec_level_t fec;         /*!< Forward Error Correction level */
} frame_cfg_t;

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static inline uint8_t pll_startup_us_to_clock_register(uint32_t pll_startup_us);
static inline uint16_t rx_timeout_us_to_register(uint32_t rx_period_us);
static inline uint8_t rx_pwr_up_delay_ns_to_register(uint16_t rx_power_up_delay_ns);
static inline uint8_t rx_pause_time_pll_cycles_to_register(uint32_t rx_pause_time_pll_cycles);
static inline uint8_t pwr_up_delay_ns_to_register(uint32_t pwr_up_delay_us);
static inline uint8_t clamp_8_bit_value(int16_t value);
static inline void uwb_reset(radio_t *radio, uint8_t delay_ms);
static inline void uwb_set_shadow_reg(radio_t *radio);
static inline void uwb_control_peripherals(radio_t *radio, peripherals_ctrl_t periph_ctrl);
static inline void uwb_transfer_blocking(radio_t *radio);
static inline void uwb_transfer_non_blocking(radio_t *radio);
static inline void uwb_write_register_8(radio_t *radio, uint8_t target_reg, uint8_t value);

/* PUBLIC FUNCTIONS ***********************************************************/
/** @brief Initialize the api .
 *
 *  @param[out] radio    Radio instance.
 *  @param[in]  no_reset If false, radio register and state will be set to default one.
 *  @param[out] error    API error instance.
 */
static inline void uwb_init(radio_t *radio, bool no_reset, sr_api_error_t *error)
{
    *error = API_ERR_NONE;

    /* 1 second delay after board power-up to allow radio xtal stabilisation. */
    while (radio->radio_hal.get_tick() <
           (uint64_t)(POWER_UP_TIME * radio->radio_hal.tick_frequency_hz / MS_TO_S)) {};

    if (!no_reset) {
        uwb_reset(radio, 10);
    }

    sr_access_append_init(&radio->access_sequence);

    uwb_control_peripherals(radio, SET_CTRL_PERIPH(CTRL_PERIPH_CLEAR, radio->std_spi,
                                                   radio->clock_source.pll_clk_source,
                                                   radio->clock_source.pll_dis,
                                                   radio->clock_source.xtal_clk_source));

    uwb_set_shadow_reg(radio);
}

/** @brief Reset the radio.
 *
 *  @param[in] radio     Radio's instance.
 *  @param[in] delay_ms  Delay in milliseconds before releasing the reset pin.
 *                       Radio is kept IDLE after reset for the same amount of time.
 */
static inline void uwb_reset(radio_t *radio, uint8_t delay_ms)
{
    radio->radio_hal.reset_reset_pin();
    radio->radio_hal.delay_ms(delay_ms);
    radio->radio_hal.set_reset_pin();
    radio->radio_hal.delay_ms(delay_ms);
}

/** @brief Reset the shadow register values to the internal radio values.
 *
 *  @param[in] radio radio_t instance.
 */
static inline void uwb_set_shadow_reg(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    uint8_t *reg_packet_cfg;
    uint8_t *reg_disables;
    uint8_t *reg_dll_tuning;
    uint8_t *reg_pll_startup;
    uint8_t *reg_resistune;
    uint8_t *reg_slp_cfg;
    uint8_t *reg_sync_word_cfg;
    uint8_t *reg_tx_params;
    uint8_t *reg_main_modem;
    uint8_t *reg_timer_cfg;

    sr_access_append_read_8(&radio->access_sequence, REG_PACKETCFG, &reg_packet_cfg, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_DISABLES, &reg_disables, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_DLLTUNING, &reg_dll_tuning, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_PLLSTARTUP, &reg_pll_startup, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_RESISTUNE, &reg_resistune, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_SLEEPCONF, &reg_slp_cfg, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_SYNCWORDCFG, &reg_sync_word_cfg, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_TXPARAMS, &reg_tx_params, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_MAINMODEM, &reg_main_modem, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_TIMERCONF, &reg_timer_cfg, &access_sequence_error);
    uwb_transfer_blocking(radio);

    radio->shadow_reg.reg_int_flag_status.short_val = REG16_INTFLG_DFLT;
    radio->shadow_reg.radio_actions                 = BIT_RXMODE | BIT_GOTOSLP;
    radio->shadow_reg.reg_modem_gain                = MOV2MASK(0b111111, BITS_MANUGAIN);
    radio->shadow_reg.reg_packet_cfg                = *reg_packet_cfg;
    radio->shadow_reg.reg_disables                  = *reg_disables;
    radio->shadow_reg.reg_dll_tuning                = *reg_dll_tuning |= BIT_ECO;
    radio->shadow_reg.reg_pll_startup               = *reg_pll_startup;
    radio->shadow_reg.reg_resistune                 = *reg_resistune;
    radio->shadow_reg.reg_slp_cfg                   = *reg_slp_cfg;
    radio->shadow_reg.reg_sync_word_cfg             = *reg_sync_word_cfg;
    radio->shadow_reg.reg_tx_params                 = *reg_tx_params;
    radio->shadow_reg.reg_main_modem                = *reg_main_modem;
    radio->shadow_reg.reg_timer_cfg                 = *reg_timer_cfg;
}

/** @brief Set the radio destination address.
 *
 *  @param[in] radio         Radio instance.
 *  @param[in] address       Destination address.
 *  @param[in] address_size  ADDRESS_LENGTH_8 for 8-bit address, ADDRESS_LENGTH_16 for 16-bit address.
 */
static inline void uwb_set_destination_address(radio_t *radio, uint16_t address, address_length_t address_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    if (address_size == ADDRESS_LENGTH_8) {
        sr_access_append_write_8(&radio->access_sequence, REG_REMOTADDR0, EXTRACT_BYTE(address, 0), &access_sequence_error);
    } else {
        sr_access_append_write_8(&radio->access_sequence, REG_REMOTADDR0, EXTRACT_BYTE(address, 0), &access_sequence_error);
        sr_access_append_write_8(&radio->access_sequence, REG_REMOTADDR1, EXTRACT_BYTE(address, 1), &access_sequence_error);
    }
}

/** @brief Set the radio local address.
 *
 *  @param[in] radio         Radio instance.
 *  @param[in] address       Local address.
 *  @param[in] address_size  ADDRESS_LENGTH_8 for 8-bit address, ADDRESS_LENGTH_16 for 16-bit address.
 */
static inline void uwb_set_local_address(radio_t *radio, uint16_t address, address_length_t address_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    if (address_size == ADDRESS_LENGTH_8) {
        sr_access_append_write_8(&radio->access_sequence, REG_LOCALADDR0, EXTRACT_BYTE(address, 0), &access_sequence_error);
    } else {
        sr_access_append_write_8(&radio->access_sequence, REG_LOCALADDR0, EXTRACT_BYTE(address, 0), &access_sequence_error);
        sr_access_append_write_8(&radio->access_sequence, REG_LOCALADDR1, EXTRACT_BYTE(address, 1), &access_sequence_error);
    }
}

/** @brief Enable the radio sleep timer.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_enable_modem_wakeup_timer(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_RXWAITTIME1, BIT_RXWAISRC, &access_sequence_error);
}

/** @brief Disable the radio sleep timer.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_disable_modem_wakeup_timer(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_RXWAITTIME1, 0, &access_sequence_error);
}

/** @brief Set the radio off time interval.
 *
 *  @param[in] radio          Radio instance.
 *  @param[in] rx_pause_time  Receiver off time interval, in pll cycles.
 */
static inline void uwb_set_rx_pause_time(radio_t *radio, uint16_t rx_pause_time)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    uint8_t reg_value = rx_pause_time_pll_cycles_to_register(rx_pause_time);

    sr_access_append_write_8(&radio->access_sequence, REG_RXPAUSETIME, reg_value, &access_sequence_error);
}

/** @brief Configure the radio timer.
 *
 *  @param[in] radio         Radio instance.
 *  @param[in] timer_config  Timer configuration instance.
 */
static inline void uwb_set_timer_config(radio_t *radio, radio_timer_config_t timer_config)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = timer_config;

    radio->shadow_reg.reg_timer_cfg &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_timer_cfg |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_TIMERCONF, radio->shadow_reg.reg_timer_cfg, &access_sequence_error);
}

/** @brief Disable the radio timer.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_disable_timer_config(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_TIMERCONF, 0, &access_sequence_error);
}

/** @brief Set the main IRQ register flag.
 *
 *  @param[in] radio         Radio instance.
 *  @param[in] int_flag_cfg  Interrupt settings.
 */
static inline void uwb_set_int_flag(radio_t *radio, int_flag_cfg_t int_flag_cfg)
{
    sr_phy_error_t error;

    volatile radio_reg_internal_16_t internal;

    internal.reg_32_u = int_flag_cfg;

    radio->shadow_reg.reg_int_flag_status.short_val &= ~internal.reg_value[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_int_flag_status.short_val |= internal.reg_value[_BIT_TO_SET];
    sr_access_append_write_16(&radio->access_sequence, REG16_IRQMASK, radio->shadow_reg.reg_int_flag_status, &error);
}

/** @brief Disable all the IRQs.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_disable_irq(radio_t *radio)
{
    uwb_set_int_flag(radio, SET_INT_FLAG_CFG(INT_FLAG_ENABLE_CLEAR, radio->irq_polarity));
}

/** @brief Setup the radio sleep level and
 *         the sleep trigger event.
 *
 *  @param[in] radio   Radio instance.
 *  @param[in] level   Sleep level instance.
 *  @param[in] events  Sleep event instance.
 */
static inline void uwb_set_sleep_config(radio_t *radio, sleep_lvl_t level, sleep_events_t events)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = level | events;
    radio->shadow_reg.reg_slp_cfg &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_slp_cfg |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_SLEEPCONF, radio->shadow_reg.reg_slp_cfg, &access_sequence_error);
}

/** @brief Sent specific commands to the radio.
 *
 *  @param[in] radio                 Radio instance.
 *  @param[in] target_radio_actions  Radio actions instance.
 */
static inline void uwb_set_radio_actions(radio_t *radio, radio_actions_t target_radio_actions)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = target_radio_actions;
    radio->shadow_reg.radio_actions &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.radio_actions |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_ACTIONS, radio->shadow_reg.radio_actions, &access_sequence_error);
}

/** @brief Get main IRQ status flag.
 *
 *  @param[in] radio  Radio instance.
 *  @return Pointers containing the IRQ flag registers.
 */
static inline read_reg_16_t uwb_get_irq_flags(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    read_reg_16_t read_reg;

    sr_access_append_read_16(&radio->access_sequence, REG_IRQMASK1, &read_reg.msb, &read_reg.lsb, &access_sequence_error);

    return read_reg;
}

/** @brief Set the payload CRC polynomial.
 *
 *  @param[in] radio           Radio instance.
 *  @param[in] crc_polynomial  Target CRC polynomial.
 */
static inline void uwb_set_crc(radio_t *radio, uint16_t crc_polynomial)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t crc_high                     = EXTRACT_BYTE(crc_polynomial, 1);
    uint8_t crc_low                      = EXTRACT_BYTE(crc_polynomial, 0);

    sr_access_append_write_8(&radio->access_sequence, REG_CRCPOLYNOM1, crc_high, &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_CRCPOLYNOM0, crc_low, &access_sequence_error);
}

/** @brief Set the radio main features.
 *
 *  @param[in] radio            Radio instance.
 *  @param[in] main_modem_feat  Radio main features instance.
 */
static inline void uwb_set_main_modem_features(radio_t *radio, main_modem_feat_t main_modem_feat)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = main_modem_feat;
    radio->shadow_reg.reg_main_modem &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_main_modem |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_MAINMODEM, radio->shadow_reg.reg_main_modem, &access_sequence_error);
}

/** @brief Set the radio constant gains.
 *
 *  @param[in] radio        Radio instance.
 *  @param[in] const_gains  Constant gains value.
 */
static inline void uwb_set_const_gains(radio_t *radio, uint8_t const_gains)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_CONSTGAINS, const_gains, &access_sequence_error);
}

/** @brief Enable the radio phase tracking.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_enable_phase_tracking(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_DEBUGMODEM, BITS_MANUPHASE, &access_sequence_error);
}

/** @brief Disable the radio phase tracking.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_disable_phase_tracking(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_DEBUGMODEM, 0, &access_sequence_error);
}

/** @brief Configure the frame preamble length.
 *
 *  @param[in] radio          Radio instance.
 *  @param[in] preamble_size  Preamble size.
 */
static inline void uwb_set_preamble_length(radio_t *radio, uint16_t preamble_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t register_preamble_size       = (preamble_size / 2) - 8;

    sr_access_append_write_8(&radio->access_sequence, REG_PREAMBLEN, register_preamble_size, &access_sequence_error);
}

/** @brief Configure the frame syncword.
 *
 *  @param[in] radio  Radio instance.
 *  @param[in] cfg    Syncword configuration instance.
 */
static inline void uwb_set_syncword_config(radio_t *radio, syncword_cfg_t *cfg)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t syncword[4]                  = {0};
    reg_16_bit_t reg_data;

    reg_data.short_val = cfg->syncword_length | SET_REGISTER(BITS_SWBITCOST, cfg->syncword_bit_cost) |
                         SET_REGISTER(BITS_SWCORRTOL, cfg->syncword_tolerance);

    radio->shadow_reg.reg_sync_word_cfg &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_sync_word_cfg |= reg_data.bytes[_BIT_TO_SET];
    serialize_uint32_to_uint8_array(cfg->syncword, syncword);

    sr_access_append_write_8(&radio->access_sequence, REG_SYNCWORDCFG, radio->shadow_reg.reg_sync_word_cfg, &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_SYNCWORD0, syncword[3], &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_SYNCWORD1, syncword[2], &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_SYNCWORD2, syncword[1], &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_SYNCWORD3, syncword[0], &access_sequence_error);
}

/** @brief Configure the TX payload size.
 *
 *  @note Useless if BIT_SIZESRC of register REG_PACKETCFG(0x3E)
 *        is set.
 *
 *  @param[in] radio        Radio instance.
 *  @param[in] packet_size  TX payload size.
 */
static inline void uwb_set_tx_packet_size(radio_t *radio, uint8_t packet_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_TXPKTSIZE, packet_size, &access_sequence_error);
}

/** @brief Configure the RX waited source.
 *
 *  @param[in] radio        Radio instance.
 *  @param[in] rx_wait_src  RX waited source.
 */
static inline void uwb_set_rx_waited_src(radio_t *radio, rx_wait_source_t rx_wait_src)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence,
                             REG_RXWAITTIME1,
                             rx_wait_src,
                             &access_sequence_error);
}

/** @brief Configure the RX payload size.
 *
 *  @note  If BIT_SIZEHDRE of register REG_PACKETCFG(0x3E)
 *         is cleared and the user does not send a header
 *         in the payload, it's the only way to tell what
 *         is the payload size.
 *
 *  @param[in] radio        Radio instance.
 *  @param[in] packet_size  RX payload size.
 */
static inline void uwb_set_rx_packet_size(radio_t *radio, uint8_t packet_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_RXPKTSIZE, packet_size, &access_sequence_error);
}

/** @brief Configure the phase-locked loop power-up waiting time.
 *
 *  @note The PLL startup time have a resolution of 30.51 us.
 *
 *  @param[in] radio           Radio instance.
 *  @param[in] pll_startup_us  PLL power-up waiting time, in us.
 */
static inline void uwb_set_pll_startup(radio_t *radio, uint16_t pll_startup_us)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint32_t register_pll_startup        = pll_startup_us_to_clock_register(pll_startup_us);

    radio->shadow_reg.reg_pll_startup = register_pll_startup;

    sr_access_append_write_8(&radio->access_sequence, REG_PLLSTARTUP, register_pll_startup, &access_sequence_error);
}

/** @brief Configure the receiver timeout and the power-up duration.
 *
 *  @note RX timeout is extended to 12-bit and the power-up duration
 *        is 4-bit long.
 *        The timeout have a resolution of 390.625025 ns per bit.
 *        Receiver power up delay have a resolution of 48.82815 ns per bit
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] rx_timeout  RX timeout and power-up duration instance.
 */
static inline void uwb_set_rx_timeout(radio_t *radio, rx_timeout_t *rx_timeout)
{
    sr_phy_error_t access_sequence_error    = ACCESS_SEQUENCE_ERR_BUSY;
    uint32_t given_timeout_to_register      = rx_timeout_us_to_register(rx_timeout->rx_period_us);
    uint32_t given_pwr_up_delay_to_register = rx_pwr_up_delay_ns_to_register(rx_timeout->rx_power_up_delay_ns);
    uint8_t rx_period_upper                 = given_timeout_to_register >> 4;
    uint8_t rx_period_lower                 = given_timeout_to_register & 0x0F;

    /* Append rx_power_up_delay to lower register */
    rx_period_lower = rx_period_lower << 4;
    rx_period_lower |= (given_pwr_up_delay_to_register & 0x0F);

    sr_access_append_write_8(&radio->access_sequence, REG_RXTIMEOUT0, rx_period_lower, &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_RXTIMEOUT1, rx_period_upper, &access_sequence_error);
}

/** @brief Control the radio peripherals.
 *
 *  @param[in] radio        Radio instance.
 *  @param[in] periph_ctrl  Peripherals control instance.
 */
static inline void uwb_control_peripherals(radio_t *radio, peripherals_ctrl_t periph_ctrl)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    reg_16_bit_t reg_data;

    reg_data.short_val = periph_ctrl;

    radio->shadow_reg.reg_disables &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_disables |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_DISABLES, radio->shadow_reg.reg_disables, &access_sequence_error);
}

/** @brief Set dll tuning.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] dll_tuning  DLL tuning value.
 */
static inline void uwb_set_dll_tuning(radio_t *radio, dll_tuning_t dll_tuning)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = dll_tuning;

    radio->shadow_reg.reg_dll_tuning &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_dll_tuning |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_DLLTUNING, radio->shadow_reg.reg_dll_tuning, &access_sequence_error);
}

/** @brief Set integgain.
 *
 *  @param[in] radio            Radio instance.
 *  @param[in] integgain_value  Integgain value.
 */
static inline void uwb_set_integgain(radio_t *radio, uint8_t integgain_value)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = SET_REGISTER_8(BITS_INTEGGAIN, integgain_value);

    radio->shadow_reg.reg_dll_tuning &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_dll_tuning |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_DLLTUNING, radio->shadow_reg.reg_dll_tuning, &access_sequence_error);
}

/** @brief Fill the radio TX FIFO in BURST mode.
 *
 *  @param[in] radio  Radio instance.
 *  @param[in] tx_buffer  Buffer to transmit OTA.
 *  @param[in] size       Size of the transmission.
 */
static inline void uwb_fill_tx_fifo(radio_t *radio, uint8_t *tx_buffer, uint16_t size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_burst(&radio->access_sequence, REG_TXFIFO, tx_buffer, size, &access_sequence_error);
}

/** @brief Read an packet from the radio's RX FIFO in BURST mode.
 *
 *  @param[in] radio  Radio instance.
 *  @param[in] size   Reception size.
 *  @return RX data pointer
 */
static inline uint8_t *uwb_read(radio_t *radio, uint8_t size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t *rx_buffer;

    sr_access_append_read_burst(&radio->access_sequence, REG_RXFIFO, &rx_buffer, size, &access_sequence_error);

    return rx_buffer;
}

/** @brief Read frame size from the radio's RX FIFO.
 *
 *  @param[in] radio  Radio instance.
 *  @return RX frame sze data pointer
 */
static inline uint8_t *uwb_read_frame_size(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t *rx_buffer;

    sr_access_append_read_8(&radio->access_sequence, REG_RXFIFO, &rx_buffer, &access_sequence_error);

    return rx_buffer;
}

/** @brief Configure the TX buffer load IRQ threshold.
 *
 *  @param[in]  radio      Radio instance.
 *  @param[in]  threshold  Buffer load threshold.
 */
static inline void uwb_set_tx_buffer_load_irq_threshold(radio_t *radio, uint8_t threshold)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t write_value;

    write_value = BIT_TXIRQEN | MOV2MASK(threshold, BITS_TXTHRESH);

    sr_access_append_write_8(&radio->access_sequence, REG_TXFIFOSTAT, write_value, &access_sequence_error);
}

/** @brief Disable the TX buffer load IRQ.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_disable_tx_buffer_load_irq(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_TXFIFOSTAT, 0, &access_sequence_error);
}

/** @brief Configure the RX buffer load IRQ threshold.
 *
 *  @param[in]  radio      Radio instance.
 *  @param[in]  threshold  Buffer load threshold.
 */
static inline void uwb_set_rx_buffer_load_irq_threshold(radio_t *radio, uint8_t threshold)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t write_value                  = 0;

    write_value = BIT_RXIRQEN | MOV2MASK(threshold, BITS_RXTHRESH);

    sr_access_append_write_8(&radio->access_sequence, REG_RXFIFOSTAT, write_value, &access_sequence_error);
}

/** @brief Disable the RX buffer load IRQ.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_disable_rx_buffer_load_irq(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_RXFIFOSTAT, 0, &access_sequence_error);
}

/** @brief Set the radio power up delay.
 *
 *  @note The register have a resolution of approximately
 *        195 ns per register increment.
 *
 *  @param[in] radio              Radio instance.
 *  @param[in] power_up_delay_ns  Power up delay, in ns.
 */
static inline void uwb_set_pwr_up_delay(radio_t *radio, uint32_t power_up_delay_ns)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint32_t power_up_delay_in_cycles;

    power_up_delay_in_cycles = pwr_up_delay_ns_to_register(power_up_delay_ns);

    sr_access_append_write_8(&radio->access_sequence, REG_PWRUPDELAY, power_up_delay_in_cycles, &access_sequence_error);
}

/** @brief Set the register REG_RXFILTERS.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] lna_peak    Low noise amplifier peak value. BIT(7..5).
 *  @param[in] rx_filters  RX filters frequency. BIT(4..0).
 */
static inline void uwb_set_rx_filters(radio_t *radio, uint8_t lna_peak, uint8_t rx_filters)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t rx_filters_value             = MOV2MASK(lna_peak, BITS_LNAPEAK) | MOV2MASK(rx_filters, BITS_RFFILFREQ);

    sr_access_append_write_8(&radio->access_sequence, REG_RXFILTERS, rx_filters_value, &access_sequence_error);
}

/** @brief Set the register REG_RXFILTERS.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] rx_filters  RX filters frequency and Low noise amplifier peak value. BIT(7..0).
 */
static inline void uwb_set_rx_filters_raw(radio_t *radio, uint8_t rx_filters)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_RXFILTERS, rx_filters, &access_sequence_error);
}

/** @brief Set collision avoidance parameters.
 *
 *  @param[in] radio        Radio instance.
 *  @param[in] rx_idle_pwr  Receiver's idle power consumption. BIT(7..6).
 *  @param[in] cac_thresh   Collision-avoidance check threshold. BIT(5..0).
 */
static inline void uwb_set_cac(radio_t *radio, rx_idle_pwr_t rx_idle_pwr, uint8_t cac_thresh)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t reg_data;

    reg_data = rx_idle_pwr | MOV2MASK(cac_thresh, BITS_CSTHRES);

    sr_access_append_write_8(&radio->access_sequence, REG_CLEARTOSEND, reg_data, &access_sequence_error);
}

/** @brief Set the radio shared pulse parameters.
 *
 *  @param[in] radio      Radio instance.
 *  @param[in] tx_params  Shared pulse parameters structure.
 */
static inline void uwb_set_tx_params(radio_t *radio, tx_params_t tx_params)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = tx_params;

    radio->shadow_reg.reg_tx_params &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_tx_params |= reg_data.bytes[_BIT_TO_SET];

    sr_access_append_write_8(&radio->access_sequence, REG_TXPARAMS, radio->shadow_reg.reg_tx_params, &access_sequence_error);
}

/** @brief Setup the packet configuration(MAC layer).
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] packet_cfg  Packet configuration struct.
 */
static inline void uwb_set_packet_config(radio_t *radio, packet_cfg_t packet_cfg)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    reg_16_bit_t reg_data;

    reg_data.short_val = packet_cfg;

    radio->shadow_reg.reg_packet_cfg &= ~reg_data.bytes[_BIT_TO_CLEAR];
    radio->shadow_reg.reg_packet_cfg |= reg_data.bytes[_BIT_TO_SET];
    sr_access_append_write_8(&radio->access_sequence, REG_PACKETCFG, radio->shadow_reg.reg_packet_cfg, &access_sequence_error);
}

/** @brief Select the radio frequency channel to use in BURST mode.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] tx_pattern  0x10 - 0x17 register pattern buffer.
 *  @param[in] pulse_size  Pulse buffer size.
 */
static inline void uwb_select_channel(radio_t *radio, uint8_t *tx_pattern, uint8_t pulse_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t write_pulse_size             = radio->shadow_reg.tx_pulse_size < pulse_size ? radio->shadow_reg.tx_pulse_size : pulse_size;

    sr_access_append_write_burst(&radio->access_sequence, REG_TXPULSE12 + write_pulse_size, tx_pattern + write_pulse_size,
                                 NB_PULSES + 1 - write_pulse_size, &access_sequence_error);

    radio->shadow_reg.tx_pulse_size = pulse_size;
}

/** @brief Get radio phase information (4 phases) in BURST mode.
 *
 *  @param[in] radio   Radio's instance.
 *  @return Pointer that will contains the received phase info.
 */
static inline uint8_t *uwb_get_phase_info(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t *phase_info;

    sr_access_append_read_burst(&radio->access_sequence, REG_PHSDATA1, &phase_info, NB_PHASES, &access_sequence_error);

    return phase_info;
}

/** @brief Get the content of the REG_RXWAITTIME1..0.
 *
 *  @param[in] radio  Radio instance.
 *  @return RX wait time structure
 */
static inline rx_wait_time_t uwb_get_rx_wait_time(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    rx_wait_time_t rx_wait_time;

    sr_access_append_read_8(&radio->access_sequence, REG_RXWAITTIME1, &rx_wait_time.rx_wait_time1, &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence, REG_RXWAITTIME0, &rx_wait_time.rx_wait_time0, &access_sequence_error);

    return rx_wait_time;
}

/** @brief Get the last received signal strength indicator.
 *
 *  @param[in] radio  Radio instance.
 *  @return Last received frame RSSI.
 */
static inline uint8_t *uwb_get_rssi(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t *rssi;

    sr_access_append_read_8(&radio->access_sequence, REG_RSSI, &rssi, &access_sequence_error);

    return rssi;
}

/** @brief Get the last received noise strength indicator.
 *
 *  @param[in] radio  Radio instance.
 *  @return Last received frame RNSI.
 */
static inline uint8_t *uwb_get_rnsi(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t *rnsi;

    sr_access_append_read_8(&radio->access_sequence, REG_RNSI, &rnsi, &access_sequence_error);

    return rnsi;
}

/** @brief Fill the header field in the radio TX_FIFO.
 *
 *  This function do not work with any of the other append function.
 *  It should be call right before calling the fill_data function.
 *  It provided an optimized way to fill the TX FIFO using a
 *  header and a payload.
 *
 *  @note The header field can be null. This function is required
 *        in conjunction with the fill_data function in order to
 *        achieve optimal filling of the radio TX FIFO.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] hdr_buffer  Header to transmit OTA.
 *  @param[in] hdr_size    Size of the header.
 */
static inline void uwb_fill_header_non_blocking(radio_t *radio, uint8_t *hdr_buffer, uint8_t hdr_size)
{
    uwb_fill_tx_fifo(radio, hdr_buffer, hdr_size);
    uwb_transfer_non_blocking(radio);
}

/** @brief Fill the header field in the radio TX_FIFO in blocking mode.
 *
 *  This function do not work with any of the other append function.
 *  It should be call right before calling the fill_data function.
 *  It provided an optimized way to fill the TX FIFO using a
 *  header and a payload.
 *
 *  @note The header field can be null. This function is required
 *        in conjunction with the fill_data function in order to
 *        achieve optimal filling of the radio TX FIFO.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] hdr_buffer  Header to transmit OTA.
 *  @param[in] hdr_size    Size of the header.
 */
static inline void uwb_fill_header_blocking(radio_t *radio, uint8_t *hdr_buffer, uint8_t hdr_size)
{
    uwb_fill_tx_fifo(radio, hdr_buffer, hdr_size);
    uwb_transfer_blocking(radio);
}

/** @brief Fill the radio TX FIFO in non-blocking mode.
 *
 *  This function do not work with any of the other append function.
 *  It should be call right after calling the fill_header function.
 *  It provided an optimized way to fill the TX FIFO using a
 *  header and a payload.
 *
 *  @param[in] radio      Radio instance.
 *  @param[in] tx_buffer  Buffer to transmit OTA.
 *  @param[in] size       Size of the transmission.
 */
static inline void uwb_fill_data_non_blocking(radio_t *radio, uint8_t *tx_buffer, uint16_t size)
{
    sr_access_spi_transfer_non_blocking(&radio->radio_hal, tx_buffer, radio->access_sequence.rx_buffer, size);
}

/** @brief Fill the radio TX FIFO in blocking mode.
 *
 *  This function do not work with any of the other append function.
 *  It should be call right after calling the fill_header function.
 *  It provided an optimized way to fill the TX FIFO using a
 *  header and a payload.
 *
 *  @param[in] radio      Radio instance.
 *  @param[in] tx_buffer  Buffer to transmit OTA.
 *  @param[in] size       Size of the transmission.
 */
static inline void uwb_fill_data_blocking(radio_t *radio, uint8_t *tx_buffer, uint16_t size)
{
    sr_access_spi_transfer_blocking(&radio->radio_hal, tx_buffer, radio->access_sequence.rx_buffer, size);
}

/** @brief Fill the radio TX FIFO for the specific cut through mode case.
 *
 *  @param[in] radio   Radio instance.
 *  @param[in] buffer  Buffer to transmit OTA.
 *  @param[in] size    Size of the transmission.
 */
static inline void uwb_fill_cut_through_data(radio_t *radio, uint8_t *buffer, uint8_t size)
{
    uwb_fill_tx_fifo(radio, buffer, size);
}

/** @brief Initiate a transfer in non blocking mode
 *         using all the appended sequence.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_transfer_non_blocking(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_transfer_non_blocking(&radio->radio_hal, &radio->access_sequence, &access_sequence_error);
}

/** @brief Initiate a transfer in blocking mode
 *         using all the appended sequence.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_transfer_blocking(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &access_sequence_error);
}

/** @brief Send raw wake-sleep period to radio.
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] sleep_time  Desired wake sleep period, in clock cycles.
 */
static inline void uwb_set_wake_sleep_raw(radio_t *radio, uint16_t sleep_time)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t wake_sleep_period_raw[2]     = {0};

    serialize_uint16_to_uint8_array(sleep_time, wake_sleep_period_raw);

    /* Sending High value - LSB - MSB to ensure no issues when using normal SPI transfer */
    sr_access_append_write_8(&radio->access_sequence, REG_TIMERCOUNT1, 0xFF, &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_TIMERCOUNT0, wake_sleep_period_raw[1], &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_TIMERCOUNT1, wake_sleep_period_raw[0], &access_sequence_error);
}

/** @brief Send raw power-up delay to radio.
 *
 *  @note  The raw value will be divided by 4
 *         as per the register conversion.
 *
 *  @param[in] radio   Radio instance.
 *  @param[in] pwr_up  Desired pwr-up delay, in clock cycles.
 */
static inline void uwb_set_pwr_up_delay_raw(radio_t *radio, int16_t pwr_up)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t pwr_up_reg_value             = pwr_up / 4;

    sr_access_append_write_8(&radio->access_sequence, REG_PWRUPDELAY, pwr_up_reg_value, &access_sequence_error);
}

/** @brief Send raw RX timeout to the radio.
 *
 *  @note  The raw RX timeout will be divided by 8
 *         as per the register conversion. The Receiver
 *         power-up delay will be appended raw.
 *
 *  @param[in] radio            Radio instance.
 *  @param[in] rx_timeout       Desired timeout, in clock cycles.
 *  @param[in] rx_pwr_up_delay  Receiver power-up delay, in clock cycles (4-bit).
 */
static inline void uwb_set_rx_timeout_raw(radio_t *radio, uint16_t rx_timeout, uint8_t rx_pwr_up_delay)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;
    uint8_t rx_timeout_lsb               = (((EXTRACT_BYTE((rx_timeout / 8), 0) & 0x0F) << 4) | (rx_pwr_up_delay & 0x0F));
    uint8_t rx_timeout_msb               = (EXTRACT_BYTE((rx_timeout / 8), 1) << 4) | (EXTRACT_BYTE((rx_timeout / 8), 0) >> 4);

    sr_access_append_write_8(&radio->access_sequence, REG_RXTIMEOUT1, rx_timeout_msb, &access_sequence_error);
    sr_access_append_write_8(&radio->access_sequence, REG_RXTIMEOUT0, rx_timeout_lsb, &access_sequence_error);
}

static inline void uwb_set_timer_cfg_burst(radio_t *radio, uint16_t sleep_time, uint16_t rx_timeout, uint8_t rx_pwr_up_delay,
                                           int16_t pwr_up)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    uint8_t pwr_up_reg_value = pwr_up / 4;
    uint8_t rx_timeout_lsb   = (((EXTRACT_BYTE((rx_timeout / 8), 0) & 0x0F) << 4) | (rx_pwr_up_delay & 0x0F));
    uint8_t rx_timeout_msb   = (EXTRACT_BYTE((rx_timeout / 8), 1) << 4) | (EXTRACT_BYTE((rx_timeout / 8), 0) >> 4);
    uint8_t timer_cfg[sizeof(sleep_time) + sizeof(rx_timeout) + sizeof(pwr_up_reg_value)];

    serialize_uint16_to_uint8_array(sleep_time, timer_cfg);

    timer_cfg[sizeof(sleep_time)]                                                   = rx_timeout_msb;
    timer_cfg[sizeof(sleep_time) + sizeof(rx_timeout_msb)]                          = rx_timeout_lsb;
    timer_cfg[sizeof(sleep_time) + sizeof(rx_timeout_msb) + sizeof(rx_timeout_lsb)] = pwr_up_reg_value;

    sr_access_append_write_burst(&radio->access_sequence, REG_TIMERCOUNT1, (uint8_t *)&timer_cfg, sizeof(timer_cfg),
                                 &access_sequence_error);
}

/** @brief Check last received frame address
 *
 *  @param[in] radio  Radio instance.
 *  @return  Last address instance.
 */
static inline last_addr_t uwb_get_last_rcv_frame_addr(radio_t *radio)
{
    last_addr_t last_rcv_address;
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_read_8(&radio->access_sequence,
                            REG_FRAMEADDR1,
                            &last_rcv_address.msb,
                            &access_sequence_error);
    sr_access_append_read_8(&radio->access_sequence,
                            REG_FRAMEADDR0,
                            &last_rcv_address.lsb,
                            &access_sequence_error);

    return last_rcv_address;
}

/** @brief Set the raw value of register REG_RESISTUNE.
 *
 *  @param[in] radio                Radio instance.
 *  @param[in] resistune_reg_value  Resistune register raw value.
 */
static inline void uwb_set_resistune(radio_t *radio, uint8_t resistune_reg_value)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    radio->shadow_reg.reg_resistune = resistune_reg_value;

    sr_access_append_write_8(&radio->access_sequence, REG_RESISTUNE, resistune_reg_value, &access_sequence_error);
}

/** @brief Set static sleep mode.
 *
 *  @param[in] radio        Radio's instance.
 *  @param[in] sleep_level  Sleep mode.
 */
static inline void uwb_set_static_sleep_mode(radio_t *radio, sleep_lvl_t sleep_level)
{
    sr_phy_error_t error;
    uint8_t *status;

    uwb_set_sleep_config(radio, sleep_level, SLEEP_EVENT_CLEAR);
    uwb_set_radio_actions(radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR));
    sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);
    do {
        sr_access_append_read_8(&radio->access_sequence, REG_PWRSTATUS, &status, &error);
        sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);
    } while (!(*status & BIT_AWAKE));

    uwb_set_radio_actions(radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, GO_TO_SLEEP));
    sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);

    uint8_t status_flag;

    switch (sleep_level) {
    case SLEEP_IDLE:
        status_flag = BIT_AWAKE;
        break;
    case SLEEP_SHALLOW:
        status_flag = BIT_DCDCEN;
        break;
    case SLEEP_DEEP:
        status_flag = BIT_PLLEN;
        break;
    default:
        status_flag = BIT_AWAKE;
        break;
    }
    do {
        sr_access_append_read_8(&radio->access_sequence, REG_PWRSTATUS, &status, &error);
        sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);
    } while (*status & status_flag);
}

/** @brief Set static RX mode.
 *
 *  @param[in] radio      Radio's instance.
 *  @param[in] rx_filter  Raw register value for RX filter.
 */
static inline void uwb_set_static_rx_mode(radio_t *radio, uint8_t rx_filter)
{
    sr_phy_error_t error;
    uint8_t *status;

    uwb_set_rx_filters_raw(radio, rx_filter);
    uwb_set_radio_actions(radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, RX_MODE_ENABLE_RECEPTION));
    sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);

    do {
        sr_access_append_read_8(&radio->access_sequence, REG_PWRSTATUS, &status, &error);
        sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);
    } while (!(*status & BIT_RXEN));
}

/** @brief Set static TX mode.
 *
 *  @param[in] radio       Radio's instance.
 *  @param[in] tx_pattern  TX pattern.
 *  @param[in] pulse_size  Pulse size.
 *  @param[in] rx_filter   RX filter.
 *  @param[in] integgain   Intergration gain.
 */
static inline void uwb_set_static_tx_mode(radio_t *radio, uint8_t *tx_pattern, uint8_t pulse_size, uint8_t rx_filter, uint8_t integgain)
{
    sr_phy_error_t error;
    uint8_t *status;

    uwb_set_tx_params(radio, SET_TX_PARAMS(HOLD_TX_ON_ENABLE));
    uwb_write_register_8(radio, REG_DEBUGMODEM, BIT_OVERRIDE);
    uwb_set_integgain(radio, integgain);
    uwb_set_rx_filters_raw(radio, rx_filter);
    uwb_select_channel(radio, tx_pattern, pulse_size);
    sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);
    uwb_set_radio_actions(radio, SET_RADIO_ACTIONS(RADIO_ACTIONS_CLEAR, RX_MODE_ENABLE_TRANSMISSION, START_TX));
    sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);

    do {
        sr_access_append_read_8(&radio->access_sequence, REG_PWRSTATUS, &status, &error);
        sr_access_transfer_blocking(&radio->radio_hal, &radio->access_sequence, &error);
    } while (!(*status & BIT_TXEN));
}

/** @brief Check last received frame address
 *
 *  @param[in] radio  Radio instance.
 *  @retval True   SPI is busy.
 *  @retval False  SPI is not busy.
 */
static inline bool uwb_is_spi_busy(radio_t *radio)
{
    return sr_access_is_spi_busy(&radio->radio_hal);
}

/** @brief Append an 8-bit write command over the SPI
 *         The register address must be odd.
 *
 *  @param[in] radio       Radio's instance.
 *  @param[in] target_reg  Target radio register.
 *  @param[in] value       Value to write in register.
 */
static inline void uwb_write_register_8(radio_t *radio, uint8_t target_reg, uint8_t value)
{
    sr_phy_error_t error;

    sr_access_append_write_8(&radio->access_sequence, target_reg, value, &error);
}

/** @brief Append a write command over the SPI
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] target_reg  Target radio register.
 *  @param[in] value       Value to write in register.
 */
static inline void uwb_write_register(radio_t *radio, uint8_t target_reg, reg_16_bit_t value)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_16(&radio->access_sequence, target_reg, value, &access_sequence_error);
}

/** @brief Append a burst write command over the SPI
 *
 *  This should be the last command before using the
 *  uwb_transfer function.
 *
 *  @param[in] radio         Radio instance.
 *  @param[in] starting_reg  Starting radio register.
 *  @param[in] buffer        Buffer to write in register.
 *  @param[in] buffer_size   Buffer size to write in burst.
 */
static inline void uwb_burst_write(radio_t *radio, uint8_t starting_reg, uint8_t *buffer, uint8_t buffer_size)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_burst(&radio->access_sequence, starting_reg, buffer, buffer_size, &access_sequence_error);
}

/** @brief Append a read command over the SPI
 *
 *  @param[in] radio       Radio instance.
 *  @param[in] target_reg  Target radio register.
 *  @return Pointer to the read value.
 */
static inline uint8_t *uwb_read_register_8(radio_t *radio, uint8_t target_reg)
{
    uint8_t *read_value_ptr;
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_read_8(&radio->access_sequence, target_reg, &read_value_ptr, &access_sequence_error);

    return read_value_ptr;
}

/** @brief Append a burst read command over the SPI
 *
 *  This should be the last command before using the
 *  uwb_transfer function.
 *
 *  @param[in] radio         Radio instance.
 *  @param[in] starting_reg  Starting radio register.
 *  @param[in] read_size     Burst read size.
 *  @return Pointer to the read buffer.
 */
static inline uint8_t *uwb_burst_read(radio_t *radio, uint8_t starting_reg, uint8_t read_size)
{
    uint8_t *read_value_ptr;
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_read_burst(&radio->access_sequence,
                                starting_reg,
                                &read_value_ptr,
                                read_size,
                                &access_sequence_error);

    return read_value_ptr;
}

/** @brief Write the resistune register from the shadow copy.
 *
 *  @param[in] radio  Radio instance.
 */
static inline void uwb_apply_saved_calibration(radio_t *radio)
{
    sr_phy_error_t access_sequence_error = ACCESS_SEQUENCE_ERR_BUSY;

    sr_access_append_write_8(&radio->access_sequence, REG_RESISTUNE, radio->shadow_reg.reg_resistune, &access_sequence_error);
}

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
/** @brief Convert given PLL startup time (us) to the equivalent register value.
 *
 *  @param[in] pll_startup_us  PLL startup time, in us.
 *  @return Equivalent 8-bit register value.
 */
static uint8_t pll_startup_us_to_clock_register(uint32_t pll_startup_us)
{
    return (pll_startup_us * CRYSTAL_CLOCK_PERIOD) / (CLOCK_FACTOR * CLOCK_FACTOR);
}

/** @brief Convert given RX timeout (us) to the equivalent register value.
 *
 *  @param[in] rx_period_us  RX timeout period, in us.
 *  @return Equivalent 16-bit register value.
 */
static uint16_t rx_timeout_us_to_register(uint32_t rx_period_us)
{
    return ((rx_period_us * SYMBOL_CLOCK_PERIOD) / (CLOCK_FACTOR)-1) / 8;
}

/** @brief Convert given RX power-up delay (ns) to the equivalent register value.
 *
 *  @param[in] rx_power_up_delay_ns  RX power-up delay, in ns.
 *  @return Equivalent 4-bit register value.
 */
static uint8_t rx_pwr_up_delay_ns_to_register(uint16_t rx_power_up_delay_ns)
{
    return ((rx_power_up_delay_ns * SYMBOL_CLOCK_PERIOD) / (CLOCK_FACTOR * CLOCK_FACTOR)) - 1;
}

/** @brief Convert given RX pause time (us) to the equivalent register value.
 *
 *  @param[in] rx_pause_time_pll_cycles  Receiver pause time, in us.
 *  @return Equivalent 8-bit register value.
 */
static uint8_t rx_pause_time_pll_cycles_to_register(uint32_t rx_pause_time_pll_cycles)
{
    int16_t raw_value      = (rx_pause_time_pll_cycles / 4) - 1;
    uint8_t register_value = clamp_8_bit_value(raw_value);

    return register_value;
}

/** @brief Convert given power-up delay (ns) to the equivalent register value.
 *
 *  @param[in] pwr_up_delay_ns  Power-up delay, in ns.
 *  @return Equivalent 8-bit register value.
 */
static uint8_t pwr_up_delay_ns_to_register(uint32_t pwr_up_delay_ns)
{
    return (pwr_up_delay_ns * SYMBOL_CLOCK_PERIOD) / (4 * CLOCK_FACTOR * CLOCK_FACTOR);
}

/** @brief Clamp 8-bit max and min to the given value.
 *
 *  @param[in] value  Value to be clamped.
 *  @return Clamped value, if necessary.
 */
static uint8_t clamp_8_bit_value(int16_t value)
{
    if (value > 255) {
        return 255;
    } else if (value < 0) {
        return 0;
    }
    return (uint8_t)value;
}

#ifdef __cplusplus
}
#endif

#endif /* SR_API_H_ */
