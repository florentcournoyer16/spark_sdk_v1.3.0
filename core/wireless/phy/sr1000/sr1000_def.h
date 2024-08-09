/** @file sr1000_def.h
 *  @brief SR1000 API definitions
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef API_SR1000_API_DEF_H_
#define API_SR1000_API_DEF_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include "sr_reg.h"

typedef uint8_t uint8_reg;
typedef uint16_t uint16_reg;

/** @brief 16 bit union for proper clear/set access.
 *
 *  @note This contains the register bits in order as seen by the radio.
 *        Use MOV2MASK/MASk2VAL macros to access.
 */
typedef union reg_16_bit {
    uint16_reg short_val;
    uint8_t bytes[2];
} reg_16_bit_t;

/** @brief 32 bit union for proper clear/set access.
 *
 *  @note This 32-bit value is 2 16-bit values which AND/OR bits in a 16-bit register.
 */
typedef union radio_reg_internal_16 {
    uint32_t reg_32_u;
    uint16_reg reg_value[2];
} radio_reg_internal_16_t;

/* GENERAL REG MACRO **********************************************************/
#define SET_REGISTER(bits, value)   _SET_REG_VALUE(_CLEAR_OFFSET_16(bits), _SET_OFFSET_16(MOV2MASK(value, bits)))
#define SET_REGISTER_8(bits, value) _SET_REG_VALUE(_CLEAR_OFFSET_8(bits), _SET_OFFSET_8(MOV2MASK(value, bits)))

/* INTERNAL MACRO *************************************************************/
#define _SET_REG_VALUE(reset, val) ((reset) | (val))
#define _SET_OFFSET_8(x)   ((x) << FIRST_BYTE_OFFSET)
#define _CLEAR_OFFSET_8(x) ((x) << SECOND_BYTE_OFFSET)

#define _SET_OFFSET_16(x)   ((x) << SHORT_FIRST_BYTE_OFFSET)
#define _CLEAR_OFFSET_16(x) ((x) << SHORT_SECOND_BYTE_OFFSET)

/** @brief Clear MACRO enum.
 *
 *  @note These should be used when one want to reset
 *        every bit of the shadow register.
 */
typedef enum clear_register {
    INT_FLAG_ENABLE_CLEAR = _CLEAR_OFFSET_16(REG16_INTFLG_INT_BITS),
    TIMER_CFG_CLEAR       = _CLEAR_OFFSET_8(0xFF),
    RADIO_ACTIONS_CLEAR   = _CLEAR_OFFSET_8(0xFF),
    PACKET_CFG_CLEAR      = _CLEAR_OFFSET_8(0xFF),
    MAIN_MODEM_FEAT_CLEAR = _CLEAR_OFFSET_8(0xFF),
    TX_PARAM_CLEAR        = _CLEAR_OFFSET_8(0xFF),
    CTRL_PERIPH_CLEAR     = _CLEAR_OFFSET_8(0xFF),
    DLL_TUNING_CLEAR      = _CLEAR_OFFSET_8(0xFF)
} clear_register_t;

/******************************************************************************/
/*                          IRQ event , 0x00 - 0x01                           */
/******************************************************************************/
/** @brief Radio's interrupt events enumeration.
 */
typedef enum radio_events {
                      /*!< Packet beginning interrupt.
                       *  Set when radio transmit or detect a sync word in reception.
                       */
    PACKET_BEGIN_IT = BIT16_PKBEGINI,
                      /*!< Receiver timeout interrupt.
                       *  Set when radio is idle and in RX mode and the selected timeout
                       *  counter reaches or exceed the value specified in the RX_PERIOD
                       *  register.
                       */
    RX_TIMEOUT_IT   = BIT16_RXTIMEOI,
                      /*!< Packet transmission end interrupt.
                       *  Set when the radio complete a transmission.
                       */
    TX_END_IT       = BIT16_TXENDI,
                     /*!< New packet reception interrupt.
                      *  Set when radio completes the reception of a frame into the RX FIFO.
                      */
    NEW_PACKET_IT   = BIT16_NEWPKTI,
                      /*!< New packet address field match interrupt.
                       *  Set when the radio completes the reception of a frame with
                       *  its address field matching the address field defined in register
                       *  LOCAL_ADDR.
                       */
    ADDR_MATCH_IT   = BIT16_ADDRMATI,
                      /*!< New broadcast packet reception end interrupt.
                       *  Set when radio completes the reception of a frame with address
                       *  field's lower byte set to 0xFF.
                       */
    BROADCAST_IT    = BIT16_BRDCASTI,
                      /*!< Set when radio complete the reception of a frame with a CRC that
                       *  matches the computed one during reception.
                       */
    CRC_PASS_IT     = BIT16_CRCPASSI,
                      /*!< Crystal oscillator timer interrupt.
                       *  Set when the crystal-clocked wake up timer's count reaches the value
                       *  in register field SLPPERIOD while the device is asleep at a sleep level
                       *  for which the DC/DC converter is off.
                       */
    XO_TIMEOUT_IT   = BIT16_XOTIMERI,
                      /*!< Wake-up interrupt.
                       *  Set when radio finishes waking up from any sleep level.
                       */
    WAKEUP_IT       = BIT16_WAKEUPI,
                      /*!< Carrier sensing check failure interrupt.
                       *  Set when the carrier sensing function detects too much energy in
                       *  the wireless channel before transmission.
                       */
    CSC_FAIL_IT     = BIT16_CSCFAILI,
                      /*!< TX FIFO underflow interrupt.
                       *  Set when the TX FIFO is not filled on time with the data required
                       *  for transmitting the current frame. This flag only signal that the
                       *  last frame transmission was aborted and the frame to transmit came
                       *  out of the chip incomplete.
                       */
    TX_UNDERFLOW_IT = BIT16_TXUDRFLI,
                      /*!< Reception buffer overflow interrupt.
                       *  Set when the RX FIFO is already full at the moment the radio intends
                       *  to push into the FIFO a byte from a frame being received.
                       */
    RX_OVRFLOW_IT   = BIT16_RXOVRFLI,
                      /*!< TX FIFO overflow interrupt.
                       *  Set when the SPI master writes another byte into the TX FIFO when
                       *  the latter is already full.
                       */
    TX_OVRFLOW_IT   = BIT16_TXOVRFLI,
                      /*!< Data buffer load threshold interrupt.
                       *  Reflects whether the load of the current direction's buffer is passed
                       *  the defined interrupt threshold. This function's main intended purpose
                       *  is to warn the on-board controller that either a TX FIFO underflow
                       *  or a RX FIFO overflow is imminent if it doesn't intervene.
                       */
    BUF_LOAD_TH_IT  = BIT16_BUFLOADI,
                      /*!< Data buffer stop interrupt
                       *  Work in a similar way to BUF_LOAD_TH_IT and can serves to prevents its
                       *  opposite problem.
                       */
    BUF_LOAD_STP_IT = BIT16_BUFSTOPE,
    NO_IRQ          = 0
} radio_events_t;

/** @brief IRQ enable flag configuration enum.
 */
typedef enum irq_enable_cfg_t {
    PKBEGIN_IT_ENABLE     = _SET_OFFSET_16(BIT16_PKBEGINE),
    PKBEGIN_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_PKBEGINE),
    RXTIMEO_IT_ENABLE     = _SET_OFFSET_16(BIT16_RXTIMEOE),
    RXTIMEO_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_RXTIMEOE),
    TXEND_IT_ENABLE       = _SET_OFFSET_16(BIT16_TXENDE),
    TXEND_IT_DISABLE      = _CLEAR_OFFSET_16(BIT16_TXENDE),
    NEWPKT_IT_ENABLE      = _SET_OFFSET_16(BIT16_NEWPKTE),
    NEWPKT_IT_DISABLE     = _CLEAR_OFFSET_16(BIT16_NEWPKTE),
    ADDRMAT_IT_ENABLE     = _SET_OFFSET_16(BIT16_ADDRMATE),
    ADDRMAT_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_ADDRMATE),
    BRDCAST_IT_ENABLE     = _SET_OFFSET_16(BIT16_BRDCASTE),
    BRDCAST_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_BRDCASTE),
    CRC_PASS_IT_ENABLE    = _SET_OFFSET_16(BIT16_CRCPASSE),
    CRC_PASS_IT_DISABLE   = _CLEAR_OFFSET_16(BIT16_CRCPASSE),
    XO_TIMEOUT_IT_ENABLE  = _SET_OFFSET_16(BIT16_XOTIMERE),
    XO_TIMEOUT_IT_DISABLE = _CLEAR_OFFSET_16(BIT16_XOTIMERE),
    WAKEUP_IT_ENABLE      = _SET_OFFSET_16(BIT16_WAKEUPE),
    WAKEUP_IT_DISABLE     = _CLEAR_OFFSET_16(BIT16_WAKEUPE),
    CSC_FAIL_IT_ENABLE    = _SET_OFFSET_16(BIT16_CSCFAILE),
    CSC_FAIL_IT_DISABLE   = _CLEAR_OFFSET_16(BIT16_CSCFAILE),
    TXUDRFL_IT_ENABLE     = _SET_OFFSET_16(BIT16_TXUDRFLE),
    TXUDRFL_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_TXUDRFLE),
    RXOVRFL_IT_ENABLE     = _SET_OFFSET_16(BIT16_RXOVRFLE),
    RXOVRFL_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_RXOVRFLE),
    TXOVRFL_IT_ENABLE     = _SET_OFFSET_16(BIT16_TXOVRFLE),
    TXOVRFL_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_TXOVRFLE),
    BUFLOAD_IT_ENABLE     = _SET_OFFSET_16(BIT16_BUFLOADE),
    BUFLOAD_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_BUFLOADE),
    BUFSTOP_IT_ENABLE     = _SET_OFFSET_16(BIT16_BUFSTOPE),
    BUFSTOP_IT_DISABLE    = _CLEAR_OFFSET_16(BIT16_BUFSTOPE)
} irq_enable_cfg_t;

/** @brief IRQ event IRQPOLAR field enum.
 */
typedef enum irq_polarity {
    IRQ_ACTIVE_LOW  = _CLEAR_OFFSET_16(BIT16_IRQPOLAR), /*!< Interrupt pin active in LOW state */
    IRQ_ACTIVE_HIGH = _SET_OFFSET_16(BIT16_IRQPOLAR)    /*!< Interrupt pin active in HIGH state */
} irq_polarity_t;

/******************************************************************************/
/*                        Buffer status, 0x02 - 0x03                          */
/******************************************************************************/
/** @brief Transmission buffer status.
 */
typedef enum txirqen {
    /* 8-bits access */
    TXIRQEN_DISABLE = _CLEAR_OFFSET_8(BIT_TXIRQEN),
    TXIRQEN_ENABLE  = _SET_OFFSET_8(BIT_TXIRQEN)
} txirqen_t;

/** @brief Reception buffer status.
 */
typedef enum rxirqen {
    /* 8-bits access */
    RXIRQEN_DISABLE = _CLEAR_OFFSET_8(BIT_RXIRQEN),
    RXIRQEN_ENABLE  = _SET_OFFSET_8(BIT_RXIRQEN)
} rxirqen_t;

/******************************************************************************/
/*                        Sleep configuration, 0x04                           */
/******************************************************************************/
/** @brief Radio's sleep level.
 *
 *  @note Sleep depth at which the radio will go when asleep.
 */
typedef enum sleep_lvl {
    SLEEP_IDLE    = _CLEAR_OFFSET_8(BITS_SLPDEPTH), /*!< Radio sleep level IDLE */
    SLEEP_SHALLOW = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_SLPDEPTH), MOV2MASK(0b01, BITS_SLPDEPTH)), /*!< Radio sleep level SHALLOW */
    SLEEP_DEEP    = _SET_OFFSET_8(BITS_SLPDEPTH)  /*!< Radio sleep level DEEP */
} sleep_lvl_t;

/** @brief Radio's sleep event enumeration.
 *
 *  @note Events that will trigger an automatic sleeping when completed.
 */
typedef enum sleep_events {
    SLEEP_RX_TIMEOUT = _SET_OFFSET_8(BIT_SLPRXTO),  /*!< Sleep event RX timeout */
    SLEEP_TX_END     = _SET_OFFSET_8(BIT_SLPTXEND), /*!< Sleep event TX end */
    SLEEP_RX_END     = _SET_OFFSET_8(BIT_SLPRXEND), /*!< Sleep event RX end */
    SLEEP_ADDR_MATCH = _SET_OFFSET_8(BIT_SLPMATCH), /*!< Sleep event address match */
    SLEEP_BROADCAST  = _SET_OFFSET_8(BIT_SLPBRDCA), /*!< Sleep event broadcast */
    SLEEP_CSC_FAIL   = _SET_OFFSET_8(BIT_SLPNOISY), /*!< Sleep event CSC fail */
    SLEEP_NO_EVENT   = 0,                           /*!< No sleep event */
    SLEEP_EVENT_CLEAR = _CLEAR_OFFSET_8(BITS8(5, 0)),
} sleep_events_t;

/******************************************************************************/
/*                     Timer configuration register, 0x05                     */
/******************************************************************************/

/** @brief Timer configuration AUTOWAKE field enum.
 */
 typedef enum auto_wake {
    /* 8-bits access */
    AUTOWAKE_UP_DISABLE = _CLEAR_OFFSET_8(BIT_AUTOWAKE), /*!< Automatically wake-up disable */
    AUTOWAKE_UP_ENABLE  = _SET_OFFSET_8(BIT_AUTOWAKE)    /*!< Automatically wake-up enable using the wake-up timer */
} auto_wake_t;

/** @brief Timer configuration WAKEONCE field enum.
 */
typedef enum wake_up_once {
    /* 8-bits access */
    WAKE_UP_ONCE_DISABLE = _CLEAR_OFFSET_8(BIT_WAKEONCE),
    WAKE_UP_ONCE_ENABLE  = _SET_OFFSET_8(BIT_WAKEONCE)
} wake_up_once_t;

/** @brief Timer configuration SYNATEND field enum.
 */
typedef enum sync_at_end_tx {
    /* 8-bits access */
    SYNC_AT_END_DISABLE = _CLEAR_OFFSET_8(BIT_SYNATEND),
    SYNC_AT_END_ENABLE  = _SET_OFFSET_8(BIT_SYNATEND)
} sync_packet_tx_t;

/** @brief Timer configuration SYNTXSTA field enum.
 */
typedef enum sync_tx_beg {
    /* 8-bits access */
    SYNC_TX_BEG_DISABLE = _CLEAR_OFFSET_8(BIT_SYNTXSTA),
    SYNC_TX_BEG_ENABLE  = _SET_OFFSET_8(BIT_SYNTXSTA)
} sync_tx_beg_t;

/** @brief Timer configuration SYNRXSTA field enum.
 */
typedef enum sync_rx_beg {
    /* 8-bits access */
    SYNC_RX_BEG_DISABLE = _CLEAR_OFFSET_8(BIT_SYNRXSTA),
    SYNC_RX_BEG_ENABLE  = _SET_OFFSET_8(BIT_SYNRXSTA)
} sync_rx_beg_t;

/** @brief Timer configuration SYNMATCH field enum.
 */
typedef enum sync_addr_match {
    /* 8-bits access */
    SYNC_ADDR_MATCH_DISABLE = _CLEAR_OFFSET_8(BIT_SYNMATCH),
    SYNC_ADDR_MATCH_ENABLE  = _SET_OFFSET_8(BIT_SYNMATCH)
} sync_addr_match_t;

/** @brief Timer configuration SYNBRDCA field enum.
 */
typedef enum sync_bcast {
    /* 8-bits access */
    SYNC_BCAST_DISABLE = _CLEAR_OFFSET_8(BIT_SYNBRDCA),
    SYNC_BCAST_ENABLE  = _SET_OFFSET_8(BIT_SYNBRDCA)
} sync_bcast_t;

/** @brief Timer configuration SYNRXCRC field enum.
 */
typedef enum sync_rxcrc {
    /* 8-bits access */
    SYNC_RXCRC_DISABLE = _CLEAR_OFFSET_8(BIT_SYNRXCRC),
    SYNC_RXCRC_ENABLE  = _SET_OFFSET_8(BIT_SYNRXCRC)
} sync_rxcrc_t;

/******************************************************************************/
/*                      Disables and clock control, 0x0E                      */
/******************************************************************************/

/** @brief Peripherals controls STDSPI field enum.
 */
typedef enum std_spi {
    /* 8-bits access */
    SPI_FAST     = _CLEAR_OFFSET_8(BIT_STDSPI),
    SPI_STANDARD = _SET_OFFSET_8(BIT_STDSPI)
} std_spi_t;

/** @brief Peripherals controls AUTOFLUSHDIS field enum.
 */
typedef enum flush_dis {
    /* 8-bits access */
    AUTO_FLUSH_DISABLE = _SET_OFFSET_8(BIT_AUTOFLUSHDIS),
    AUTO_FLUSH_ENABLE  = _CLEAR_OFFSET_8(BIT_AUTOFLUSHDIS)
} flush_dis_t;

/** @brief Peripherals controls 1VSWDIS field enum.
 */
typedef enum one_vsw_dis {
    /* 8-bits access */
    VSWDIS_DISABLE = _SET_OFFSET_8(BIT_1VSWDIS),
    VSWDIS_ENABLE  = _CLEAR_OFFSET_8(BIT_1VSWDIS)
} one_vsw_dis_t;

/** @brief Peripherals controls DCDCDIS field enum.
 */
typedef enum dcdc_dis {
    /* 8-bits access */
    DCDCDIS_DISABLE = _SET_OFFSET_8(BIT_DCDCDIS),
    DCDCDIS_ENABLE  = _CLEAR_OFFSET_8(BIT_DCDCDIS)
} dcdc_dis_t;

/** @brief Peripherals controls PLLDIS field enum.
 */
typedef enum pll_dis {
    /* 8-bits access */
    PLL_DISABLE = _SET_OFFSET_8(BIT_PLLDIS),
    PLL_ENABLE  = _CLEAR_OFFSET_8(BIT_PLLDIS)
} pll_dis_t;

/** @brief Peripherals controls SYMBCSRC field enum.
 */
typedef enum symbol_rate_clk_source {
    /* 8-bits access */
    SYMBSRC_INTERNAL = _CLEAR_OFFSET_8(BIT_SYMBCSRC),
    SYMBSRC_EXTERNAL = _SET_OFFSET_8(BIT_SYMBCSRC)
} symbol_rate_clk_source_t;

/** @brief Peripherals controls XTALCSRC field enum.
 */
typedef enum xtal_osc_clk_source {
    /* 8-bits access */
   XTALSRC_EXTERNAL = _SET_OFFSET_8(BIT_XTALCSRC),
   XTALSRC_INTERNAL = _CLEAR_OFFSET_8(BIT_XTALCSRC)
} xtal_osc_clk_source_t;

/** @brief Peripherals controls OUTPXTAL field enum.
 */
typedef enum output_xtal {
    /* 8-bits access */
   OUTPUTXTAL_DISABLE = _CLEAR_OFFSET_8(BIT_OUTPXTAL),
   OUTPUTXTAL_ENABLE  = _SET_OFFSET_8(BIT_OUTPXTAL)
} output_xtal_t;

/******************************************************************************/
/*                      Transmitter parameters, 0x1C                          */
/******************************************************************************/

/** @brief Transmitter parameters HOLDTXON field enum.
 */
typedef enum hold_tx_on {
    /* 8-bits access */
   HOLD_TX_ON_DISABLE = _CLEAR_OFFSET_8(BIT_HOLDTXON),
   HOLD_TX_ON_ENABLE  = _SET_OFFSET_8(BIT_HOLDTXON)
} hold_tx_on_t;

/** @brief Transmitter parameters RNDPHASE field enum.
 */
typedef enum rnd_phase {
    /* 8-bits access */
   RND_PHASE_DISABLE = _CLEAR_OFFSET_8(BIT_RNDPHASE),
   RND_PHASE_ENABLE  = _SET_OFFSET_8(BIT_RNDPHASE)
} rnd_phase_t;

/** @brief Transmitter parameters IFFILTEN field enum.
 */
typedef enum if_filt_en {
    /* 8-bits access */
   IF_FILT_DISABLE = _CLEAR_OFFSET_8(BIT_IFFILTEN),
   IF_FILT_ENABLE  = _SET_OFFSET_8(BIT_IFFILTEN)
} if_filt_en_t;

/** @brief Radio's TX power level enumeration in dBFs.
 */
typedef enum tx_power {
    TX_PWR_0_0_DB = _CLEAR_OFFSET_8(BITS_TXPOWER),
    TX_PWR_0_6_DB = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_TXPOWER), MOV2MASK(0b01, BITS_TXPOWER)),
    TX_PWR_1_2_DB = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_TXPOWER), MOV2MASK(0b10, BITS_TXPOWER)),
    TX_PWR_1_8_DB = _SET_OFFSET_8(BITS_TXPOWER),

    /* Config Spectrum Preset */
    TX_PWR_MINUS_9_9_DBFS = 0,        /*!< TX power -9.9 dBFs             */
    TX_PWR_MINUS_9_0_DBFS,            /*!< TX power -9.0 dBFs             */
    TX_PWR_MINUS_8_1_DBFS,            /*!< TX power -8.1 dBFs             */
    TX_PWR_MINUS_7_2_DBFS,            /*!< TX power -7.2 dBFs             */
    TX_PWR_MINUS_6_3_DBFS,            /*!< TX power -6.3 dBFs             */
    TX_PWR_MINUS_5_4_DBFS,            /*!< TX power -5.4 dBFs             */
    TX_PWR_MINUS_4_5_DBFS,            /*!< TX power -4.5 dBFs             */
    TX_PWR_MINUS_3_6_DBFS,            /*!< TX power -3.6 dBFs             */
    TX_PWR_MINUS_2_7_DBFS,            /*!< TX power -2.7 dBFs             */
    TX_PWR_MINUS_1_8_DBFS,            /*!< TX power -1.8 dBFs             */
    TX_PWR_MINUS_0_9_DBFS,            /*!< TX power -0.9 dBFs             */
    TX_PWR_MINUS_0_0_DBFS,            /*!< TX power -0.0 dBFs             */
    TX_PWR_MINUS_9_9_DBFS_RANGING,    /*!< TX power -9.9 dBFs for ranging */
    TX_PWR_MINUS_9_0_DBFS_RANGING,    /*!< TX power -9.0 dBFs for ranging */
    TX_PWR_MINUS_8_1_DBFS_RANGING,    /*!< TX power -8.1 dBFs for ranging */
    TX_PWR_MINUS_7_2_DBFS_RANGING,    /*!< TX power -7.2 dBFs for ranging */
    TX_PWR_MINUS_6_3_DBFS_RANGING,    /*!< TX power -6.3 dBFs for ranging */
    TX_PWR_MINUS_5_4_DBFS_RANGING,    /*!< TX power -5.4 dBFs for ranging */
    TX_PWR_MINUS_4_5_DBFS_RANGING,    /*!< TX power -4.5 dBFs for ranging */
    TX_PWR_MINUS_3_6_DBFS_RANGING,    /*!< TX power -3.6 dBFs for ranging */
    TX_PWR_MINUS_2_7_DBFS_RANGING,    /*!< TX power -2.7 dBFs for ranging */
    TX_PWR_MINUS_1_8_DBFS_RANGING,    /*!< TX power -1.8 dBFs for ranging */
    TX_PWR_MINUS_0_9_DBFS_RANGING,    /*!< TX power -0.9 dBFs for ranging */
    TX_PWR_MINUS_0_0_DBFS_RANGING,    /*!< TX power -0.0 dBFs for ranging */
    TX_PWR_LOW_OUTPUT_POWER_RANGING,  /*!< Recommended short-range power setting */
    TX_PWR_HIGH_OUTPUT_POWER_RANGING, /*!< Recommended long-range power setting */
} tx_power_t;

/******************************************************************************/
/*                          Delay line tuning, 0x1D                           */
/******************************************************************************/

/** @brief Delay line tuning INTEGLEN field enum.
 */
typedef enum integlen {
    /* 8-bits access */
   INTEGLEN_FULL  = _CLEAR_OFFSET_8(BIT_INTEGLEN),
   INTEGLEN_SHORT = _SET_OFFSET_8(BIT_INTEGLEN)
} integlen_t;

/** @brief Delay line tuning ECO field enum.
 */
typedef enum eco {
    /* 8-bits access */
   ECO_DISABLE = _CLEAR_OFFSET_8(BIT_ECO),
   ECO_ENABLE  = _SET_OFFSET_8(BIT_ECO)
} eco_t;

#define CLAMP_DL_TUNE(dl_tune) (((dl_tune) > 0xF) ? 0xF : (dl_tune))
#define DLTUNE_RAW_TO_REG(dl_tune) _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_DLTUNE), MOV2MASK(CLAMP_DL_TUNE(dl_tune), BITS_DLTUNE))
#define INTEGGAIN_RAW_TO_REG(integgain) _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_INTEGGAIN), MOV2MASK(integgain, BITS_INTEGGAIN))
/******************************************************************************/
/*              Power status and actions register, 0x1F                       */
/******************************************************************************/

/** @brief Power status and actions CALIBRAT field enum.
 */
typedef enum calibrat {
    /* 8-bits access */
    CALIBRATION_DCRO_DISABLE = _CLEAR_OFFSET_8(BIT_CALIBRAT), /*!< Disable the digitally-controlled ring oscillator calibration */
    CALIBRATION_DCRO_ENABLE  = _SET_OFFSET_8(BIT_CALIBRAT)    /*!< Trigger the calibration block counting process of */
} calibrat_t;                                                 /*   the digitally-controlled ring oscillator */

/** @brief Power status and actions SKIPWAKE field enum.
 */
typedef enum skipwake {
    /* 8-bits access */
    SKIP_WAKE_DISABLE = _CLEAR_OFFSET_8(BIT_SKIPWAKE), /*!< Don't skip the next wake up event */
    SKIP_WAKE_ENABLE  = _SET_OFFSET_8(BIT_SKIPWAKE)    /*!< Skip the automated wake-up process and remain asleep */
} skipwake_t;

/** @brief Power status and actions RXMODE field enum.
 */
typedef enum rxmode {
    /* 8-bits access */
    RX_MODE_ENABLE_TRANSMISSION = _CLEAR_OFFSET_8(BIT_RXMODE), /*!< Configure the device for frame transmission */
    RX_MODE_ENABLE_RECEPTION    = _SET_OFFSET_8(BIT_RXMODE)    /*!< Configure the device for frame reception */
} rxmode_t;

/** @brief Power status and actions STARTTX field enum.
 */
typedef enum starttx {
    /* 8-bits access */
    START_TX_NO_EFFECT = _CLEAR_OFFSET_8(BIT_STARTTX), /*!< Has no effect on the register */
    START_TX           = _SET_OFFSET_8(BIT_STARTTX)    /*!< Schedule a frame transmission the next time the radio is awake */
} starttx_t;

/** @brief Power status and actions INITTIME field enum.
 */
typedef enum  inittimer {
    /* 8-bits access */
    INIT_TIMER_NO_EFFECT                = _CLEAR_OFFSET_8(BIT_INITTIME), /*!< Has no effect on the register */
    INIT_TIMER_RESET_BOTH_WAKE_UP_TIMER = _SET_OFFSET_8(BIT_INITTIME)    /*!< Reset both wake-up timers to 0x0000 */
} inittimer_t;

/** @brief Power status and actions GOTOSLP field enum.
 */
typedef enum gotoslp {
    /* 8-bits access */
    GO_TO_SLEEP_WAKE_UP = _CLEAR_OFFSET_8(BIT_GOTOSLP), /*!< Wake up the radio only if bit AUTOWAKE of register 0x05 is cleared */
    GO_TO_SLEEP         = _SET_OFFSET_8(BIT_GOTOSLP)    /*!< Put the radio to sleep if it is awake and idle */
} gotoslp_t;

/** @brief Power status and actions FLUSHRX field enum.
 */
typedef enum flushrx {
    /* 8-bits access */
    FLUSH_RX_NO_EFFECT       = _CLEAR_OFFSET_8(BIT_FLUSHRX), /*!< Has no effect on register */
    FLUSH_RX_RESET_RX_BUFFER = _SET_OFFSET_8(BIT_FLUSHRX)    /*!< Flush and reset the reception buffer */
} flushrx_t;

/** @brief Power status and actions FLUSHTX field enum.
 */
typedef enum flushtx {
    /* 8-bits access */
    FLUSH_TX_NO_EFFECT       = _CLEAR_OFFSET_8(BIT_FLUSHTX), /*!< Has no effect on register */
    FLUSH_TX_RESET_TX_BUFFER = _SET_OFFSET_8(BIT_FLUSHTX)    /*!< Flush and reset the transmission buffer */
} flushtx_t;

/******************************************************************************/
/*                      Receiver last waited time, 0x24                       */
/******************************************************************************/

/** @brief Receiver last waited time RXWAISRC field enum.
 */
typedef enum rx_wait_source {
                              /*!< Default receiver idle time counter reusing the
                               *   output shift register of the modem is used
                               */
    RX_WAIT_SOURCE_DEFAULT  = _CLEAR_OFFSET_8(BIT_RXWAISRC),
                              /*!< Modem’s wake-up timer value is the one that
                               * gets stored in the RX WAITED register.
                               */
    RX_WAIT_SOURCE_REGISTER = _SET_OFFSET_8(BIT_RXWAISRC)
} rx_wait_source_t;

/******************************************************************************/
/*                       Modem debug features, 0x2B                           */
/******************************************************************************/

/** @brief Override modem register field.
 */
typedef enum modem_debug_override {
    OVERRIDE_DISABLE = _CLEAR_OFFSET_8(BIT_OVERRIDE), /*!< Do not override modem */
    OVERRIDE_ENABLE  = _SET_OFFSET_8(BIT_OVERRIDE)    /*!< Override modem */
} modem_debug_override_t;

/** @brief Manual receiver phase selection / Phase tracking register field.
 */
typedef enum manuphase {
                                        /*!< Use integration window 1 for decoding of the received packet
                                         *   and propagate to the STRM_OUT pins
                                         */
    MANUPHASE_INTEG_WINDOW_1         = _CLEAR_OFFSET_8(BITS_MANUPHASE),
                                        /*!< Use integration window 2 for decoding of the received packet
                                         *   and propagate to the STRM_OUT pins
                                         */
    MANUPHASE_INTEG_WINDOW_2         = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_MANUPHASE), MOV2MASK(0b01, BITS_MANUPHASE)),
                                        /*!< Use integration window 3 for decoding of the received packet
                                         *   and propagate to the STRM_OUT pins
                                         */
    MANUPHASE_INTEG_WINDOW_3         = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_MANUPHASE), MOV2MASK(0b10, BITS_MANUPHASE)),
                                        /*!< Use integration window 4 for decoding of the received packet
                                         *   and propagate to the STRM_OUT pins
                                         */
    MANUPHASE_INTEG_WINDOW_4         = _SET_OFFSET_8(BITS_MANUPHASE),
    MANUPHASE_PHASE_TRACKING_DISABLE = _CLEAR_OFFSET_8(BITS_MANUPHASE), /*!< Disable phase tracking */
    MANUPHASE_PHASE_TRACKING_ENABLE  = _SET_OFFSET_8(BITS_MANUPHASE), /*!< Enable phase tracking */
} manuphase_t;

/** @brief Manual bit decision threshold enum.
 */
typedef enum man_bit_thresh {
                                    /* Radio will select the bit decision threshold */
    MANUAL_BIT_THRESHOLD_DISABLE = _CLEAR_OFFSET_8(BIT_MANBITHR),
                                    /* Use the bit decision threshold in the BITTHRADJ field of this register */
    MANUAL_BIT_THRESHOLD_ENABLE  = _SET_OFFSET_8(BIT_MANBITHR)
} man_bit_thresh_t;

/******************************************************************************/
/*                          Modem Main Features , 0x2C                        */
/******************************************************************************/
/** @brief Radio's modulation type enumeration.
 */
typedef enum modulation {
    MODULATION_OOK     = _CLEAR_OFFSET_8(BITS_MODULATION),      /*!< Frame modulation OOK (On-off keying) */
    MODULATION_IOOK    = SET_REGISTER_8(BITS_MODULATION, 0b01), /*!< Frame modulation OOK (On-off keying) */
    MODULATION_PPM     = SET_REGISTER_8(BITS_MODULATION, 0b10), /*!< Frame modulation PPM (Pulse-position modulation) */
    MODULATION_2BITPPM = _SET_OFFSET_8(BITS_MODULATION),        /*!< Frame modulation 2BITPPM(2-bit Pulse-position modulation) */
} modulation_t;

/** @brief Radio's Forward error correction enumeration.
 */
typedef enum fec_level {
    FEC_LVL_0 = _CLEAR_OFFSET_8(BITS_FECLEVEL),      /*!< Forward error correction ratio 1.00 */
    FEC_LVL_1 = SET_REGISTER_8(BITS_FECLEVEL, 0b01), /*!< Forward error correction ratio 1.33 */
    FEC_LVL_2 = SET_REGISTER_8(BITS_FECLEVEL, 0b10), /*!< Forward error correction ratio 1.66 */
    FEC_LVL_3 = _SET_OFFSET_8(BITS_FECLEVEL)         /*!< Forward error correction ratio 2.00 */
} fec_level_t;

#define FEC_TYPE_TO_RAW(fec_level) MASK2VAL(fec_level, BITS_FECLEVEL)

/** @brief Inter-symbol interference mitigation enum
 */
typedef enum isi_mitig {
    ISI_MITIG_0 = _CLEAR_OFFSET_8(BITS_ISIMITIG),      /*!< Inter-symbol interference mitigation level 0 */
    ISI_MITIG_1 = SET_REGISTER_8(BITS_ISIMITIG, 0b01), /*!< Inter-symbol interference mitigation level 1 */
    ISI_MITIG_2 = SET_REGISTER_8(BITS_ISIMITIG, 0b10), /*!< Inter-symbol interference mitigation level 2 */
    ISI_MITIG_3 = _SET_OFFSET_8(BITS_ISIMITIG)         /*!< Inter-symbol interference mitigation level 3 */
} isi_mitig_t;

/** @brief Automatic transmission enumeration
 */
typedef enum auto_tx {
                      /*!< Disable automatic transmission and follow the one
                       *  based on the START_TX of register 0x1F
                       */
    AUTO_TX_DISABLE = _CLEAR_OFFSET_8(BIT_AUTOTX),
                      /*!< Enable automatic transmission on wake-up event of predetermined pattern */
    AUTO_TX_ENABLE  = _SET_OFFSET_8(BIT_AUTOTX)
} auto_tx_t;

/** @brief Enable/Disable Automatic frame reply enumeration
 */
typedef enum auto_reply {
    AUTO_REPLY_DISABLE =  _CLEAR_OFFSET_8(BIT_AUTORPLY), /*!< Disable automatic reply on received frame */
    AUTO_REPLY_ENABLE  =  _SET_OFFSET_8(BIT_AUTORPLY)    /*!< Enable automatic reply on received frame depending on certain condition */
} auto_reply_t;

/******************************************************************************/
/*                          Clear-to-send, 0x2D                               */
/******************************************************************************/

/** @brief Clear-to-send IDLERXPWR field enum.
 */
typedef enum rx_idle_pwr {
    /*!< The receiver will remain on for only 8 symbol clock cycle after the RXPUDELAY before turning off again */
    RX_IDLE_PWR_MIN  = 0,
    /*!< The receive r will remain on for only 16 symbol clock cycle after the RXPUDELAY before turning off again */
    RX_IDLE_PWR_MED  = MOV2MASK(0b01, BITS_IDLERXPWR),
    /*!< The receive r will always remain on when waiting for preamble */
    RX_IDLE_PWR_HIGH = BITS_IDLERXPWR
} rx_idle_pwr_t;

/******************************************************************************/
/*                 Synchronization word settings , 0x31                       */
/******************************************************************************/

/** @brief Radio's syncword length enumeration.
 */
typedef enum syncword_length {
    SYNCWORD_LENGTH_16 = _CLEAR_OFFSET_8(BIT_SWLENGTH), /*!< Syncword length on 16-bit */
    SYNCWORD_LENGTH_32 = _SET_OFFSET_8(BIT_SWLENGTH)    /*!< Syncword length on 32-bit */
} syncword_length_t;

/******************************************************************************/
/*                      Packet configuration, 0x3E                            */
/******************************************************************************/

/** @brief Packet configuration ADDRFILT field enum.
 */
typedef enum addr_filt {
                                      /*!< Disable address filtering for both receive packet and auto-reply */
    ADDR_FILT_DISABLE               = _CLEAR_OFFSET_8(BITS_ADDRFILT),
                                      /*!< Enable address filtering on receive packet only */
    ADDR_FILT_ON_RX_FRAME_ENABLE    = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_ADDRFILT), MOV2MASK(0b01, BITS_ADDRFILT)),
                                      /*!< Enable auto-reply only when receive packet match the local address
                                       *  and is not the multicast/broadcast one
                                       */
    ADDR_FILT_FOR_AUTO_REPLY_ENABLE = _SET_REG_VALUE(_CLEAR_OFFSET_8(BITS_ADDRFILT), MOV2MASK(0b10, BITS_ADDRFILT)),
                                      /*!< Enable address filtering for auto-reply and received packet */
    ADDR_FILT_ENABLE                = _SET_OFFSET_8(BITS_ADDRFILT),
} addr_filt_t;

/** @brief Radio's address length enumeration.
 */
typedef enum address_length {
    ADDRESS_LENGTH_8  = _CLEAR_OFFSET_8(BIT_ADDRLEN), /*!< Radio address on 8-bit */
    ADDRESS_LENGTH_16 = _SET_OFFSET_8(BIT_ADDRLEN)    /*!< Radio address on 16-bit */
} address_length_t;

/** @brief Frame address field header.
 */
typedef enum addr_hdre {
    ADDR_HEADER_DISABLE = _CLEAR_OFFSET_8(BIT_ADDRHDRE), /*!< Disable frame address field header */
    ADDR_HEADER_ENABLE  = _SET_OFFSET_8(BIT_ADDRHDRE),   /*!< Enable frame address field header */
} addr_hdre_t;

/** @brief Payload size field header.
 */
typedef enum size_hdre {
    SIZE_HEADER_DISABLE = _CLEAR_OFFSET_8(BIT_SIZEHDRE), /*!< Disable payload size field header */
    SIZE_HEADER_ENABLE  = _SET_OFFSET_8(BIT_SIZEHDRE)    /*!< Insert a payload size field before frame payload */
} size_hdre_t;

/** @brief Transmitted frame payload size source.
 */
typedef enum size_src {
    SIZE_SRC_REG_TXPKTSIZE   = _CLEAR_OFFSET_8(BIT_SIZESRC), /*!< Use value of TXPKTSIZE register for the frame payload size*/
    SIZE_SRC_TX_PAYLOAD_SIZE = _SET_OFFSET_8(BIT_SIZESRC),   /*!< Use value in packet header for frame payload size */
} size_src_t;

/** @brief Save frame address field in reception buffer.
 */
typedef enum save_addr {
                        /*!< Disable save of address field */
    SAVE_ADDR_DISABLE = _CLEAR_OFFSET_8(BIT_SAVEADDR),
                        /*!< Enable save of address field of each received frame into the
                         *  reception buffer with the frame payload
                         */
    SAVE_ADDR_ENABLE  = _SET_OFFSET_8(BIT_SAVEADDR)
} save_addr_t;

/** @brief Save frame payload size field in reception buffer.
 */
typedef enum save_size {
                        /*!< Disable saving of payload size */
    SAVE_SIZE_DISABLE = _CLEAR_OFFSET_8(BIT_SAVESIZE),
                        /*!< Enable saving of payload size of each received frame into the
                         *  reception buffer with the frame payload
                         */
    SAVE_SIZE_ENABLE  = _SET_OFFSET_8(BIT_SAVESIZE)
} save_size_t;

/** @brief Transceiver chip rate enumeration.
 */
typedef enum chip_rate_cfg {
                         /*!< 20.48 MHz is the only possible option for SR1000 */
    CHIP_RATE_20_48_MHZ = 0
} chip_rate_cfg_t;

#define DEFAULT_PACKET_CONFIGURATION SET_PACKET_CFG(PACKET_CFG_CLEAR,       \
                                                    ADDR_FILT_ENABLE,       \
                                                    ADDRESS_LENGTH_16,      \
                                                    ADDR_HEADER_ENABLE,     \
                                                    SIZE_HEADER_ENABLE,     \
                                                    SIZE_SRC_REG_TXPKTSIZE, \
                                                    SAVE_ADDR_DISABLE,      \
                                                    SAVE_SIZE_ENABLE)
/******************************************************************************/
/*                                 PLL                                        */
/******************************************************************************/
#define PLL_RATIO               625
#define PLL_FREQ_HZ(chip_rate)  ((chip_rate) == (CHIP_RATE_20_48_MHZ) ? (20480000) : (20480000))
#define PLL_FREQ_KHZ(chip_rate) ((chip_rate) == (CHIP_RATE_20_48_MHZ) ? (20480)    : (20480))
#define PHASE_OFFSET_BYTE_COUNT 1

/******************************************************************************/
/*                             Frame outcome                                  */
/******************************************************************************/
/** @brief Frame outcome enumeration.
 */
typedef enum frame_outcome {
    FRAME_RECEIVED,          /*!< Frame received */
    FRAME_LOST,              /*!< Frame lost */
    FRAME_REJECTED,          /*!< Frame rejected */
    FRAME_SENT_ACK,          /*!< Frame sent and acknowledged */
    FRAME_SENT_ACK_LOST,     /*!< Frame sent and ack is lost*/
    FRAME_SENT_ACK_REJECTED, /*!< Frame sent and ack is rejected */
    FRAME_WAIT               /*!< No frame sent or received */
} frame_outcome_t;

/******************************************************************************/
/*                                  CCA                                       */
/******************************************************************************/
#define CCA_DISABLE           0xFF
#define CCA_DEFAULT_MARGIN    12
#define CCA_MAX_RETRY_TIME_US 50

/******************************************************************************/
/*                                RF CHANNEL                                  */
/******************************************************************************/

#define NB_CHANNEL            10
#define NB_PULSES             12
#define NB_PHASES             4
#define TX_PATTERN_BYTE_COUNT 12
#define MAX_PULSE_WIDTH       7
#define MAX_INTEGGAIN         3

/** @brief Radio's RF channel ID enumeration.
 */
typedef enum rf_channel_id {
    CHANNEL_0 = 0, /*!< RF Channel 0 */
    CHANNEL_1,     /*!< RF Channel 1 */
    CHANNEL_2,     /*!< RF Channel 2 */
    CHANNEL_3,     /*!< RF Channel 3 */
    CHANNEL_4,     /*!< RF Channel 4 */
    CHANNEL_5,     /*!< RF Channel 5 */
    CHANNEL_6,     /*!< RF Channel 6 */
    CHANNEL_7,     /*!< RF Channel 7 */
    CHANNEL_8,     /*!< RF Channel 8 */
    CHANNEL_9,     /*!< RF Channel 9 */
    CHANNEL_ALL    /*!< Reference to all RF channel */
} rf_channel_id_t;

/******************************************************************************/
/*                                  MISC                                      */
/******************************************************************************/
#define MAX_FRAMESIZE     128
#define BROADCAST_ADDRESS 0xFF

/* PRIVATE MACRO **************************************************************/
/** @brief Implementation details for NUM_VAR_ARGS
 */
#define NUM_VA_ARGS_IMPL(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, N, ...) N

/** @brief Macro to get the number of arguments in a call variadic macro call
 *
 *   @param[in]    ...     List of arguments
 *   @retval  Number of variadic arguments in the argument l
 */
#define NUM_VA_ARGS(...) NUM_VA_ARGS_IMPL(__VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

#define OR(...) OR_(__VA_ARGS__)
#define OR_(...) OR_N(NUM_VA_ARGS(__VA_ARGS__), __VA_ARGS__)
#define OR_N(N, ...) OR_N_(N, __VA_ARGS__)
#define OR_N_(N, ...) OR_##N(__VA_ARGS__)
#define OR_1(a, ...)  (a)
#define OR_2(a, ...)  ((a) | OR_1(__VA_ARGS__))
#define OR_3(a, ...)  ((a) | OR_2(__VA_ARGS__))
#define OR_4(a, ...)  ((a) | OR_3(__VA_ARGS__))
#define OR_5(a, ...)  ((a) | OR_4(__VA_ARGS__))
#define OR_6(a, ...)  ((a) | OR_5(__VA_ARGS__))
#define OR_7(a, ...)  ((a) | OR_6(__VA_ARGS__))
#define OR_8(a, ...)  ((a) | OR_7(__VA_ARGS__))
#define OR_9(a, ...)  ((a) | OR_8(__VA_ARGS__))
#define OR_10(a, ...) ((a) | OR_9(__VA_ARGS__))
#define OR_11(a, ...) ((a) | OR_10(__VA_ARGS__))
#define OR_12(a, ...) ((a) | OR_11(__VA_ARGS__))
#define OR_13(a, ...) ((a) | OR_12(__VA_ARGS__))
#define OR_14(a, ...) ((a) | OR_13(__VA_ARGS__))
#define OR_15(a, ...) ((a) | OR_14(__VA_ARGS__))
#define OR_16(a, ...) ((a) | OR_15(__VA_ARGS__))

#define _BIT_TO_SET   0
#define _BIT_TO_CLEAR 1

#endif /* API_SR1000_API_DEF_H_ */
