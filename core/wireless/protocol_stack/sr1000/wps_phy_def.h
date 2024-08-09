/** @file  wps_phy_def.h
 *  @brief Include every definition needed by the WPS PHY layer.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef WPS_PHY_DEF_H_
#define WPS_PHY_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "circular_queue.h"
#include "sr_api.h"
#include "xlayer.h"
#include "wps_def.h"

/* CONSTANTS ******************************************************************/
#define PHY_STATE_Q_SIZE 10 /*!< Queue size for PHY layer state machine. */
#define PHY_MAX_CERTIF_HEADER_SIZE 10 /*!< Queue size for PHY layer state machine. */
#define PHY_CERTIF_BYTE0           0x6F /*!< Max power certification pattern byte 0. */
#define PHY_CERTIF_BYTE1           0x0A /*!< Max power certification pattern byte 1. */

/* TYPES **********************************************************************/
/** @brief Wireless protocol stack PHY Layer input signal.
 */
typedef enum phy_input_signal {
    PHY_SIGNAL_RADIO_IRQ = 0, /*!< PHY Radio IRQ signal */
    PHY_SIGNAL_DMA_CMPLT,     /*!< PHY DMA transfer complete signal */
    PHY_SIGNAL_PREPARE_RADIO, /*!< PHY prepare radio signal */
    PHY_SIGNAL_SYNCING        /*!< PHY syncing signal */
} phy_input_signal_t;

/** @brief Wireless protocol stack PHY Layer output signal.
 */
typedef enum phy_output_signal {
    PHY_SIGNAL_NONE = 0,             /*!< PHY no signal */
    PHY_SIGNAL_PROCESSING,           /*!< PHY processing signal */
    PHY_SIGNAL_YIELD,                /*!< PHY yield signal */
    PHY_SIGNAL_CONFIG_COMPLETE,      /*|< PHY config has been sent */
    PHY_SIGNAL_BLOCKING_CONFIG_DONE, /*|< PHY config has been sent (blocking)*/
    PHY_SIGNAL_CONNECT,              /*!< PHY connect signal */
    PHY_SIGNAL_PREPARE_DONE,         /*!< PHY prepare done signal */
    PHY_SIGNAL_FRAME_SENT_ACK,       /*!< PHY frame sent and ack signal */
    PHY_SIGNAL_FRAME_SENT_NACK,      /*!< PHY frame sent and nack signal */
    PHY_SIGNAL_FRAME_NOT_SENT,       /*!< PHY frame not sent signal */
    PHY_SIGNAL_FRAME_RECEIVED,       /*!< PHY frame received signal */
    PHY_SIGNAL_FRAME_MISSED,         /*!< PHY frame missed signal */
    PHY_SIGNAL_ERROR                 /*!< PHY error signal */
} phy_output_signal_t;

/** @brief WPS PHY instance.
 */
typedef struct wps_phy wps_phy_t;

/** @brief Layer one state machine function pointer type.
 */
typedef void (*wps_phy_state_t)(wps_phy_t *phy);

/** @brief WPS PHY handle function pointer.
 */
typedef void (*wps_phy_handle_t)(wps_phy_t *phy);

/** @brief PHY layer configuration.
 */
typedef struct phy_cfg {
    radio_t *radio;              /*!< Radio instance */
    uint16_t local_address;      /*!< Local address */
    syncword_cfg_t syncword_cfg; /*!< Sync word configuration */
    uint32_t preamble_len;       /*!< Preamble length */
    sleep_lvl_t sleep_lvl;       /*!< Sleep level */
    uint32_t crc_polynomial;     /*!< CRC polynomial */
    chip_rate_cfg_t chip_rate;   /*!< Radio chip rate, only 20.48MHz is supported on SR1XXX */
    uint8_t rx_gain;             /*!< Radio RX gain */
} wps_phy_cfg_t;

/** @brief TX configuration settings for the phy Layer.
 */
typedef struct phy_tx_frame {
    modulation_t *modulation;          /*!< modulation */
    fec_level_t *fec;                  /*!< FEC level */
    uint8_t *cca_threshold;            /*!< Clear Channel Assessment threshold */
    xlayer_frame_t *frame;             /*!< Cross layer frame */
    phy_output_signal_t *signal;       /*!< Wireless protocol stack phy Layer output signal */
    uint8_t payload_size;              /*!< TX payload size (used for cut-through mode) */
    uint16_t cca_retry_time;           /*!< CCA retry time */
    uint8_t cca_max_try_count;         /*!< CCA max try count */
    uint8_t *cca_try_count;            /*!< CCA try count */
    cca_fail_action_t cca_fail_action; /*!< CCA fail action */
} phy_tx_frame_t;

/** @brief RX configuration settings for the phy Layer.
 */
typedef struct phy_rx_frame {
    uint8_t *rx_constgain;       /*!< Receiver constant gain */
    uint32_t *rnsi_raw;          /*!< RNSI in 1/10 dB */
    uint32_t *rssi_raw;          /*!< RSSI in 1/10 dB */
    xlayer_frame_t *frame;       /*!< Cross layer frame */
    phy_output_signal_t *signal; /*!< Wireless protocol stack phy Layer output signal */
    uint8_t payload_size;        /*!< RX payload size (used for cut-through mode) */
    uint8_t header_size;         /*!< RX header size (use when custom protocols are append to mac header) */
} phyl1_rx_frame_t;

/** @brief General configuration settings for the phy Layer.
 */
typedef struct phy_frame_cfg {
    uint16_t *destination_address;  /*!< Main frame destination address */
    uint16_t *source_address;       /*!< Main frame source address */
    rf_channel_t *channel;          /*!< RF channel settings */
    uint16_t *power_up_delay;       /*!< Power up delay */
    uint16_t *rx_timeout;           /*!< RX timeout */
    uint32_t *sleep_time;           /*!< Sleep time in PLL cycles */
    uint16_t *rx_wait_time;         /*!< RX wait time */
    sleep_lvl_t sleep_level;        /*!< Sleep Level */
    radio_actions_t radio_actions;  /*!< Radio actions register fields settings */
    phase_info_t *phase_info;       /*!< Phases information */
    uint8_t *header_size;           /*!< Header size */
} phy_frame_cfg_t;

/** @brief WPS PHY instance.
 */
struct wps_phy {
    phy_input_signal_t input_signal; /*!< Wireless protocol stack phy Layer input signal*/

    phy_output_signal_t signal_main; /*!< Wireless protocol stack phy Layer output signal main*/
    phy_output_signal_t signal_auto; /*!< Wireless protocol stack phy Layer output signal auto*/

    radio_t *radio; /*!< Already initialized radio instance. */

    uint16_t source_address;      /*!< Node source address*/
    auto_reply_t auto_reply_mode; /*!< Autoreply mode*/
    xlayer_t *xlayer_main;        /*!< Main cross layer*/
    xlayer_t *xlayer_auto;        /*!< Auto cross layer*/
    xlayer_cfg_internal_t *config;               /*!< Configuration */

    /* Internal Variables */
    wps_phy_state_t *current_state;                     /*!< Current state machine state*/
    wps_phy_state_t end_state;                          /*!< Current state machine state*/
    circular_queue_t next_states;                       /*!< Next state machine state queue*/
    wps_phy_state_t *next_state_pool[PHY_STATE_Q_SIZE]; /*!< Next state machine state pool*/
    uint8_t state_step;                                 /*!< State index*/

    phy_frame_cfg_t cfg; /*!< General configuration settings for the phy Layer*/
    phy_tx_frame_t tx;   /*!< TX configuration settings for the phy Layer*/
    phyl1_rx_frame_t rx; /*!< RX configuration settings for the phy Layer*/

    rx_wait_time_t rx_wait;  /*!< RX wait time */
    uint8_t *rssi;           /*!< RSSI in 1/10 dB */
    uint8_t *rnsi;           /*!< RNSI in 1/10 dB */
    uint8_t *irq_status_1;   /*!< radio_events byte 1 */
    uint8_t *irq_status_2;   /*!< radio_events byte 2*/
    uint8_t *rx_frame_size;  /*!< RX frame size*/
    uint8_t *pwr_status_cmd; /*!< Pwr status and command register value */

    uint8_t partial_frame_count; /*!< The number of parts the frame will be divided in for cut-through mode */
    uint8_t partial_frame_index; /*!< Current partial frame for cut through mode */

    uint16_t syncing_period_pll_cycles; /*!< Syncing period in PLL cycles */

    uint8_t *phase_info;  /*!< Radio current phases information */
    bool wait_for_ack_tx; /*!< Wait for end of transmission of ack frame */
    uint8_t certification_header[PHY_MAX_CERTIF_HEADER_SIZE]; /*!< Certification header buffer */

    wps_write_request_info_t write_request_info; /*!< Contains info about a write register access */
    wps_read_request_info_t read_request_info;   /*!< Contains info about a read register access */
    wps_phy_handle_t phy_handle;                 /*!< Current state machine state*/

    void *mac;                                               /*!< MAC pointer */
    void (*wps_phy_callback)(void *mac, wps_phy_t *wps_phy); /*!< PHY callback */
};

/** @brief SR1100 PHY debugging registers.
 */
typedef struct phy_debug_cfg {
    bool enable; /*!< Enable/Disable debugging feature of the radio */
} phy_debug_cfg_t;

#if WPS_RADIO_COUNT > 1

/** @brief WPS Dual radio function pointer configuration structure.
 */
typedef struct wps_multi_cfg {
    void (*timer_start)(void);                 /*!< Radio timer start interface. */
    void (*timer_stop)(void);                  /*!< Radio timer stop interface. */
    void (*timer_set_period)(uint16_t period); /*!< Radio timer set period interface. */
    void (*timer_set_max_period)(void);        /*!< Radio timer set max period interface. */
    void (*disable_timer_irq)(void);           /*!< Disable Multi radio interrupt source */
    void (*enable_timer_irq)(void);            /*!< Enable Multi radio interrupt source */
    uint32_t timer_frequency_hz;               /*!< Radio timer frequency in Hz*/
    uint16_t avg_sample_count;                 /*!< Replying radio selection average sample count */
    uint8_t mode;                              /*!< Replying radio selection mode */
    uint8_t rssi_threshold;                    /*!< Replying radio selection RSSI threshold */
} wps_multi_cfg_t;

#endif

#ifdef __cplusplus
}
#endif

#endif /* WPS_PHY_DEF_H_ */
