/** @file  swc_cfg_coord.h
 *  @brief Application specific configuration constants for the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_CFG_COORD_H_
#define SWC_CFG_COORD_H_

/* CONSTANTS ******************************************************************/
#define NETWORK_ROLE   SWC_ROLE_COORDINATOR
#define PAN_ID         0xBCD
#define COORD_ADDRESS  0x01
#define NODE_ADDRESS   0x02
#define LOCAL_ADDRESS  COORD_ADDRESS
#define REMOTE_ADDRESS NODE_ADDRESS
#define LOW_OUTPUT_POWER_MODE

/* Output power configuration:
 *  - LOW_OUTPUT_POWER_MODE: Short range. Ideal for Indoor usage.
 *  - HIGH_OUTPUT_POWER_MODE: Long range. Ideal for outdoor usage.
 */
#ifdef LOW_OUTPUT_POWER_MODE
    #define TX_DATA_PULSE_COUNT 1
    #define TX_DATA_PULSE_WIDTH 0
    #define TX_DATA_PULSE_GAIN  1
#elif defined(HIGH_OUTPUT_POWER_MODE)
    #define TX_DATA_PULSE_COUNT 3
    #define TX_DATA_PULSE_WIDTH 3
    #define TX_DATA_PULSE_GAIN  0
#else
#error "No output power level defined."
#endif

/* Input power configuration:
 *  - LOW_OUTPUT_POWER_MODE: Short range. Ideal for Indoor usage.
 *  - HIGH_OUTPUT_POWER_MODE: Long range. Ideal for outdoor usage.
 */
#ifdef LOW_OUTPUT_POWER_MODE
    #define RX_AUTO_REPLY_PULSE_COUNT 1 /* Pulses configuration of received auto-reply frames */
#elif defined(HIGH_OUTPUT_POWER_MODE)
    #define RX_AUTO_REPLY_PULSE_COUNT 3 /* Pulses configuration of received auto-reply frames */
#else
#error "No input power level defined."
#endif

/* SWC queue size */
#define TX_DATA_QUEUE_SIZE 2
#define RX_DATA_QUEUE_SIZE 2

/* Misc configurations */
#define SWC_MODULATION  SWC_MOD_2BITPPM
#define SWC_FEC_LEVEL   SWC_FEC_2
#define SWC_SLEEP_LEVEL SWC_SLEEP_IDLE

/* Schedule configuration */
#define SCHEDULE { \
    2000 \
}
#define TX_TIMESLOTS { \
    MAIN_TIMESLOT(0) \
}
#define RX_TIMESLOTS { \
    AUTO_TIMESLOT(0) \
}

/* Channels */
#define CHANNEL_FREQ { \
    165, \
}
#define CHANNEL_SEQUENCE { \
    0, \
}


#endif /* SWC_CFG_COORD_H_ */
