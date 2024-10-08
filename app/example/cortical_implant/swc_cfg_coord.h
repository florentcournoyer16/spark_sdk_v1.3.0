/** @file  swc_cfg_coord.h
 *  @brief Application specific configuration constants for the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_CFG_COORD_H_
#define SWC_CFG_COORD_H_

/* CONSTANTS ******************************************************************/
#define NETWORK_ROLE        SWC_ROLE_COORDINATOR
#define PAN_ID              0xBCD
#define COORDINATOR_ADDRESS 0x01
#define NODE_ADDRESS        0x02
#define LOCAL_ADDRESS       COORDINATOR_ADDRESS
#define REMOTE_ADDRESS      NODE_ADDRESS

/* Output power configuration */
#define TX_DATA_PULSE_COUNT 3
#define TX_DATA_PULSE_WIDTH 6
#define TX_DATA_PULSE_GAIN  0
#define TX_ACK_PULSE_COUNT  3
#define TX_ACK_PULSE_WIDTH  6
#define TX_ACK_PULSE_GAIN   0

/* Input power configuration */
#define RX_ACK_PULSE_COUNT  3 /* Pulses configuration of received ACK frames */
#define RX_DATA_PULSE_COUNT 3 /* Pulses configuration of received data frames */

/* SWC queue size */
#define TX_DATA_QUEUE_SIZE 2
#define RX_DATA_QUEUE_SIZE 2

/* Misc configurations */
#define SWC_MODULATION  SWC_MOD_2BITPPM
#define SWC_FEC_LEVEL   SWC_FEC_2
#define SWC_SLEEP_LEVEL SWC_SLEEP_IDLE

/* Schedule configuration */
#define SCHEDULE { \
    1000, 1000 \
}
#define TX_TIMESLOTS { \
    MAIN_TIMESLOT(0) \
}
#define RX_TIMESLOTS { \
    MAIN_TIMESLOT(1) \
}

/* Channels */
#define CHANNEL_FREQ { \
    164, 171, 178, 185, 192 \
}
#define CHANNEL_SEQUENCE { \
    0, 1, 2, 3, 4 \
}


#endif /* SWC_CFG_COORD_H_ */
