/** @file  swc_cfg_coord.h
 *  @brief Application specific configuration constants for the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_CFG_COORD_H_
#define SWC_CFG_COORD_H_

/* CONSTANTS ******************************************************************/
#define NETWORK_ROLE              SWC_ROLE_COORDINATOR
#define PAN_ID                    0xBCD
#define COORDINATOR_ADDRESS       0x00
#define NODE1_ADDRESS             0x01
#define NODE2_ADDRESS             0x02
#define NETWORK_BROADCAST_ADDRESS 0xFF

#define LOCAL_ADDRESS             COORDINATOR_ADDRESS
#define REMOTE_ADDRESS_1          NODE1_ADDRESS
#define REMOTE_ADDRESS_2          NODE2_ADDRESS

/* Output power configuration */
#define TX_BEACON_PULSE_COUNT 3
#define TX_BEACON_PULSE_WIDTH 6
#define TX_BEACON_PULSE_GAIN  0
#define TX_ACK_PULSE_COUNT    3
#define TX_ACK_PULSE_WIDTH    6
#define TX_ACK_PULSE_GAIN     0

/* Input power configuration */
#define RX_ACK_PULSE_COUNT  3 /* Pulses configuration of received ACK frames */
#define RX_DATA_PULSE_COUNT 2 /* Pulses configuration of received data frames */

/* SWC queue size */
#define TX_DATA_QUEUE_SIZE 2
#define RX_DATA_QUEUE_SIZE 2

/* Misc configurations */
#define SWC_MODULATION       SWC_MOD_2BITPPM
#define SWC_FEC_LEVEL        SWC_FEC_1
#define SWC_SLEEP_LEVEL      SWC_SLEEP_IDLE

/* Schedule configuration */
#define SCHEDULE {           \
    333, 333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333, \
         333, 333, 333, 333  \
}
#define TX_BEACON_TIMESLOTS { \
    MAIN_TIMESLOT(0)          \
}
#define RX_FROM_NODE1_TIMESLOTS {          \
    MAIN_TIMESLOT(1),  MAIN_TIMESLOT(3),   \
    MAIN_TIMESLOT(5),  MAIN_TIMESLOT(7),   \
    MAIN_TIMESLOT(9),  MAIN_TIMESLOT(11),  \
    MAIN_TIMESLOT(13),  MAIN_TIMESLOT(15), \
    MAIN_TIMESLOT(17),  MAIN_TIMESLOT(19), \
    MAIN_TIMESLOT(21),  MAIN_TIMESLOT(23), \
    MAIN_TIMESLOT(25),  MAIN_TIMESLOT(27), \
    MAIN_TIMESLOT(29),  MAIN_TIMESLOT(31), \
    MAIN_TIMESLOT(33),  MAIN_TIMESLOT(35), \
    MAIN_TIMESLOT(37),  MAIN_TIMESLOT(39), \
    MAIN_TIMESLOT(41),  MAIN_TIMESLOT(43)  \
}
#define RX_FROM_NODE2_TIMESLOTS {          \
    MAIN_TIMESLOT(2),  MAIN_TIMESLOT(4),   \
    MAIN_TIMESLOT(6),  MAIN_TIMESLOT(8),   \
    MAIN_TIMESLOT(10),  MAIN_TIMESLOT(12), \
    MAIN_TIMESLOT(14),  MAIN_TIMESLOT(16), \
    MAIN_TIMESLOT(18),  MAIN_TIMESLOT(20), \
    MAIN_TIMESLOT(22),  MAIN_TIMESLOT(24), \
    MAIN_TIMESLOT(26),  MAIN_TIMESLOT(28), \
    MAIN_TIMESLOT(30),  MAIN_TIMESLOT(32), \
    MAIN_TIMESLOT(34),  MAIN_TIMESLOT(36), \
    MAIN_TIMESLOT(38),  MAIN_TIMESLOT(40), \
    MAIN_TIMESLOT(42),  MAIN_TIMESLOT(44)  \
}

/* Channels */
#define CHANNEL_FREQ { \
    167, 174, 181, 188 \
}
#define CHANNEL_SEQUENCE { \
    0, 1, 2, 3             \
}


#endif /* SWC_CFG_COORD_H_ */
