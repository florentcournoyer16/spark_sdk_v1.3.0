/** @file  pairing_message.h
 *  @brief This file contains the constants and structures for transferred pairing messages.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef PAIRING_MESSAGES_H_
#define PAIRING_MESSAGES_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
/*! Structured transferred to the wireless core needs to be packed to avoid issues. */
#define PACKED __attribute__((__packed__))

/*! Bytes position common to all pairing messages. */
#define PAIRING_BYTE_COMMAND 0

/* TYPES **********************************************************************/
/** @brief Pairing commands used in every transferred message.
 */
typedef enum pairing_command {
    /*! The pairing command's initial value. */
    PAIRING_COMMAND_NONE,
    /*! The authentication message command. */
    PAIRING_COMMAND_AUTHENTICATION_MESSAGE,
    /*! The authentication response command. */
    PAIRING_COMMAND_AUTHENTICATION_RESPONSE,
    /*! The identification message command. */
    PAIRING_COMMAND_IDENTIFICATION_MESSAGE,
    /*! The identification response command. */
    PAIRING_COMMAND_IDENTIFICATION_RESPONSE,
    /*! The addressing message command. */
    PAIRING_COMMAND_ADDRESSING_MESSAGE,
    /*! The addressing response command. */
    PAIRING_COMMAND_ADDRESSING_RESPONSE,
} pairing_command_t;

/** @brief The possible authentication actions.
 */
typedef enum pairing_authentication_action {
    /*! The authentication action initial value. */
    PAIRING_AUTHENTICATION_ACTION_NONE,
    /*! The authentication phase is successful. */
    PAIRING_AUTHENTICATION_ACTION_SUCCESS,
    /*! The authentication phase failed. */
    PAIRING_AUTHENTICATION_ACTION_FAIL,
} pairing_authentication_action_t;

/** @brief The possible identification actions.
 */
typedef enum pairing_identification_action {
    /*! The identification action initial value. */
    PAIRING_IDENTIFICATION_ACTION_NONE,
    /*! The authentication phase is successful. */
    PAIRING_IDENTIFICATION_ACTION_SUCCESS,
    /*! The authentication phase failed. */
    PAIRING_IDENTIFICATION_ACTION_FAIL,
} pairing_identification_action_t;

/** @brief The possible addressing actions.
 */
typedef enum pairing_addressing_action {
    /*! The identification action initial value. */
    PAIRING_ADDRESSING_ACTION_NONE,
    /*! The identification phase is successful. */
    PAIRING_ADDRESSING_ACTION_SUCCESS,
    /*! The identification phase failed. */
    PAIRING_ADDRESSING_ACTION_FAIL,
} pairing_addressing_action_t;

/** @brief Pairing authentication message sent by the coordinator to share the application code.
 */
typedef struct PACKED pairing_authentication_message {
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! Application code to verify the devices' compatibility. */
    uint64_t app_code;
} pairing_authentication_message_t;

/** @brief The pairing authentication response sent by the node.
 */
typedef struct PACKED pairing_authentication_response {
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! The action chosen by the node. */
    uint8_t pairing_authentication_action;
} pairing_authentication_response_t;

/** @brief The pairing identification message sent by the node to share node's device role and unique ID.
 */
typedef struct PACKED pairing_identification_message {
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! The pairing device's application specific role. */
    uint8_t device_role;
    /*! Unique ID generated by the device's radio. */
    uint64_t unique_id;
} pairing_identification_message_t;

/** @brief The pairing identification response sent by the coordinator.
 */
typedef struct PACKED pairing_identification_response {
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! The action chosen by the coordinator. */
    uint8_t pairing_identification_action;
} pairing_identification_response_t;

/** @brief The pairing addressing message sent by the coordinator to share pairing addresses to the node.
 */
typedef struct PACKED pairing_addressing_message {
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! PAN ID sent to the node. */
    uint16_t pan_id;
    /*! Coordinator ID inside the PAN sent to the node. */
    uint8_t coordinator_id;
    /*! Node ID available to be used by the node. */
    uint8_t node_id;
} pairing_addressing_message_t;

/** @brief The pairing addressing response sent by the node.
 */
typedef struct PACKED pairing_addressing_response {
    /*! The pairing command associated with the message. */
    uint8_t pairing_command;
    /*! The action chosen by the node. */
    uint8_t pairing_addressing_action;
} pairing_addressing_response_t;

#ifdef __cplusplus
}
#endif

#endif /* PAIRING_MESSAGES_H_ */
