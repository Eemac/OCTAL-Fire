#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_lib.h"

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t* data;
} can_frame_t;

typedef struct {
    uint32_t id;
    uint32_t mask;
} can_filter_t;

/*
 * Enums
 */

enum bms_state_e { 
    BMS_STATE_ACTIVE,
    BMS_STATE_CHARGING,
    BMS_STATE_FAULT,
};


/*
 * Initialize the bms_a CAN peripheral
 */
void can_init_bms_a(FDCAN_HandleTypeDef *fdcan2);


extern uint8_t thrust_data[];
extern can_frame_t thrust_msg;
extern volatile struct can_lib_thrust_t thrust;

/*
 * Send the thrust message
 *
 * First it takes the data struct `thrust_t` and packs it into an
 * array of bytes in thrust_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thrust(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_status_data[];
extern can_frame_t bms_status_msg;
extern volatile struct can_lib_bms_status_t bms_status;

/*
 * Send the bms_status message
 *
 * First it takes the data struct `bms_status_t` and packs it into an
 * array of bytes in bms_status_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_status(FDCAN_HandleTypeDef *hfdcan2);




extern uint8_t msg1_data[];
extern can_frame_t msg1_msg;
extern struct can_lib_msg1_t msg1;

int can_receive(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_status_b_data[];
extern can_frame_t bms_status_b_msg;
extern struct can_lib_bms_status_b_t bms_status_b;

int can_receive(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_status_b_ex_e_data[];
extern can_frame_t bms_status_b_ex_e_msg;
extern struct can_lib_bms_status_b_ex_e_t bms_status_b_ex_e;

int can_receive(FDCAN_HandleTypeDef *hfdcan2);



#ifdef __cplusplus
}
#endif