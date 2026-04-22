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

enum core_state_e { 
    CORE_STATE_STARTUP,
    CORE_STATE_IDLE,
    CORE_STATE_ACTIVE,
    CORE_STATE_FAULT,
};


/*
 * Initialize the core CAN peripheral
 */
void can_init_core(FDCAN_HandleTypeDef *fdcan2);


extern uint8_t core_status_data[];
extern can_frame_t core_status_msg;
extern volatile struct can_lib_core_status_t core_status;

/*
 * Send the core_status message
 *
 * First it takes the data struct `core_status_t` and packs it into an
 * array of bytes in core_status_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_core_status(FDCAN_HandleTypeDef *hfdcan2);





#ifdef __cplusplus
}
#endif