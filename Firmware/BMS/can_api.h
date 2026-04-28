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
    BMS_STATE_CHARGING_IDLE,
    BMS_STATE_CHARGING_ACTIVE,
    BMS_STATE_BALANCING,
    BMS_STATE_ACTIVE,
    BMS_STATE_IDLE,
    BMS_STATE_FAULTED,
    BMS_STATE_SURVIVAL,
};


/*
 * Initialize the bms_west CAN peripheral
 */
void can_init_bms_west(FDCAN_HandleTypeDef *fdcan2);


extern uint8_t bms_status_west_data[];
extern can_frame_t bms_status_west_msg;
extern volatile struct can_lib_bms_status_west_t bms_status_west;

/*
 * Send the bms_status_west message
 *
 * First it takes the data struct `bms_status_west_t` and packs it into an
 * array of bytes in bms_status_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_status_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_temp_metrics_west_data[];
extern can_frame_t bms_temp_metrics_west_msg;
extern volatile struct can_lib_bms_temp_metrics_west_t bms_temp_metrics_west;

/*
 * Send the bms_temp_metrics_west message
 *
 * First it takes the data struct `bms_temp_metrics_west_t` and packs it into an
 * array of bytes in bms_temp_metrics_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_temp_metrics_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_voltage_metrics_west_data[];
extern can_frame_t bms_voltage_metrics_west_msg;
extern volatile struct can_lib_bms_voltage_metrics_west_t bms_voltage_metrics_west;

/*
 * Send the bms_voltage_metrics_west message
 *
 * First it takes the data struct `bms_voltage_metrics_west_t` and packs it into an
 * array of bytes in bms_voltage_metrics_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_voltage_metrics_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t cell_voltages_0_west_data[];
extern can_frame_t cell_voltages_0_west_msg;
extern volatile struct can_lib_cell_voltages_0_west_t cell_voltages_0_west;

/*
 * Send the cell_voltages_0_west message
 *
 * First it takes the data struct `cell_voltages_0_west_t` and packs it into an
 * array of bytes in cell_voltages_0_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_cell_voltages_0_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t cell_voltages_1_west_data[];
extern can_frame_t cell_voltages_1_west_msg;
extern volatile struct can_lib_cell_voltages_1_west_t cell_voltages_1_west;

/*
 * Send the cell_voltages_1_west message
 *
 * First it takes the data struct `cell_voltages_1_west_t` and packs it into an
 * array of bytes in cell_voltages_1_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_cell_voltages_1_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t cell_voltages_2_west_data[];
extern can_frame_t cell_voltages_2_west_msg;
extern volatile struct can_lib_cell_voltages_2_west_t cell_voltages_2_west;

/*
 * Send the cell_voltages_2_west message
 *
 * First it takes the data struct `cell_voltages_2_west_t` and packs it into an
 * array of bytes in cell_voltages_2_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_cell_voltages_2_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_0_west_data[];
extern can_frame_t thermistors_0_west_msg;
extern volatile struct can_lib_thermistors_0_west_t thermistors_0_west;

/*
 * Send the thermistors_0_west message
 *
 * First it takes the data struct `thermistors_0_west_t` and packs it into an
 * array of bytes in thermistors_0_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_0_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_1_west_data[];
extern can_frame_t thermistors_1_west_msg;
extern volatile struct can_lib_thermistors_1_west_t thermistors_1_west;

/*
 * Send the thermistors_1_west message
 *
 * First it takes the data struct `thermistors_1_west_t` and packs it into an
 * array of bytes in thermistors_1_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_1_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_2_west_data[];
extern can_frame_t thermistors_2_west_msg;
extern volatile struct can_lib_thermistors_2_west_t thermistors_2_west;

/*
 * Send the thermistors_2_west message
 *
 * First it takes the data struct `thermistors_2_west_t` and packs it into an
 * array of bytes in thermistors_2_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_2_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_3_west_data[];
extern can_frame_t thermistors_3_west_msg;
extern volatile struct can_lib_thermistors_3_west_t thermistors_3_west;

/*
 * Send the thermistors_3_west message
 *
 * First it takes the data struct `thermistors_3_west_t` and packs it into an
 * array of bytes in thermistors_3_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_3_west(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_4_west_data[];
extern can_frame_t thermistors_4_west_msg;
extern volatile struct can_lib_thermistors_4_west_t thermistors_4_west;

/*
 * Send the thermistors_4_west message
 *
 * First it takes the data struct `thermistors_4_west_t` and packs it into an
 * array of bytes in thermistors_4_west_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_4_west(FDCAN_HandleTypeDef *hfdcan2);





#ifdef __cplusplus
}
#endif