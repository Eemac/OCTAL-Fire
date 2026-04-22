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
 * Initialize the bms_a CAN peripheral
 */
void can_init_bms_a(FDCAN_HandleTypeDef *fdcan2);


extern uint8_t bms_status_a_data[];
extern can_frame_t bms_status_a_msg;
extern volatile struct can_lib_bms_status_a_t bms_status_a;

/*
 * Send the bms_status_a message
 *
 * First it takes the data struct `bms_status_a_t` and packs it into an
 * array of bytes in bms_status_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_status_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_temp_metrics_a_data[];
extern can_frame_t bms_temp_metrics_a_msg;
extern volatile struct can_lib_bms_temp_metrics_a_t bms_temp_metrics_a;

/*
 * Send the bms_temp_metrics_a message
 *
 * First it takes the data struct `bms_temp_metrics_a_t` and packs it into an
 * array of bytes in bms_temp_metrics_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_temp_metrics_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t bms_voltage_metrics_a_data[];
extern can_frame_t bms_voltage_metrics_a_msg;
extern volatile struct can_lib_bms_voltage_metrics_a_t bms_voltage_metrics_a;

/*
 * Send the bms_voltage_metrics_a message
 *
 * First it takes the data struct `bms_voltage_metrics_a_t` and packs it into an
 * array of bytes in bms_voltage_metrics_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_bms_voltage_metrics_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t cell_voltages_0_a_data[];
extern can_frame_t cell_voltages_0_a_msg;
extern volatile struct can_lib_cell_voltages_0_a_t cell_voltages_0_a;

/*
 * Send the cell_voltages_0_a message
 *
 * First it takes the data struct `cell_voltages_0_a_t` and packs it into an
 * array of bytes in cell_voltages_0_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_cell_voltages_0_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t cell_voltages_1_a_data[];
extern can_frame_t cell_voltages_1_a_msg;
extern volatile struct can_lib_cell_voltages_1_a_t cell_voltages_1_a;

/*
 * Send the cell_voltages_1_a message
 *
 * First it takes the data struct `cell_voltages_1_a_t` and packs it into an
 * array of bytes in cell_voltages_1_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_cell_voltages_1_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t cell_voltages_2_a_data[];
extern can_frame_t cell_voltages_2_a_msg;
extern volatile struct can_lib_cell_voltages_2_a_t cell_voltages_2_a;

/*
 * Send the cell_voltages_2_a message
 *
 * First it takes the data struct `cell_voltages_2_a_t` and packs it into an
 * array of bytes in cell_voltages_2_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_cell_voltages_2_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_0_a_data[];
extern can_frame_t thermistors_0_a_msg;
extern volatile struct can_lib_thermistors_0_a_t thermistors_0_a;

/*
 * Send the thermistors_0_a message
 *
 * First it takes the data struct `thermistors_0_a_t` and packs it into an
 * array of bytes in thermistors_0_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_0_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_1_a_data[];
extern can_frame_t thermistors_1_a_msg;
extern volatile struct can_lib_thermistors_1_a_t thermistors_1_a;

/*
 * Send the thermistors_1_a message
 *
 * First it takes the data struct `thermistors_1_a_t` and packs it into an
 * array of bytes in thermistors_1_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_1_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_2_a_data[];
extern can_frame_t thermistors_2_a_msg;
extern volatile struct can_lib_thermistors_2_a_t thermistors_2_a;

/*
 * Send the thermistors_2_a message
 *
 * First it takes the data struct `thermistors_2_a_t` and packs it into an
 * array of bytes in thermistors_2_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_2_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_3_a_data[];
extern can_frame_t thermistors_3_a_msg;
extern volatile struct can_lib_thermistors_3_a_t thermistors_3_a;

/*
 * Send the thermistors_3_a message
 *
 * First it takes the data struct `thermistors_3_a_t` and packs it into an
 * array of bytes in thermistors_3_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_3_a(FDCAN_HandleTypeDef *hfdcan2);


extern uint8_t thermistors_4_a_data[];
extern can_frame_t thermistors_4_a_msg;
extern volatile struct can_lib_thermistors_4_a_t thermistors_4_a;

/*
 * Send the thermistors_4_a message
 *
 * First it takes the data struct `thermistors_4_a_t` and packs it into an
 * array of bytes in thermistors_4_a_data, which is a part of the
 * can_frame_t. Then the can_frame_t is sent.
 */
void can_send_thermistors_4_a(FDCAN_HandleTypeDef *hfdcan2);





#ifdef __cplusplus
}
#endif