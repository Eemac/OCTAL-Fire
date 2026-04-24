#include "can_api.h"
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_lib.h"

void can_init_bms_a(FDCAN_HandleTypeDef *hfdcan2) {
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    //Initializes the peripherals clocks to run from 48MHz PLLQ
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        //Error_Handler();
    }

    hfdcan2->Instance = FDCAN2;
    hfdcan2->Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan2->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2->Init.Mode = FDCAN_MODE_NORMAL;//FDCAN_MODE_INTERNAL_LOOPBACK;
    hfdcan2->Init.AutoRetransmission = DISABLE;//Enable this for actual bus
    hfdcan2->Init.TransmitPause = DISABLE;
    hfdcan2->Init.ProtocolException = DISABLE;
    hfdcan2->Init.NominalPrescaler = 6;
    hfdcan2->Init.NominalSyncJumpWidth = 1;
    hfdcan2->Init.NominalTimeSeg1 = 13;
    hfdcan2->Init.NominalTimeSeg2 = 2;

    hfdcan2->Init.StdFiltersNbr = 16;
    hfdcan2->Init.ExtFiltersNbr = 3;
    //Data+ timeSeg/SJW/Prescaler values only for extended can frames
    hfdcan2->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(hfdcan2) != HAL_OK) {
        //Error_Handler();
    }
    
    HAL_FDCAN_ConfigGlobalFilter(
        hfdcan2,
        FDCAN_REJECT,   // Reject non-matching standard IDs
        FDCAN_REJECT,   // Reject non-matching extended IDs
        FDCAN_FILTER_REMOTE,
        FDCAN_FILTER_REMOTE
    );

    // Set up filters for the chip
    FDCAN_FilterTypeDef sFilterConfig;
    
    //     /**FDCAN2 GPIO Configuration for stm32g473CCT3
    //     PB12     ------> FDCAN2_RX
    //     PB13     ------> FDCAN2_TX
    //     */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    if(HAL_FDCAN_Start(hfdcan2) != HAL_OK)
    {
      //Error_Handler();
    }
}






/*
 * Transmit messages
 */

uint8_t bms_status_a_data[8] = {0};

can_frame_t bms_status_a_msg = {
    .id = 16,
    .data = bms_status_a_data,
    .dlc = 8,
};

volatile struct can_lib_bms_status_a_t bms_status_a = {0};

void can_send_bms_status_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_bms_status_a_pack(
        bms_status_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_bms_status_a_t*) &bms_status_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 16;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, bms_status_a_data)!= HAL_OK) {

        }
    }
}








uint8_t bms_temp_metrics_a_data[8] = {0};

can_frame_t bms_temp_metrics_a_msg = {
    .id = 17,
    .data = bms_temp_metrics_a_data,
    .dlc = 8,
};

volatile struct can_lib_bms_temp_metrics_a_t bms_temp_metrics_a = {0};

void can_send_bms_temp_metrics_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_bms_temp_metrics_a_pack(
        bms_temp_metrics_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_bms_temp_metrics_a_t*) &bms_temp_metrics_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 17;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, bms_temp_metrics_a_data)!= HAL_OK) {

        }
    }
}








uint8_t bms_voltage_metrics_a_data[8] = {0};

can_frame_t bms_voltage_metrics_a_msg = {
    .id = 18,
    .data = bms_voltage_metrics_a_data,
    .dlc = 8,
};

volatile struct can_lib_bms_voltage_metrics_a_t bms_voltage_metrics_a = {0};

void can_send_bms_voltage_metrics_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_bms_voltage_metrics_a_pack(
        bms_voltage_metrics_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_bms_voltage_metrics_a_t*) &bms_voltage_metrics_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 18;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, bms_voltage_metrics_a_data)!= HAL_OK) {

        }
    }
}








uint8_t cell_voltages_0_a_data[8] = {0};

can_frame_t cell_voltages_0_a_msg = {
    .id = 19,
    .data = cell_voltages_0_a_data,
    .dlc = 8,
};

volatile struct can_lib_cell_voltages_0_a_t cell_voltages_0_a = {0};

void can_send_cell_voltages_0_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_cell_voltages_0_a_pack(
        cell_voltages_0_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_cell_voltages_0_a_t*) &cell_voltages_0_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 19;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, cell_voltages_0_a_data)!= HAL_OK) {

        }
    }
}








uint8_t cell_voltages_1_a_data[8] = {0};

can_frame_t cell_voltages_1_a_msg = {
    .id = 20,
    .data = cell_voltages_1_a_data,
    .dlc = 8,
};

volatile struct can_lib_cell_voltages_1_a_t cell_voltages_1_a = {0};

void can_send_cell_voltages_1_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_cell_voltages_1_a_pack(
        cell_voltages_1_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_cell_voltages_1_a_t*) &cell_voltages_1_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 20;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, cell_voltages_1_a_data)!= HAL_OK) {

        }
    }
}








uint8_t cell_voltages_2_a_data[6] = {0};

can_frame_t cell_voltages_2_a_msg = {
    .id = 21,
    .data = cell_voltages_2_a_data,
    .dlc = 6,
};

volatile struct can_lib_cell_voltages_2_a_t cell_voltages_2_a = {0};

void can_send_cell_voltages_2_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_cell_voltages_2_a_pack(
        cell_voltages_2_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_cell_voltages_2_a_t*) &cell_voltages_2_a,
        6
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 21;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, cell_voltages_2_a_data)!= HAL_OK) {

        }
    }
}








uint8_t thermistors_0_a_data[8] = {0};

can_frame_t thermistors_0_a_msg = {
    .id = 22,
    .data = thermistors_0_a_data,
    .dlc = 8,
};

volatile struct can_lib_thermistors_0_a_t thermistors_0_a = {0};

void can_send_thermistors_0_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_thermistors_0_a_pack(
        thermistors_0_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_thermistors_0_a_t*) &thermistors_0_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 22;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, thermistors_0_a_data)!= HAL_OK) {

        }
    }
}








uint8_t thermistors_1_a_data[8] = {0};

can_frame_t thermistors_1_a_msg = {
    .id = 23,
    .data = thermistors_1_a_data,
    .dlc = 8,
};

volatile struct can_lib_thermistors_1_a_t thermistors_1_a = {0};

void can_send_thermistors_1_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_thermistors_1_a_pack(
        thermistors_1_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_thermistors_1_a_t*) &thermistors_1_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 23;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, thermistors_1_a_data)!= HAL_OK) {

        }
    }
}








uint8_t thermistors_2_a_data[8] = {0};

can_frame_t thermistors_2_a_msg = {
    .id = 24,
    .data = thermistors_2_a_data,
    .dlc = 8,
};

volatile struct can_lib_thermistors_2_a_t thermistors_2_a = {0};

void can_send_thermistors_2_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_thermistors_2_a_pack(
        thermistors_2_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_thermistors_2_a_t*) &thermistors_2_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 24;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, thermistors_2_a_data)!= HAL_OK) {

        }
    }
}








uint8_t thermistors_3_a_data[8] = {0};

can_frame_t thermistors_3_a_msg = {
    .id = 25,
    .data = thermistors_3_a_data,
    .dlc = 8,
};

volatile struct can_lib_thermistors_3_a_t thermistors_3_a = {0};

void can_send_thermistors_3_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_thermistors_3_a_pack(
        thermistors_3_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_thermistors_3_a_t*) &thermistors_3_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 25;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, thermistors_3_a_data)!= HAL_OK) {

        }
    }
}








uint8_t thermistors_4_a_data[8] = {0};

can_frame_t thermistors_4_a_msg = {
    .id = 26,
    .data = thermistors_4_a_data,
    .dlc = 8,
};

volatile struct can_lib_thermistors_4_a_t thermistors_4_a = {0};

void can_send_thermistors_4_a(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_thermistors_4_a_pack(
        thermistors_4_a_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_thermistors_4_a_t*) &thermistors_4_a,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 26;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, thermistors_4_a_data)!= HAL_OK) {

        }
    }
}









/*
 * Receive messages
 */


int can_receive(FDCAN_HandleTypeDef *hfdcan2) {
    
    //Do RX decode if packet present
    FDCAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};

    int rc = HAL_FDCAN_GetRxFifoFillLevel(hfdcan2, FDCAN_RX_FIFO0);
    if(rc > 0) {
        HAL_FDCAN_GetRxMessage(hfdcan2, FDCAN_RX_FIFO0, &rx_msg_header, rx_msg_data);
        switch (rx_msg_header.Identifier) {
            
                default:
                    break;
            
        }
      }

    return rc;
}

