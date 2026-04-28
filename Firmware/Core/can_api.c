#include "can_api.h"
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_lib.h"

void can_init_core(FDCAN_HandleTypeDef *hfdcan2) {
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
    
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 16; 
    sFilterConfig.FilterID2 = 0x7FF; 
    if (HAL_FDCAN_ConfigFilter(hfdcan2, &sFilterConfig) != HAL_OK) {
        // Error_Handler();
    }
    
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 18; 
    sFilterConfig.FilterID2 = 0x7FF; 
    if (HAL_FDCAN_ConfigFilter(hfdcan2, &sFilterConfig) != HAL_OK) {
        // Error_Handler();
    }
    
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

uint8_t core_status_data[1] = {0};

can_frame_t core_status_msg = {
    .id = 1,
    .data = core_status_data,
    .dlc = 1,
};

volatile struct can_lib_core_status_t core_status = {0};

void can_send_core_status(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_core_status_pack(
        core_status_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_core_status_t*) &core_status,
        1
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 1;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, core_status_data)!= HAL_OK) {

        }
    }
}








uint8_t core_batt_ctrl_data[1] = {0};

can_frame_t core_batt_ctrl_msg = {
    .id = 2,
    .data = core_batt_ctrl_data,
    .dlc = 1,
};

volatile struct can_lib_core_batt_ctrl_t core_batt_ctrl = {0};

void can_send_core_batt_ctrl(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_core_batt_ctrl_pack(
        core_batt_ctrl_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_core_batt_ctrl_t*) &core_batt_ctrl,
        1
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 2;
    TxHeader.IdType = FDCAN_STANDARD_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, core_batt_ctrl_data)!= HAL_OK) {

        }
    }
}









/*
 * Receive messages
 */

uint8_t bms_status_north_data[8] = {0};

can_frame_t bms_status_north_msg = {
    .data = bms_status_north_data,
};

can_filter_t bms_status_north_filter = {
    .id = 16,
    .mask = 2047
};

struct can_lib_bms_status_north_t bms_status_north = {0};


uint8_t bms_voltage_metrics_north_data[8] = {0};

can_frame_t bms_voltage_metrics_north_msg = {
    .data = bms_voltage_metrics_north_data,
};

can_filter_t bms_voltage_metrics_north_filter = {
    .id = 18,
    .mask = 2047
};

struct can_lib_bms_voltage_metrics_north_t bms_voltage_metrics_north = {0};



int can_receive(FDCAN_HandleTypeDef *hfdcan2) {
    
    //Do RX decode if packet present
    FDCAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};

    int rc = HAL_FDCAN_GetRxFifoFillLevel(hfdcan2, FDCAN_RX_FIFO0);
    if(rc > 0) {
        HAL_FDCAN_GetRxMessage(hfdcan2, FDCAN_RX_FIFO0, &rx_msg_header, rx_msg_data);
        switch (rx_msg_header.Identifier) {
            
                case 16:
                    can_lib_bms_status_north_unpack(&bms_status_north, rx_msg_data, 8);
                    break;
            
                case 18:
                    can_lib_bms_voltage_metrics_north_unpack(&bms_voltage_metrics_north, rx_msg_data, 8);
                    break;
            
                default:
                    break;
            
        }
      }

    return rc;
}

