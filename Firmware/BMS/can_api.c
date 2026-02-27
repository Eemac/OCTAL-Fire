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
    
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 525226497; 
    sFilterConfig.FilterID2 = 0x1FFFFFFF; 
    if (HAL_FDCAN_ConfigFilter(hfdcan2, &sFilterConfig) != HAL_OK) {
        // Error_Handler();
    }
    
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 17; 
    sFilterConfig.FilterID2 = 0x7FF; 
    if (HAL_FDCAN_ConfigFilter(hfdcan2, &sFilterConfig) != HAL_OK) {
        // Error_Handler();
    }
    
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 4355; 
    sFilterConfig.FilterID2 = 0x1FFFFFFF; 
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

uint8_t thrust_data[8] = {0};

can_frame_t thrust_msg = {
    .id = 5145601,
    .mob = 0,
    .data = thrust_data,
    .dlc = 8,
};

volatile struct can_lib_thrust_t thrust = {0};

void can_send_thrust(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_thrust_pack(
        thrust_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_thrust_t*) &thrust,
        8
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 5145601;
    TxHeader.IdType = FDCAN_EXTENDED_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, thrust_data)!= HAL_OK) {

        }
    }
}








uint8_t bms_status_data[7] = {0};

can_frame_t bms_status_msg = {
    .id = 4096,
    .mob = 0,
    .data = bms_status_data,
    .dlc = 7,
};

volatile struct can_lib_bms_status_t bms_status = {0};

void can_send_bms_status(FDCAN_HandleTypeDef *hfdcan2) {
    // We can be sure here that the CAN data struct won't change here

    can_lib_bms_status_pack(
        bms_status_data,
        // We can safely discard the volatile qualifier because we are in an
        // ATOMIC block, so the value will not be changed in an ISR
        (const struct can_lib_bms_status_t*) &bms_status,
        7
    );

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 4096;
    TxHeader.IdType = FDCAN_EXTENDED_ID;  
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan2) > 0) {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan2, &TxHeader, bms_status_data)!= HAL_OK) {

        }
    }
}









/*
 * Receive messages
 */

uint8_t msg1_data[7] = {0};

can_frame_t msg1_msg = {
    .mob = 0,
    .data = msg1_data,
};

can_filter_t msg1_filter = {
    .id = 525226497,
    .mask = 2047
};

struct can_lib_msg1_t msg1 = {0};


uint8_t bms_status_b_data[1] = {0};

can_frame_t bms_status_b_msg = {
    .mob = 0,
    .data = bms_status_b_data,
};

can_filter_t bms_status_b_filter = {
    .id = 17,
    .mask = 2047
};

struct can_lib_bms_status_b_t bms_status_b = {0};


uint8_t bms_status_b_ex_e_data[1] = {0};

can_frame_t bms_status_b_ex_e_msg = {
    .mob = 0,
    .data = bms_status_b_ex_e_data,
};

can_filter_t bms_status_b_ex_e_filter = {
    .id = 4355,
    .mask = 2047
};

struct can_lib_bms_status_b_ex_e_t bms_status_b_ex_e = {0};



int can_receive(FDCAN_HandleTypeDef *hfdcan2) {
    
    //Do RX decode if packet present
    FDCAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};

    int rc = HAL_FDCAN_GetRxFifoFillLevel(hfdcan2, FDCAN_RX_FIFO0);
    if(rc > 0) {
        HAL_FDCAN_GetRxMessage(hfdcan2, FDCAN_RX_FIFO0, &rx_msg_header, rx_msg_data);
        switch (rx_msg_header.Identifier) {
            
                case 525226497:
                    can_lib_msg1_unpack(&msg1, rx_msg_data, 7);
                    break;
            
                case 17:
                    can_lib_bms_status_b_unpack(&bms_status_b, rx_msg_data, 1);
                    break;
            
                case 4355:
                    can_lib_bms_status_b_ex_e_unpack(&bms_status_b_ex_e, rx_msg_data, 1);
                    break;
            
                default:
                    break;
            
        }
      }

    return rc;
}

