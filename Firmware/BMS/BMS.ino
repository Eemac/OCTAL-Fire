#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_api.h"

#define LED PB9   // QFP48 socket PCB
FDCAN_HandleTypeDef hfdcan2;

void OCTAL_Set_Clock(void) {
  // set flash latency for target (144MHz -> FLASH_LATENCY_4) and up voltage to HF range
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  // Request HSE ON (clock source mode from external high-speed crystal)
  RCC->CR |= RCC_CR_HSEON;

  //Move PLL to source off of HSE
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE);

  //Setup PLL for 48MHz peripheral clock
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4; // 12MHz after div
  RCC_OscInitStruct.PLL.PLLN = 24; //288Mhz after mult
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; //144MHz CPU clock
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6; //Peripherals at 48MHz (required by USB (48MHz only), CAN ahas a bit more leeway)
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  //Enable PLL
  SET_BIT(RCC->CR, RCC_CR_PLLON);

  //Set CPU and bus clocks to source 144MHz from PLL
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  //push new clock settings to time-dependent code/structs
  SystemCoreClockUpdate();

  //Start tick counter
  HAL_InitTick(TICK_INT_PRIORITY);
}

void setup() 
{
  OCTAL_Set_Clock();

  pinMode(LED,OUTPUT);
  pinMode(PB1,OUTPUT);
  pinMode(PB2,OUTPUT);
  pinMode(PB3,OUTPUT);
  pinMode(PB4,OUTPUT);
  pinMode(PB5,OUTPUT);
  pinMode(PB6,OUTPUT);
  pinMode(PB7,OUTPUT);
  pinMode(PB9,OUTPUT);
  pinMode(PB11,OUTPUT);
  pinMode(PB14,OUTPUT);
  pinMode(PB15,OUTPUT);
  pinMode(PA15,OUTPUT);

  digitalToggle(PB9);
  digitalToggle(PB2);

  Serial.begin(115200);

  can_init_bms_a(&hfdcan2);

  // initTemp(tempStruct);
  //InitBMS(adStruct);
  //doDebug(ledStruct, tempStruct, adStruct);
  //setCurrentSense(ivStruct);
  //setupFans(tempStruct);
}

uint16_t parsedValue = 0;

void loop()
{
  digitalToggle(PB1);
  digitalToggle(PB2);

  delay(50);
  // bms_status.bms_fault_code = 2;
  // can_send_bms_status(&hfdcan2);
  uint8_t pvb = parsedValue & 0xFF;
  uint8_t pvt = (parsedValue >> 8) & 0x3F;
  Serial.println(parsedValue, HEX);
  //long total = pvb | pvt << 10 | pvb
  thrust.b1 = pvb;
  thrust.b2 =pvt << 2;
  // thrust.b3 = 0xA0;
  // thrust.m3 = parsedValue;
  // thrust.m4 = parsedValue;
  thrust.tail_byte = 0xC0;
  can_send_thrust(&hfdcan2);

   if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // remove whitespace

    // Check that it starts with 'S

      long value = input.toInt();  // convert to integer

      // Ensure it fits in uint16_t
      if (value >= 0 && value <= 65535) {
        parsedValue = value;

        Serial.print("Parsed uint16_t value: ");
        Serial.println(parsedValue, HEX);
      } else {
        Serial.println("Value out of uint16_t range.");
      }
  }
    Serial.print(msg1.pwm);
    Serial.print(" ");

  if(can_receive(&hfdcan2) > 0) {
   
  }
}

