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

  // pinMode(LED,OUTPUT);
  pinMode(PB1,OUTPUT);
  pinMode(PB2,OUTPUT);
  // pinMode(PB3,OUTPUT);
  pinMode(PB4,OUTPUT);
  // pinMode(PB5,OUTPUT);
  // pinMode(PB6,OUTPUT);
  // pinMode(PB7,OUTPUT);
  // pinMode(PB9,OUTPUT);
  // pinMode(PB11,OUTPUT);

  pinMode(PA10,OUTPUT);
  // pinMode(PB14,OUTPUT);
  // pinMode(PB15,OUTPUT);
  // pinMode(PA15,OUTPUT);

  // digitalToggle(PB9);
  // digitalToggle(PB2);

  Serial.begin(115200);

  can_init_bms_a(&hfdcan2);

  digitalToggle(PA10);

  // initTemp(tempStruct);
  //InitBMS(adStruct);
  //doDebug(ledStruct, tempStruct, adStruct);
  //setCurrentSense(ivStruct);
  //setupFans(tempStruct);
  digitalToggle(PB2);
}

uint16_t parsedValue = 0;
uint8_t wrapper = 0;

void loop()
{
  digitalToggle(PB1);
  digitalToggle(PB2);
  // Serial.println("Hello");
  
  delay(50);
  uint16_t motors[8] = {parsedValue,parsedValue,parsedValue,parsedValue,parsedValue,parsedValue,parsedValue,parsedValue};

  uint8_t f1[8], f2[8], f3[8];


  buildOctoRawCommand(motors, f1, f2, f3, wrapper); // transfer_id = 0..31
  wrapper++;
  if(wrapper == 32) {wrapper = 0;}

  // bms_status.bms_fault_code = 2;
  // can_send_bms_status(&hfdcan2);
  // uint8_t pvb = parsedValue & 0xFF;
  // uint8_t pvt = (parsedValue >> 8) & 0x3F;
  // Serial.println(parsedValue, HEX);
  // long total = pvb | pvt << 10 | pvb
  // thrust.b1 = pvb;
  // thrust.b2 =pvt << 2;
  // // thrust.b3 = 0xA0;
  // // thrust.m3 = parsedValue;
  // // thrust.m4 = parsedValue;
  // thrust.b1 = f1[2];
  // thrust.b2 = f1[3];
  // thrust.b3 = f1[4];
  // thrust.b4 = f1[5];
  // thrust.b5 = f1[6];
  // thrust.b6 = f2[0];
  // thrust.b7 = f2[1];
  // thrust.tail_byte = 0xC0;
  // can_send_thrust(&hfdcan2);

  // thrust.b1 = f2[0];
  // thrust.b2 = f2[1];
  // thrust.b3 = f2[2];
  // thrust.b4 = f2[3];
  // thrust.b5 = f2[4];
  // thrust.b6 = f2[5];
  // thrust.b7 = f2[6];
  // thrust.tail_byte = f2[7];
  // can_send_thrust(&hfdcan2);

  // thrust.b1 = f3[0];
  // thrust.b2 = f3[1];
  // thrust.b3 = f3[7];
  // thrust.b4 = 0;
  // thrust.b5 = 0;
  // thrust.b6 = 0;
  // thrust.b7 = 0;
  // thrust.tail_byte = 0;

  // can_send_thrust(&hfdcan2);
  // thrust.tail_byte = wrapper;
  // can_send_thrust(&hfdcan2);
  // can_send_thrust4(&hfdcan2);

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





// ---- CRC update (CRC-16/CCITT-FALSE) ----
static uint16_t crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}


// ---- MAIN FUNCTION ----
void buildOctoRawCommand(uint16_t motors[8],
                         uint8_t frame1[8],
                         uint8_t frame2[8],
                         uint8_t frame3[8],
                         uint8_t transfer_id)
{
    uint8_t packed[14];
    memset(packed, 0, sizeof(packed));

    // ---- SPEC-CORRECT 8 MOTOR PACKING ----
  uint8_t lowBytes[8] = {0x00};
  uint8_t highBytes[8] = {0x00};
  uint16_t combined[8] = {0x00};
  
  
  for(int i = 0; i < 8; i++) {
    lowBytes[i] = (uint8_t)motors[i];
    highBytes[i] = (uint8_t)((motors[i] & 0x3FFF) >> 8);
    combined[i] = highBytes[i] << 2 | lowBytes[i] << 8;
  }
  
  	uint64_t f1_b = (uint64_t)combined[0] << 48 | (uint64_t)combined[1] << 34 | (uint64_t)combined[2] << 20 | (uint64_t)combined[3] << 6;
  	uint64_t f2_b = (uint64_t)combined[4] << 48 | (uint64_t)combined[5] << 34 | (uint64_t)combined[6] << 20 | (uint64_t)combined[7] << 6;
  
  for(int i = 0; i < 7; i++) {
    packed[i] = (uint8_t)(f1_b >> 56 - i * 8);
    packed[i + 7] = (uint8_t)(f2_b >> 56 - i * 8);
  }
  
  // for(int i = 0; i < 14; i++) {
  // 	Serial.print(packed[i], HEX);
  //   Serial.print(" ");
  // }
  
  // Serial.println();

    // ---- CRC with Hobbywing signature ----
    uint16_t crc = 0xFFFF;

    const uint8_t signature[8] = {
        0xAD, 0x40, 0x66, 0x9F,
        0xC7, 0x86, 0xF0, 0xBD
    };

    for (int i = 0; i < 8; i++) {
        crc = crc16_update(crc, signature[i]);
    }

    for (int i = 0; i < 14; i++) {
        crc = crc16_update(crc, packed[i]);
    }


    // ---- Tail bytes ----
    uint8_t tail_start = (1 << 7) | (0 << 6) | (0 << 5) | (transfer_id & 0x1F);
    uint8_t tail_mid   = (0 << 7) | (0 << 6) | (1 << 5) | (transfer_id & 0x1F);
    uint8_t tail_end   = (0 << 7) | (1 << 6) | (0 << 5) | (transfer_id & 0x1F);


    // ---- FRAME 1 ----
    frame1[0] = crc & 0xFF;
    frame1[1] = (crc >> 8) & 0xFF;

    for (int i = 0; i < 5; i++) {
        frame1[2 + i] = packed[i];
    }

    frame1[7] = tail_start;


    // ---- FRAME 2 ----
    for (int i = 0; i < 7; i++) {
        frame2[i] = packed[5 + i];
    }

    frame2[7] = tail_mid;


    // ---- FRAME 3 ----
    frame3[0] = packed[12];
    frame3[1] = packed[13];

    for (int i = 2; i < 7; i++) {
        frame3[i] = 0;
    }

    frame3[7] = tail_end;
}

