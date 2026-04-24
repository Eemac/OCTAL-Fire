#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED


#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_api.h"
#include "MCP23S17.h"

// STM32G473CCTx pin defs
#define HEARTBEAT PA8
#define ASENSE_5V PA0
#define ASENSE_3V3 PA1

#define SHDN_N PB0 
#define SHDN_NE PB1 
#define SHDN_E PB2 
#define SHDN_SE PB3 
#define SHDN_S PB4 
#define SHDN_SW PB5 
#define SHDN_W PB9 
#define SHDN_NW PB10 

#define RESET_IO PA10

#define CAN_COMS PB6 
#define USBC_COMS PB7 
#define BOOT0 PB8 

#define CUBE_HANDOFF PB11 
#define RXCAN PB12 
#define TXCAN PB13 

#define CUBE_IO_RESET PB14 
#define CTRL_NONCRIT_CHECK PB15 
#define SENSE_POWER_CYCLE PA15

// MCP Pin Defs
// port B; all unlabeled are input
#define RESET_CUBE_1 8 // out
#define SHDN_REMOTE 9
#define SHDN_TILT 10
#define SHDN_WIRED 11
#define SHDN_FRAME 12
#define SHDN_TSMS 13
#define SHDN_CRIT_RIDER 14
#define SHDN_CTRL_LOBATT 15

// port A; all unlabeled are input
#define DEBUG1 0 // out
#define RESET_CUBE_2 1 // out
#define SHDN_LOWER_RIDER 3
#define SHDN_NUC 2
#define SHDN_BMS_N 4
#define SHDN_BMS_E 5
#define SHDN_BMS_S 6
#define SHDN_BMS_ALL 7

FDCAN_HandleTypeDef hfdcan2;

// CS, MISO, MOSI, SCK
#define CS PA4
#define MISO PA6
#define MOSI PA7
#define SCK PA5

MCP23S17 MCP(CS);

bool mcpbegin = 0;

void OCTAL_Set_Clock(void) {
  // set flash latency for target (144MHz -> FLASH_LATENCY_4) and up voltage to HF range
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  // Request HSE ON (clock source mode from external high-speed crystal)
  RCC->CR |= RCC_CR_HSEON;

  //Move PLL to source off of HSE
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE);

  //Setup PLL for 48MHz peripheral clock
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;  // 12MHz after div
  RCC_OscInitStruct.PLL.PLLN = 24;             //2ds88Mhz after mult
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  //144MHz CPU clock
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;  //Peripherals at 48MHz (required by USB (48MHz only), CAN ahas a bit more leeway)
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  //Enable PLL
  SET_BIT(RCC->CR, RCC_CR_PLLON);

  //Set CPU and bus clocks to source 144MHz from PLL
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  //push new clock settings to time-dependent code/structs
  SystemCoreClockUpdate();

  //Start tick counter
  HAL_InitTick(TICK_INT_PRIORITY);
}

void setup() {
  OCTAL_Set_Clock();

  pinMode(SHDN_N, INPUT);
  pinMode(SHDN_NE, INPUT);
  pinMode(SHDN_E, INPUT);
  pinMode(SHDN_SE, INPUT);
  pinMode(SHDN_S, INPUT);
  pinMode(SHDN_SW, INPUT);
  pinMode(SHDN_W, INPUT);
  pinMode(SHDN_NW, INPUT);

  pinMode(CAN_COMS, OUTPUT);
  pinMode(USBC_COMS, OUTPUT);
  pinMode(BOOT0, INPUT);

  pinMode(RESET_IO, OUTPUT);

  pinMode(CUBE_HANDOFF, OUTPUT);
  pinMode(RXCAN, INPUT);
  pinMode(TXCAN, OUTPUT);

  pinMode(CUBE_IO_RESET, OUTPUT);
  pinMode(CTRL_NONCRIT_CHECK, OUTPUT);
  pinMode(SENSE_POWER_CYCLE, OUTPUT);

  pinMode(HEARTBEAT, OUTPUT);  // Core Heartbeat

  Serial.begin(112500);

  SPI.begin();
  mcpbegin = MCP.begin();
  delay(50);
  if (mcpbegin) {
    // GPA outputs: DEBUG1, RESET_CUBEs_2
    MCP.pinMode1(DEBUG1, OUTPUT);
    MCP.pinMode1(RESET_CUBE_2, OUTPUT);

    // GPB output: RESET_CUBE_1
    MCP.pinMode1(RESET_CUBE_1, OUTPUT);

    // everything else input (SHDN signals)
    MCP.pinMode1(SHDN_REMOTE, INPUT);
    MCP.pinMode1(SHDN_TILT, INPUT);
    MCP.pinMode1(SHDN_WIRED, INPUT);
    MCP.pinMode1(SHDN_FRAME, INPUT);
    MCP.pinMode1(SHDN_TSMS, INPUT);
    MCP.pinMode1(SHDN_CRIT_RIDER, INPUT);
    MCP.pinMode1(SHDN_CTRL_LOBATT, INPUT);

    MCP.pinMode1(SHDN_LOWER_RIDER, INPUT);
    MCP.pinMode1(SHDN_NUC, INPUT);
    MCP.pinMode1(SHDN_BMS_N, INPUT);
    MCP.pinMode1(SHDN_BMS_E, INPUT);
    MCP.pinMode1(SHDN_BMS_S, INPUT);
    MCP.pinMode1(SHDN_BMS_ALL, INPUT);


    // // Port 0 (GPA): Debug GPA0, ResetCube2 GPA1 are Outputs (0), others are SHDN Inputs (1)
    // MCP.pinMode8(0, 0b11111100); 
    
    // // Port 1 (GPB): ResetCube1 GPB0 is Output (0), others are SHDN Inputs (1)
    // MCP.pinMode8(1, 0b11111110); 

    MCP.write1(DEBUG1, HIGH); // prime output
    MCP.write1(RESET_CUBE_2, LOW); // prime output
    MCP.write1(RESET_CUBE_2, LOW); // prime output
    
    Serial.println("MCP23S17 Initialized.");
  } else {
    Serial.println("MCP23S17 not found.");
  }
  delay(100);


  can_init_core(&hfdcan2);
}

uint16_t parsedValue = 0;
float current_5v = 0;
float current_3v3 = 0;

const int shdnPins[] = {
  SHDN_N, SHDN_NE, SHDN_E, SHDN_SE, 
  SHDN_S, SHDN_SW, SHDN_W, SHDN_NW
};
const char* shdnLabels[] = {
  "N ", "NE", "E ", "SE", 
  "S ", "SW", "W ", "NW"
};

void printShutdownStatus() {
    if (mcpbegin) {
    Serial.println("--- MCP SHDN Status ---");

    const int mcpPins[] = {
      SHDN_REMOTE,
      SHDN_TILT,
      SHDN_WIRED,
      SHDN_FRAME,
      SHDN_TSMS,
      SHDN_CRIT_RIDER,
      SHDN_CTRL_LOBATT,

      SHDN_LOWER_RIDER,
      SHDN_NUC,
      SHDN_BMS_N,
      SHDN_BMS_E,
      SHDN_BMS_S,
      SHDN_BMS_ALL
    };

    const char* mcpLabels[] = {
      "REMOTE",
      "TILT",
      "WIRED",
      "FRAME",
      "TSMS",
      "CRIT",
      "LOBATT",

      "LOWER",
      "NUC",
      "BMS_N",
      "BMS_E",
      "BMS_S",
      "BMS_ALL"
    };

    for (int i = 0; i < 13; i++) {
      int state = MCP.read1(mcpPins[i]);

      Serial.print(mcpLabels[i]);
      Serial.print(": ");
      Serial.print(state == HIGH ? "CLOSED" : "OPEN");

      if ((i + 1) % 4 == 0) {
        Serial.println();
      } else {
        Serial.print(" | ");
      }
    }

    Serial.println();
  }

  Serial.println("--- SHDN Pin Status ---");
  
  for (int i = 0; i < 8; i++) {
    int state = digitalRead(shdnPins[i]);
    
    Serial.print(shdnLabels[i]);
    Serial.print(": ");
    Serial.print(state == HIGH ? "CLOSED" : "OPEN ");
    
    if ((i + 1) % 4 == 0) {
      Serial.println();
    } else {
      Serial.print(" | ");
    }
  }
  Serial.println("-----------------------");
}

float MAX_VPACK_DIFF = 0.080; // 80mV pack difference

void loop() {

  digitalToggle(HEARTBEAT);
  MCP.write1(DEBUG1, HIGH);
  delay(1000);


  current_5v = (analogRead(ASENSE_5V) * 3.3) / 1023.0;
  current_3v3 = (analogRead(ASENSE_3V3) * 3.3) / 1023.0;
  // Serial.print("Current 5V: ");
  // Serial.println(current_5v);
  // Serial.print("Current 3V3: ");
  // Serial.println(current_3v3);

  // printShutdownStatus();

  delay(100);
  core_status.core_state = 0;
  Serial.print("DEBUG1 raw read: ");
  Serial.println(MCP.read1(DEBUG1), HEX);

  digitalWrite(RESET_IO, HIGH);

  if (Serial.available()) {
    digitalWrite(USBC_COMS, HIGH);
    String input = Serial.readStringUntil('\n');
    input.trim();

    long value = input.toInt();

    if (value >= 0 && value <= 65535) {
      parsedValue = value;

      Serial.print("Parsed uint16_t value: ");
      Serial.println(parsedValue, HEX);
    } else {
      Serial.println("Value out of uint16_t range.");
    }
  } else {
    digitalWrite(USBC_COMS, LOW);
  }

  if (can_receive(&hfdcan2) > 0) {
    Serial.print("CAN RECEIVED");
    digitalWrite(CAN_COMS, HIGH);
    
    // // pack voltage reading
    // float min_volt = bms_status_n.pack_voltage;
    // if (bms_status_e.pack_voltage < min_volt) min_volt = bms_status_e.pack_voltage;
    // if (bms_status_s.pack_voltage < min_volt) min_volt = bms_status_s.pack_voltage;
    // if (bms_status_w.pack_voltage < min_volt) min_volt = bms_status_w.pack_voltage;

    // core_batt_ctrl.north = (bms_status_n.pack_voltage <= (min_volt + MAX_VPACK_DIFF)) ? 1 : 0;
    // core_batt_ctrl.east  = (bms_status_e.pack_voltage <= (min_volt + MAX_VPACK_DIFF)) ? 1 : 0;
    // core_batt_ctrl.south = (bms_status_s.pack_voltage <= (min_volt + MAX_VPACK_DIFF)) ? 1 : 0;
    // core_batt_ctrl.west  = (bms_status_w.pack_voltage <= (min_volt + MAX_VPACK_DIFF)) ? 1 : 0;

    // if (core_batt_ctrl == 1111) {
    //   MCP.write1(DEBUG1, HIGH);
    // }
    
    Serial.print(CAN_COMS);
  } else {
    digitalWrite(CAN_COMS, LOW);
  }

  can_send_core_status(&hfdcan2);
  can_send_core_batt_ctrl(&hfdcan2);
}
