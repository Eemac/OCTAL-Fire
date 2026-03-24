#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_api.h"
#include "ADBMS181x.h"
#include "bms_hardware.h"
#include <SPI.h>
#include <Wire.h>

#define LED PB9   // QFP48 socket PCB
#define TOTAL_IC 1

#define THERM_A1_PIN PA0
#define THERM_B1_PIN PA1
#define THERM_A2_PIN PA2
#define THERM_B2_PIN PA3

#define MUX_SDA_PIN PA8
#define MUX_SCL_PIN PA9

#define MUX1_ADDR 0x4C
#define MUX2_ADDR 0x4E

#define MAX14661_CMD_A 0x14
#define MAX14661_CMD_B 0x15

cell_asic bms_ic[TOTAL_IC];
FDCAN_HandleTypeDef hfdcan2;

const char *mux1_names[16] = {
  "TH_1_A", "TH_1_B", "TH_1_C", "TH_1_D",
  "TH_2_A", "TH_2_B", "TH_2_C", "TH_2_D",
  "TH_3_C", "TH_3_D", "TH_3_A", "TH_3_B",
  "TH_4_A", "TH_4_B", "TH_4_C", "TH_4_D"
};

const char *mux2_names[16] = {
  "TH_5_A", "TH_5_B", "TH_5_C", "TH_5_D",
  "TH_6_A", "TH_6_B", "TH_6_C", "TH_6_D",
  "TH_7_A", "TH_7_B", "TH_7_C", "TH_7_D",
  "THERM_BUSBAR", "THERM_AMBIENT", "UNUSED_15", "UNUSED_16"
};

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

void init_bms_chip_array()
{
  for (int i = 0; i < TOTAL_IC; i++)
  {
    bms_ic[i].isospi_reverse = false;

    bms_ic[i].ic_reg.cell_channels = 18;
    bms_ic[i].ic_reg.stat_channels = 4;
    bms_ic[i].ic_reg.aux_channels = 9;

    bms_ic[i].ic_reg.num_cv_reg = 6;
    bms_ic[i].ic_reg.num_gpio_reg = 4;
    bms_ic[i].ic_reg.num_stat_reg = 2;

    bms_ic[i].system_open_wire = 0;

    for (int j = 0; j < 18; j++)
    {
      bms_ic[i].cells.c_codes[j] = 0;
    }

    for (int j = 0; j < 6; j++)
    {
      bms_ic[i].cells.pec_match[j] = 0;
    }
  }

  ADBMS181x_init_cfg(TOTAL_IC, bms_ic);
  ADBMS181x_reset_crc_count(TOTAL_IC, bms_ic);
}

void read_and_print_cell_voltages()
{
  //Voltage values will read as max(somehwere around 6.55 if no battery source connected to board)
  wakeup_idle(TOTAL_IC);

  ADBMS181x_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
  ADBMS181x_pollAdc();
  

  int8_t pec_error = ADBMS181x_rdcv(0, TOTAL_IC, bms_ic);

  Serial.print("PEC status: ");
  Serial.println(pec_error);

  for (int ic = 0; ic < TOTAL_IC; ic++)
  {
    Serial.print("IC ");
    Serial.println(ic + 1);

    for (int cell = 0; cell < 18; cell++)
    {
      uint16_t raw = bms_ic[ic].cells.c_codes[cell];
      float volts = raw * 0.0001f;

      Serial.print("C");
      Serial.print(cell + 1);
      Serial.print(": raw=");
      Serial.print(raw);
      Serial.print("  V=");
      Serial.println(volts, 4);
    }
  }

  Serial.println();
}

void init_thermistors()
{
  pinMode(THERM_A1_PIN, INPUT_ANALOG);
  pinMode(THERM_B1_PIN, INPUT_ANALOG);
  pinMode(THERM_A2_PIN, INPUT_ANALOG);
  pinMode(THERM_B2_PIN, INPUT_ANALOG);

  Wire.setSDA(MUX_SDA_PIN);
  Wire.setSCL(MUX_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  analogReadResolution(12);
}

bool set_mux_channels(uint8_t mux_addr, uint8_t chA, uint8_t chB)
{
  if (chA > 0x10 || chB > 0x10) return false;

  Wire.beginTransmission(mux_addr);
  Wire.write(MAX14661_CMD_A);
  Wire.write(chA);
  Wire.write(chB);
  return (Wire.endTransmission() == 0);
}

float raw_to_therm_voltage(uint16_t raw)
{
  return (3.3f * raw) / 4095.0f;
}

void print_single_therm_reading(const char *name, uint16_t raw)
{
  Serial.print(name);
  Serial.print(": raw=");
  Serial.print(raw);
  Serial.print("  V=");
  Serial.println(raw_to_therm_voltage(raw), 4);
}

void read_and_print_thermistors_for_index(uint8_t index)
{
  bool mux1_ok = set_mux_channels(MUX1_ADDR, index, index);
  bool mux2_ok = set_mux_channels(MUX2_ADDR, index, index);

  if (!mux1_ok || !mux2_ok)
  {
    Serial.println("Thermistor mux write failed");
    return;
  }

  delay(5);

  uint16_t raw_a1 = analogRead(THERM_A1_PIN);
  uint16_t raw_b1 = analogRead(THERM_B1_PIN);
  uint16_t raw_a2 = analogRead(THERM_A2_PIN);
  uint16_t raw_b2 = analogRead(THERM_B2_PIN);

  Serial.print("Thermistor index ");
  Serial.println(index + 1);

  Serial.print("THERM_A1 -> ");
  print_single_therm_reading(mux1_names[index], raw_a1);

  Serial.print("THERM_B1 -> ");
  print_single_therm_reading(mux1_names[index], raw_b1);

  Serial.print("THERM_A2 -> ");
  print_single_therm_reading(mux2_names[index], raw_a2);

  Serial.print("THERM_B2 -> ");
  print_single_therm_reading(mux2_names[index], raw_b2);

  Serial.println();
}

void scan_all_thermistors()
{
  for (uint8_t index = 0; index < 16; index++)
  {
    read_and_print_thermistors_for_index(index);
    delay(20);
  }
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
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  digitalToggle(PB9);
  digitalToggle(PB2);

  Serial.begin(115200);
  delay(200);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  init_bms_chip_array();
  init_thermistors();
  //can_init_bms_a(&hfdcan2);

  read_and_print_cell_voltages();
  scan_all_thermistors();


  // initTemp(tempStruct);
  //InitBMS(adStruct);
  //doDebug(ledStruct, tempStruct, adStruct);
  //setCurrentSense(ivStruct);
  //setupFans(tempStruct);
}

uint16_t parsedValue = 0;

void loop()
{
  //digitalToggle(PB1);
  //digitalToggle(PB2);
  Serial.println("Starting cell read...");
  read_and_print_cell_voltages();
  Serial.println("Starting thermistor read...");
  scan_all_thermistors();

  delay(1000);
  // bms_status.bms_fault_code = 2;
  // can_send_bms_status(&hfdcan2);
  //uint8_t pvb = parsedValue & 0xFF;
  //uint8_t pvt = (parsedValue >> 8) & 0x3F;
  //Serial.println(parsedValue, HEX);
  //long total = pvb | pvt << 10 | pvb
  //thrust.b1 = pvb;
  //thrust.b2 =pvt << 2;
  // thrust.b3 = 0xA0;
  // thrust.m3 = parsedValue;
  // thrust.m4 = parsedValue;
  //thrust.tail_byte = 0xC0;
  //can_send_thrust(&hfdcan2);

   //if (Serial.available()) {
    //String input = Serial.readStringUntil('\n');
    //input.trim();  // remove whitespace

    // Check that it starts with 'S

      //long value = input.toInt();  // convert to integer

      // Ensure it fits in uint16_t
      //if (value >= 0 && value <= 65535) {
        //parsedValue = value;
        //Serial.print(parsedValue);
        //Serial.print("Parsed uint16_t value: ");
        //Serial.println(parsedValue, HEX);
      //} //else {
        //Serial.println("Value out of uint16_t range.");
      //}
  //}

  //if(can_receive(&hfdcan2) > 0) {
    //Serial.print(msg1.pwm);
    //Serial.print(" ");
  //}
}

