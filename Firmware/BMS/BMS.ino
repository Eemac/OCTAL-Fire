#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_api.h"
#include "ADBMS181x.h"
#include "bms_hardware.h"
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#define LED PB9
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

enum BmsState {
  IDLE,
  FAULTED
};

enum BmsFaultBits {
  FAULT_ADBMS_COMMS    = 2,
  FAULT_MUX            = 4,
  FAULT_CELL_UNDERTEMP = 5,
  FAULT_CELL_OVERTEMP  = 6,
  FAULT_CELL_UNDERVOLT = 7,
  FAULT_CELL_OVERVOLT  = 8,
  FAULT_PEC            = 15
};

BmsState bms_state = IDLE;
uint16_t bms_faults = 0;

const float CELL_VOLTAGE_LOW  = 2.5f; // Min value is can handle is 2.5(discharge)
const float CELL_VOLTAGE_HIGH = 4.25f; // Max value it can handle is 4.25(discharge)

//Must add the over current later aswell
const float TEMP_LOW_C  = -20.0f; // Min value it cna take is -20.0f(discharge)
const float TEMP_HIGH_C = 55.0f; // Max value it can tolerate = 55.0f(discharge)

// Stored measurement summaries
float min_cell_voltage = 100.0f;
float max_cell_voltage = 0.0f;
float pack_voltage = 0.0f;

float min_temp_c = 1000.0f;
float max_temp_c = -1000.0f;

int8_t last_pec_error = 0;
bool last_mux_error = false;

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
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC->CR |= RCC_CR_HSEON;
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE);

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  SET_BIT(RCC->CR, RCC_CR_PLLON);

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  SystemCoreClockUpdate();
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

float therm_voltage_to_temp_c(float v_out)
{
  if (v_out <= 0.0f || v_out >= 3.3f) {
    return -1000.0f;
  }

  float temp_k = 1.0f / (
    (1.0f / 298.15f) +
    (1.0f / 4250.0f) * log(v_out / (3.3f - v_out))
  );

  return temp_k - 273.15f;
}

void set_fault_bit(uint8_t bit)
{
  bms_faults |= (1U << bit);
}

bool read_cell_voltages()
{
  min_cell_voltage = 100.0f;
  max_cell_voltage = 0.0f;
  pack_voltage = 0.0f;

  wakeup_idle(TOTAL_IC);
  ADBMS181x_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
  ADBMS181x_pollAdc();

  last_pec_error = ADBMS181x_rdcv(0, TOTAL_IC, bms_ic);

  for (int ic = 0; ic < TOTAL_IC; ic++)
  {
    for (int cell = 0; cell < 18; cell++)
    {
      float volts = bms_ic[ic].cells.c_codes[cell] * 0.0001f;

      if (volts < min_cell_voltage) min_cell_voltage = volts;
      if (volts > max_cell_voltage) max_cell_voltage = volts;

      pack_voltage += volts;
    }
  }

  return (last_pec_error == 0);
}

bool read_all_thermistors()
{
  min_temp_c = 1000.0f;
  max_temp_c = -1000.0f;
  last_mux_error = false;

  for (uint8_t index = 0; index < 16; index++)
  {
    bool mux1_ok = set_mux_channels(MUX1_ADDR, index, index);
    bool mux2_ok = set_mux_channels(MUX2_ADDR, index, index);

    if (!mux1_ok || !mux2_ok)
    {
      last_mux_error = true;
      return false;
    }

    delay(5);

    float t1 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_A1_PIN)));
    float t2 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_B1_PIN)));
    float t3 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_A2_PIN)));
    float t4 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_B2_PIN)));

    if (t1 < min_temp_c) min_temp_c = t1;
    if (t2 < min_temp_c) min_temp_c = t2;
    if (t3 < min_temp_c) min_temp_c = t3;
    if (t4 < min_temp_c) min_temp_c = t4;

    if (t1 > max_temp_c) max_temp_c = t1;
    if (t2 > max_temp_c) max_temp_c = t2;
    if (t3 > max_temp_c) max_temp_c = t3;
    if (t4 > max_temp_c) max_temp_c = t4;

    delay(20);
  }

  return true;
}

void update_faults()
{
  bms_faults = 0;

  if (last_pec_error != 0)
  {
    set_fault_bit(FAULT_ADBMS_COMMS);
    set_fault_bit(FAULT_PEC);
  }

  if (last_mux_error)
  {
    set_fault_bit(FAULT_MUX);
  }

  if (min_cell_voltage < CELL_VOLTAGE_LOW)
  {
    set_fault_bit(FAULT_CELL_UNDERVOLT);
  }

  if (max_cell_voltage > CELL_VOLTAGE_HIGH)
  {
    set_fault_bit(FAULT_CELL_OVERVOLT);
  }

  if (min_temp_c < TEMP_LOW_C)
  {
    set_fault_bit(FAULT_CELL_UNDERTEMP);
  }

  if (max_temp_c > TEMP_HIGH_C)
  {
    set_fault_bit(FAULT_CELL_OVERTEMP);
  }
}

void print_faults()
{
  Serial.print("Fault mask: 0b");
  Serial.println(bms_faults, BIN);
  if (bms_faults & (1U << FAULT_ADBMS_COMMS))    Serial.println("FAULT: ADBMS comms");
  if (bms_faults & (1U << FAULT_MUX))            Serial.println("FAULT: Mux");
  if (bms_faults & (1U << FAULT_PEC))            Serial.println("FAULT: PEC");
  if (bms_faults & (1U << FAULT_CELL_UNDERVOLT)) Serial.println("FAULT: Cell undervoltage");
  if (bms_faults & (1U << FAULT_CELL_OVERVOLT))  Serial.println("FAULT: Cell overvoltage");
  if (bms_faults & (1U << FAULT_CELL_UNDERTEMP)) Serial.println("FAULT: Cell undertemp");
  if (bms_faults & (1U << FAULT_CELL_OVERTEMP))  Serial.println("FAULT: Cell overtemp");
}

void setup()
{
  OCTAL_Set_Clock();

  pinMode(LED, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PB2, OUTPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB11, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PA15, OUTPUT);
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

  // can_init_bms_a(&hfdcan2);
}

void loop()
{
  switch (bms_state)
  {
    case IDLE:
    {
      bool cells_ok = read_cell_voltages();
      bool therms_ok = read_all_thermistors();

      update_faults();

      Serial.println("BMS state: IDLE");
      Serial.print("Min cell voltage: ");
      Serial.println(min_cell_voltage, 4);
      Serial.print("Max cell voltage: ");
      Serial.println(max_cell_voltage, 4);
      Serial.print("Min temp C: ");
      Serial.println(min_temp_c, 2);
      Serial.print("Max temp C: ");
      Serial.println(max_temp_c, 2);

      if (!cells_ok || !therms_ok || bms_faults != 0)
      {
        bms_state = FAULTED;
      }

      break;
    }

    case FAULTED:
    {
      Serial.println("BMS state: FAULTED");
      print_faults();

      digitalToggle(PB9);
      delay(500);
      break;
    }
  }

  delay(200);
}