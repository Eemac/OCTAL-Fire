#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_OPAMP_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_opamp.h"

#include "can_api.h"
#include "can_lib.h"
#include "ADBMS181x.h"
#include "bms_hardware.h"

#include <SPI.h>
#include <Wire.h>
#include <math.h>

#define LED PB9
#define ERROR_LED PB4
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

#define ISENSE_PIN PB0

cell_asic bms_ic;
FDCAN_HandleTypeDef hfdcan2;
OPAMP_HandleTypeDef hopamp2;
ADC_HandleTypeDef hadc2;

enum BmsState {
  CHARGING_IDLE = 0,
  CHARGING_ACTIVE = 1,
  BALANCING = 2,
  ACTIVE = 3,
  IDLE = 4,
  FAULTED = 5,
  SURVIVAL = 6
};

enum BmsFaultBits {
  FAULT_CAN_ERROR       = 0,
  FAULT_LOOP_TIMING     = 1,
  FAULT_ADBMS_COMMS     = 2,
  FAULT_ADBMS_OVERTEMP  = 3,
  FAULT_MUX             = 4,
  FAULT_CELL_UNDERTEMP  = 5,
  FAULT_CELL_OVERTEMP   = 6,
  FAULT_CELL_UNDERVOLT  = 7,
  FAULT_CELL_OVERVOLT   = 8,
  FAULT_OPEN_WIRE       = 9,
  FAULT_CURRENT_SENSE   = 10,
  FAULT_OVERCURRENT     = 11,
  FAULT_3V3_BROWNOUT    = 12,
  FAULT_5V_BROWNOUT     = 13,
  FAULT_CHARGER_COMMS   = 14,
  FAULT_PEC             = 15
};

uint16_t bms_faults = 0;

const float CELL_VOLTAGE_LOW  = 2.5f;
const float CELL_VOLTAGE_HIGH = 4.20f;

const float TEMP_LOW_C  = -20.0f;
const float TEMP_HIGH_C = 60.0f;

int8_t last_pec_error = 0;
bool last_mux_error = false;

// Isense Current reading 2x

const float ISENSE_ADC_REF_V = 3.3f;
const float ISENSE_ADC_MAX = 4095.0f;

// Might Recalibrate since this was made with 4x values
const float ISENSE_ZERO_RAW = 241.18f;

// Rough 2x gain estimate from your 4x calibration.
const float ISENSE_COUNTS_PER_AMP = 7.32f;
const float ISENSE_SIGN = 1.0f;
const int ISENSE_AVERAGE_SAMPLES = 100;
const int ISENSE_SAMPLE_DELAY_US = 50;
const float ISENSE_DEADBAND_A = 0.15f;
const float OVERCURRENT_LIMIT_A = 500.0f;
const int OVERCURRENT_TRIP_COUNT = 3;
const int PEC_TRIP_COUNT = 3;
uint8_t overcurrent_counter = 0;
uint8_t pec_error_counter = 0;
bool overcurrent_fault_latched = false;
bool pec_fault_latched = false;
float last_isense_raw = 0.0f;
float last_isense_2x_voltage = 0.0f;
float last_isense_current_A = 0.0f;

// Thermistors

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

extern "C" void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp)
{
  if (hopamp->Instance == OPAMP2)
  {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

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
    bms_ic.isospi_reverse = false;

    bms_ic.ic_reg.cell_channels = 18;
    bms_ic.ic_reg.stat_channels = 4;
    bms_ic.ic_reg.aux_channels = 9;

    bms_ic.ic_reg.num_cv_reg = 6;
    bms_ic.ic_reg.num_gpio_reg = 4;
    bms_ic.ic_reg.num_stat_reg = 2;

    bms_ic.system_open_wire = 0;

    for (int j = 0; j < 18; j++)
    {
      bms_ic.cells.c_codes[j] = 0;
    }

    for (int j = 0; j < 6; j++)
    {
      bms_ic.cells.pec_match[j] = 0;
    }
  }

  ADBMS181x_init_cfg(1, &bms_ic);
  ADBMS181x_reset_crc_count(1, &bms_ic);
}

bool set_mux_channels(uint8_t mux_addr, uint8_t chA, uint8_t chB)
{
  if (chA > 15 || chB > 15) return false;

  Wire.beginTransmission(mux_addr);
  Wire.write(MAX14661_CMD_A);
  Wire.write(chA);
  Wire.write(chB);
  return (Wire.endTransmission() == 0);
}

uint16_t thermistorEncode(float t) {
  if (t <= -20.0f) return 0;
  if (t >= 120.0f) return 65535;
  return (uint16_t)((t + 20.0f) / 140.0f * 65535.0f);
}

float thermistorDecode(uint16_t t) {
  return (float)(t * 140.0f / 65535.0f - 20.0f);
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

  set_mux_channels(MUX1_ADDR, 0, 1);
  set_mux_channels(MUX2_ADDR, 0, 1);

  bms_temp_metrics_a.max_cell_temp_ever = thermistorEncode(0.0f);
  bms_temp_metrics_a.min_cell_temp_ever = thermistorEncode(120.0f);
}

float raw_to_therm_voltage(uint16_t raw)
{
  return (3.3f * raw) / 4095.0f;
}

float therm_voltage_to_temp_c(float v_out)
{
  if (v_out <= 0.0f || v_out >= 3.3f) {
    return 0.0f;
  }

  float temp_k = 1.0f / (
    (1.0f / 298.15f) +
    (1.0f / 4250.0f) * log(v_out / (3.3f - v_out))
  );

  return temp_k - 273.15f;
}

float isenseRawToVoltage(float raw)
{
  return (raw * ISENSE_ADC_REF_V) / ISENSE_ADC_MAX;
}

float isenseRawToCurrentAmps(float raw)
{
  float current_A = (raw - ISENSE_ZERO_RAW) / ISENSE_COUNTS_PER_AMP;
  current_A *= ISENSE_SIGN;

  if (fabs(current_A) < ISENSE_DEADBAND_A)
  {
    current_A = 0.0f;
  }

  return current_A;
}

void update_pec_counter()
{
  if (last_pec_error != 0)
  {
    if (pec_error_counter < PEC_TRIP_COUNT)
    {
      pec_error_counter++;
    }
  }
  else
  {
    pec_error_counter = 0;
  }

  if (pec_error_counter >= PEC_TRIP_COUNT)
  {
    pec_fault_latched = true;
  }
}

void update_overcurrent_counter()
{
  float abs_current_A = fabs(last_isense_current_A);

  if (abs_current_A > OVERCURRENT_LIMIT_A)
  {
    if (overcurrent_counter < OVERCURRENT_TRIP_COUNT)
    {
      overcurrent_counter++;
    }
  }
  else
  {
    overcurrent_counter = 0;
  }

  if (overcurrent_counter >= OVERCURRENT_TRIP_COUNT)
  {
    overcurrent_fault_latched = true;
  }
}

bool configureADC2ForVOPAMP2Channel()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Serial.println("ERROR: ADC2 VOPAMP2 channel config failed");
    return false;
  }

  return true;
}

bool reinitADC2ForVOPAMP2()
{
  HAL_ADC_Stop(&hadc2);
  HAL_ADC_DeInit(&hadc2);

  __HAL_RCC_ADC12_CLK_ENABLE();

  hadc2.Instance = ADC2;

  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Serial.println("ERROR: ADC2 re-init failed");
    return false;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Serial.println("ERROR: ADC2 re-calibration failed");
    return false;
  }

  return configureADC2ForVOPAMP2Channel();
}

void initISense()
{
  pinMode(ISENSE_PIN, INPUT_ANALOG);

  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;

  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp2.Init.InternalOutput = ENABLE;

  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;

  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp2.Init.InvertingInputSecondary = OPAMP_SEC_INVERTINGINPUT_PGA;
  hopamp2.Init.NonInvertingInputSecondary = OPAMP_SEC_NONINVERTINGINPUT_IO2;

  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;

  // 2x gain
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;

  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  hopamp2.Init.TrimmingValueP = 0;
  hopamp2.Init.TrimmingValueN = 0;

  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Serial.println("ERROR: OPAMP2 init failed");
  }

  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    Serial.println("ERROR: OPAMP2 start failed");
  }

  delay(20);

  if (!reinitADC2ForVOPAMP2())
  {
    Serial.println("ERROR: initial ADC2 VOPAMP2 setup failed");
  }

  Serial.println("ISENSE OPAMP2 2x + ADC2 VOPAMP2 initialized");
}

int32_t readVOPAMP2Once()
{
  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc2);
    return -1;
  }

  if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc2);
    return -1;
  }

  uint16_t raw = HAL_ADC_GetValue(&hadc2);
  HAL_ADC_Stop(&hadc2);

  return raw;
}

float readVOPAMP2Average(int samples)
{
  uint32_t sum = 0;
  int good_samples = 0;

  if (!reinitADC2ForVOPAMP2())
  {
    return -1.0f;
  }

  for (int i = 0; i < samples; i++)
  {
    int32_t raw = readVOPAMP2Once();

    if (raw >= 0)
    {
      sum += raw;
      good_samples++;
    }

    delayMicroseconds(ISENSE_SAMPLE_DELAY_US);
  }

  if (good_samples == 0)
  {
    return -1.0f;
  }

  return (float)sum / (float)good_samples;
}

void readIsense()
{
  float raw = readVOPAMP2Average(ISENSE_AVERAGE_SAMPLES);

  if (raw < 0.0f)
  {
    Serial.println("ERROR: ISENSE ADC read failed");
    return;
  }

  last_isense_raw = raw;
  last_isense_2x_voltage = isenseRawToVoltage(last_isense_raw);
  last_isense_current_A = isenseRawToCurrentAmps(last_isense_raw);

  update_overcurrent_counter();

  float current_for_can_A = last_isense_current_A;

  if (current_for_can_A < 0.0f)
  {
    current_for_can_A = 0.0f;
  }

  if (current_for_can_A > 511.0f)
  {
    current_for_can_A = 511.0f;
  }

  bms_status_a.pack_current = (int16_t)roundf(current_for_can_A);
}

void update_error_light()
{
  bool pec_error_active = (bms_faults & (1U << FAULT_PEC));

  if (pec_error_active)
  {
    digitalWrite(ERROR_LED, HIGH);
  }
  else
  {
    digitalWrite(ERROR_LED, LOW);
  }
}

void printIsenseDebug()
{
  Serial.print("ISENSE raw ADC: ");
  Serial.print(last_isense_raw, 2);

  Serial.print(" | ISENSE 2x voltage: ");
  Serial.print(last_isense_2x_voltage, 4);
  Serial.print(" V");

  Serial.print(" | Current actual: ");
  Serial.print(last_isense_current_A, 3);
  Serial.print(" A");

  Serial.print(" | OC count: ");
  Serial.print(overcurrent_counter);

  Serial.print(" | PEC count: ");
  Serial.print(pec_error_counter);

  Serial.print(" | Fault mask: 0b");
  Serial.print(bms_faults, BIN);

  Serial.print(" | State: ");
  Serial.print(bms_status_a.bms_state);

  Serial.print(" | CAN pack_current raw: ");
  Serial.println(bms_status_a.pack_current);
}

void set_fault_bit(uint8_t bit)
{
  bms_faults |= (1U << bit);
}

bool read_cell_voltages() {
  bms_voltage_metrics_a.min_cell_voltage =
    can_lib_bms_voltage_metrics_a_min_cell_voltage_encode(5000.0f);

  bms_voltage_metrics_a.max_cell_voltage =
    can_lib_bms_voltage_metrics_a_max_cell_voltage_encode(0.0f);

  float pack_sum_mv = 0.0f;

  wakeup_idle(1);
  ADBMS181x_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
  ADBMS181x_pollAdc();

  last_pec_error = ADBMS181x_rdcv(0, 1, &bms_ic);
  update_pec_counter();

  for (int cell = 0; cell < 14; cell++)
  {
    float volts = bms_ic.cells.c_codes[cell] * 0.0001f;
    float mv = volts * 1000.0f;

    switch (cell)
    {
      case 0: cell_voltages_0_a.cell_1 = can_lib_cell_voltages_0_a_cell_1_encode(mv); break;
      case 1: cell_voltages_0_a.cell_2 = can_lib_cell_voltages_0_a_cell_2_encode(mv); break;
      case 2: cell_voltages_0_a.cell_3 = can_lib_cell_voltages_0_a_cell_3_encode(mv); break;
      case 3: cell_voltages_0_a.cell_4 = can_lib_cell_voltages_0_a_cell_4_encode(mv); break;
      case 4: cell_voltages_0_a.cell_5 = can_lib_cell_voltages_0_a_cell_5_encode(mv); break;

      case 5: cell_voltages_1_a.cell_6 = can_lib_cell_voltages_1_a_cell_6_encode(mv); break;
      case 6: cell_voltages_1_a.cell_7 = can_lib_cell_voltages_1_a_cell_7_encode(mv); break;
      case 7: cell_voltages_1_a.cell_8 = can_lib_cell_voltages_1_a_cell_8_encode(mv); break;
      case 8: cell_voltages_1_a.cell_9 = can_lib_cell_voltages_1_a_cell_9_encode(mv); break;
      case 9: cell_voltages_1_a.cell_10 = can_lib_cell_voltages_1_a_cell_10_encode(mv); break;

      case 10: cell_voltages_2_a.cell_11 = can_lib_cell_voltages_2_a_cell_11_encode(mv); break;
      case 11: cell_voltages_2_a.cell_12 = can_lib_cell_voltages_2_a_cell_12_encode(mv); break;
      case 12: cell_voltages_2_a.cell_13 = can_lib_cell_voltages_2_a_cell_13_encode(mv); break;
      case 13: cell_voltages_2_a.cell_14 = can_lib_cell_voltages_2_a_cell_14_encode(mv); break;
    }

    if (mv < can_lib_bms_voltage_metrics_a_min_cell_voltage_decode(bms_voltage_metrics_a.min_cell_voltage))
    {
      bms_voltage_metrics_a.min_cell_voltage =
        can_lib_bms_voltage_metrics_a_min_cell_voltage_encode(mv);
    }

    if (mv > can_lib_bms_voltage_metrics_a_max_cell_voltage_decode(bms_voltage_metrics_a.max_cell_voltage))
    {
      bms_voltage_metrics_a.max_cell_voltage =
        can_lib_bms_voltage_metrics_a_max_cell_voltage_encode(mv);
    }

    pack_sum_mv += mv;
  }

  bms_status_a.pack_voltage =
    can_lib_bms_status_a_pack_voltage_encode(pack_sum_mv);

  bms_voltage_metrics_a.cell_voltage_threshold_high =
    can_lib_bms_voltage_metrics_a_cell_voltage_threshold_high_encode(CELL_VOLTAGE_HIGH * 1000.0f);

  bms_voltage_metrics_a.cell_voltage_threshold_low =
    can_lib_bms_voltage_metrics_a_cell_voltage_threshold_low_encode(CELL_VOLTAGE_LOW * 1000.0f);

  if (last_pec_error == 0) {
    digitalToggle(PB11);
  }

  return (last_pec_error == 0);
}

uint8_t thermistors_index = 0;

bool read_thermistors() {
  last_mux_error = false;

  if (thermistors_index == 0)
  {
    bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(120.0f);
    bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(-20.0f);
  }

  float t1 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_A1_PIN)));
  float t2 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_B1_PIN)));
  float t3 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_A2_PIN)));
  float t4 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_B2_PIN)));

  float temps[4] = {t1, t2, t3, t4};

  float min_current = thermistorDecode(bms_temp_metrics_a.min_current_cell_temp);
  float max_current = thermistorDecode(bms_temp_metrics_a.max_current_cell_temp);
  float min_ever = thermistorDecode(bms_temp_metrics_a.min_cell_temp_ever);
  float max_ever = thermistorDecode(bms_temp_metrics_a.max_cell_temp_ever);

  for (int i = 0; i < 4; i++)
  {
    if (temps[i] < min_current) min_current = temps[i];
    if (temps[i] > max_current) max_current = temps[i];

    if (temps[i] < min_ever) min_ever = temps[i];
    if (temps[i] > max_ever) max_ever = temps[i];
  }

  bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(min_current);
  bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(max_current);
  bms_temp_metrics_a.min_cell_temp_ever = thermistorEncode(min_ever);
  bms_temp_metrics_a.max_cell_temp_ever = thermistorEncode(max_ever);

  thermistors_index += 2;
  if (thermistors_index >= 16)
  {
    thermistors_index = 0;
  }

  bool mux1_ok = set_mux_channels(MUX1_ADDR, thermistors_index, thermistors_index + 1);
  if (mux1_ok) {
    digitalToggle(PB7);
  }

  bool mux2_ok = set_mux_channels(MUX2_ADDR, thermistors_index, thermistors_index + 1);
  if (mux2_ok) {
    digitalToggle(PA15);
  }

  if (!mux1_ok || !mux2_ok)
  {
    last_mux_error = true;
    return false;
  }

  return true;
}

void update_faults() {
  bms_faults = 0;

  if (pec_fault_latched)
  {
    set_fault_bit(FAULT_ADBMS_COMMS);
    set_fault_bit(FAULT_PEC);
  }

  if (last_mux_error)
  {
    set_fault_bit(FAULT_MUX);
  }

  if (overcurrent_fault_latched)
  {
    set_fault_bit(FAULT_OVERCURRENT);
  }

  if (can_lib_bms_voltage_metrics_a_min_cell_voltage_decode(bms_voltage_metrics_a.min_cell_voltage) < CELL_VOLTAGE_LOW * 1000.0f)
  {
    set_fault_bit(FAULT_CELL_UNDERVOLT);
  }

  if (can_lib_bms_voltage_metrics_a_max_cell_voltage_decode(bms_voltage_metrics_a.max_cell_voltage) > CELL_VOLTAGE_HIGH * 1000.0f)
  {
    set_fault_bit(FAULT_CELL_OVERVOLT);
  }

  // Use CURRENT thermistor scan for active temp faulting.
  if (thermistorDecode(bms_temp_metrics_a.min_current_cell_temp) < TEMP_LOW_C)
  {
    set_fault_bit(FAULT_CELL_UNDERTEMP);
  }

  if (thermistorDecode(bms_temp_metrics_a.max_current_cell_temp) > TEMP_HIGH_C)
  {
    set_fault_bit(FAULT_CELL_OVERTEMP);
  }

  bms_status_a.bms_faults = bms_faults;
}

void print_faults() {
  Serial.print("Fault mask: 0b");
  Serial.println(bms_faults, BIN);

  if (bms_faults & (1U << FAULT_ADBMS_COMMS))    Serial.println("FAULT: ADBMS comms");
  if (bms_faults & (1U << FAULT_MUX))            Serial.println("FAULT: Mux");
  if (bms_faults & (1U << FAULT_PEC))            Serial.println("FAULT: PEC");
  if (bms_faults & (1U << FAULT_OVERCURRENT))    Serial.println("FAULT: Overcurrent");
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
  pinMode(ERROR_LED, OUTPUT);
  pinMode(PB5, OUTPUT);
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PB11, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PA10, OUTPUT);
  pinMode(PA15, OUTPUT);

  digitalWrite(ERROR_LED, LOW);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // DISABLE BMS CONTROL OF FETS
  digitalToggle(PA10);

  Serial.begin(115200);
  delay(200);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  init_bms_chip_array();
  init_thermistors();
  initISense();

  can_init_bms_a(&hfdcan2);

  bms_status_a.bms_state = IDLE;

  Serial.println("BMS started with ISENSE OPAMP2 2x current reading.");
}

void run_bms_cycle()
{
  bool cells_ok = read_cell_voltages();

  // Read current before thermistors because thermistors use analogRead().
  readIsense();

  bool therms_ok = read_thermistors();

  update_faults();

  // ERROR LED is only controlled by PEC error now.
  update_error_light();

  can_send_bms_status_a(&hfdcan2);
  can_send_bms_temp_metrics_a(&hfdcan2);
  can_send_bms_voltage_metrics_a(&hfdcan2);
  can_send_cell_voltages_0_a(&hfdcan2);
  can_send_cell_voltages_1_a(&hfdcan2);
  can_send_cell_voltages_2_a(&hfdcan2);

  digitalToggle(PB2);

  switch (bms_status_a.bms_state)
  {
    case IDLE:
    {
      if (!cells_ok || !therms_ok || bms_faults != 0)
      {
        bms_status_a.bms_state = FAULTED;
      }

      break;
    }

    case FAULTED:
    {
      // For bring-up only: allow return to IDLE if faults clear.
      // Later, we can make faults latch until reset.
      if (cells_ok && therms_ok && bms_faults == 0)
      {
        bms_status_a.bms_state = IDLE;
      }

      break;
    }

    default:
    {
      bms_status_a.bms_state = IDLE;
      break;
    }
  }
}

unsigned long ms50 = 0;
unsigned long ms200 = 0;
unsigned long ms1000 = 0;

void loop()
{
  unsigned long now = millis();

  if (now - ms50 >= 50)
  {
    ms50 = now;
    run_bms_cycle();
  }

  if (now - ms200 >= 200)
  {
    ms200 = now;
    digitalToggle(PB9);
  }

  if (now - ms1000 >= 1000)
  {
    ms1000 = now;
    printIsenseDebug();
  }

  delay(1);
}