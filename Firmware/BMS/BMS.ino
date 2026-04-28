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

// #define NORTH
// #define EAST
// #define SOUTH
#define WEST

#if (defined(NORTH) && defined(EAST)) || (defined(NORTH) && defined(SOUTH)) || (defined(NORTH) && defined(WEST)) || (defined(EAST) && defined(SOUTH)) || (defined(EAST) && defined(WEST)) || (defined(SOUTH) && defined(WEST))
    #error "More than one BMS config selected"
#endif

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

#define ISENSE_PIN PB0

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
uint8_t overcurrent_counter = 0;

cell_asic bms_ic;
FDCAN_HandleTypeDef hfdcan2;
OPAMP_HandleTypeDef hopamp2;
ADC_HandleTypeDef hadc2;

//Init all variables:
//bms_status
uint8_t bms_state = 0;
uint16_t bms_faults = 0, pack_voltage = 0, balancing_mask = 0;
int16_t pack_current = 0;

//bms_temp_metrics
uint16_t max_current_cell_temp = 0, min_current_cell_temp = 0, max_cell_temp_ever = 0, min_cell_temp_ever = 0, temp_threshold_low = 0, temp_threshold_high = 0;

//bms_voltage_metrics
uint16_t max_cell_voltage = 0, min_cell_voltage = 0, cell_voltage_threshold_high = 0, cell_voltage_threshold_low = 0, charger_target_voltage = 0;

//cell_voltages
uint16_t cell_1 = 0, cell_2 = 0, cell_3 = 0, cell_4 = 0, cell_5 = 0, cell_6 = 0, cell_7 = 0, cell_8 = 0, cell_9 = 0, cell_10 = 0, cell_11 = 0, cell_12 = 0, cell_13 = 0, cell_14 = 0;

//thermistors
uint16_t thermistor_1 = 0, thermistor_2 = 0, thermistor_3 = 0, thermistor_4 = 0, thermistor_5 = 0, thermistor_6 = 0, thermistor_7 = 0, thermistor_8 = 0, thermistor_9 = 0, thermistor_10 = 0, thermistor_11 = 0, thermistor_12 = 0, thermistor_13 = 0, thermistor_14 = 0, thermistor_15 = 0, thermistor_16 = 0, thermistor_17 = 0, thermistor_18 = 0, thermistor_19 = 0, thermistor_20 = 0, thermistor_21 = 0, thermistor_22 = 0, thermistor_23 = 0, thermistor_24 = 0, thermistor_25 = 0, thermistor_26 = 0, thermistor_27 = 0, thermistor_28 = 0, thermistor_29 = 0, thermistor_30 = 0;


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
  FAULT_OVERCURRENT    = 12,
  FAULT_PEC            = 15
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

const float CELL_VOLTAGE_LOW  = 2.5f; // Min value is can handle is 2.5(discharge)
const float CELL_VOLTAGE_HIGH = 4.20f; // Max value it can handle is 4.25(discharge)

//Must add the over current later aswell
const float TEMP_LOW_C  = -20.0f; // Min value it cna take is -20.0f(discharge)
const float TEMP_HIGH_C = 60.0f; // Max value it can tolerate = 55.0f(discharge)

int8_t last_pec_error = 0;
bool last_mux_error = false;

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

void init_bms_chip_array() {
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

void initISense() {
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

  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK) {
    Serial.println("ERROR: OPAMP2 init failed");
  }

  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK) {
    Serial.println("ERROR: OPAMP2 start failed");
  }

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

  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Serial.println("ERROR: ADC2 re-init failed");
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    Serial.println("ERROR: ADC2 re-calibration failed");
  }

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Serial.println("ERROR: ADC2 VOPAMP2 channel config failed");
  }
}

void init_thermistors() {
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

  max_cell_temp_ever = thermistorEncode(0);
  min_cell_temp_ever = thermistorEncode(120);
}

bool set_mux_channels(uint8_t mux_addr, uint8_t chA, uint8_t chB) {
  if (chA > 0x10 || chB > 0x10) return false;

  Wire.beginTransmission(mux_addr);
  Wire.write(MAX14661_CMD_A);
  Wire.write(chA);
  Wire.write(chB);
  return (Wire.endTransmission() == 0);
}

float therm_raw_to_temp_c(float raw) {
  float v_out = (3.3f * raw) / 4095.0f;
  if (v_out <= 0.0f || v_out >= 3.3f) {
    return 0.0;
  }

  float temp_k = 1.0f / (
    (1.0f / 298.15f) +
    (1.0f / 4250.0f) * log(v_out / (3.3f - v_out))
  );

  return temp_k - 273.15f;
}

void set_fault_bit(uint8_t bit) {
  bms_faults |= (1U << bit);
}

bool read_cell_voltages() {
  min_cell_voltage = 5000.0f;
  max_cell_voltage = 0.0f;
  pack_voltage = 0.0f;

  wakeup_idle(1);
  ADBMS181x_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
  ADBMS181x_pollAdc();

  last_pec_error = ADBMS181x_rdcv(0, 1, &bms_ic);

    for (int cell = 0; cell < 14; cell++)
    {
      double volts = bms_ic.cells.c_codes[cell] * 0.0001f;

      switch (cell)
      {
        // -------- group 0 --------
        case 0: cell_1 = can_lib_cell_voltages_0_north_cell_1_encode(volts * 1000); break;
        case 1: cell_2 = can_lib_cell_voltages_0_north_cell_2_encode(volts * 1000); break;
        case 2: cell_3 = can_lib_cell_voltages_0_north_cell_3_encode(volts * 1000); break;
        case 3: cell_4 = can_lib_cell_voltages_0_north_cell_4_encode(volts * 1000); break;
        case 4: cell_5 = can_lib_cell_voltages_0_north_cell_5_encode(volts * 1000); break;
        // -------- group 1 --------
        case 5: cell_6 = can_lib_cell_voltages_1_north_cell_6_encode(volts * 1000); break;
        case 6: cell_7 = can_lib_cell_voltages_1_north_cell_7_encode(volts * 1000); break;
        case 7: cell_8 = can_lib_cell_voltages_1_north_cell_8_encode(volts * 1000); break;
        case 8: cell_9 = can_lib_cell_voltages_1_north_cell_9_encode(volts * 1000); break;
        case 9: cell_10 = can_lib_cell_voltages_1_north_cell_10_encode(volts * 1000); break;
        // -------- group 2 --------
        case 10: cell_11 = can_lib_cell_voltages_2_north_cell_11_encode(volts * 1000); break;
        case 11: cell_12 = can_lib_cell_voltages_2_north_cell_12_encode(volts * 1000); break;
        case 12: cell_13 = can_lib_cell_voltages_2_north_cell_13_encode(volts * 1000); break;
        case 13: cell_14 = can_lib_cell_voltages_2_north_cell_14_encode(volts * 1000); break;
      }

      if (volts * 1000 < can_lib_bms_voltage_metrics_north_max_cell_voltage_decode(min_cell_voltage)) min_cell_voltage = can_lib_bms_voltage_metrics_north_max_cell_voltage_encode(volts * 1000);
      if (volts * 1000 > can_lib_bms_voltage_metrics_north_min_cell_voltage_decode(max_cell_voltage)) max_cell_voltage = can_lib_bms_voltage_metrics_north_min_cell_voltage_encode(volts * 1000);

     pack_voltage += can_lib_bms_status_north_pack_voltage_encode(volts * 1000);
   
    }

    cell_voltage_threshold_high = can_lib_bms_voltage_metrics_north_cell_voltage_threshold_high_encode(CELL_VOLTAGE_HIGH * 1000);
    cell_voltage_threshold_low = can_lib_bms_voltage_metrics_north_cell_voltage_threshold_high_encode(CELL_VOLTAGE_LOW * 1000);

    digitalWrite(PB11, last_pec_error == 0);

  return (last_pec_error == 0);
}

uint16_t thermistorEncode(float t) {
  return (uint16_t)((t + 20.0) / 140.0 * 65536.0);
}

float thermistorDecode(uint16_t t) {
  return (float)(t * 140.0 / 65536.0 - 20.0);
}

uint8_t thermistors_index = 14;
bool read_thermistors() {
  last_mux_error = false;

  float t1 = therm_raw_to_temp_c(analogRead(THERM_A1_PIN));
  float t2 = therm_raw_to_temp_c(analogRead(THERM_B1_PIN));
  float t3 = therm_raw_to_temp_c(analogRead(THERM_A2_PIN));
  float t4 = therm_raw_to_temp_c(analogRead(THERM_B2_PIN));

  thermistors_index += 2;
  if(thermistors_index == 14) {
    thermistors_index = 0;

      Serial.println(thermistorDecode(max_current_cell_temp));
  // Serial.println(thermistorDecode(min_current_cell_temp));

    min_current_cell_temp = thermistorEncode(120);
    max_current_cell_temp = thermistorEncode(-20);
  }


  if (t1 < min_current_cell_temp) min_current_cell_temp = thermistorEncode(t1);
  if (t2 < min_current_cell_temp) min_current_cell_temp = thermistorEncode(t2);
  if (t3 < min_current_cell_temp) min_current_cell_temp = thermistorEncode(t3);
  if (t4 < min_current_cell_temp) min_current_cell_temp = thermistorEncode(t4);

  if (t1 > max_current_cell_temp) max_current_cell_temp = thermistorEncode(t1);
  if (t2 > max_current_cell_temp) max_current_cell_temp = thermistorEncode(t2);
  if (t3 > max_current_cell_temp) max_current_cell_temp = thermistorEncode(t3);
  if (t4 > max_current_cell_temp) max_current_cell_temp = thermistorEncode(t4);

  bool mux1_ok = set_mux_channels(MUX1_ADDR, thermistors_index, thermistors_index + 1);
  digitalWrite(PB7, mux1_ok);
  // if(mux1_ok) {digitalToggle(PB7);}
  bool mux2_ok = set_mux_channels(MUX2_ADDR, thermistors_index, thermistors_index + 1);
  digitalWrite(PA15, mux2_ok);
  // if(mux2_ok) {digitalToggle(PA15);}

  //timer delay compensates for mux warmup
  if ((!mux1_ok || !mux2_ok) && millis() > 15000)
  {
    last_mux_error = true;
    return false;
  }

  return true;
}


float currentRollingAvg = 0.0;
#define ROLLING_AVG_PERCENT 0.90

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


void readIsense() {
  float raw = readVOPAMP2Average(50);
  // ADC_ChannelConfTypeDef sConfig = {0};
  // sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  // sConfig.Rank = ADC_REGULAR_RANK_1;
  // sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  // sConfig.SingleDiff = ADC_SINGLE_ENDED;
  // sConfig.OffsetNumber = ADC_OFFSET_NONE;
  // sConfig.Offset = 0;

  // if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
  //   Serial.println("ERROR: ADC2 VOPAMP2 channel config failed");
  // }

  // //IIR rolling average
  // if (HAL_ADC_Start(&hadc2) != HAL_OK) {HAL_ADC_Stop(&hadc2);}
  // if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK){HAL_ADC_Stop(&hadc2);}
  // uint16_t raw = HAL_ADC_GetValue(&hadc2);
  // HAL_ADC_Stop(&hadc2);

  currentRollingAvg = currentRollingAvg * ROLLING_AVG_PERCENT + (float)raw * (1-ROLLING_AVG_PERCENT);



  //Calcualte current in dA
  float current_A = ((currentRollingAvg  - ISENSE_ZERO_RAW) * 3.3 / 4096.0) / ISENSE_COUNTS_PER_AMP * 1000 * 1.22;

  Serial.println(current_A);
  
  //Do overcurrent catching
  if(current_A > 5000) {overcurrent_counter++;}
  if(overcurrent_counter > 50) {set_fault_bit(FAULT_OVERCURRENT);}

  pack_current = (int16_t)current_A;
  Serial.println(pack_current);
}

void update_faults() {
  if (last_pec_error != 0)
  {
    set_fault_bit(FAULT_ADBMS_COMMS);
    set_fault_bit(FAULT_PEC);
  }

  if (last_mux_error)
  {
    set_fault_bit(FAULT_MUX);
  }

  if (can_lib_bms_voltage_metrics_north_min_cell_voltage_decode(min_cell_voltage) < CELL_VOLTAGE_LOW * 1000)
  {
    set_fault_bit(FAULT_CELL_UNDERVOLT);
  }

  if (can_lib_bms_voltage_metrics_north_max_cell_voltage_decode(max_cell_voltage) > CELL_VOLTAGE_HIGH  * 1000.0)
  {
    set_fault_bit(FAULT_CELL_OVERVOLT);
  }

  if (thermistorDecode(min_cell_temp_ever) < TEMP_LOW_C)
  {
    set_fault_bit(FAULT_CELL_UNDERTEMP);
  }

  if (thermistorDecode(max_cell_temp_ever) > TEMP_HIGH_C)
  {
    set_fault_bit(FAULT_CELL_OVERTEMP);
  }
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

  //DISABLE BMS CONTROL OF FETS
  pinMode(PA10, OUTPUT);
  digitalToggle(PA10);

  Serial.begin(115200);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  init_bms_chip_array();
  init_thermistors();

  initISense();

  #ifdef NORTH
    can_init_bms_north(&hfdcan2);
  #elif defined(EAST)
    can_init_bms_east(&hfdcan2);
  #elif defined(SOUTH)
    can_init_bms_south(&hfdcan2);
  #elif defined(WEST)
    can_init_bms_west(&hfdcan2);
  #endif
}

void update_can_structs() {
  Serial.println("===== CELL VOLTAGES DEBUG =====");

  // Group 0
  Serial.print("Cell 1: ");  Serial.println(cell_1);
  Serial.print("Cell 2: ");  Serial.println(cell_2);
  Serial.print("Cell 3: ");  Serial.println(cell_3);
  Serial.print("Cell 4: ");  Serial.println(cell_4);
  Serial.print("Cell 5: ");  Serial.println(cell_5);

  // Group 1
  Serial.print("Cell 6: ");  Serial.println(cell_6);
  Serial.print("Cell 7: ");  Serial.println(cell_7);
  Serial.print("Cell 8: ");  Serial.println(cell_8);
  Serial.print("Cell 9: ");  Serial.println(cell_9);
  Serial.print("Cell 10: "); Serial.println(cell_10);

  // Group 2
  Serial.print("Cell 11: "); Serial.println(cell_11);
  Serial.print("Cell 12: "); Serial.println(cell_12);
  Serial.print("Cell 13: "); Serial.println(cell_13);
  Serial.print("Cell 14: "); Serial.println(cell_14);

  Serial.println("================================");
   #ifdef NORTH
   //bms_status_north
    bms_status_north.bms_state = bms_state;
    bms_status_north.bms_faults = bms_faults;
    bms_status_north.pack_voltage = pack_voltage;
    bms_status_north.pack_current = pack_current;
    bms_status_north.balancing_mask = balancing_mask;

    //bms_temp_metrics_north
    bms_temp_metrics_north.max_current_cell_temp = max_current_cell_temp;
    bms_temp_metrics_north.min_current_cell_temp = min_current_cell_temp;
    bms_temp_metrics_north.max_cell_temp_ever = max_cell_temp_ever;
    bms_temp_metrics_north.min_cell_temp_ever = min_cell_temp_ever;
    bms_temp_metrics_north.temp_threshold_high = temp_threshold_high;
    bms_temp_metrics_north.temp_threshold_low = temp_threshold_low;

    //bms_voltage_metrics_north
    bms_voltage_metrics_north.max_cell_voltage = max_cell_voltage;
    bms_voltage_metrics_north.min_cell_voltage = min_cell_voltage;
    bms_voltage_metrics_north.cell_voltage_threshold_high = cell_voltage_threshold_high;
    bms_voltage_metrics_north.cell_voltage_threshold_low = cell_voltage_threshold_low;
    bms_voltage_metrics_north.charger_target_voltage = charger_target_voltage;

    //cell_voltages_0_north
    cell_voltages_0_north.cell_1 = cell_1;
    cell_voltages_0_north.cell_2 = cell_2;
    cell_voltages_0_north.cell_3 = cell_3;
    cell_voltages_0_north.cell_4 = cell_4;
    cell_voltages_0_north.cell_5 = cell_5;

    //cell_voltages_1_north
    cell_voltages_1_north.cell_6 = cell_6;
    cell_voltages_1_north.cell_7 = cell_7;
    cell_voltages_1_north.cell_8 = cell_8;
    cell_voltages_1_north.cell_9 = cell_9;
    cell_voltages_1_north.cell_10 = cell_10;

    //cell_voltages_2_north
    cell_voltages_2_north.cell_11 = cell_11;
    cell_voltages_2_north.cell_12 = cell_12;
    cell_voltages_2_north.cell_13 = cell_13;
    cell_voltages_2_north.cell_14 = cell_14;

//     void print_cell_voltages_debug()
// {
  
// }

    //thermistors_0_north
    thermistors_0_north.thermistor_1 = thermistor_1;
    thermistors_0_north.thermistor_2 = thermistor_2;
    thermistors_0_north.thermistor_3 = thermistor_3;
    thermistors_0_north.thermistor_4 = thermistor_4;
    thermistors_0_north.thermistor_5 = thermistor_5;
    thermistors_0_north.thermistor_6 = thermistor_6;

    //thermistors_1_north
    thermistors_1_north.thermistor_7 = thermistor_7;
    thermistors_1_north.thermistor_8 = thermistor_8;
    thermistors_1_north.thermistor_9 = thermistor_9;
    thermistors_1_north.thermistor_10 = thermistor_10;
    thermistors_1_north.thermistor_11 = thermistor_11;
    thermistors_1_north.thermistor_12 = thermistor_12;

    //thermistors_2_north
    thermistors_2_north.thermistor_13 = thermistor_13;
    thermistors_2_north.thermistor_14 = thermistor_14;
    thermistors_2_north.thermistor_15 = thermistor_15;
    thermistors_2_north.thermistor_16 = thermistor_16;
    thermistors_2_north.thermistor_17 = thermistor_17;
    thermistors_2_north.thermistor_18 = thermistor_18;

    //thermistors_3_north
    thermistors_3_north.thermistor_19 = thermistor_19;
    thermistors_3_north.thermistor_20 = thermistor_20;
    thermistors_3_north.thermistor_21 = thermistor_21;
    thermistors_3_north.thermistor_22 = thermistor_22;
    thermistors_3_north.thermistor_23 = thermistor_23;
    thermistors_3_north.thermistor_24 = thermistor_24;

    //thermistors_4_north
    thermistors_4_north.thermistor_25 = thermistor_25;
    thermistors_4_north.thermistor_26 = thermistor_26;
    thermistors_4_north.thermistor_27 = thermistor_27;
    thermistors_4_north.thermistor_28 = thermistor_28;
    thermistors_4_north.thermistor_29 = thermistor_29;
    thermistors_4_north.thermistor_30 = thermistor_30;
  #elif defined(EAST)
    bms_temp_metrics_east.max_cell_temp_ever = max_cell_temp_ever;
    bms_temp_metrics_east.min_cell_temp_ever = min_cell_temp_ever;
  #elif defined(SOUTH)
    bms_temp_metrics_south.max_cell_temp_ever = max_cell_temp_ever;
    bms_temp_metrics_south.min_cell_temp_ever = min_cell_temp_ever;
  #elif defined(WEST)
    bms_temp_metrics_west.max_cell_temp_ever = max_cell_temp_ever;
    bms_temp_metrics_west.min_cell_temp_ever = min_cell_temp_ever;
  #endif
}

void run_bms_cycle()
{
  bool cells_ok = read_cell_voltages();
  bool therms_ok = read_thermistors();
  
  readIsense();

  update_faults();

  update_can_structs();

  #ifdef NORTH
    can_send_bms_status_north(&hfdcan2);
    can_send_bms_temp_metrics_north(&hfdcan2);
    can_send_bms_voltage_metrics_north(&hfdcan2);
    can_send_cell_voltages_0_north(&hfdcan2);
    can_send_cell_voltages_1_north(&hfdcan2);
    can_send_cell_voltages_2_north(&hfdcan2);
  #elif defined(EAST)
    can_send_bms_status_east(&hfdcan2);
    can_send_bms_temp_metrics_east(&hfdcan2);
    can_send_bms_voltage_metrics_east(&hfdcan2);
    can_send_cell_voltages_0_east(&hfdcan2);
    can_send_cell_voltages_1_east(&hfdcan2);
    can_send_cell_voltages_2_east(&hfdcan2);
  #elif defined(SOUTH)
    can_send_bms_status_south(&hfdcan2);
    can_send_bms_temp_metrics_south(&hfdcan2);
    can_send_bms_voltage_metrics_south(&hfdcan2);
    can_send_cell_voltages_0_south(&hfdcan2);
    can_send_cell_voltages_1_south(&hfdcan2);
    can_send_cell_voltages_2_south(&hfdcan2);
  #elif defined(WEST)
    can_send_bms_status_west(&hfdcan2);
    can_send_bms_temp_metrics_west(&hfdcan2);
    can_send_bms_voltage_metrics_west(&hfdcan2);
    can_send_cell_voltages_0_west(&hfdcan2);
    can_send_cell_voltages_1_west(&hfdcan2);
    can_send_cell_voltages_2_west(&hfdcan2);
  #endif

  digitalToggle(PB2);

  switch (bms_state) {
    case IDLE:
      if (!cells_ok || !therms_ok || bms_faults != 0) {
        bms_state = FAULTED;
      }

      break;

    case FAULTED:
      digitalToggle(PB4);
      print_faults();
      break;
  }
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

unsigned long ms50 = 0;
unsigned long ms200 = 0;

void loop() {
  unsigned long now = millis();

  if (now - ms50 >= 50)
  {
    ms50 = now;

    run_bms_cycle();
  }

  if (now - ms200 >= 200) {
    ms200 = now;

    digitalToggle(PB9);
  }

  delay(1);
}