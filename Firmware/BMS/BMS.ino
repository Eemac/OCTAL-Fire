#define HAL_FDCAN_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can_api.h"
#include "can_lib.h"
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

cell_asic bms_ic;
FDCAN_HandleTypeDef hfdcan2;
OPAMP_HandleTypeDef hopamp2;

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

uint16_t bms_faults = 0;

const float CELL_VOLTAGE_LOW  = 2.5f; // Min value is can handle is 2.5(discharge)
const float CELL_VOLTAGE_HIGH = 4.20f; // Max value it can handle is 4.25(discharge)

//Must add the over current later aswell
const float TEMP_LOW_C  = -20.0f; // Min value it cna take is -20.0f(discharge)
const float TEMP_HIGH_C = 60.0f; // Max value it can tolerate = 55.0f(discharge)

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

  bms_temp_metrics_a.max_cell_temp_ever = thermistorEncode(0);
  bms_temp_metrics_a.min_cell_temp_ever = thermistorEncode(120);
}

void initISense() {
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_4_OR_MINUS_3;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  //AMP SENSE
  pinMode(PB0, INPUT_ANALOG);
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
    return 0.0;
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

bool read_cell_voltages() {
  bms_voltage_metrics_a.min_cell_voltage = 5000.0f;
  bms_voltage_metrics_a.max_cell_voltage = 0.0f;
  bms_status_a.pack_voltage = 0.0f;

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
        case 0: cell_voltages_0_a.cell_1 = can_lib_cell_voltages_0_a_cell_1_encode(volts * 1000); break;
        case 1: cell_voltages_0_a.cell_2 = can_lib_cell_voltages_0_a_cell_2_encode(volts * 1000); break;
        case 2: cell_voltages_0_a.cell_3 = can_lib_cell_voltages_0_a_cell_3_encode(volts * 1000); break;
        case 3: cell_voltages_0_a.cell_4 = can_lib_cell_voltages_0_a_cell_4_encode(volts * 1000); break;
        case 4: cell_voltages_0_a.cell_5 = can_lib_cell_voltages_0_a_cell_5_encode(volts * 1000); break;
        // -------- group 1 --------
        case 5: cell_voltages_1_a.cell_6 = can_lib_cell_voltages_1_a_cell_6_encode(volts * 1000); break;
        case 6: cell_voltages_1_a.cell_7 = can_lib_cell_voltages_1_a_cell_7_encode(volts * 1000); break;
        case 7: cell_voltages_1_a.cell_8 = can_lib_cell_voltages_1_a_cell_8_encode(volts * 1000); break;
        case 8: cell_voltages_1_a.cell_9 = can_lib_cell_voltages_1_a_cell_9_encode(volts * 1000); break;
        case 9: cell_voltages_1_a.cell_10 = can_lib_cell_voltages_1_a_cell_10_encode(volts * 1000); break;
        // -------- group 2 --------
        case 10: cell_voltages_2_a.cell_11 = can_lib_cell_voltages_2_a_cell_11_encode(volts * 1000); break;
        case 11: cell_voltages_2_a.cell_12 = can_lib_cell_voltages_2_a_cell_12_encode(volts * 1000); break;
        case 12: cell_voltages_2_a.cell_13 = can_lib_cell_voltages_2_a_cell_13_encode(volts * 1000); break;
        case 13: cell_voltages_2_a.cell_14 = can_lib_cell_voltages_2_a_cell_14_encode(volts * 1000); break;
      }

      if (volts * 1000 < can_lib_bms_voltage_metrics_a_max_cell_voltage_decode(bms_voltage_metrics_a.min_cell_voltage)) bms_voltage_metrics_a.min_cell_voltage = can_lib_bms_voltage_metrics_a_max_cell_voltage_encode(volts * 1000);
      if (volts * 1000 > can_lib_bms_voltage_metrics_a_min_cell_voltage_decode(bms_voltage_metrics_a.max_cell_voltage)) bms_voltage_metrics_a.max_cell_voltage = can_lib_bms_voltage_metrics_a_min_cell_voltage_encode(volts * 1000);

      bms_status_a.pack_voltage += can_lib_bms_status_a_pack_voltage_encode(volts * 1000);
   
    }

    bms_voltage_metrics_a.cell_voltage_threshold_high = can_lib_bms_voltage_metrics_a_cell_voltage_threshold_high_encode(CELL_VOLTAGE_HIGH * 1000);
    bms_voltage_metrics_a.cell_voltage_threshold_low = can_lib_bms_voltage_metrics_a_cell_voltage_threshold_high_encode(CELL_VOLTAGE_LOW * 1000);

    if(last_pec_error == 0) {
      digitalToggle(PB11);
    }

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

  float t1 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_A1_PIN)));
  float t2 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_B1_PIN)));
  float t3 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_A2_PIN)));
  float t4 = therm_voltage_to_temp_c(raw_to_therm_voltage(analogRead(THERM_B2_PIN)));

  thermistors_index += 2;
  if(thermistors_index == 14) {
    thermistors_index = 0;

  //     Serial.println(thermistorDecode(bms_temp_metrics_a.max_current_cell_temp));
  // Serial.println(thermistorDecode(bms_temp_metrics_a.min_current_cell_temp));

    bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(120);
    bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(-20);
  }


  if (t1 < bms_temp_metrics_a.min_current_cell_temp) bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(t1);
  if (t2 < bms_temp_metrics_a.min_current_cell_temp) bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(t2);
  if (t3 < bms_temp_metrics_a.min_current_cell_temp) bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(t3);
  if (t4 < bms_temp_metrics_a.min_current_cell_temp) bms_temp_metrics_a.min_current_cell_temp = thermistorEncode(t4);

  if (t1 > bms_temp_metrics_a.max_current_cell_temp) bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(t1);
  if (t2 > bms_temp_metrics_a.max_current_cell_temp) bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(t2);
  if (t3 > bms_temp_metrics_a.max_current_cell_temp) bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(t3);
  if (t4 > bms_temp_metrics_a.max_current_cell_temp) bms_temp_metrics_a.max_current_cell_temp = thermistorEncode(t4);

  bool mux1_ok = set_mux_channels(MUX1_ADDR, thermistors_index, thermistors_index + 1);
  if(mux1_ok) {digitalToggle(PB7);}
  bool mux2_ok = set_mux_channels(MUX2_ADDR, thermistors_index, thermistors_index + 1);
  if(mux2_ok) {digitalToggle(PA15);}

  if (!mux1_ok || !mux2_ok)
  {
    last_mux_error = true;
    return false;
  }

  return true;
}

void readIsense() {
  float sum = 0;

  for (int i = 0; i < 20; i++){sum += (float)analogRead(PB0);}
  sum = sum / 20.0;
  bms_status_a.pack_current = (uint16_t)((sum - 121.0) * 0.265 * 10.0); //100mA in ones place
  // Serial.println(bms_status_a.pack_current); //144 = 6.10  ---------- / 4096.0 * 3.3
}

void update_faults() {
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

  if (can_lib_bms_voltage_metrics_a_min_cell_voltage_decode(bms_voltage_metrics_a.min_cell_voltage) < CELL_VOLTAGE_LOW * 1000)
  {
    set_fault_bit(FAULT_CELL_UNDERVOLT);
  }

  if (can_lib_bms_voltage_metrics_a_max_cell_voltage_decode(bms_voltage_metrics_a.max_cell_voltage) > CELL_VOLTAGE_HIGH  * 1000.0)
  {
    set_fault_bit(FAULT_CELL_OVERVOLT);
  }

  if (thermistorDecode(bms_temp_metrics_a.min_cell_temp_ever) < TEMP_LOW_C)
  {
    set_fault_bit(FAULT_CELL_UNDERTEMP);
  }

  if (thermistorDecode(bms_temp_metrics_a.max_cell_temp_ever) > TEMP_HIGH_C)
  {
    set_fault_bit(FAULT_CELL_OVERTEMP);
  }
}

void print_faults() {
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

  //DISABLE BMS CONTROL OF FETS
  digitalToggle(PA10);

  Serial.begin(115200);
  delay(200);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  init_bms_chip_array();
  init_thermistors();

  can_init_bms_a(&hfdcan2);
}

void run_bms_cycle()
{
  bool cells_ok = read_cell_voltages();
  bool therms_ok = read_thermistors();
  readIsense();

  update_faults();

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
      digitalToggle(PB4);
      // handle_faulted_state_nonblocking(); // NEW
      break;
    }
  }
}

unsigned long ms50 = 0;
unsigned long ms200 = 0;

void loop()
{
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