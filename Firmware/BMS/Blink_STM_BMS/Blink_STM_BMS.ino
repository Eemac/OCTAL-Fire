#include <Wire.h>
#include <meFDCAN.h>

#include <SPI.h>
#include "ADBMS1818.h"
#include "ADBMS181x.h"


//               MOSI  MISO  SCLK   SSEL
// SPIClass SPI(PA7, PA6, PA5, PA4);

// How many BMS ICs in chain (usually 1 for testing)
#define TOTAL_IC 1

cell_asic bms_ic[TOTAL_IC];


void BAD_Set_Clock(void) {
  // set flash latency for target (144MHz -> FLASH_LATENCY_4) and up voltage to HF range
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_4);

  /* Request HSE ON (clock source mode from external high-speed crystal) */
  RCC->CR |= RCC_CR_HSEON;

  /* Wait for HSERDY (crystal spin-up finished) */
  while (!(RCC->CR & RCC_CR_HSERDY)) {
    __NOP();
  }

  //Move PLL to source off of HSE
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE);

  //Wait for PLL to ACTUALLY source from HFE
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) {
    __NOP();
  }

  // Disable PLL before reconfiguring
  CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
  while (RCC->CR & RCC_CR_PLLRDY) {
    __NOP();
  }  // wait until off

  // Reset PLLCFGR to reset state (RM0440: 0x00001000)
  RCC->PLLCFGR = 0x00001000;

  //CONFIG PLL
  // Source = HSE (bits PLLSRC = 0b11)
  RCC->PLLCFGR |= (3U << RCC_PLLCFGR_PLLSRC_Pos);

  // M = 4  → write (M-1) = 3 into PLLM[3:0]
  RCC->PLLCFGR |= (3U << RCC_PLLCFGR_PLLM_Pos);

  // N = 24 → write N directly into PLLN[7:0]
  RCC->PLLCFGR |= (24U << RCC_PLLCFGR_PLLN_Pos);

  // R = 2  → write 0 into PLLR[1:0] (00 = /2)
  RCC->PLLCFGR |= (0U << RCC_PLLCFGR_PLLR_Pos);

  // Q = 6 → PLLQ = 10 (binary)
  RCC->PLLCFGR |= (2U << RCC_PLLCFGR_PLLQ_Pos);

  // Enable PLL R output (for CPU) and Q output (for USB)
  SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN);
  SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQEN);

  //Enable PLL
  SET_BIT(RCC->CR, RCC_CR_PLLON);

  // Wait for PLL ready
  while (!(RCC->CR & RCC_CR_PLLRDY)) {
    __NOP();
  }

  // Switch SYSCLK to PLL
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

  // Wait until switch completes
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    __NOP();
  }

  SystemCoreClockUpdate();
  HAL_InitTick(TICK_INT_PRIORITY);
}

void setup() {
  BAD_Set_Clock();
  Serial.begin(112500);
  Wire.begin();

  meFDCAN_init(500, 2, PB12, PB13);  


  pinMode(PB5, OUTPUT);

    // Set the System Select pin as an output
  pinMode(PA4, OUTPUT);
  digitalWrite(PA4, HIGH);

  delay(2000);
  Serial.println("ADBMS1818 Connection Test Starting...");

  // SPI setup — required for ADBMS181x library
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  // Initialize structure defaults
  for (int i = 0; i < TOTAL_IC; i++) {
    memset(&bms_ic[i], 0, sizeof(cell_asic));
  }

  // Wake up the chip (important)
  wakeup_sleep(TOTAL_IC);
  delay(10);

  Serial.println("Reading configuration registers...");
}
void loop()
{

wakeup_sleep(1);
delay(5);
  // --- Read CFGRA ---
  int8_t pecA = ADBMS181x_rdcfg(TOTAL_IC, bms_ic);
  Serial.print("CFGRA PEC: ");
  Serial.println(pecA);

  Serial.println("CFGRA bytes:");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(bms_ic[0].config.rx_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // --- Read CFGRB ---
  int8_t pecB = ADBMS181x_rdcfgb(TOTAL_IC, bms_ic);
  Serial.print("CFGRB PEC: ");
  Serial.println(pecB);

  Serial.println("CFGRB bytes:");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(bms_ic[0].configb.rx_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\n");

  // --- Interpretation ---
  if (pecA == 0 && pecB == 0) {
    Serial.println("SUCCESS: ADBMS1818 appears to be connected correctly.\n");
  } else {
    Serial.println("ERROR: PEC mismatch — check wiring (CS, SCK, MISO, MOSI, isoSPI).\n");
  }




    byte error, address;
  int found = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Found device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
    // else if (error == 4) Serial.print("Unknown error at 0x"); // optional debug
  }

  if (found == 0) Serial.println("No I2C devices found");
  Serial.println();






 // --------------------------------------------------------
  // 1) Receive
  // --------------------------------------------------------
  CANFDMessage rx;
  if (ACANFD_STM32::can1.receive(rx)) {
    Serial.print("RX ID: 0x");
    Serial.print(rx.id, HEX);
    Serial.print("  LEN: ");
    Serial.print(rx.len);
    Serial.print("  DATA: ");
    for (uint32_t i = 0; i < rx.len; i++) {
      Serial.print(rx.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // --------------------------------------------------------
  // 2) Transmit once per second
  // --------------------------------------------------------
  static uint32_t last = 0;
  if (millis() - last > 1000) {
    last = millis();

    CANFDMessage tx;
    tx.id = 0x123;
    tx.len = 8;
    for (uint8_t i = 0; i < 8; i++) {
      tx.data[i] = i;
    }

    if (!ACANFD_STM32::can1.tryToSend(tx)) {
      Serial.println("TX FIFO full!");
    } else {
      Serial.println("Sent CAN FD frame");
    }
  }



  delay(2000);
}


// void loop() {

//   // Data to send to the slave (if any) and to hold the received data
//   byte receivedByte;

//   // Select the slave device by pulling its Slave Select pin LOW
//   digitalWrite(PA4, LOW);

//   // Perform an SPI transfer to read data
//   // When reading, you typically send a dummy byte (e.g., 0x00)
//   // The SPI.transfer() function simultaneously sends a byte and receives a byte
//   receivedByte = SPI.transfer(0x00);

//   // Deselect the slave device by pulling its Slave Select pin HIGH
//   digitalWrite(PA4, HIGH);

//   // Print the received byte to the Serial Monitor
//   Serial.print("Received: ");
//   Serial.println(receivedByte, HEX); // Print in hexadecimal format


//   delay(200);
//   digitalWrite(PB5, LOW);
//   delay(200);
//   digitalWrite(PB5, HIGH);
//   Serial.print("Hello World");
//   Serial.println(micros());
// }
