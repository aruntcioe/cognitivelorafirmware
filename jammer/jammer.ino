/***********************************************************************
 * LoRa Test Jammer (Bare-metal SX1276/SX1278)  (Corrected)
 *
 * Board:  Arduino (Uno/Nano class)
 * Radio:  SX1276/SX1278 module (Ra-02)
 *
 * PURPOSE
 *   A controlled channel-occupancy aggressor used ONLY to test the
 *   resilience / channel-hopping behaviour of your OWN cognitive-radio
 *   link on the bench.
 *
 *   REGULATORY NOTE: This continuously radiates at high power. Operate
 *   only into a dummy load / attenuator or a shielded setup, and stay
 *   within your local ISM band and power limits. Do not radiate over
 *   the air in a way that disrupts third-party equipment.
 *
 * Buttons:
 *   PIN_BTN_CHANNEL -> step frequency channel (433.0 .. 434.6 MHz)
 *   PIN_BTN_SF      -> step spreading factor  (7 .. 12, valid SX127x range)
 *
 * ----- FIXES vs. original -----
 *  - Spreading factor is now clamped to the valid SX127x LoRa range
 *    (7..12). SF5 does not exist and SF6 needs special register setup,
 *    so both are excluded (the original wrapped down to SF5).
 *  - Low Data Rate Optimization (LDRO) is now toggled automatically for
 *    SF11/SF12 at BW125 kHz, as the datasheet requires.
 *  - Single sketch: PAYLOAD_MODE selects a 255-byte max-airtime payload
 *    or a short "PACKET" string (consolidates the two original files).
 ***********************************************************************/

#include <SPI.h>

// -------- Payload selection --------
//   1 = maximum airtime (255 bytes)   -> strongest channel occupancy
//   0 = short "PACKET" string
#define PAYLOAD_MODE 0

// ---------------- Pins ----------------
#define PIN_NSS   10
#define PIN_RESET 9
#define PIN_DIO0  2

#define PIN_LED_BLUE 6
#define PIN_LED_RED  7

#define PIN_BTN_CHANNEL 4
#define PIN_BTN_SF      5

// ---------------- SX127x Registers ----------------
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FRF_MSB           0x06
#define REG_FRF_MID           0x07
#define REG_FRF_LSB           0x08
#define REG_PA_CONFIG         0x09
#define REG_OCP               0x0B
#define REG_FIFO_ADDR_PTR     0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_IRQ_FLAGS         0x12
#define REG_MODEM_CONFIG_1    0x1D
#define REG_MODEM_CONFIG_2    0x1E
#define REG_MODEM_CONFIG_3    0x26
#define REG_PREAMBLE_MSB      0x20
#define REG_PREAMBLE_LSB      0x21
#define REG_PAYLOAD_LENGTH    0x22
#define REG_VERSION           0x42
#define REG_PA_DAC            0x4D

// ---------------- Modes ----------------
#define LONG_RANGE_MODE 0x80
#define MODE_SLEEP      0x00
#define MODE_STDBY      0x01
#define MODE_TX         0x03

// ---------------- Frequency Channels ----------------
const uint32_t channelFreqKHz[15] = {
  432900, 433000,433200,433250, 433400,433500, 433700,433750, 434000, 344200, 434250, 434400, 434500, 434750,
};

uint8_t currentChannel = 0;

// Valid SX127x LoRa spreading factors: 7..12
uint8_t currentSF = 7;

// ---------------- Packet ----------------
#if PAYLOAD_MODE == 1
  #define PACKET_LEN 160
  uint8_t TX_PACKET[PACKET_LEN];
  void createPacket()
  {
    for (uint16_t i = 0; i < PACKET_LEN; i++)
      TX_PACKET[i] = 0x55;              // 01010101 pattern
  }
#else
  const char TX_PACKET[] = "PACKET";
  const uint8_t PACKET_LEN = 6;
  void createPacket() { /* static payload */ }
#endif

// ---------------- Buttons ----------------
bool btnChannelPrev = HIGH;
bool btnSFPrev      = HIGH;
unsigned long lastButtonTime = 0;
#define DEBOUNCE_TIME 200

// ---------------- SPI Register Functions ----------------
void writeRegister(uint8_t reg, uint8_t value)
{
  digitalWrite(PIN_NSS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(value);
  digitalWrite(PIN_NSS, HIGH);
}

uint8_t readRegister(uint8_t reg)
{
  digitalWrite(PIN_NSS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_NSS, HIGH);
  return value;
}

// ---------------- LEDs ----------------
void bootAnimation()
{
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_BLUE, LOW);
    delay(150);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_BLUE, HIGH);
    delay(150);
  }
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
}

void parameterChangeBlink()
{
  digitalWrite(PIN_LED_RED, HIGH);
  delay(80);
  digitalWrite(PIN_LED_RED, LOW);
}

// ---------------- SX127x Control ----------------
void resetSX1276()
{
  digitalWrite(PIN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_RESET, HIGH);
  delay(20);
}

void setFrequency(uint32_t freqKHz)
{
  // Fstep = 32 MHz / 2^19 = 61.03515625 Hz ; Frf = freqHz / Fstep
  uint64_t frf = ((uint64_t)freqKHz * 1000ULL) / 61.03515625;

  writeRegister(REG_FRF_MSB, (frf >> 16) & 0xFF);
  writeRegister(REG_FRF_MID, (frf >> 8)  & 0xFF);
  writeRegister(REG_FRF_LSB,  frf        & 0xFF);
}

// Enable Low Data Rate Optimization for SF11/SF12 at BW125 kHz.
void setLowDataRateOptimize(uint8_t sf)
{
  uint8_t cfg3 = 0x04;                 // AGC auto on
  if (sf >= 11) cfg3 |= 0x08;          // LowDataRateOptimize
  writeRegister(REG_MODEM_CONFIG_3, cfg3);
}

void setSpreadingFactor(uint8_t sf)
{
  if (sf < 7)  sf = 7;                 // clamp to valid SX127x LoRa range
  if (sf > 12) sf = 12;

  uint8_t value = readRegister(REG_MODEM_CONFIG_2);
  value &= 0x0F;
  value |= (sf << 4);
  writeRegister(REG_MODEM_CONFIG_2, value);

  setLowDataRateOptimize(sf);
}

bool initSX1276()
{
  bootAnimation();
  resetSX1276();

  if (readRegister(REG_VERSION) != 0x12)
    return false;

  writeRegister(REG_OP_MODE, LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  writeRegister(REG_OP_MODE, LONG_RANGE_MODE | MODE_STDBY);

  writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);

  // -------- Maximum power (+20 dBm via PA_BOOST) --------
  writeRegister(REG_PA_CONFIG, 0x8F);
  writeRegister(REG_PA_DAC,    0x87);
  writeRegister(REG_OCP,       0x2B);   // raise over-current limit

  // BW125 kHz, CR 4/5, explicit header
  writeRegister(REG_MODEM_CONFIG_1, 0x72);

  // SF + CRC enabled
  writeRegister(REG_MODEM_CONFIG_2, (currentSF << 4) | 0x04);

  // LDRO for the current SF
  setLowDataRateOptimize(currentSF);

  // Preamble = 8
  writeRegister(REG_PREAMBLE_MSB, 0);
  writeRegister(REG_PREAMBLE_LSB, 8);

  setFrequency(channelFreqKHz[currentChannel]);
  return true;
}

// ---------------- Transmission ----------------
void startTransmission()
{
  writeRegister(REG_OP_MODE, LONG_RANGE_MODE | MODE_STDBY);
  writeRegister(REG_FIFO_ADDR_PTR, 0);

  for (uint16_t i = 0; i < PACKET_LEN; i++)
    writeRegister(REG_FIFO, TX_PACKET[i]);

  writeRegister(REG_PAYLOAD_LENGTH, PACKET_LEN);
  writeRegister(REG_IRQ_FLAGS, 0xFF);                 // clear flags
  writeRegister(REG_OP_MODE, LONG_RANGE_MODE | MODE_TX);
}

// ---------------- Setup ----------------
void setup()
{
  Serial.begin(115200);

  pinMode(PIN_NSS, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BTN_CHANNEL, INPUT_PULLUP);
  pinMode(PIN_BTN_SF, INPUT_PULLUP);

  SPI.begin();
  digitalWrite(PIN_NSS, HIGH);

  if (!initSX1276())
  {
    digitalWrite(PIN_LED_RED, HIGH);
    Serial.println("SX1276 INIT FAILED");
    while (1) { delay(1000); }
  }

  Serial.println("SX1276 READY");

  createPacket();
  startTransmission();
}

// ---------------- Main Loop ----------------
void loop()
{
  // TX complete -> immediately retransmit (continuous occupancy)
  if (readRegister(REG_IRQ_FLAGS) & 0x08)
  {
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    digitalWrite(PIN_LED_BLUE, !digitalRead(PIN_LED_BLUE));
    digitalWrite(PIN_LED_RED, LOW);

    Serial.print("TX:SUCCESS | FREQ:");
    Serial.print(channelFreqKHz[currentChannel]);
    Serial.print("kHz | CH:");
    Serial.print(currentChannel);
    Serial.print(" | SF:");
    Serial.println(currentSF);

    startTransmission();
  }

  // -------- Button handling --------
  unsigned long now = millis();
  bool b1 = digitalRead(PIN_BTN_CHANNEL);
  bool b2 = digitalRead(PIN_BTN_SF);

  if (now - lastButtonTime > DEBOUNCE_TIME)
  {
    // Channel button
    if (b1 == LOW && btnChannelPrev == HIGH)
    {
      currentChannel++;
      if (currentChannel >= 9) currentChannel = 0;

      setFrequency(channelFreqKHz[currentChannel]);
      parameterChangeBlink();

      Serial.print("FREQ changed to ");
      Serial.print(channelFreqKHz[currentChannel]);
      Serial.print(" kHz (CH ");
      Serial.print(currentChannel);
      Serial.println(")");

      lastButtonTime = now;
    }

    // SF button  (cycle 7..12)
    if (b2 == LOW && btnSFPrev == HIGH)
    {
      currentSF++;
      if (currentSF > 12) currentSF = 7;

      setSpreadingFactor(currentSF);
      parameterChangeBlink();

      Serial.print("SF changed to ");
      Serial.println(currentSF);

      lastButtonTime = now;
    }
  }

  btnChannelPrev = b1;
  btnSFPrev      = b2;
}
