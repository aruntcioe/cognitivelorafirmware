/***********************************************************************
 * AI-Assisted Cognitive Radio LPWAN
 * TRANSMITTER NODE  (Corrected)
 *
 * Board:  ESP32 Dev Module   (VSPI)
 * Radios: 2x SX1278 (Ra-02)
 *   DATA  PLANE -> starts at 433 MHz  (hops on AI command)
 *   CTRL  PLANE -> fixed 445 MHz      (outside the jammer band)
 *
 * DATA packets are never acknowledged.
 * CONTROL packets are validated (magic + checksum), de-duplicated
 * (replay protection) and acknowledged.
 *
 * ----- FIXES vs. original -----
 *  - ControlPacket layout is now byte-identical to the receiver
 *    (packed struct, frequency carried as uint32_t kHz).
 *  - Frequency handled as uint32_t kHz everywhere (no uint8_t truncation).
 *  - Added lightweight magic + checksum authentication on the control
 *    plane and duplicate-commandID rejection (replay protection).
 *  - DIO0 is polled (RadioLib SX127x has no isPacketReceived()).
 ***********************************************************************/

#include <RadioLib.h>
#include <SPI.h>

//======================================================================
// SPI BUS PINS
//======================================================================
#define SPI_SCK     18
#define SPI_MISO    19
#define SPI_MOSI    23

//======================================================================
// DATA RADIO (starts 433 MHz)
//======================================================================
#define DATA_NSS    5
#define DATA_RST    14
#define DATA_DIO0   26

//======================================================================
// CONTROL RADIO (445 MHz)
//======================================================================
#define CTRL_NSS    4
#define CTRL_RST    13
#define CTRL_DIO0   27

//======================================================================
// STATUS LEDs
//======================================================================
#define LED_TX      32
#define LED_ERROR   33

//======================================================================
// SHARED SPI BUS
//======================================================================
SPIClass sharedSPI(VSPI);

//======================================================================
// RADIO OBJECTS
//======================================================================
SX1278 dataRadio = new Module(
    DATA_NSS, DATA_DIO0, DATA_RST, RADIOLIB_NC, sharedSPI);

SX1278 controlRadio = new Module(
    CTRL_NSS, CTRL_DIO0, CTRL_RST, RADIOLIB_NC, sharedSPI);

//======================================================================
// SHARED PROTOCOL  (MUST be byte-identical on TX and RX)
//======================================================================
static const uint32_t PROTO_MAGIC  = 0xC0DEA5A5UL;   // network id / token
static const uint32_t PROTO_SECRET = 0x5A17C0DEUL;   // pre-shared key

#pragma pack(push, 1)
struct DataPacket
{
    uint32_t magic; 
    uint32_t sequence;
    uint32_t timestamp;
    uint16_t sensorValue;
};

struct ControlPacket
{
    uint32_t magic;
    uint32_t commandID;
    uint8_t  command;
    uint32_t newFrequencyKHz;
    uint8_t  newSF;
    uint8_t  newCR;
    uint32_t checksum;
};

struct ControlAck
{
    uint32_t magic;
    uint32_t commandID;
    uint8_t  applied;
    uint32_t checksum;
};
#pragma pack(pop)

enum CommandType
{
    CMD_NONE        = 0,
    CMD_CHANNEL_HOP = 1,
    CMD_LINK_ADAPT  = 2
};

//======================================================================
// CHECKSUMS  (identical algorithm on both nodes)
//======================================================================
uint32_t controlChecksum(const ControlPacket &p)
{
    uint32_t s = PROTO_SECRET;
    s += p.magic;
    s += p.commandID;
    s += p.command;
    s += p.newFrequencyKHz;
    s += p.newSF;
    s += p.newCR;
    return s;
}

uint32_t ackChecksum(const ControlAck &a)
{
    uint32_t s = PROTO_SECRET;
    s += a.magic;
    s += a.commandID;
    s += a.applied;
    return s;
}

//======================================================================
// CURRENT RADIO CONFIGURATION (kHz throughout)
//======================================================================
uint32_t currentFrequencyKHz = 433000;
uint8_t  currentSF           = 12;
uint8_t  currentCR           = 8;

//======================================================================
// CONTROL CHANNEL CONFIGURATION
//======================================================================
const float   CONTROL_FREQUENCY = 445.0;   // MHz
const uint8_t CONTROL_SF         = 12;
const uint8_t CONTROL_CR         = 8;

//======================================================================
// STATE
//======================================================================
uint32_t sequenceNumber      = 0;
uint32_t lastAppliedCommandID = 0;         // replay / duplicate guard

const uint32_t DATA_INTERVAL = 10;        // ms
uint32_t lastTransmissionTime = 0;

//======================================================================
// FUNCTION DECLARATIONS
//======================================================================
bool     initializeDataRadio();
bool     initializeControlRadio();
void     applyRadioConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr);
void     sendDataPacket();
void     checkControlPlane();
void     sendControlAck(uint32_t commandID);
uint16_t readSensor();

//======================================================================
// SETUP
//======================================================================
void setup()
{
    Serial.begin(115200);

    pinMode(LED_TX, OUTPUT);
    pinMode(LED_ERROR, OUTPUT);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_ERROR, LOW);

    Serial.println();
    Serial.println("=========================================");
    Serial.println(" Cognitive Radio Transmitter");
    Serial.println("=========================================");

    sharedSPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

    if (!initializeDataRadio())
    {
        Serial.println("DATA RADIO FAILED");
        while (true) { delay(2000); }
    }

    if (!initializeControlRadio())
    {
        Serial.println("CONTROL RADIO FAILED");
        while (true) { delay(1000); }
    }

    controlRadio.startReceive();     // arm control plane (DIO0 -> RxDone)

    Serial.println();
    Serial.println("System Ready");
    Serial.println();
}

//======================================================================
// INITIALIZE DATA RADIO
//======================================================================
bool initializeDataRadio()
{
    Serial.println("-----------------------------------------");
    Serial.println("Initializing DATA Radio...");
    Serial.println("-----------------------------------------");

    float freqMHz = currentFrequencyKHz / 1000.0f;

    int state = dataRadio.begin(
                    freqMHz,
                    125.0,
                    currentSF,
                    currentCR,
                    RADIOLIB_SX127X_SYNC_WORD,
                    17,              // TX power (dBm)
                    8);

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("DATA Radio Init Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("DATA Radio Initialized");
    Serial.print("Frequency (kHz) : "); Serial.println(currentFrequencyKHz);
    Serial.print("SF : ");              Serial.println(currentSF);
    Serial.print("CR : 4/");            Serial.println(currentCR);
    Serial.println();
    return true;
}

//======================================================================
// INITIALIZE CONTROL RADIO
//======================================================================
bool initializeControlRadio()
{
    Serial.println("-----------------------------------------");
    Serial.println("Initializing CONTROL Radio...");
    Serial.println("-----------------------------------------");

    int state = controlRadio.begin(
                    CONTROL_FREQUENCY,
                    125.0,
                    CONTROL_SF,
                    CONTROL_CR,
                    RADIOLIB_SX127X_SYNC_WORD,
                    10,              // lower power is enough
                    8);

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("CONTROL Radio Init Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("CONTROL Radio Initialized");
    Serial.print("Frequency : "); Serial.println(CONTROL_FREQUENCY);
    Serial.print("SF : ");        Serial.println(CONTROL_SF);
    Serial.print("CR : 4/");      Serial.println(CONTROL_CR);
    Serial.println();
    return true;
}

//======================================================================
// APPLY NEW RADIO CONFIGURATION (data plane only)
//======================================================================
void applyRadioConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr)
{
    float freqMHz = frequencyKHz / 1000.0f;

    Serial.println();
    Serial.println("========== APPLYING NEW CONFIGURATION ==========");
    Serial.print("Old Frequency (kHz) : "); Serial.println(currentFrequencyKHz);
    Serial.print("Old SF : ");              Serial.println(currentSF);
    Serial.print("Old CR : 4/");            Serial.println(currentCR);

    currentFrequencyKHz = frequencyKHz;
    currentSF           = sf;
    currentCR           = cr;

    dataRadio.setFrequency(freqMHz);
    dataRadio.setSpreadingFactor(currentSF);
    dataRadio.setCodingRate(currentCR);

    Serial.println();
    Serial.print("New Frequency (kHz) : "); Serial.println(currentFrequencyKHz);
    Serial.print("New SF : ");              Serial.println(currentSF);
    Serial.print("New CR : 4/");            Serial.println(currentCR);
    Serial.println("Configuration Updated");
    Serial.println("===============================================");
    Serial.println();
}

//======================================================================
// READ SENSOR (placeholder)
//======================================================================
uint16_t readSensor()
{
    return random(0, 4096);          // matches ESP32 12-bit ADC range
}

//======================================================================
// SEND DATA PACKET
//======================================================================
void sendDataPacket()
{
    DataPacket packet;
    packet.sequence    = sequenceNumber++;
    packet.timestamp   = millis();
    packet.sensorValue = readSensor();
    packet.magic = PROTO_SECRET;

    digitalWrite(LED_TX, HIGH);
    int state = dataRadio.transmit((uint8_t*)&packet, sizeof(packet));
    digitalWrite(LED_TX, LOW);

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.print("[DATA] Seq=");    Serial.print(packet.sequence);
        Serial.print("  Sensor=");      Serial.print(packet.sensorValue);
        Serial.print("  Time=");        Serial.println(packet.timestamp);
    }
    else
    {
        Serial.print("[DATA] Transmission Failed : ");
        Serial.println(state);
        digitalWrite(LED_ERROR, HIGH);
        delay(50);
        digitalWrite(LED_ERROR, LOW);
    }
}

//======================================================================
// SEND CONTROL ACK
//======================================================================
void sendControlAck(uint32_t commandID)
{
    ControlAck ack;
    ack.magic     = PROTO_MAGIC;
    ack.commandID = commandID;
    ack.applied   = 1;
    ack.checksum  = ackChecksum(ack);

    int state = controlRadio.transmit((uint8_t*)&ack, sizeof(ack));

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.print("[CTRL] ACK Sent : ");
        Serial.println(commandID);
    }
    else
    {
        Serial.print("[CTRL] ACK Failed : ");
        Serial.println(state);
    }

    controlRadio.startReceive();     // back to listening
}

//======================================================================
// CHECK CONTROL CHANNEL (DIO0 polled)
//======================================================================
void checkControlPlane()
{
    // No packet yet? (DIO0 is mapped to RxDone in receive mode.)
    if (digitalRead(CTRL_DIO0) == LOW)
        return;

    ControlPacket command;
    int state = controlRadio.readData((uint8_t*)&command, sizeof(command));

    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        Serial.println("[CTRL] CRC Error");
        controlRadio.startReceive();
        return;
    }
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[CTRL] Receive Error : ");
        Serial.println(state);
        controlRadio.startReceive();
        return;
    }

    // ---- Authenticate ----
    if (command.magic != PROTO_MAGIC ||
        command.checksum != controlChecksum(command))
    {
        Serial.println("[CTRL] Rejected: bad magic/checksum");
        controlRadio.startReceive();
        return;
    }

    // ---- Replay / duplicate guard ----
    if (command.commandID == lastAppliedCommandID)
    {
        // Already applied; the previous ACK was probably lost -> re-ACK only.
        Serial.println("[CTRL] Duplicate command -> re-ACK");
        sendControlAck(command.commandID);
        return;
    }
    if (command.commandID < lastAppliedCommandID)
    {
        Serial.println("[CTRL] Rejected: stale commandID");
        controlRadio.startReceive();
        return;
    }

    Serial.println();
    Serial.println("====================================");
    Serial.println("AI CONTROL COMMAND RECEIVED");
    Serial.println("====================================");
    Serial.print("Command ID : ");      Serial.println(command.commandID);
    Serial.print("Command Type : ");    Serial.println(command.command);
    Serial.print("Frequency (kHz) : "); Serial.println(command.newFrequencyKHz);
    Serial.print("SF : ");              Serial.println(command.newSF);
    Serial.print("CR : 4/");            Serial.println(command.newCR);
    Serial.println();

    switch (command.command)
    {
        case CMD_CHANNEL_HOP:
            Serial.println("Executing Channel Hop...");
            applyRadioConfiguration(command.newFrequencyKHz,
                                    currentSF, currentCR);
            break;

        case CMD_LINK_ADAPT:
            Serial.println("Executing Link Adaptation...");
            applyRadioConfiguration(command.newFrequencyKHz,
                                    command.newSF, command.newCR);
            break;

        default:
            Serial.println("Unknown Command");
            controlRadio.startReceive();
            return;
    }

    lastAppliedCommandID = command.commandID;
    sendControlAck(command.commandID);

    Serial.println("Configuration Applied Successfully");
    Serial.println("====================================");
    Serial.println();
}

//======================================================================
// MAIN LOOP
//======================================================================
void loop()
{
    // 1. Service the control plane.
    checkControlPlane();

    // 2. Periodically transmit application data.
    if (millis() - lastTransmissionTime >= DATA_INTERVAL)
    {
        lastTransmissionTime = millis();
        sendDataPacket();
    }
}
