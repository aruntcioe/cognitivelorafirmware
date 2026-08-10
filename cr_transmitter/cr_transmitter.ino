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

enum ControlPhase : uint8_t
{
    PHASE_NONE    = 0,
    PHASE_PREPARE = 1,   // "proposing this config - don't apply yet"
    PHASE_COMMIT  = 2    // "confirmed - both sides apply after guardTimeMs"
};

struct ControlPacket
{
    uint32_t magic;
    uint32_t commandID;
    uint8_t  command;          // CMD_CHANNEL_HOP / CMD_LINK_ADAPT / CMD_SYNC_PING
    uint8_t  phase;             // PHASE_PREPARE / PHASE_COMMIT (unused for SYNC)
    uint32_t newFrequencyKHz;
    uint8_t  newSF;
    uint8_t  newCR;
    uint32_t guardTimeMs;       // delay after COMMIT before applying (0 for PREPARE/SYNC)
    uint32_t checksum;
};

struct ControlAck
{
    uint32_t magic;
    uint32_t commandID;
    uint8_t  phase;             // echoes which phase this acks (or SYNC_PONG)
    uint8_t  applied;           // 1 = accepted, 0 = rejected
    uint32_t reportedFreqKHz;   // ALWAYS the transmitter's real current config
    uint8_t  reportedSF;
    uint8_t  reportedCR;
    uint32_t checksum;
};
#pragma pack(pop)

enum CommandType
{
    CMD_NONE        = 0,
    CMD_CHANNEL_HOP = 1,
    CMD_LINK_ADAPT  = 2,
    CMD_SYNC_PING   = 3    // heartbeat: "what config are you actually running?"
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
    s += p.phase;
    s += p.newFrequencyKHz;
    s += p.newSF;
    s += p.newCR;
    s += p.guardTimeMs;
    return s;
}

uint32_t ackChecksum(const ControlAck &a)
{
    uint32_t s = PROTO_SECRET;
    s += a.magic;
    s += a.commandID;
    s += a.phase;
    s += a.applied;
    s += a.reportedFreqKHz;
    s += a.reportedSF;
    s += a.reportedCR;
    return s;
}

//======================================================================
// GUARD-TIME APPLY STATE (non-blocking - set by COMMIT, consumed in loop())
//======================================================================
bool     pendingApply     = false;
uint32_t pendingApplyAtMs = 0;
uint32_t pendingFreqKHz   = 0;
uint8_t  pendingSF        = 0;
uint8_t  pendingCR        = 0;

//======================================================================
// CURRENT RADIO CONFIGURATION (kHz throughout)
//======================================================================
uint32_t currentFrequencyKHz = 433000;
uint8_t  currentSF           = 8;
uint8_t  currentCR           = 5;

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
uint32_t lastCommandID       = 0;         // highest commandID we've PREPAREd/COMMITted
uint8_t  lastPhaseHandled    = PHASE_NONE; // which phase of lastCommandID we last acked
uint32_t preparedFreqKHz     = 0;         // config proposed by the pending PREPARE
uint8_t  preparedSF          = 0;
uint8_t  preparedCR          = 0;

const uint32_t DATA_INTERVAL = 2000;        // ms
uint32_t lastTransmissionTime = 0;

//======================================================================
// FUNCTION DECLARATIONS
//======================================================================
bool     initializeDataRadio();
bool     initializeControlRadio();
void     applyRadioConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr);
void     sendDataPacket();
void     checkControlPlane();
void     sendControlAck(uint32_t commandID, uint8_t phase, uint8_t applied);
void     serviceGuardApply();
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
                    17,              // lower power is enough
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
        Serial.print("CSVROW,"); Serial.print(packet.sequence);
Serial.print(","); Serial.print(packet.sensorValue);
Serial.print(","); Serial.println(packet.timestamp);
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
// SEND CONTROL ACK  (also doubles as SYNC_PONG - always reports the truth)
//======================================================================
void sendControlAck(uint32_t commandID, uint8_t phase, uint8_t applied)
{
    ControlAck ack;
    ack.magic          = PROTO_MAGIC;
    ack.commandID      = commandID;
    ack.phase          = phase;
    ack.applied        = applied;
    ack.reportedFreqKHz = currentFrequencyKHz;   // always the REAL current config
    ack.reportedSF      = currentSF;
    ack.reportedCR      = currentCR;
    ack.checksum       = ackChecksum(ack);

    int state = controlRadio.transmit((uint8_t*)&ack, sizeof(ack));

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.print("[CTRL] ACK Sent : commandID="); Serial.print(commandID);
        Serial.print(" phase="); Serial.println(phase);
    }
    else
    {
        Serial.print("[CTRL] ACK Failed : ");
        Serial.println(state);
    }

    controlRadio.startReceive();     // back to listening
}

//======================================================================
// CHECK CONTROL CHANNEL (DIO0 polled) - two-phase commit + SYNC heartbeat
//======================================================================
void checkControlPlane()
{
    if (digitalRead(CTRL_DIO0) == LOW)
        return;

    ControlPacket command;
    int state = controlRadio.readData((uint8_t*)&command, sizeof(command));

    if (state != RADIOLIB_ERR_NONE) {
        controlRadio.startReceive();
        return;
    }

    if (command.magic != PROTO_MAGIC || command.checksum != controlChecksum(command)) {
        controlRadio.startReceive();
        return;
    }

    if (command.command == CMD_SYNC_PING)
    {
        sendControlAck(command.commandID, PHASE_NONE, 1);
        return;
    }

    // ---- Duplicate / progression handling for a commandID we've seen ----
    if (command.commandID == lastCommandID)
    {
        if (command.phase == lastPhaseHandled)
        {
            // True duplicate retransmission of a phase we already acked.
            Serial.print("[CTRL] Duplicate phase="); Serial.print(command.phase);
            Serial.println(" -> re-ACK");
            sendControlAck(command.commandID, command.phase, 1);
            return;
        }

        if (command.phase == PHASE_COMMIT && lastPhaseHandled == PHASE_PREPARE)
        {
            // Legitimate progression: this is the COMMIT for our current PREPARE.
            Serial.println();
            Serial.println("====================================");
            Serial.println("AI CONTROL COMMAND COMMITTED");
            Serial.println("====================================");
            Serial.print("Command ID : "); Serial.println(command.commandID);
            Serial.print("Applying in (ms) : "); Serial.println(command.guardTimeMs);
            Serial.println();

            Serial.print("CTRLROW,TX,PREPARE_RX,"); Serial.print(command.commandID);
            Serial.print(","); Serial.print(command.newFrequencyKHz);
            Serial.print(","); Serial.print(command.newSF);
            Serial.print(","); Serial.println(command.newCR);

            lastPhaseHandled  = PHASE_COMMIT;
            pendingFreqKHz    = preparedFreqKHz;
            pendingSF         = preparedSF;
            pendingCR         = preparedCR;
            pendingApply      = true;
            pendingApplyAtMs  = millis() + command.guardTimeMs;

            sendControlAck(command.commandID, PHASE_COMMIT, 1);
            return;
        }

        // A late/retransmitted PREPARE arriving after we already moved to
        // COMMIT for this same commandID - harmless, just re-ack PREPARE.
        if (command.phase == PHASE_PREPARE && lastPhaseHandled == PHASE_COMMIT)
        {
            sendControlAck(command.commandID, PHASE_PREPARE, 1);
            return;
        }
        // --- FIX 1: UNHANDLED EDGE CASE ---
        // If same commandID arrives with an unexpected phase combination:
        controlRadio.startReceive(); // REQUIRED: Re-arm before exiting!
        return;
    }
    if (command.commandID < lastCommandID)
    {
        Serial.println("[CTRL] Rejected: stale commandID");
        controlRadio.startReceive();
        return;
    }

    // ---- New commandID ----
    if (command.phase == PHASE_PREPARE)
    {
        Serial.println();
        Serial.println("====================================");
        Serial.println("AI CONTROL COMMAND PROPOSED (PREPARE)");
        Serial.println("====================================");
        Serial.print("Command ID : ");      Serial.println(command.commandID);
        Serial.print("Command Type : ");    Serial.println(command.command);
        Serial.print("Frequency (kHz) : "); Serial.println(command.newFrequencyKHz);
        Serial.print("SF : ");              Serial.println(command.newSF);
        Serial.print("CR : 4/");            Serial.println(command.newCR);
        Serial.println("(not applying yet - waiting for COMMIT)");
        Serial.println();

        Serial.print("CTRLROW,TX,COMMITTED,"); Serial.print(command.commandID);
        Serial.print(","); Serial.print(command.newFrequencyKHz);
        Serial.print(","); Serial.print(command.newSF);
        Serial.print(","); Serial.println(command.newCR);

        lastCommandID    = command.commandID;
        lastPhaseHandled = PHASE_PREPARE;
        preparedFreqKHz  = command.newFrequencyKHz;
        preparedSF       = command.newSF;
        preparedCR       = command.newCR;

        sendControlAck(command.commandID, PHASE_PREPARE, 1);
        return;
    }

    // A COMMIT arrived for a commandID we never PREPAREd - can't trust it.
    Serial.println("[CTRL] Rejected: COMMIT with no matching PREPARE");
    controlRadio.startReceive();
}

//======================================================================
// SERVICE GUARD-TIME APPLY - non-blocking, call every loop() iteration.
// The actual hardware reconfiguration happens here, exactly guardTimeMs
// after the COMMIT exchange, in step with the receiver's own guard timer.
//======================================================================
void serviceGuardApply()
{
    if (!pendingApply) return;
    if ((int32_t)(millis() - pendingApplyAtMs) < 0) return;

    Serial.println();
    Serial.println("========== GUARD TIME ELAPSED - APPLYING ==========");
    applyRadioConfiguration(pendingFreqKHz, pendingSF, pendingCR);
    Serial.println("====================================================");
    Serial.println();

   Serial.print("CTRLROW,TX,APPLIED,"); Serial.print(lastCommandID);
    Serial.print(","); Serial.print(pendingFreqKHz);
    Serial.print(","); Serial.print(pendingSF);
    Serial.print(","); Serial.println(pendingCR);

    pendingApply = false;
}

//======================================================================
// MAIN LOOP
//======================================================================
void loop()
{
    // 1. Service the control plane (PREPARE/COMMIT/SYNC) - non-blocking.
    checkControlPlane();

    // 2. Apply any committed config once its guard time has elapsed.
    serviceGuardApply();

    // 3. Periodically transmit application data.
    if (millis() - lastTransmissionTime >= DATA_INTERVAL)
    {
        lastTransmissionTime = millis();
        sendDataPacket();
    }
}