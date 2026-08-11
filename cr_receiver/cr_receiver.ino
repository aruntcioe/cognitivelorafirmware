/***********************************************************************
 * AI-Assisted Cognitive Radio LPWAN
 * RECEIVER NODE  (Corrected)
 *
 * Board:  ESP32-S3 Dev Module   (FSPI)
 * Radios: 2x SX1278 (Ra-02)
 *   DATA  PLANE -> starts at 433 MHz  (follows AI hop commands)
 *   CTRL  PLANE -> fixed 445 MHz      (outside the jammer band)
 *
 * Pipeline:
 *   receive DATA -> extract metrics -> fill feature window ->
 *   compute feature vector -> run inference -> execute policy
 *   (channel hop / link adaptation) over the control plane.
 *
 * ----- FIXES vs. original -----
 *  - Removed all compile errors: missing semicolons, 'unit8_t' typo,
 *    out-of-scope 'command'/'prediction', out-of-scope 'processed'.
 *  - Replaced non-existent isPacketReceived() with DIO0 polling.
 *  - ControlPacket layout is now byte-identical to the transmitter
 *    (packed struct, frequency as uint32_t kHz).
 *  - Frequency handled as uint32_t kHz everywhere (no uint8_t truncation).
 *  - Hop-escape channels moved OUTSIDE the jammer band (433.0-434.6 MHz).
 *  - Control commands are authenticated (magic + checksum) and retried;
 *    inference runs exactly once per completed window.
 ***********************************************************************/

#include <RadioLib.h>
#include <rf_model.h>
#include "jam_engine.h"
#include <SPI.h>

//======================================================================
// SPI BUS PINS
//======================================================================
#define SPI_SCK      12
#define SPI_MISO     11
#define SPI_MOSI     13

//======================================================================
// DATA RADIO (starts 433 MHz)
//======================================================================
#define NSS_DATA     10
#define RST_DATA     14
#define DIO0_DATA    16

//======================================================================
// CONTROL RADIO (445 MHz)
//======================================================================
#define NSS_CTRL     9
#define RST_CTRL     15
#define DIO0_CTRL    18

//======================================================================
// STATUS LEDs
//======================================================================
#define LED_ACTIVITY 17
#define LED_ERROR    4

//======================================================================
// FEATURE WINDOW SIZE
//======================================================================
#define FEATURE_WINDOW_SIZE 10

//======================================================================
// RADIO CONFIGURATION (kHz throughout, must match transmitter)
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
// CHANNEL-HOP ESCAPE TABLE
//
// The jammer occupies 433.0 - 434.6 MHz (9 channels, 200 kHz spacing).
// Escape channels are chosen OUTSIDE that band so a hop actually leaves
// the jammed spectrum.  Verify these against your local ISM/regulatory
// limits before radiating.
//======================================================================
const uint32_t HOP_TABLE_KHZ[] = { 434800, 435000, 435200, 435400 };
const uint8_t  HOP_TABLE_SIZE  = sizeof(HOP_TABLE_KHZ) / sizeof(HOP_TABLE_KHZ[0]);
int hopIndex = -1;

uint32_t windowStartTime = 0;

// ---- add these globals near your other window-state variables ----
float sumForeignRSSI = 0, sumForeignSNR = 0, sumForeignCFO = 0;
int   foreignCount = 0;


uint32_t pendingGuardMs = 0;   // the guardTimeMs we advertised in this command's packet
//======================================================================
// SHARED SPI BUS
//======================================================================
SPIClass sharedSPI(FSPI);

//======================================================================
// RADIO OBJECTS
//======================================================================
SX1278 dataRadio = new Module(
    NSS_DATA, DIO0_DATA, RST_DATA, RADIOLIB_NC, sharedSPI);

SX1278 controlRadio = new Module(
    NSS_CTRL, DIO0_CTRL, RST_CTRL, RADIOLIB_NC, sharedSPI);

//======================================================================
// SHARED PROTOCOL  (MUST be byte-identical on TX and RX)
//======================================================================
static const uint32_t PROTO_MAGIC  = 0xC0DEA5A5UL;
static const uint32_t PROTO_SECRET = 0x5A17C0DEUL;

uint32_t settleUntilMs      = 0;   // no jamming actions allowed until this time
uint8_t  windowsSinceHop    = 0;   // fresh windows observed on the new config
const uint8_t  SETTLE_WINDOWS  = 2;      // require >=2 fresh windows before acting again
const uint32_t SETTLE_MIN_MS   = 8000;   // and at least this long (covers SF12 settle)

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
    uint8_t  phase;             // echoes which phase this acks, or SYNC_PONG
    uint8_t  applied;           // 1 = accepted, 0 = rejected
    uint32_t reportedFreqKHz;   // for SYNC_PONG: transmitter's ACTUAL current config
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
// RAW FEATURE (one entry per received packet)
//======================================================================
struct PacketFeature
{
    uint32_t sequence;
    uint32_t timestamp;
    float    rssi;
    float    snr;
    float    cfo;
    bool     crcOK;
};

PacketFeature featureWindow[FEATURE_WINDOW_SIZE];

//======================================================================
// FEATURE VECTOR (ML input)
//======================================================================
struct FeatureVector
{
    float    meanRSSI;
    float    varRSSI;
    float    meanSNR;
    float    varSNR;
    float    CFO;
    float    PLR;
    uint16_t consecutiveCRCFailures;
    uint8_t  currentSF;
    uint8_t  currentCR;
  };

FeatureVector features;

//======================================================================
// WINDOW STATE
//======================================================================
uint8_t  windowIndex             = 0;
uint8_t  packetsInWindow         = 0;
uint16_t lostPacketsWindow       = 0;
uint16_t crcFailuresWindow       = 0;
uint16_t consecutiveCRCFailures  = 0;
uint32_t previousSequence        = 0;
bool     firstPacket             = true;

uint32_t commandCounter          = 0;

//======================================================================
// NON-BLOCKING CONTROL STATE MACHINE (two-phase commit + guard time)
//======================================================================
enum CtrlState
{
    CTRL_IDLE,
    CTRL_AWAIT_PREPARE_ACK,
    CTRL_AWAIT_COMMIT_ACK,
    CTRL_WAIT_GUARD
};

CtrlState ctrlState        = CTRL_IDLE;
ControlPacket ctrlInFlight;              // packet currently being (re)transmitted
uint8_t  ctrlAttempt        = 0;
const uint8_t CTRL_MAX_ATTEMPTS = 4;
uint32_t ctrlDeadlineMs     = 0;          // when the current attempt is considered timed out
uint32_t ctrlAckTimeoutMs   = 0;          // computed from time-on-air, set per attempt

const uint32_t GUARD_TIME_MS = 3000;      // both sides apply this long after COMMIT exchange
uint32_t ctrlGuardApplyAtMs = 0;
uint32_t pendingFreqKHz     = 0;
uint8_t  pendingSF          = 0;
uint8_t  pendingCR          = 0;

//======================================================================
// SYNC HEARTBEAT STATE
//======================================================================
uint32_t lastSyncSentMs      = 0;
const uint32_t SYNC_INTERVAL_MS   = 30000;   // only fires while ctrlState == CTRL_IDLE
const uint32_t SYNC_REPLY_WINDOW_MS = 2500;
bool     syncAwaitingPong     = false;
uint32_t syncDeadlineMs       = 0;
uint32_t syncCommandID        = 0;
uint8_t  syncMismatchStreak   = 0;
const uint8_t SYNC_MISMATCH_LIMIT = 2;   // consecutive mismatches before forcing a fix
//======================================================================
// SYNC HEARTBEAT STATE
//======================================================================
enum SyncPhase : uint8_t
{
    SYNC_PHASE_IDLE   = 0,
    SYNC_PHASE_TXING  = 1,   // PING is on the air (non-blocking transmit in progress)
    SYNC_PHASE_WAIT   = 2    // PING sent, waiting for PONG
};
SyncPhase syncPhase = SYNC_PHASE_IDLE;

//======================================================================
// FUNCTION DECLARATIONS
//======================================================================
bool  initializeDataRadio();
bool  initializeControlRadio();
void  receiveDataPacket();
void  processFeatureWindow();
void  extractFeatures();
int   runInference();
void  executeDecision(int prediction);
void  beginControlCommand(uint8_t commandType, uint32_t frequencyKHz,
                           uint8_t sf, uint8_t cr);
void  serviceControlPlane();
void  serviceSyncHeartbeat();
void  applyReceiverConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr);
float calculateToA();
float readFrequencyError();

uint32_t getControlTimeoutMs() {
    uint32_t pingToa = controlRadio.getTimeOnAir(sizeof(ControlPacket)) / 1000UL;
    uint32_t pongToa = controlRadio.getTimeOnAir(sizeof(ControlAck)) / 1000UL;
    // Add 500ms safety margin for processing/SPI delay
    return (pingToa + pongToa) + 800UL; 
}
void resetModule(int rstPin) {
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, LOW);
    delay(10);
    digitalWrite(rstPin, HIGH);
    delay(20);   // give the crystal/oscillator time to stabilize before any
                 // SPI register read is attempted (10ms was marginal and is
                 // the likely cause of the intermittent -2 CHIP_NOT_FOUND)
}

// In Receiver & Transmitter: Dynamically calculate guard time for Link Adaptation
uint32_t calculateSafeGuardTimeMs(uint8_t sf) {
    // High SF (SF10-SF12) requires longer guard time for 2-phase commit sync
    if (sf >= 10) return 6000; 
    return 3000;
}

// Replace the fixed constant with a function
uint32_t getWindowTimeoutMs()
{
    // Expected time for a full window of packets, plus margin.
    // getTimeOnAir returns microseconds for the CURRENT data-radio config.
    uint32_t toaMs = dataRadio.getTimeOnAir(sizeof(DataPacket)) / 1000UL;

    // Assume TX sends roughly one packet per (ToA + fixed inter-packet gap).
    // Tune INTERPKT_GAP_MS to match your transmitter's send cadence.
    // const uint32_t INTERPKT_GAP_MS = 500;
    uint32_t perPacket = 500;

    // Wait long enough to *reasonably* fill a window, then flush.
    // 1.5x gives slack for a few missed packets without over-waiting.
    uint32_t timeout = (uint32_t)(perPacket * FEATURE_WINDOW_SIZE * 1.5f);

    // Clamp to sane bounds so a bad config can't hang or thrash the loop.
    if (timeout < 3000)  timeout = 3000;
    if (timeout > 10000) timeout = 10000;
    return timeout;
}
//======================================================================
// SETUP
//======================================================================
void setup()
{
    Serial.begin(115200);

    pinMode(LED_ACTIVITY, OUTPUT);
    pinMode(LED_ERROR, OUTPUT);
    digitalWrite(LED_ACTIVITY, LOW);
    digitalWrite(LED_ERROR, LOW);

    pinMode(NSS_DATA, OUTPUT);
    pinMode(NSS_CTRL, OUTPUT);
    digitalWrite(NSS_DATA, HIGH);
    digitalWrite(NSS_CTRL, HIGH);
    delay(10);

    Serial.println();
    Serial.println("======================================");
    Serial.println(" AI Cognitive Radio Receiver");
    Serial.println("======================================");
    resetModule(RST_DATA);
    resetModule(RST_CTRL);

    sharedSPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
    delay(20);   // let the bus/power rail settle before the first transaction

    if (!initializeDataRadio())
    {
        Serial.println("DATA RADIO FAILED");
        while (true) { delay(1000); }
    }

    if (!initializeControlRadio())
    {
        Serial.println("CONTROL RADIO FAILED");
        while (true) { delay(1000); }
    }

    dataRadio.startReceive();        // arm data plane (DIO0 -> RxDone)
    jamEngineInit(currentFrequencyKHz, currentSF);
    Serial.println();
    Serial.println("Receiver Ready");
    Serial.println();
}

//======================================================================
// INITIALIZE DATA RADIO
//======================================================================
bool initializeDataRadio()
{
    Serial.println("--------------------------------------");
    Serial.println("Initializing DATA Radio...");
    Serial.println("--------------------------------------");

    float freqMHz = currentFrequencyKHz / 1000.0f;

    const uint8_t MAX_INIT_ATTEMPTS = 5;
    int state = RADIOLIB_ERR_UNKNOWN;

    for (uint8_t attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++)
    {
        state = dataRadio.begin(
                        freqMHz,
                        125.0,
                        currentSF,
                        currentCR,
                        RADIOLIB_SX127X_SYNC_WORD,
                        10,
                        8);

        if (state == RADIOLIB_ERR_NONE) break;

        Serial.print("Attempt "); Serial.print(attempt); Serial.print("/");
        Serial.print(MAX_INIT_ATTEMPTS);
        Serial.print(" failed : "); Serial.print(state);
       
        resetModule(RST_DATA);   // re-toggle RST before retrying
        delay(50);
    }

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("Initialization Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("DATA Radio Ready");
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
    Serial.println("--------------------------------------");
    Serial.println("Initializing CONTROL Radio...");
    Serial.println("--------------------------------------");

    const uint8_t MAX_INIT_ATTEMPTS = 5;
    int state = RADIOLIB_ERR_UNKNOWN;

    for (uint8_t attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++)
    {
        state = controlRadio.begin(
                        CONTROL_FREQUENCY,
                        125.0,
                        CONTROL_SF,
                        CONTROL_CR,
                        RADIOLIB_SX127X_SYNC_WORD,
                       17,
                        8);

        if (state == RADIOLIB_ERR_NONE) break;

        Serial.print("Attempt "); Serial.print(attempt); Serial.print("/");
        Serial.print(MAX_INIT_ATTEMPTS);
        Serial.print(" failed : "); Serial.print(state);
   
        resetModule(RST_CTRL);   // re-toggle RST before retrying
        delay(50);
    }

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("Initialization Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("CONTROL Radio Ready");
    Serial.println();
    return true;
}

//======================================================================
// TIME ON AIR (uses current radio configuration)
//======================================================================
float calculateToA()
{
    size_t payloadLength = sizeof(DataPacket);
    return dataRadio.getTimeOnAir(payloadLength);
}

//======================================================================
// FREQUENCY ERROR (CFO)
//
// RadioLib does not expose SX1278 frequency error; returns 0 for now.
// Replace later with direct RegFei register reads.
//======================================================================

 float readFrequencyError() {
  return dataRadio.getFrequencyError();
}



//======================================================================
// RECEIVE DATA PACKET (fixed: magic-checked capture-effect handling)
//======================================================================

void receiveDataPacket()
{
    if (windowStartTime == 0) windowStartTime = millis();

    if (digitalRead(DIO0_DATA) == LOW)
    {
        if (ctrlState == CTRL_WAIT_GUARD || ctrlState == CTRL_AWAIT_COMMIT_ACK) return;
        uint32_t windowTimeout = getWindowTimeoutMs();
        // no packet yet -- check if window has timed out
        if (millis() - windowStartTime > windowTimeout &&
            (packetsInWindow > 0 || foreignCount > 0))
        {
            processFeatureWindow();   // flush, using real OR foreign-capture stats
            windowIndex = 0; 
            packetsInWindow = 0;
            lostPacketsWindow = 0; 
            crcFailuresWindow = 0;
            sumForeignRSSI = 0;
            sumForeignSNR = 0; 
            sumForeignCFO = 0; 
            foreignCount = 0;
            windowStartTime = millis();
        }
        else if (millis() - windowStartTime > windowTimeout &&
                 packetsInWindow == 0 && foreignCount == 0)
        {
            // This really is undefined -- keep the sentinel only for this case.
            Serial.println("CSVROW,-999,0,-999,0,0,1.0000,0,0,0,0");
            windowStartTime = millis();
        }
        return;
    }

    DataPacket packet;
    int state = dataRadio.readData((uint8_t*)&packet, sizeof(packet));

    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        Serial.println("[DATA] CRC Failure");
        Serial.println("PKTEVT,CRC_FAIL");
        crcFailuresWindow++;
        consecutiveCRCFailures++;
        lostPacketsWindow++;
        windowIndex++;
        dataRadio.startReceive();
        return;
    }

    // ---- Other receive error ----
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[DATA] RX Error : ");
        lostPacketsWindow++;
        Serial.println(state);
        dataRadio.startReceive();
        return;
    }

    if (packet.magic != PROTO_SECRET)
    {
        // Foreign/jammer packet -- CRC passed so RadioLib's RSSI/SNR/CFO
        // readings ARE valid measurements of the interferer's signal.
        // Capture them instead of throwing them away.
        float fRSSI = dataRadio.getRSSI();
        float fSNR  = dataRadio.getSNR();
        float fCFO  = readFrequencyError();

        Serial.print("[DATA] Foreign/Jammer packet captured (magic mismatch) RSSI=");
        Serial.print(fRSSI); Serial.print(" SNR="); Serial.println(fSNR);

        Serial.print("PKTEVT,JAMMER,");
        Serial.print(fRSSI, 2); Serial.print(",");
        Serial.print(fSNR, 2);  Serial.print(",");
        Serial.println(fCFO, 2);

        sumForeignRSSI += fRSSI;
        sumForeignSNR  += fSNR;
        sumForeignCFO  += fCFO;
        foreignCount++;

        crcFailuresWindow++;
        consecutiveCRCFailures++;
        lostPacketsWindow++;
        dataRadio.startReceive();
        return;
    }

    // ---- Good, verified packet ----
    consecutiveCRCFailures = 0;
    digitalWrite(LED_ACTIVITY, HIGH);

    if (firstPacket)
    {
        previousSequence = packet.sequence;
        firstPacket = false;
    }
    else if (packet.sequence > previousSequence + 1)
    {
        lostPacketsWindow += (packet.sequence - previousSequence - 1);
        previousSequence = packet.sequence;
    }
    else
    {
        previousSequence = packet.sequence;
    }

    featureWindow[windowIndex].sequence  = packet.sequence;
    featureWindow[windowIndex].timestamp = packet.timestamp;
    featureWindow[windowIndex].rssi      = dataRadio.getRSSI();
    featureWindow[windowIndex].snr       = dataRadio.getSNR();
    featureWindow[windowIndex].cfo       = readFrequencyError();
    featureWindow[windowIndex].crcOK     = true;

    Serial.println("--------------------------------");
    Serial.print("Sequence : "); Serial.println(packet.sequence);
    Serial.print("RSSI : ");     Serial.println(featureWindow[windowIndex].rssi);
    Serial.print("SNR : ");      Serial.println(featureWindow[windowIndex].snr);
    Serial.print("Sensor : ");   Serial.println(packet.sensorValue);
    Serial.print("CFO : ");      Serial.println(featureWindow[windowIndex].cfo);
    Serial.println("--------------------------------");

    // ---- Machine-readable per-packet row for the PC-side dashboard ----
    // PKTROW,sequence,rssi,snr,cfo,sensorValue,timestamp
    Serial.print("PKTROW,");
    Serial.print(packet.sequence);                          Serial.print(",");
    Serial.print(featureWindow[windowIndex].rssi, 2);        Serial.print(",");
    Serial.print(featureWindow[windowIndex].snr, 2);         Serial.print(",");
    Serial.print(featureWindow[windowIndex].cfo, 2);         Serial.print(",");
    Serial.print(packet.sensorValue);                        Serial.print(",");
    Serial.println(packet.timestamp);

    windowIndex++;
    packetsInWindow++;

    if (windowIndex >= FEATURE_WINDOW_SIZE)
    {
        processFeatureWindow();
        windowIndex        = 0;
        packetsInWindow    = 0;
        lostPacketsWindow  = 0;
        crcFailuresWindow  = 0;
        sumForeignRSSI = 0; sumForeignSNR = 0; sumForeignCFO = 0; foreignCount = 0;
        windowStartTime    = millis();   // <-- Bug 1 fix
    }

    digitalWrite(LED_ACTIVITY, LOW);
    dataRadio.startReceive();
}



//======================================================================
// PROCESS FEATURE WINDOW
//======================================================================
void processFeatureWindow()
{
    if (windowsSinceHop < 255) windowsSinceHop++;
    Serial.println();
    Serial.println("=======================================");
    Serial.println(" Feature Window Complete");
    Serial.println("=======================================");

    extractFeatures();

    Serial.println();
    Serial.println("========== FEATURE VECTOR ==========");
    Serial.print("Mean RSSI : ");     Serial.println(features.meanRSSI);
    Serial.print("Variance RSSI : "); Serial.println(features.varRSSI);
    Serial.print("Mean SNR : ");      Serial.println(features.meanSNR);
    Serial.print("Variance SNR : ");  Serial.println(features.varSNR);
    Serial.print("Mean CFO : ");      Serial.println(features.CFO);
    Serial.print("PLR : ");           Serial.println(features.PLR);
    Serial.print("CRC Failures : ");  Serial.println(features.consecutiveCRCFailures);
    Serial.print("Current SF : ");    Serial.println(features.currentSF);
    Serial.print("Current CR : ");    Serial.println(features.currentCR);
    Serial.println("====================================");

    // ---- Machine-readable line for the PC-side logger ----
    // Order MUST match HEADER in getcsv.py:
    // meanRSSI,varRSSI,meanSNR,varSNR,CFO,PLR,CRC,SF,CR
    Serial.print("CSVROW,");
    Serial.print(features.meanRSSI, 4);        Serial.print(",");
    Serial.print(features.varRSSI, 4);         Serial.print(",");
    Serial.print(features.meanSNR, 4);         Serial.print(",");
    Serial.print(features.varSNR, 4);          Serial.print(",");
    Serial.print(features.CFO, 4);             Serial.print(",");
    Serial.print(features.PLR, 4);             Serial.print(",");
    Serial.print(features.consecutiveCRCFailures); Serial.print(",");
    Serial.print(features.currentSF);          Serial.print(",");
    Serial.print(features.currentCR);          Serial.print(",");


    int prediction = runInference();
    Serial.print("Prediction = ");
    Serial.println(prediction);
    Serial.println();

    // ---- Machine-readable line for the PC-side dashboard ----
    Serial.print("AIROW,");
    Serial.println(prediction);
      jamEngineRecordWindow(prediction,
                          features.PLR,
                          features.consecutiveCRCFailures,
                          currentFrequencyKHz,
                          currentSF);
    executeDecision(prediction);
}
//======================================================================
// FEATURE EXTRACTION
//======================================================================
void extractFeatures()
{
    int n = packetsInWindow;
    if (n == 0)
    {
        // Total loss: no packets arrived this window at all.
        // RSSI/SNR/CFO/ToA are undefined -- use sentinel values rather
        // than silently dividing by zero (which on most toolchains
        // yields NaN/inf and corrupts everything downstream, including
        // the ML feature vector and any CSV row printed from it).
          if (foreignCount > 0)
        {
            // No real data got through, but we DID measure the jammer's
            // signal repeatedly -- use that as the RSSI/SNR/CFO signature
            // for this window. This is the useful "exact SF/freq match,
            // jammer totally dominates" case.
            features.meanRSSI = sumForeignRSSI / foreignCount;
            features.varRSSI  = 0.0f;   // single-source variance; see note below
            features.meanSNR  = sumForeignSNR  / foreignCount;
            features.varSNR   = 0.0f;
            features.CFO      = sumForeignCFO  / foreignCount;
                features.PLR      = 1.0f;   // 100% loss of real TX data
        }
            else if (crcFailuresWindow > 0)
        {
            // Nothing decoded cleanly, but corrupted frames WERE heard --
            // heavy-noise / desync signature, distinct from true silence.
            features.meanRSSI = -999.0f;   // RadioLib doesn't expose RSSI on CRC failure
            features.varRSSI  = 0.0f;
            features.meanSNR  = -999.0f;
            features.varSNR   = 0.0f;
            features.CFO      = 0.0f;
           
            features.PLR      = 1.0f;
        }
        else
        {
        features.meanRSSI = -999.0f;
        features.varRSSI  = 0.0f;
        features.meanSNR  = -999.0f;
        features.varSNR   = 0.0f;
        features.CFO      = 0.0f;
     
        features.PLR      = 1.0f;   // 100% loss
        }
        features.consecutiveCRCFailures = consecutiveCRCFailures;
        features.currentSF = currentSF;
        features.currentCR = currentCR;
        return;
    }

    float sumRSSI = 0, sumSNR = 0, sumCFO = 0;

    for (int i = 0; i < n; i++)
    {
        sumRSSI += featureWindow[i].rssi;
        sumSNR  += featureWindow[i].snr;
        sumCFO  += featureWindow[i].cfo;
    }

    features.meanRSSI = sumRSSI / n;
    features.meanSNR  = sumSNR  / n;
    features.CFO      = sumCFO  / n;

    float rssiVariance = 0, snrVariance = 0;
    for (int i = 0; i < n; i++)
    {
        float dR = featureWindow[i].rssi - features.meanRSSI;
        rssiVariance += dR * dR;
        float dS = featureWindow[i].snr - features.meanSNR;
        snrVariance += dS * dS;
    }
    features.varRSSI = rssiVariance / n;
    features.varSNR  = snrVariance / n;

    // Packet loss rate over the window
    // (n successfully-decoded packets out of n + lostPacketsWindow expected)
    features.PLR = (float)lostPacketsWindow /
                   (n + lostPacketsWindow);

    features.consecutiveCRCFailures = consecutiveCRCFailures;
    features.currentSF = currentSF;
    features.currentCR = currentCR;
}


int runInference()
{
    // 1. Create a float array matching the required feature count
    float x[RF_N_FEATURES];

    // 2. Map struct members into the array using the macro indices
    x[F_meanRSSI]  = features.meanRSSI;
    x[F_varRSSI]   = features.varRSSI;
    x[F_meanSNR]   = features.meanSNR;
    x[F_varSNR]    = features.varSNR;
    x[F_CFO]       = features.CFO;
    x[F_PLR]       = features.PLR;
    
    // Explicitly cast non-float members to float
    x[F_CRC]       = (float)features.consecutiveCRCFailures;
    x[F_SF]        = (float)features.currentSF;
    x[F_CR]        = (float)features.currentCR;
    
    x[F_link_lost] = 0.0f;

    uint8_t predicted_class = rf_predict(x);

    return (int)predicted_class;
}

//======================================================================
// APPLY RECEIVER CONFIGURATION (data plane)
//======================================================================
void applyReceiverConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr)
{
    float freqMHz = frequencyKHz / 1000.0f;

    dataRadio.setFrequency(freqMHz);
    dataRadio.setSpreadingFactor(sf);
    dataRadio.setCodingRate(cr);
    dataRadio.startReceive();

    Serial.println("Receiver Reconfigured");
}

//======================================================================
// TRANSMIT ONE CONTROL PACKET (PREPARE or COMMIT phase) - fire and forget,
// timing/retry handled by serviceControlPlane()
//======================================================================
void transmitCtrlPacket()
{
    int state = controlRadio.transmit((uint8_t*)&ctrlInFlight, sizeof(ctrlInFlight));
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[CTRL] TX Failed : ");
        Serial.println(state);
    }

    // Timeout must cover actual LoRa time-on-air, not a guessed constant
    // (see earlier fix - fixed 1500ms was shorter than a single SF12 packet).
    uint32_t cmdToaUs = controlRadio.getTimeOnAir(sizeof(ControlPacket));
    uint32_t ackToaUs = controlRadio.getTimeOnAir(sizeof(ControlAck));
    ctrlAckTimeoutMs  = ((cmdToaUs + ackToaUs) / 1000UL) * 2 + 500;
    ctrlDeadlineMs    = millis() + ctrlAckTimeoutMs;

    controlRadio.startReceive();
}

//======================================================================
// BEGIN A NEW CONTROL COMMAND (non-blocking - returns immediately)
// Kicks off the PREPARE phase. Call this from executeDecision() instead
// of the old blocking sendControlPacket().
//======================================================================
void beginControlCommand(uint8_t commandType, uint32_t frequencyKHz,
                          uint8_t sf, uint8_t cr)
{
    if (ctrlState != CTRL_IDLE)
    {
        Serial.println("[CTRL] Busy with a previous command - skipping this window");
        Serial.println("CTRLROW,RX,BUSY_SKIP,0,0,0,0");
        return;
    }

    pendingFreqKHz = frequencyKHz;
    pendingSF      = sf;
    pendingCR      = cr;

    ctrlInFlight.magic           = PROTO_MAGIC;
    ctrlInFlight.commandID       = ++commandCounter;
    ctrlInFlight.command         = commandType;
    ctrlInFlight.phase           = PHASE_PREPARE;
    ctrlInFlight.newFrequencyKHz = frequencyKHz;
    ctrlInFlight.newSF           = sf;
    ctrlInFlight.newCR           = cr;
    ctrlInFlight.guardTimeMs     = calculateSafeGuardTimeMs(sf);
    pendingGuardMs               = ctrlInFlight.guardTimeMs; 
    ctrlInFlight.checksum        = controlChecksum(ctrlInFlight);

    ctrlAttempt = 1;
    Serial.print("[CTRL] PREPARE sent (commandID="); Serial.print(ctrlInFlight.commandID);
    Serial.print("CTRLROW,RX,PREPARE_SENT,"); Serial.print(ctrlInFlight.commandID);
    Serial.print(","); Serial.print(ctrlInFlight.newFrequencyKHz);
    Serial.print(","); Serial.print(ctrlInFlight.newSF);
    Serial.print(","); Serial.println(ctrlInFlight.newCR);
    Serial.println(", attempt 1/" + String(CTRL_MAX_ATTEMPTS) + ")");

    transmitCtrlPacket();
    ctrlState = CTRL_AWAIT_PREPARE_ACK;
}

//======================================================================
// SERVICE CONTROL PLANE - call every loop() iteration. Non-blocking:
// advances the PREPARE -> COMMIT -> guard-wait -> apply state machine
// and handles retries/timeouts without ever stalling DATA reception.
//======================================================================
void serviceControlPlane()
{
    // ---- Guard-time expiry: apply the already-committed config ----
    if (ctrlState == CTRL_WAIT_GUARD)
    {
        if ((int32_t)(millis() - ctrlGuardApplyAtMs) >= 0)
        {
            currentFrequencyKHz = pendingFreqKHz;
            currentSF           = pendingSF;
            currentCR           = pendingCR;
            applyReceiverConfiguration(currentFrequencyKHz, currentSF, currentCR);
               // Discard any partial window built on the OLD config / during transition
            windowIndex        = 0;
            packetsInWindow    = 0;
            lostPacketsWindow  = 0;
            crcFailuresWindow  = 0;
            sumForeignRSSI = 0; sumForeignSNR = 0; sumForeignCFO = 0; foreignCount = 0;
            firstPacket        = true;      // <-- critical: don't count the seq gap across the hop
            settleUntilMs    = millis() + SETTLE_MIN_MS;
            windowsSinceHop  = 0;
            Serial.println("[CTRL] Entering post-hop settling window");
            windowStartTime    = millis();
            Serial.println("[CTRL] Guard time elapsed - configuration applied");
            Serial.print("CTRLROW,RX,APPLIED,"); Serial.print(ctrlInFlight.commandID);
            Serial.print(","); Serial.print(currentFrequencyKHz);
            Serial.print(","); Serial.print(currentSF);
            Serial.print(","); Serial.println(currentCR);
            ctrlState = CTRL_IDLE;
        }
        return;   // nothing else to do while waiting out the guard
    }

    if (ctrlState == CTRL_IDLE) return;

    // ---- Check for an inbound ACK ----
    if (digitalRead(DIO0_CTRL) == HIGH)
    {
        ControlAck ack;
        int state = controlRadio.readData((uint8_t*)&ack, sizeof(ack));

        if (state == RADIOLIB_ERR_NONE &&
            ack.magic == PROTO_MAGIC &&
            ack.checksum == ackChecksum(ack) &&
            ack.commandID == ctrlInFlight.commandID &&
            ack.applied != 0)
        {
            if (ctrlState == CTRL_AWAIT_PREPARE_ACK && ack.phase == PHASE_PREPARE)
            {
                Serial.println("[CTRL] PREPARE_ACK received -> sending COMMIT");
                Serial.print("CTRLROW,RX,PREPARE_ACK,"); Serial.print(ctrlInFlight.commandID);
                Serial.print(","); Serial.print(ctrlInFlight.newFrequencyKHz);
                Serial.print(","); Serial.print(ctrlInFlight.newSF);
                Serial.print(","); Serial.println(ctrlInFlight.newCR);

                ctrlInFlight.phase    = PHASE_COMMIT;
                ctrlInFlight.checksum = controlChecksum(ctrlInFlight);
                ctrlAttempt = 1;
                transmitCtrlPacket();
                ctrlState = CTRL_AWAIT_COMMIT_ACK;
                return;
            }

            if (ctrlState == CTRL_AWAIT_COMMIT_ACK && ack.phase == PHASE_COMMIT)
            {
                Serial.println("[CTRL] COMMIT_ACK received -> both sides now guard-waiting");
                Serial.print("CTRLROW,RX,COMMIT_ACK,"); Serial.print(ctrlInFlight.commandID);
                Serial.print(","); Serial.print(ctrlInFlight.newFrequencyKHz);
                Serial.print(","); Serial.print(ctrlInFlight.newSF);
                Serial.print(","); Serial.println(ctrlInFlight.newCR);
                uint32_t cmdToaMs = controlRadio.getTimeOnAir(sizeof(ControlPacket)) / 1000UL;
                uint32_t ackToaMs = controlRadio.getTimeOnAir(sizeof(ControlAck))    / 1000UL;
                uint32_t roundTripMs = cmdToaMs + ackToaMs;   // COMMIT out + COMMIT_ACK back
               
                uint32_t remainingGuard = (pendingGuardMs > roundTripMs)
                                ? (pendingGuardMs - roundTripMs)
                                : 0;
                ctrlGuardApplyAtMs = millis() + remainingGuard;
                ctrlState = CTRL_WAIT_GUARD;
                return;
            }
        }

        controlRadio.startReceive();   // stray/foreign packet - keep waiting
    }

    // ---- Retry / give up on timeout ----
    if ((int32_t)(millis() - ctrlDeadlineMs) >= 0)
    {
        if (ctrlAttempt >= CTRL_MAX_ATTEMPTS)
        {
            Serial.println("[CTRL] Command failed after retries (config unchanged)");
            Serial.print("CTRLROW,RX,FAILED,"); Serial.print(ctrlInFlight.commandID);
            Serial.println(",0,0,0");
            ctrlState = CTRL_IDLE;
            dataRadio.startReceive();   // re-arm data plane just in case
            return;
        }

        ctrlAttempt++;
        Serial.print("[CTRL] Timeout - retrying (attempt ");
        Serial.print(ctrlAttempt); Serial.print("/");
        Serial.print(CTRL_MAX_ATTEMPTS); Serial.println(")");
        transmitCtrlPacket();
    }
}

void serviceSyncHeartbeat()
{
    if (ctrlState != CTRL_IDLE) return;

    // ================================================================
    // PHASE: PING transmit in progress (NON-BLOCKING) -- CHANGE #1
    // We started the PING with startTransmit(); wait for TxDone on
    // DIO0_CTRL. Crucially, receiveDataPacket() keeps running every
    // loop() iteration while this air-time (~1.5s at SF12) elapses.
    // ================================================================
    if (syncPhase == SYNC_PHASE_TXING)
    {
        if (digitalRead(DIO0_CTRL) == HIGH)
        {
            controlRadio.finishTransmit();          // clear TxDone / flush
            controlRadio.startReceive();            // now listen for the PONG
            syncPhase      = SYNC_PHASE_WAIT;
            syncDeadlineMs = millis() + getControlTimeoutMs();
            Serial.println("SYNCROW,PING_SENT");
        }
        return;   // do nothing else until the PING finishes going out
    }

    // ================================================================
    // PHASE: waiting for PONG (unchanged logic, driven by syncPhase)
    // ================================================================
    if (syncPhase == SYNC_PHASE_WAIT)
    {
        if (digitalRead(DIO0_CTRL) == HIGH)
        {
            ControlAck pong;
            int state = controlRadio.readData((uint8_t*)&pong, sizeof(pong));

            if (state == RADIOLIB_ERR_NONE &&
                pong.magic == PROTO_MAGIC &&
                pong.checksum == ackChecksum(pong) &&
                pong.commandID == syncCommandID)
            {
                bool matches = (pong.reportedFreqKHz == currentFrequencyKHz &&
                                pong.reportedSF      == currentSF &&
                                pong.reportedCR      == currentCR);

                if (matches)
                {
                    syncMismatchStreak = 0;
                    Serial.println("SYNCROW,MATCH");
                }
                else
                {
                    syncMismatchStreak++;
                    if (syncMismatchStreak >= SYNC_MISMATCH_LIMIT)
                    {
                        currentFrequencyKHz = pong.reportedFreqKHz;
                        currentSF           = pong.reportedSF;
                        currentCR           = pong.reportedCR;
                        applyReceiverConfiguration(currentFrequencyKHz, currentSF, currentCR);
                        // Discard partial window built on OLD config
                        windowIndex = 0; packetsInWindow = 0;
                        lostPacketsWindow = 0; crcFailuresWindow = 0;
                        sumForeignRSSI = 0; sumForeignSNR = 0; sumForeignCFO = 0; foreignCount = 0;
                        firstPacket = true;
                        settleUntilMs    = millis() + SETTLE_MIN_MS;
                        windowsSinceHop  = 0;
                        Serial.println("[CTRL] Entering post-hop settling window");
                        windowStartTime = millis();
                        syncMismatchStreak = 0;
                        Serial.println("SYNCROW,DESYNC_FIXED");
                    }
                }
                syncPhase = SYNC_PHASE_IDLE;
                controlRadio.startReceive();
                return;
            }
            controlRadio.startReceive();   // stray packet, keep waiting
        }
        else if ((int32_t)(millis() - syncDeadlineMs) >= 0)
        {
            Serial.println("SYNCROW,TIMEOUT");
            syncPhase = SYNC_PHASE_IDLE;
            controlRadio.startReceive();
        }
        return;
    }

    // ================================================================
    // PHASE: IDLE -- decide whether to KICK OFF a new PING -- CHANGE #2
    // ================================================================
    if (millis() - lastSyncSentMs < SYNC_INTERVAL_MS) return;

    // Don't even START a PING if a DATA packet is currently pending on
    // the data radio -- service it first this iteration.
    if (digitalRead(DIO0_DATA) == HIGH) return;

    lastSyncSentMs = millis();

    ControlPacket ping;
    ping.magic           = PROTO_MAGIC;
    ping.commandID       = ++commandCounter;
    ping.command         = CMD_SYNC_PING;
    ping.phase           = PHASE_NONE;
    ping.newFrequencyKHz = currentFrequencyKHz;
    ping.newSF           = currentSF;
    ping.newCR           = currentCR;
    ping.guardTimeMs     = 0;
    ping.checksum        = controlChecksum(ping);

    // --- CHANGE #3: NON-BLOCKING transmit ---
    // startTransmit() returns immediately; the radio sends in the
    // background and raises DIO0_CTRL when done. We handle that above.
    int state = controlRadio.startTransmit((uint8_t*)&ping, sizeof(ping));
    if (state == RADIOLIB_ERR_NONE)
    {
        syncCommandID = ping.commandID;
        syncPhase     = SYNC_PHASE_TXING;   // wait for TxDone, don't block
    }
    else
    {
        Serial.print("[SYNC] startTransmit failed: "); Serial.println(state);
        controlRadio.startReceive();
        syncPhase = SYNC_PHASE_IDLE;
    }
}

//======================================================================
// EXECUTE AI DECISION
//======================================================================
// void executeDecision(int prediction)
// {
//     switch (prediction)
//     {
//         case 0:  // Normal
//             Serial.println("[AI] Normal Channel");
//             break;

//         case 1:  // Jammer -> hop to a channel outside the jammer band
//         {
//             Serial.println("[AI] Jammer Detected");
//             hopIndex = (hopIndex + 1) % HOP_TABLE_SIZE;
//             uint32_t target = HOP_TABLE_KHZ[hopIndex];
//             Serial.print("Channel Hopping to (kHz) : ");
//             Serial.println(target);
//             sendControlPacket(CMD_CHANNEL_HOP, target, currentSF, currentCR);
//             break;
//         }

//         case 2:  // Weak link -> increase robustness (only if not already robust)
//             Serial.println("[AI] Weak Link");
//             if (currentSF != 10 || currentCR != 8)
//             {
//                 Serial.println("Increasing Robustness");
//                 sendControlPacket(CMD_LINK_ADAPT, currentFrequencyKHz, 10, 8);
//             }
//             else
//             {
//                 Serial.println("Already at robust settings");
//             }
//             break;

//         case 3:  // Excellent link -> optimize throughput (only if not already fast)
//             Serial.println("[AI] Excellent Link");
//             if (currentSF != 7 || currentCR != 5)
//             {
//                 Serial.println("Optimizing Throughput");
//                 sendControlPacket(CMD_LINK_ADAPT, currentFrequencyKHz, 7, 5);
//             }
//             else
//             {
//                 Serial.println("Already at optimal settings");
//             }
//             break;

//         default:
//             Serial.println("Unknown AI Prediction");
//             break;
//     }
// }
bool actionAllowed() {
    return ((int32_t)(millis() - settleUntilMs) >= 0) &&
           (windowsSinceHop >= SETTLE_WINDOWS) &&
           (ctrlState == CTRL_IDLE);
}

//======================================================================
// EXECUTE AI DECISION
//======================================================================
void executeDecision(int prediction) {
  switch (prediction) {
    case 0:
      Serial.println("\n[AI] Normal Channel");
      jamEngineResetStreak();
      break;
case 1:
{

    if (!actionAllowed())
    {
        Serial.print("[AI] Jammer verdict IGNORED - still settling (");
        Serial.print(windowsSinceHop); Serial.print("/");
        Serial.print(SETTLE_WINDOWS); Serial.println(" windows)");
        Serial.println("CTRLROW,RX,JAM_SETTLING_SKIP,0,0,0,0");
        break;   // observe more before acting
    }

    Serial.println("\n[AI] Jammer Detected -> cognitive re-select");
    uint32_t f; uint8_t sf;
    jamEngineSelect(currentFrequencyKHz, currentSF, &f, &sf);
    Serial.print("CTRLROW,RX,JAM_HOP,streak="); Serial.print(jamEngineStreak());
    Serial.print(","); Serial.print(f);
    Serial.print(","); Serial.println(sf);
    beginControlCommand(CMD_CHANNEL_HOP, f, sf, currentCR);
    break;
}
    case 2:
      { 
         // Added curly braces to allow local variable declarations safely
        Serial.println("\n[AI] Weak Link\nIncreasing Robustness");
        
        if (!actionAllowed()) {
           Serial.println("CTRLROW,RX,LINK_SETTLING_SKIP,0,0,0,0");
            break; 
           }
        // If it's below 10, force start at 10.
        // If it's 10 or 11, step up.
        // If it's 12 or higher, cap at 12.
        int sf = (currentSF < 10) ? 10 : ((currentSF < 12) ? currentSF + 1 : 12);

        // Same logic for CR: below 6 -> 6. Between 6 and 7 -> step up. 8+ -> cap at 8.
        int cr = (currentCR < 6) ? 6 : ((currentCR < 8) ? currentCR + 1 : 8);

        beginControlCommand(CMD_LINK_ADAPT, currentFrequencyKHz, sf, cr);
        jamEngineResetStreak(); // fading != jamming: reset escalation
        break;
      }

    default:
      Serial.println("\nUnknown AI Prediction");
      break;
  }
}
//======================================================================
// LOOP
//======================================================================
void loop()
{
    // Inference and policy execution run once per completed feature
    // window, triggered from inside receiveDataPacket().
    receiveDataPacket();

    // Non-blocking: advances any in-flight PREPARE/COMMIT exchange and
    // applies the config once its guard time elapses. Never stalls DATA rx.
    serviceControlPlane();

    // Non-blocking: periodic config-agreement check between the two nodes.
    serviceSyncHeartbeat();
}
