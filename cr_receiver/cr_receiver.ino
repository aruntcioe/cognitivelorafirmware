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
const uint32_t WINDOW_TIMEOUT_MS = 15000; 
// ---- add these globals near your other window-state variables ----
float sumForeignRSSI = 0, sumForeignSNR = 0, sumForeignCFO = 0;
int   foreignCount = 0;

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
// RAW FEATURE (one entry per received packet)
//======================================================================
struct PacketFeature
{
    uint32_t sequence;
    uint32_t timestamp;
    float    rssi;
    float    snr;
    float    cfo;
    float    toa;
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
    float    meanToA;
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
// FUNCTION DECLARATIONS
//======================================================================
bool  initializeDataRadio();
bool  initializeControlRadio();
void  receiveDataPacket();
void  processFeatureWindow();
void  extractFeatures();
int   runInference();
void  executeDecision(int prediction);
void  sendControlPacket(uint8_t commandType, uint32_t frequencyKHz,
                        uint8_t sf, uint8_t cr);
bool  waitForControlAck(uint32_t expectedID);
void  applyReceiverConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr);
float calculateToA();
float readFrequencyError();

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

    Serial.println();
    Serial.println("======================================");
    Serial.println(" AI Cognitive Radio Receiver");
    Serial.println("======================================");

    sharedSPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

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

    int state = dataRadio.begin(
                    freqMHz,
                    125.0,
                    currentSF,
                    currentCR,
                    RADIOLIB_SX127X_SYNC_WORD,
                    10,
                    8);

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

    int state = controlRadio.begin(
                    CONTROL_FREQUENCY,
                    125.0,
                    CONTROL_SF,
                    CONTROL_CR,
                    RADIOLIB_SX127X_SYNC_WORD,
                    10,
                    8);

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
        // no packet yet -- check if window has timed out
        if (millis() - windowStartTime > WINDOW_TIMEOUT_MS &&
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
        else if (millis() - windowStartTime > WINDOW_TIMEOUT_MS &&
                 packetsInWindow == 0 && foreignCount == 0)
        {
            // TRUE silence: nothing decoded at all, not even jammer preamble.
            // This really is undefined -- keep the sentinel only for this case.
            Serial.println("CSVROW,-999,0,-999,0,0,1.0000,0,0,0,0");
            windowStartTime = millis();
        }
        return;
    }

    DataPacket packet;
    int state = dataRadio.readData((uint8_t*)&packet, sizeof(packet));

    // ---- CRC error ----
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        Serial.println("[DATA] CRC Failure");
        crcFailuresWindow++;
        consecutiveCRCFailures++;
        lostPacketsWindow++;
        dataRadio.startReceive();
        return;
    }

    // ---- Other receive error ----
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[DATA] RX Error : ");
        Serial.println(state);
        dataRadio.startReceive();
        return;
    }

    // ---- CRC passed, but check WHO actually sent it ----
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
    featureWindow[windowIndex].toa       = calculateToA();
    featureWindow[windowIndex].crcOK     = true;

    Serial.println("--------------------------------");
    Serial.print("Sequence : "); Serial.println(packet.sequence);
    Serial.print("RSSI : ");     Serial.println(featureWindow[windowIndex].rssi);
    Serial.print("SNR : ");      Serial.println(featureWindow[windowIndex].snr);
    Serial.print("Sensor : ");   Serial.println(packet.sensorValue);
    Serial.print("TOA : ");      Serial.println(featureWindow[windowIndex].toa);
    Serial.print("CFO : ");      Serial.println(featureWindow[windowIndex].cfo);
    Serial.println("--------------------------------");

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
    Serial.print("Mean ToA : ");      Serial.println(features.meanToA);
    Serial.print("Mean CFO : ");      Serial.println(features.CFO);
    Serial.print("PLR : ");           Serial.println(features.PLR);
    Serial.print("CRC Failures : ");  Serial.println(features.consecutiveCRCFailures);
    Serial.print("Current SF : ");    Serial.println(features.currentSF);
    Serial.print("Current CR : ");    Serial.println(features.currentCR);
    Serial.println("====================================");

    // ---- Machine-readable line for the PC-side logger ----
    // Order MUST match HEADER in getcsv.py:
    // meanRSSI,varRSSI,meanSNR,varSNR,CFO,PLR,CRC,SF,CR,meanToA
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
    Serial.println(features.meanToA, 4);

    // ---- Inference (replace with TinyML / Random Forest later) ----
    int prediction = runInference();
    Serial.print("Prediction = ");
    Serial.println(prediction);
    Serial.println();

    executeDecision(prediction);
}
//======================================================================
// FEATURE EXTRACTION
//======================================================================
void extractFeatures()
{
    // Use the ACTUAL packet count, not the fixed window size --
    // a timed-out window (heavy jamming) may close with fewer than
    // FEATURE_WINDOW_SIZE packets, or even zero.
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
            features.meanToA  = 0.0f;   // ToA not meaningful for foreign captures
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
            features.meanToA  = 0.0f;
            features.PLR      = 1.0f;
        }
        else
        {
        features.meanRSSI = -999.0f;
        features.varRSSI  = 0.0f;
        features.meanSNR  = -999.0f;
        features.varSNR   = 0.0f;
        features.CFO      = 0.0f;
        features.meanToA  = 0.0f;
        features.PLR      = 1.0f;   // 100% loss
        }
        features.consecutiveCRCFailures = consecutiveCRCFailures;
        features.currentSF = currentSF;
        features.currentCR = currentCR;
        return;
    }

    float sumRSSI = 0, sumSNR = 0, sumToA = 0, sumCFO = 0;

    for (int i = 0; i < n; i++)
    {
        sumRSSI += featureWindow[i].rssi;
        sumSNR  += featureWindow[i].snr;
        sumToA  += featureWindow[i].toa;
        sumCFO  += featureWindow[i].cfo;
    }

    features.meanRSSI = sumRSSI / n;
    features.meanSNR  = sumSNR  / n;
    features.meanToA  = sumToA  / n;
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

//======================================================================
// RUN AI INFERENCE  (placeholder heuristic)
//
// Output class:
//   0 = Normal
//   1 = Jammer
//   2 = Weak Link
//   3 = Excellent Link
//
// Replace with Random Forest / TinyML / TensorFlow Lite / Edge Impulse.
//======================================================================
int runInference()
{
    // // Jammer: sustained CRC failures, or low SNR combined with high loss.
    // if (features.consecutiveCRCFailures >= 3 ||
    //     (features.meanSNR < -7.0f && features.PLR > 0.30f))
    //     return 1;

    // // Weak link: low RSSI or negative SNR without heavy loss.
    // if (features.meanRSSI < -100.0f || features.meanSNR < 0.0f)
    //     return 2;

    // // Excellent link: strong RSSI and high SNR.
    // if (features.meanRSSI > -60.0f && features.meanSNR > 8.0f)
    //     return 3;

    return 0;
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
// SEND CONTROL PACKET (with authentication + retries)
//======================================================================
void sendControlPacket(uint8_t commandType, uint32_t frequencyKHz,
                       uint8_t sf, uint8_t cr)
{
    ControlPacket packet;
    packet.magic           = PROTO_MAGIC;
    packet.commandID       = ++commandCounter;
    packet.command         = commandType;
    packet.newFrequencyKHz = frequencyKHz;
    packet.newSF           = sf;
    packet.newCR           = cr;
    packet.checksum        = controlChecksum(packet);

    const uint8_t MAX_ATTEMPTS = 4;

    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)
    {
        Serial.print("[CTRL] Sending command (attempt ");
        Serial.print(attempt); Serial.print("/");
        Serial.print(MAX_ATTEMPTS); Serial.println(")...");

        int state = controlRadio.transmit((uint8_t*)&packet, sizeof(packet));
        if (state != RADIOLIB_ERR_NONE)
        {
            Serial.print("Control TX Failed : ");
            Serial.println(state);
            continue;                 // retry
        }

        if (waitForControlAck(packet.commandID))
        {
            Serial.println("ACK Received");

            // Only NOW commit the receiver to the new configuration,
            // keeping it in step with the (already acknowledged) TX.
            currentFrequencyKHz = frequencyKHz;
            currentSF           = sf;
            currentCR           = cr;

            applyReceiverConfiguration(currentFrequencyKHz,
                                       currentSF, currentCR);
            return;
        }

        Serial.println("ACK Timeout");
    }

    Serial.println("[CTRL] Command failed after retries (config unchanged)");
    // Re-arm the data plane; configuration stays as-is so TX and RX
    // remain on the same (old) channel if the command never landed.
    dataRadio.startReceive();
}

//======================================================================
// WAIT FOR CONTROL ACK (DIO0 polled + authenticated)
//======================================================================
bool waitForControlAck(uint32_t expectedID)
{
    controlRadio.startReceive();
    uint32_t startTime = millis();

    while (millis() - startTime < 1500)
    {
        if (digitalRead(DIO0_CTRL) == HIGH)
        {
            ControlAck ack;
            int state = controlRadio.readData((uint8_t*)&ack, sizeof(ack));

            if (state == RADIOLIB_ERR_NONE &&
                ack.magic == PROTO_MAGIC &&
                ack.checksum == ackChecksum(ack) &&
                ack.commandID == expectedID)
            {
                return (ack.applied != 0);
            }

            controlRadio.startReceive();   // bad/foreign packet -> keep waiting
        }
    }
    return false;
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


//======================================================================
// EXECUTE AI DECISION
//======================================================================
void executeDecision(int prediction) {
  switch (prediction) {
    case 0:
      Serial.println("\n[AI] Normal Channel");
      break;
    case 1:
    {
      Serial.println("\n[AI] Jammer Detected\nChannel Hopping...");
      // 1. Calculate the next targeted frequency (+200 kHz)
      float nextFrequency = currentFrequencyKHz + 200.0f;

      // 2. Check if the hop pushes us past the highest channel (434600 kHz)
      if (nextFrequency > 434600.0f) {
        Serial.println("[System] Frequency upper limit reached. Wrapping back to Channel 0.");
        nextFrequency = 433000.0f;  // Reset to the base channel
      }
      // 3. Send the command using the freshly calculated frequency
      sendControlPacket(CMD_CHANNEL_HOP, (uint32_t)nextFrequency, currentSF, currentCR);
      break;
    }
    case 2:
      {  // Added curly braces to allow local variable declarations safely
        Serial.println("\n[AI] Weak Link\nIncreasing Robustness");
        // If it's below 10, force start at 10.
        // If it's 10 or 11, step up.
        // If it's 12 or higher, cap at 12.
        int sf = (currentSF < 10) ? 10 : ((currentSF < 12) ? currentSF + 1 : 12);

        // Same logic for CR: below 6 -> 6. Between 6 and 7 -> step up. 8+ -> cap at 8.
        int cr = (currentCR < 6) ? 6 : ((currentCR < 8) ? currentCR + 1 : 8);

        sendControlPacket(CMD_LINK_ADAPT, currentFrequencyKHz, sf, cr);
        break;
      }

    case 3:
      {  // Added curly braces
        Serial.println("\n[AI] Excellent Link\nOptimizing Throughput");

        // If it's above 12, force start at 12.
        // If it's between 9 and 12, step down.
        // If it's 8 or lower, floor at 8.
        int sf = (currentSF > 10) ? 10 : ((currentSF > 8) ? currentSF - 1 : 8);

        // For CR: if above 8, force 8. Between 6 and 8 -> step down. 5 or lower -> floor at 5.
        int cr = (currentCR >= 8) ? 7 : ((currentCR > 5) ? currentCR - 1 : 5);

        // FIXED: Passing variables now instead of hardcoded 7, 5
        sendControlPacket(CMD_LINK_ADAPT, currentFrequencyKHz, sf, cr);
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
}
