/***********************************************************************
 * AI-Assisted Cognitive Radio LPWAN
 * RECEIVER NODE (Fixed & Enhanced)
 ***********************************************************************/

#include <RadioLib.h>
#include <SPI.h>

//======================================================================
// SHARED SPI BUS
//======================================================================
#define SPI_SCK 12
#define SPI_MISO 11
#define SPI_MOSI 13

//======================================================================
// DATA RADIO (433 MHz)
//======================================================================
#define NSS_DATA 10
#define RST_DATA 14
#define DIO0_DATA 16

//======================================================================
// CONTROL RADIO (445 MHz)
//======================================================================
#define NSS_CTRL 9
#define RST_CTRL 15
#define DIO0_CTRL 18

//======================================================================
// STATUS LEDs
//======================================================================
#define LED_ACTIVITY 17
#define LED_ERROR 4

//======================================================================
// FEATURE WINDOW SIZE
//======================================================================
#define FEATURE_WINDOW_SIZE 5
#define DATA_PACKET_MAGIC 0xDEADBEEF

//======================================================================
// RADIO HEALTH / FAULT-HALT STATE
//======================================================================
const uint8_t RADIO_FAULT_THRESHOLD = 5;  // consecutive failures before halt

uint8_t dataRadioFailStreak = 0;
uint8_t controlRadioFailStreak = 0;

uint32_t lastGoodRxSequence = 0;    // last data packet successfully received
uint32_t lastGoodAckCommandID = 0;  // last control command successfully ACKed

bool systemHalted = false;
//======================================================================
// RADIO CONFIGURATION
//======================================================================
float currentFrequency = 433000;  // Stored in kHz
uint8_t currentSF = 8;
uint8_t currentCR = 5;

//======================================================================
// CONTROL CHANNEL CONFIGURATION
//======================================================================
const float CONTROL_FREQUENCY = 445.0;
const uint8_t CONTROL_SF = 12;
const uint8_t CONTROL_CR = 8;

bool featureCollectionEnabled = true;

//======================================================================
// SHARED SPI BUS
//======================================================================
SPIClass sharedSPI(FSPI);

//======================================================================
// RADIO OBJECTS
//======================================================================
SX1278 dataRadio = new Module(NSS_DATA, DIO0_DATA, RST_DATA, RADIOLIB_NC, sharedSPI);
SX1278 controlRadio = new Module(NSS_CTRL, DIO0_CTRL, RST_CTRL, RADIOLIB_NC, sharedSPI);

//======================================================================
// COMMAND TYPES
//======================================================================
enum CommandType {
  CMD_NONE = 0,
  CMD_CHANNEL_HOP = 1,
  CMD_LINK_ADAPT = 2
};

//======================================================================
// DATA PACKET
//======================================================================
struct DataPacket {
  uint32_t magic;
  uint32_t sequence;
  uint32_t timestamp;
  uint16_t sensorValue;
};

//======================================================================
// CONTROL PACKET (Fixed frequency sizing to prevent truncation)
//======================================================================
struct ControlPacket {
  uint32_t commandID;
  uint8_t command;
  uint32_t newFrequency;
  uint8_t newSF;
  uint8_t newCR;
};

//======================================================================
// CONTROL ACK
//======================================================================
struct ControlAck {
  uint32_t commandID;
  bool applied;
};

//======================================================================
// RAW FEATURE
//======================================================================
struct PacketFeature {
  uint32_t sequence;
  uint32_t timestamp;
  float rssi;
  float snr;
  float cfo;
  float toa;
  bool crcOK;
};

PacketFeature featureWindow[FEATURE_WINDOW_SIZE];

//======================================================================
// WINDOW VARIABLES
//======================================================================
uint8_t windowIndex = 0;
uint32_t expectedSequence = 0;
uint16_t packetsReceived = 0;
uint16_t packetsLost = 0;
uint16_t crcFailures = 0;
uint32_t commandCounter = 0;

//======================================================================
// NON-BLOCKING CONTROL PLANE STATE
//======================================================================
enum ControlTxState { CTRL_IDLE,
                      CTRL_AWAITING_ACK };
ControlTxState controlTxState = CTRL_IDLE;

ControlPacket pendingControlPacket;
uint32_t controlSentTime = 0;
uint8_t controlRetryCount = 0;

const uint8_t CONTROL_MAX_RETRIES = 4;
const uint32_t CONTROL_ACK_TIMEOUT_MS = 1500;  // must cover SF12 command+ack air time

//======================================================================
// DIO0 INTERRUPT FLAGS (replaces digitalRead polling)
//======================================================================
volatile bool dataPacketFlag = false;
volatile bool ctrlPacketFlag = false;

//======================================================================
// FEATURE VECTOR
//======================================================================
struct FeatureVector {
  float meanRSSI;
  float varRSSI;
  float meanSNR;
  float varSNR;
  float CFO;
  float PLR;
  uint16_t consecutiveCRCFailures;
  uint8_t currentSF;
  uint8_t currentCR;
  float meanToA;
};

FeatureVector features;

//======================================================================
// WINDOW STATISTICS
//======================================================================
uint8_t packetsInWindow = 0;
uint16_t lostPacketsWindow = 0;
uint16_t crcFailuresWindow = 0;
uint16_t consecutiveCRCFailures = 0;
uint32_t previousSequence = 0;
bool firstPacket = true;
bool windowReady = false;  // Explicit flag for processing

//======================================================================
// FUNCTION DECLARATIONS
//======================================================================
bool initializeDataRadio();
bool initializeControlRadio();
void receiveDataPacket();
void processFeatureWindow();
void extractFeatures();
int runInference();
void logFeatureVectorCSV(String label);

float calculateToA();
float readFrequencyError();
void applyReceiverConfiguration(uint32_t frequency, uint8_t sf, uint8_t cr);
void executeDecision(int prediction);

void sendControlCommand(uint8_t command, uint32_t frequency, uint8_t sf, uint8_t cr);
void transmitPendingControlPacket();
void serviceControlPlane();

//======================================================================
// PACKET VALIDITY CHECK
//======================================================================
bool isPacketPlausible(const DataPacket& pkt) {
  return pkt.magic == DATA_PACKET_MAGIC;
}
void IRAM_ATTR onDataDio0() {
  dataPacketFlag = true;
}

void IRAM_ATTR onCtrlDio0() {
  ctrlPacketFlag = true;
}
//======================================================================
// SETUP
//======================================================================
void setup() {
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

  const int MAX_RETRIES = 5;
  int retryCount = 0;

  // --- INITIALIZE DATA RADIO (Max 5 Retries) ---
  while (!initializeDataRadio()) {
    retryCount++;
    digitalWrite(LED_ERROR, HIGH);  // Visual error warning
    Serial.print("DATA RADIO FAILED (Attempt ");
    Serial.print(retryCount);
    Serial.print("/");
    Serial.print(MAX_RETRIES);
    Serial.println(")");

    if (retryCount >= MAX_RETRIES) {
      Serial.println("\n[FATAL ERROR] Data Radio failed to initialize 5 times. Halting execution.");
      while (true)
        ;  // Lock up securely here
    }

    Serial.println("Retrying in 2 seconds...");
    delay(2000);
  }
  digitalWrite(LED_ERROR, LOW);  // Clear error LED on success
  retryCount = 0;                // Reset counter for the next radio

  // --- INITIALIZE CONTROL RADIO (Max 5 Retries) ---
  while (!initializeControlRadio()) {
    retryCount++;
    digitalWrite(LED_ERROR, HIGH);
    Serial.print("CONTROL RADIO FAILED (Attempt ");
    Serial.print(retryCount);
    Serial.print("/");
    Serial.print(MAX_RETRIES);
    Serial.println(")");

    if (retryCount >= MAX_RETRIES) {
      Serial.println("\n[FATAL ERROR] Control Radio failed to initialize 5 times. Halting execution.");
      while (true)
        ;
    }

    Serial.println("Retrying in 2 seconds...");
    delay(2000);
  }
  digitalWrite(LED_ERROR, LOW);

  // --- SET UP INTERRUPTS ---
  dataRadio.startReceive();
  dataRadio.setDio0Action(onDataDio0, RISING);

  controlRadio.startReceive();
  controlRadio.setDio0Action(onCtrlDio0, RISING);

  Serial.println("\nReceiver Ready\n");

  // Print CSV Header to easily capture into terminal files
  Serial.println("--- CSV DATASET OUTPUT START ---");
  Serial.println("meanRSSI,varRSSI,meanSNR,varSNR,CFO,PLR,CRC,SF,CR,meanToA,label");
}

//======================================================================
// INITIALIZE DATA RADIO
//======================================================================
bool initializeDataRadio() {
  Serial.println("--------------------------------------");
  Serial.println("Initializing DATA Radio...");
  Serial.println("--------------------------------------");
  float freqMHz = currentFrequency / 1000.0f;

  int state = dataRadio.begin(
    freqMHz,
    125.0,
    currentSF,
    currentCR,
    RADIOLIB_SX127X_SYNC_WORD,
    10,
    8);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Initialization Failed : ");
    Serial.println(state);
    return false;
  }

  Serial.println("DATA Radio Ready");
  Serial.print("Frequency : ");
  Serial.println(currentFrequency);
  Serial.print("SF : ");
  Serial.println(currentSF);
  Serial.print("CR : 4/");
  Serial.println(currentCR);
  Serial.println();
  return true;
}

//======================================================================
// INITIALIZE CONTROL RADIO
//======================================================================
bool initializeControlRadio() {
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

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Initialization Failed : ");
    Serial.println(state);
    return false;
  }

  Serial.println("CONTROL Radio Ready\n");
  return true;
}

float calculateToA() {
  size_t payloadLength = sizeof(DataPacket);
  return dataRadio.getTimeOnAir(payloadLength);
}

float readFrequencyError() {
  return dataRadio.getFrequencyError();
}

//======================================================================
// RECEIVE DATA PACKET
//======================================================================
void receiveDataPacket() {
  if (!dataPacketFlag) {
    return;
  }
  dataPacketFlag = false;

  DataPacket packet = {};
  int state = dataRadio.readData((uint8_t*)&packet, sizeof(packet));

  if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[DATA] CRC Failure");
    crcFailuresWindow++;
    consecutiveCRCFailures++;
    dataRadioFailStreak++;
    if (dataRadioFailStreak >= RADIO_FAULT_THRESHOLD) {
      haltSystem("DATA radio (RX)", lastGoodRxSequence);
      return;
    }
    dataRadio.startReceive();
    return;
  }

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[DATA] RX Error : ");
    Serial.println(state);
    dataRadioFailStreak++;
    if (dataRadioFailStreak >= RADIO_FAULT_THRESHOLD) {
      haltSystem("DATA radio (RX)", lastGoodRxSequence);
      return;
    }
    dataRadio.startReceive();
    return;
  }

  if (!isPacketPlausible(packet)) {
    Serial.print("[DATA] Implausible packet rejected - Seq=");
    Serial.print(packet.sequence);
    Serial.print(" Sensor=");
    Serial.println(packet.sensorValue);

    dataRadioFailStreak++;
    if (dataRadioFailStreak >= RADIO_FAULT_THRESHOLD) {
      haltSystem("DATA radio (RX)", lastGoodRxSequence);
      return;
    }
    dataRadio.startReceive();
    return;
  }

  consecutiveCRCFailures = 0;
  dataRadioFailStreak = 0;
  lastGoodRxSequence = packet.sequence;
  digitalWrite(LED_ACTIVITY, HIGH);

  if (firstPacket) {
    previousSequence = packet.sequence;
    firstPacket = false;
  } else {
    if (packet.sequence > previousSequence + 1) {
      lostPacketsWindow += (packet.sequence - previousSequence - 1);
    }
    previousSequence = packet.sequence;
  }

  if (featureCollectionEnabled) {
    featureWindow[windowIndex].sequence = packet.sequence;
    featureWindow[windowIndex].timestamp = packet.timestamp;
    featureWindow[windowIndex].rssi = dataRadio.getRSSI();
    featureWindow[windowIndex].snr = dataRadio.getSNR();
    featureWindow[windowIndex].cfo = readFrequencyError();
    featureWindow[windowIndex].toa = calculateToA();
    featureWindow[windowIndex].crcOK = true;


    Serial.println("--------------------------------");
    Serial.print("sequence Id: ");
    Serial.println(packet.sequence);
    Serial.print("rxTimestamp: ");
    Serial.println(millis());  // rxTimestampMs
    Serial.print("txtimestamp: ");
    Serial.println(packet.timestamp);  // txTimestampMs (from payload)
    Serial.print("RSSI: ");
    Serial.println(featureWindow[windowIndex].rssi, 2);
    Serial.print("SNR: ");
    Serial.println(featureWindow[windowIndex].snr, 2);
    Serial.print("CFO: ");
    Serial.println(featureWindow[windowIndex].cfo, 2);
    Serial.print("TOA: ");
    Serial.println(featureWindow[windowIndex].toa, 1);
    Serial.print("CRC: ");
    Serial.println(1);  // crcOK
    Serial.print("SF: ");
    Serial.println(currentSF);
    Serial.print("CR: ");
    Serial.println(currentCR);
    Serial.print("sensor: ");
    Serial.println(packet.sensorValue);
    Serial.println("--------------------------------");
    windowIndex++;
    packetsInWindow++;
  } else {
    Serial.println("--------------------------------");
    Serial.print("Feature collection is halted: ");
    Serial.print("sequence Id: ");
    Serial.println(packet.sequence);
    Serial.print("rxTimestamp: ");
    Serial.println(millis());  // rxTimestampMs
    Serial.print("txtimestamp: ");
    Serial.println(packet.timestamp);
    Serial.print("sensor: ");
    Serial.println(packet.sensorValue);
    Serial.println("--------------------------------");
  }

  if (windowIndex >= FEATURE_WINDOW_SIZE) {
    processFeatureWindow();
    windowIndex = 0;
    packetsInWindow = 0;
    lostPacketsWindow = 0;
    crcFailuresWindow = 0;
    windowReady = true;  // Signal that loop() can now safely evaluate ML
  }

  digitalWrite(LED_ACTIVITY, LOW);
  dataRadio.startReceive();
}

//======================================================================
// PROCESS FEATURE WINDOW
//======================================================================
void processFeatureWindow() {
  Serial.println("\n=======================================");
  Serial.println(" Feature Window Complete");
  Serial.println("=======================================");

  extractFeatures();

  Serial.println("\n========== FEATURE VECTOR ==========");
  Serial.print("Mean RSSI : ");
  Serial.println(features.meanRSSI);
  Serial.print("Variance RSSI : ");
  Serial.println(features.varRSSI);
  Serial.print("Mean SNR : ");
  Serial.println(features.meanSNR);
  Serial.print("Variance SNR : ");
  Serial.println(features.varSNR);
  Serial.print("Mean ToA : ");
  Serial.println(features.meanToA);
  Serial.print("Mean CFO : ");
  Serial.println(features.CFO);
  Serial.print("PLR : ");
  Serial.println(features.PLR);
  Serial.print("CRC Failures : ");
  Serial.println(features.consecutiveCRCFailures);
  Serial.print("Current SF : ");
  Serial.println(features.currentSF);
  Serial.print("Current CR : ");
  Serial.println(features.currentCR);
  Serial.println("====================================");
}

//======================================================================
// FEATURE EXTRACTION
//======================================================================
void extractFeatures() {
  float sumRSSI = 0, sumSNR = 0, sumToA = 0, sumCFO = 0;

  for (int i = 0; i < FEATURE_WINDOW_SIZE; i++) {
    sumRSSI += featureWindow[i].rssi;
    sumSNR += featureWindow[i].snr;
    sumToA += featureWindow[i].toa;
    sumCFO += featureWindow[i].cfo;
  }

  features.meanRSSI = sumRSSI / FEATURE_WINDOW_SIZE;
  features.meanSNR = sumSNR / FEATURE_WINDOW_SIZE;
  features.meanToA = sumToA / FEATURE_WINDOW_SIZE;
  features.CFO = sumCFO / FEATURE_WINDOW_SIZE;

  float rssiVariance = 0, snrVariance = 0;
  for (int i = 0; i < FEATURE_WINDOW_SIZE; i++) {
    float rssiDifference = featureWindow[i].rssi - features.meanRSSI;
    rssiVariance += rssiDifference * rssiDifference;

    float snrDifference = featureWindow[i].snr - features.meanSNR;
    snrVariance += snrDifference * snrDifference;
  }

  features.varRSSI = rssiVariance / FEATURE_WINDOW_SIZE;
  features.varSNR = snrVariance / FEATURE_WINDOW_SIZE;
  features.PLR = (float)lostPacketsWindow / (FEATURE_WINDOW_SIZE + lostPacketsWindow);
  features.consecutiveCRCFailures = consecutiveCRCFailures;
  features.currentSF = currentSF;
  features.currentCR = currentCR;
}

//======================================================================
// DATA LOGGING LAYER (CSV Generator)
//======================================================================
//======================================================================
// DATA LOGGING LAYER (CSV Generator) - streams over USB serial only
//======================================================================
void logFeatureVectorCSV(String label) {
  String row = String(features.meanRSSI, 1) + "," + String(features.varRSSI, 1) + "," + String(features.meanSNR, 1) + "," + String(features.varSNR, 1) + "," + String(features.CFO, 1) + "," + String(features.PLR, 3) + "," + String(features.consecutiveCRCFailures) + "," + String(features.currentSF) + "," + String(features.currentCR) + "," + String(features.meanToA, 1) + "," + label;

  Serial.print("CSVROW,");  // tag so the PC script can pick out data rows
  Serial.println(row);
}

//======================================================================
// RUN AI INFERENCE
//======================================================================
//======================================================================
// TEMPORARY THRESHOLD LABELER (NOT REAL AI)
//======================================================================
// This exists ONLY to auto-label the CSV during controlled data-collection
// runs so all 4 classes get exercised and captured on disk. It is a stand-in
// for the real classifier - swap this out once you're training on collected
// data. Thresholds below are placeholders; tune them against your actual
// captured RSSI/SNR/PLR distributions for the test environment you're in.
//======================================================================
//======================================================================
// CALIBRATED COGNITIVE RADIO THRESHOLDS (SX1278 LoRa)
//======================================================================
// Jamming: Low SNR combined with high/normal RSSI indicates active interference
const float THRESH_JAM_SNR = 10.0f;  // Below this (e.g. -12dB to -20dB), noise/interference is overriding the signal

// Fading: Signal attenuation due to distance or physical obstructions
const float THRESH_FADE_RSSI = -105.0f;  // Standard SX1278 sensitivity limit starts showing heavy drops here
const float THRESH_FADE_PLR = 0.25f;     // Packet Loss Rate > 25% indicates a dropping, unstable fading link

// Excellent Link: High-quality line-of-sight conditions allowing high throughput
const float THRESH_EXCELLENT_RSSI = -80.0f;  // Strong signal level (closer to 0 is stronger, e.g., -70dB is excellent)
const float THRESH_EXCELLENT_SNR = 8.0f;     // SX1278 SNR realistically saturates near +10dB; 8.0dB is pristine

int runInference() {
  // --- Class 1: Jammed / Primary User Active ---
  // Collapsed SNR relative to noise floor is the signature of jamming/
  // co-channel interference, independent of RSSI.

  // if (features.meanSNR < THRESH_JAM_SNR) {
  //     return 1;
  // }

  // // --- Class 2: Severe Link Fading / High Path Loss ---
  // // Either weak received power OR high packet loss (which weak power
  // // alone doesn't always capture, e.g. multipath fading) counts.
  // if (features.meanRSSI < THRESH_FADE_RSSI || features.PLR > THRESH_FADE_PLR) {
  //     return 2;
  // }

  // // --- Class 3: Exceptional / High Margin Link ---
  // // Strong signal AND clean SNR - safe to shrink SF/CR for throughput.
  // if (features.meanRSSI > THRESH_EXCELLENT_RSSI &&
  //     features.meanSNR  > THRESH_EXCELLENT_SNR) {
  //     return 3;
  // }

  // --- Class 0: Clear / Nominal ---
  // Doesn't fall into any of the above buckets - operate as-is.
  return 0;
}
//======================================================================
// HALT SYSTEM ON RADIO FAULT
//======================================================================
void haltSystem(const char* radioName, uint32_t lastKnownGoodValue) {
  if (systemHalted) return;  // already halted, don't spam
  systemHalted = true;

  digitalWrite(LED_ACTIVITY, LOW);
  digitalWrite(LED_ERROR, HIGH);  // steady ON, not blinking - distinguishes from transient error flash

  Serial.println();
  Serial.println("=====================================================");
  Serial.print("SYSTEM HALTED - ");
  Serial.print(radioName);
  Serial.println(" malfunction detected (connection/antenna/hardware).");
  Serial.print("Last known-good sequence/ID : ");
  Serial.println(lastKnownGoodValue);
  Serial.println("All TX/RX operations stopped. Power-cycle to recover.");
  Serial.println("=====================================================");
}

void applyReceiverConfiguration(uint32_t frequencyKHz, uint8_t sf, uint8_t cr) {
  float freqMHz = frequencyKHz / 1000.0f;
  dataRadio.setFrequency(freqMHz);
  dataRadio.setSpreadingFactor(sf);
  dataRadio.setCodingRate(cr);
  dataRadio.startReceive();
  Serial.println("Receiver Reconfigured");
}

//======================================================================
// TRANSMIT (OR RETRANSMIT) THE PENDING CONTROL PACKET
//======================================================================
void transmitPendingControlPacket() {
  Serial.print("\n[CTRL] Sending command (attempt ");
  Serial.print(controlRetryCount + 1);
  Serial.print("/");
  Serial.print(CONTROL_MAX_RETRIES);
  Serial.println(")...");

  int state = controlRadio.transmit((uint8_t*)&pendingControlPacket, sizeof(pendingControlPacket));

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[CTRL] TX Failed : ");
    Serial.println(state);
  }
  featureCollectionEnabled = false;
  ctrlPacketFlag = false;  // NEW: discard the self-triggered TxDone interrupt from this transmit
  controlRadio.startReceive();
  controlSentTime = millis();
}

//======================================================================
// KICK OFF A NEW CONTROL COMMAND (non-blocking)
//======================================================================
void sendControlCommand(uint8_t command, uint32_t frequency, uint8_t sf, uint8_t cr) {
  if (controlTxState != CTRL_IDLE) {
    Serial.println("[CTRL] Busy with previous command, skipping this decision cycle.");
    return;
  }

  pendingControlPacket.commandID = ++commandCounter;
  pendingControlPacket.command = command;
  pendingControlPacket.newFrequency = frequency;
  pendingControlPacket.newSF = sf;
  pendingControlPacket.newCR = cr;

  controlRetryCount = 0;
  controlTxState = CTRL_AWAITING_ACK;

  transmitPendingControlPacket();
}

//======================================================================
// SERVICE CONTROL PLANE - call every loop() iteration, non-blocking
//======================================================================
void serviceControlPlane() {
  if (controlTxState != CTRL_AWAITING_ACK) {
    return;
  }

  if (ctrlPacketFlag) {
    ctrlPacketFlag = false;
    ControlAck ack;
    int state = controlRadio.readData((uint8_t*)&ack, sizeof(ack));
    controlRadio.startReceive();

    if (state == RADIOLIB_ERR_NONE && ack.commandID == pendingControlPacket.commandID) {
      Serial.println("[CTRL] ACK Received");
      featureCollectionEnabled = true;
      controlRadioFailStreak = 0;
      lastGoodAckCommandID = ack.commandID;
      if (ack.applied) {
        currentFrequency = pendingControlPacket.newFrequency;
        currentSF = pendingControlPacket.newSF;
        currentCR = pendingControlPacket.newCR;
        applyReceiverConfiguration(currentFrequency, currentSF, currentCR);
      }

      controlTxState = CTRL_IDLE;
      return;
    }
    // Mismatched/malformed packet on control channel - ignore, keep waiting.
  }

  if (millis() - controlSentTime >= CONTROL_ACK_TIMEOUT_MS) {
    controlRetryCount++;

    if (controlRetryCount >= CONTROL_MAX_RETRIES) {
      Serial.println("[CTRL] ACK Timeout - giving up after max retries.");
      controlTxState = CTRL_IDLE;
      featureCollectionEnabled = true;
      controlRadioFailStreak++;
      if (controlRadioFailStreak >= RADIO_FAULT_THRESHOLD) {
        haltSystem("CONTROL radio (RX)", lastGoodAckCommandID);
      }
      return;
    }

    Serial.println("[CTRL] ACK Timeout - retrying...");
    transmitPendingControlPacket();
  }
}

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
      float nextFrequency = currentFrequency + 200.0f;

      // 2. Check if the hop pushes us past the highest channel (434600 kHz)
      if (nextFrequency > 434600.0f) {
        Serial.println("[System] Frequency upper limit reached. Wrapping back to Channel 0.");
        nextFrequency = 433000.0f;  // Reset to the base channel
      }
      // 3. Send the command using the freshly calculated frequency
      sendControlCommand(CMD_CHANNEL_HOP, (uint32_t)nextFrequency, currentSF, currentCR);
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

        sendControlCommand(CMD_LINK_ADAPT, currentFrequency, sf, cr);
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
        sendControlCommand(CMD_LINK_ADAPT, currentFrequency, sf, cr);
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
void loop() {
  if (systemHalted) {
    return;  // radios stopped permanently - power cycle to recover
  }
  // Always listen for DATA packets
  receiveDataPacket();
  serviceControlPlane();  // NEW - non-blocking ACK check/retry, every iteration

  // Triggered seamlessly when processFeatureWindow completes a block
  if (windowReady) {
    // Step 1: Calculate the temporary heuristic prediction to use as a dataset auto-label
    int heuristicPrediction = runInference();

    // Step 2: Log out the complete CSV structure directly to the stream
    logFeatureVectorCSV(String(heuristicPrediction));

    // Step 3: Act on policy decision
    executeDecision(heuristicPrediction);

    // Step 4: Clear down state for the next feature gathering block
    windowReady = false;
  }
}