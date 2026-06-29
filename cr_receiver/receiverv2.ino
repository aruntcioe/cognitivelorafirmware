/***********************************************************************
 * AI-Assisted Cognitive Radio LPWAN
 * RECEIVER NODE (Fixed & Enhanced)
 ***********************************************************************/

#include <RadioLib.h>
#include <SPI.h>

//======================================================================
// SHARED SPI BUS
//======================================================================
#define SPI_SCK      12
#define SPI_MISO     11
#define SPI_MOSI     13

//======================================================================
// DATA RADIO (433 MHz)
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
#define FEATURE_WINDOW_SIZE 5

//======================================================================
// RADIO CONFIGURATION
//======================================================================
float currentFrequency = 433000; // Stored in kHz
uint8_t currentSF = 8;
uint8_t currentCR = 5;

//======================================================================
// CONTROL CHANNEL CONFIGURATION
//======================================================================
const float CONTROL_FREQUENCY = 445.0;
const uint8_t CONTROL_SF = 12;
const uint8_t CONTROL_CR = 8;

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
bool windowReady = false; // Explicit flag for processing

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
void sendControlPacket(uint8_t command, uint32_t frequency, uint8_t sf, uint8_t cr);
bool waitForControlAck();
float calculateToA();
float readFrequencyError();
void applyReceiverConfiguration(uint32_t frequency, uint8_t sf, uint8_t cr);
void executeDecision(int prediction);

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

    if(!initializeDataRadio()) {
        Serial.println("DATA RADIO FAILED");
        while(true);
    }

    if(!initializeControlRadio()) {
        Serial.println("CONTROL RADIO FAILED");
        while(true);
    }

    dataRadio.startReceive();
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
                    8
                );

    if(state != RADIOLIB_ERR_NONE) {
        Serial.print("Initialization Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("DATA Radio Ready");
    Serial.print("Frequency : "); Serial.println(currentFrequency);
    Serial.print("SF : "); Serial.println(currentSF);
    Serial.print("CR : 4/"); Serial.println(currentCR);
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
                    8
                );

    if(state != RADIOLIB_ERR_NONE) {
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
    return 0.0; // Place holder for direct register access in Part 5
}

//======================================================================
// RECEIVE DATA PACKET
//======================================================================
void receiveDataPacket() {
    if(!dataRadio.isPacketReceived())
        return;

    DataPacket packet;
    int state = dataRadio.readData((uint8_t*)&packet, sizeof(packet));

    if(state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("[DATA] CRC Failure");
        crcFailuresWindow++;
        consecutiveCRCFailures++;
        dataRadio.startReceive();
        return;
    }

    if(state != RADIOLIB_ERR_NONE) {
        Serial.print("[DATA] RX Error : ");
        Serial.println(state);
        dataRadio.startReceive();
        return;
    }

    consecutiveCRCFailures = 0;
    digitalWrite(LED_ACTIVITY, HIGH);

    if(firstPacket) {
        previousSequence = packet.sequence;
        firstPacket = false;
    } else {
        if(packet.sequence > previousSequence + 1) {
            lostPacketsWindow += (packet.sequence - previousSequence - 1);
        }
        previousSequence = packet.sequence;
    }

    featureWindow[windowIndex].sequence = packet.sequence;
    featureWindow[windowIndex].timestamp = packet.timestamp;
    featureWindow[windowIndex].rssi = dataRadio.getRSSI();
    featureWindow[windowIndex].snr = dataRadio.getSNR();
    featureWindow[windowIndex].cfo = readFrequencyError();
    featureWindow[windowIndex].toa = calculateToA();
    featureWindow[windowIndex].crcOK = true;

    Serial.println("--------------------------------");
    Serial.print("Sequence : "); Serial.println(packet.sequence);
    Serial.print("RSSI : ");     Serial.println(featureWindow[windowIndex].rssi);
    Serial.print("SNR : ");      Serial.println(featureWindow[windowIndex].snr);
    Serial.print("Sensor : ");   Serial.println(packet.sensorValue);
    Serial.println("--------------------------------");

    windowIndex++;
    packetsInWindow++;

    if(windowIndex >= FEATURE_WINDOW_SIZE) {
        processFeatureWindow();
        windowIndex = 0;
        packetsInWindow = 0;
        lostPacketsWindow = 0;
        crcFailuresWindow = 0;
        windowReady = true; // Signal that loop() can now safely evaluate ML
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
}

//======================================================================
// FEATURE EXTRACTION
//======================================================================
void extractFeatures() {
    float sumRSSI = 0, sumSNR = 0, sumToA = 0, sumCFO = 0;

    for(int i = 0; i < FEATURE_WINDOW_SIZE; i++) {
        sumRSSI += featureWindow[i].rssi;
        sumSNR  += featureWindow[i].snr;
        sumToA  += featureWindow[i].toa;
        sumCFO  += featureWindow[i].cfo;
    }

    features.meanRSSI = sumRSSI / FEATURE_WINDOW_SIZE;
    features.meanSNR  = sumSNR / FEATURE_WINDOW_SIZE;
    features.meanToA  = sumToA / FEATURE_WINDOW_SIZE;
    features.CFO      = sumCFO / FEATURE_WINDOW_SIZE;

    float rssiVariance = 0, snrVariance = 0;
    for(int i = 0; i < FEATURE_WINDOW_SIZE; i++) {
        float rssiDifference = featureWindow[i].rssi - features.meanRSSI;
        rssiVariance += rssiDifference * rssiDifference;

        float snrDifference = featureWindow[i].snr - features.meanSNR;
        snrVariance += snrDifference * snrDifference;
    }

    features.varRSSI = rssiVariance / FEATURE_WINDOW_SIZE;
    features.varSNR  = snrVariance / FEATURE_WINDOW_SIZE;
    features.PLR     = (float)lostPacketsWindow / (FEATURE_WINDOW_SIZE + lostPacketsWindow);
    features.consecutiveCRCFailures = consecutiveCRCFailures;
    features.currentSF = currentSF;
    features.currentCR = currentCR;
}

//======================================================================
// DATA LOGGING LAYER (CSV Generator)
//======================================================================
void logFeatureVectorCSV(String label) {
    // Prints a single clean line of comma-separated values to the terminal
    Serial.print(features.meanRSSI, 1); Serial.print(",");
    Serial.print(features.varRSSI, 1);  Serial.print(",");
    Serial.print(features.meanSNR, 1);   Serial.print(",");
    Serial.print(features.varSNR, 1);   Serial.print(",");
    Serial.print(features.CFO, 1);      Serial.print(",");
    Serial.print(features.PLR, 3);      Serial.print(",");
    Serial.print(features.consecutiveCRCFailures); Serial.print(",");
    Serial.print(features.currentSF);   Serial.print(",");
    Serial.print(features.currentCR);   Serial.print(",");
    Serial.print(features.meanToA, 1);  Serial.print(",");
    Serial.println(label); 
}

//======================================================================
// RUN AI INFERENCE
//======================================================================
int runInference() {
    if(features.meanSNR < -8)    return 1; // Jammer
    if(features.meanRSSI < -105) return 2; // Weak Link
    return 0;                              // Normal
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
// SEND CONTROL PACKET
//======================================================================
void sendControlPacket(uint8_t command, uint32_t frequency, uint8_t sf, uint8_t cr) {
    ControlPacket packet;
    packet.commandID = ++commandCounter;
    packet.command = command;
    packet.newFrequency = frequency;
    packet.newSF = sf;
    packet.newCR = cr;

    Serial.println("\nSending Control Command...");
    int state = controlRadio.transmit((uint8_t*)&packet, sizeof(packet));
    
    if(state != RADIOLIB_ERR_NONE) {
        Serial.print("Control TX Failed : ");
        Serial.println(state);
        return;
    }

    Serial.println("Waiting for ACK...");

    if(waitForControlAck()) {
        Serial.println("ACK Received");
        currentFrequency = frequency;
        currentSF = sf;
        currentCR = cr;
        applyReceiverConfiguration(currentFrequency, currentSF, currentCR);
    } else {
        Serial.println("ACK Timeout");
    }
}

//======================================================================
// WAIT FOR CONTROL ACK
//======================================================================
bool waitForControlAck() {
    controlRadio.startReceive();
    uint32_t startTime = millis();
    while(millis() - startTime < 1500) {
        if(controlRadio.isPacketReceived()) {
            ControlAck ack;
            int state = controlRadio.readData((uint8_t*)&ack, sizeof(ack));
            if(state == RADIOLIB_ERR_NONE) {
                if(ack.commandID == commandCounter) {
                    return ack.applied;
                }
            }
            controlRadio.startReceive();
        }
    }
    return false;
}

//======================================================================
// EXECUTE AI DECISION
//======================================================================
void executeDecision(int prediction) {
    switch(prediction) {
        case 0:
            Serial.println("\n[AI] Normal Channel");
            break;
        case 1:
            Serial.println("\n[AI] Jammer Detected\nChannel Hopping...");
            sendControlPacket(CMD_CHANNEL_HOP, 434000, currentSF, currentCR);
            break;
        case 2:
            Serial.println("\n[AI] Weak Link\nIncreasing Robustness");
            sendControlPacket(CMD_LINK_ADAPT, currentFrequency, 10, 8);
            break;
        case 3:
            Serial.println("\n[AI] Excellent Link\nOptimizing Throughput");
            sendControlPacket(CMD_LINK_ADAPT, currentFrequency, 7, 5);
            break;
        default:
            Serial.println("\nUnknown AI Prediction");
            break;
    }
}

//======================================================================
// LOOP
//======================================================================
void loop() {
    // Always listen for DATA packets
    receiveDataPacket();

    // Triggered seamlessly when processFeatureWindow completes a block
    if(windowReady) {
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