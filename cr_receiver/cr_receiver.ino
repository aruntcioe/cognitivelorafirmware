/***********************************************************************
 *  AI-Assisted Cognitive Radio LPWAN
 *  RECEIVER NODE
 *
 * --------------------------------------------------------------------
 *  DATA PLANE
 *      Frequency : 433 MHz
 *      Purpose   : Receive continuous application data
 *
 * --------------------------------------------------------------------
 *  CONTROL PLANE
 *      Frequency : 445 MHz
 *      Purpose   :
 *          - Send AI adaptation commands
 *          - Receive Control ACK from transmitter
 *
 * --------------------------------------------------------------------
 *  Receiver Responsibilities
 *
 *  1. Receive DATA packets
 *  2. Extract radio metrics
 *  3. Store metrics in feature window
 *  4. Calculate window statistics
 *  5. Run AI classifier
 *  6. Send adaptation command
 *
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

// Must always match transmitter

float currentFrequency = 433000;

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

SX1278 dataRadio = new Module(
    NSS_DATA,
    DIO0_DATA,
    RST_DATA,
    RADIOLIB_NC,
    sharedSPI
);

SX1278 controlRadio = new Module(
    NSS_CTRL,
    DIO0_CTRL,
    RST_CTRL,
    RADIOLIB_NC,
    sharedSPI
);



//======================================================================
// COMMAND TYPES
//======================================================================

enum CommandType
{
    CMD_NONE = 0,

    CMD_CHANNEL_HOP = 1,

    CMD_LINK_ADAPT = 2
};



//======================================================================
// DATA PACKET
//
// Must be IDENTICAL to transmitter
//======================================================================

struct DataPacket
{
    uint32_t sequence;

    uint32_t timestamp;

    uint16_t sensorValue;
};



//======================================================================
// CONTROL PACKET
//
// Sent TO transmitter
//======================================================================

struct ControlPacket
{
    uint32_t commandID;

    uint8_t command;

    uint8_t newFrequency;

    uint8_t newSF;

    uint8_t newCR;
};



//======================================================================
// CONTROL ACK
//
// Returned BY transmitter
//======================================================================

struct ControlAck
{
    uint32_t commandID;

    bool applied;
};



//======================================================================
// RAW FEATURE
//
// One entry per received packet
//======================================================================

struct PacketFeature
{
    uint32_t sequence;

    uint32_t timestamp;

    float rssi;

    float snr;

    float cfo;

    float toa;

    bool crcOK;
};



//======================================================================
// WINDOW BUFFER
//======================================================================

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
//
// This will become ML input
//======================================================================

struct FeatureVector
{
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

//======================================================================
// FUNCTION DECLARATIONS
//======================================================================

bool initializeDataRadio();

bool initializeControlRadio();

void receiveDataPacket();

void processFeatureWindow();

void extractFeatures();

int runInference();

void sendControlPacket(
        uint8_t command,
        unit8_t frequency,
        uint8_t sf,
        uint8_t cr
);

bool waitForControlAck();

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



    sharedSPI.begin(
        SPI_SCK,
        SPI_MISO,
        SPI_MOSI,
        -1
    );



    if(!initializeDataRadio())
    {
        Serial.println("DATA RADIO FAILED");

        while(true);
    }



    if(!initializeControlRadio())
    {
        Serial.println("CONTROL RADIO FAILED");

        while(true);
    }



    dataRadio.startReceive();

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
    float freqMHz = currentFrequency / 1000.0f

    int state = dataRadio.begin(
                    freqMHz,
                    125.0,
                    currentSF,
                    currentCR,
                    RADIOLIB_SX127X_SYNC_WORD,
                    10,
                    8
                );

    if(state != RADIOLIB_ERR_NONE)
    {
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
                    8
                );

    if(state != RADIOLIB_ERR_NONE)
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
// CALCULATE TIME ON AIR
//
// This uses the CURRENT radio configuration.
//
// Later if SF/CR changes,
// this automatically changes too.
//======================================================================

float calculateToA()
{
    size_t payloadLength = sizeof(DataPacket);

    float toa =
        dataRadio.getTimeOnAir(payloadLength);

    return toa;
}



//======================================================================
// READ FREQUENCY ERROR
//
// SX1278 stores Frequency Error in:
//
// RegFeiMsb
// RegFeiMid
// RegFeiLsb
//
// NOTE:
//
// RadioLib currently does not expose a public API
// for SX1278 Frequency Error.
//
// For Version 1 we return 0.
//
// In Part 5 we will replace this with direct
// register access using SPI.
//======================================================================

float readFrequencyError()
{
    return 0.0;
}


//======================================================================
// RECEIVE DATA PACKET
//======================================================================

void receiveDataPacket()
{

    //----------------------------------------------------------
    // No packet?
    //----------------------------------------------------------

    if(!dataRadio.isPacketReceived())
        return;



    //----------------------------------------------------------
    // Read received packet
    //----------------------------------------------------------

    DataPacket packet;

    int state = dataRadio.readData(
                    (uint8_t*)&packet,
                    sizeof(packet)
                );



    //----------------------------------------------------------
    // CRC Error
    //----------------------------------------------------------

    if(state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        Serial.println("[DATA] CRC Failure");

        crcFailuresWindow++;

        consecutiveCRCFailures++;

        dataRadio.startReceive();

        return;
    }



    //----------------------------------------------------------
    // Other receive error
    //----------------------------------------------------------

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[DATA] RX Error : ");

        Serial.println(state);

        dataRadio.startReceive();

        return;
    }



    //----------------------------------------------------------
    // Successful packet
    //----------------------------------------------------------

    consecutiveCRCFailures = 0;

    digitalWrite(LED_ACTIVITY, HIGH);



    //----------------------------------------------------------
    // Packet Loss Calculation
    //----------------------------------------------------------

    if(firstPacket)
    {
        previousSequence = packet.sequence;

        firstPacket = false;
    }
    else
    {

        if(packet.sequence > previousSequence + 1)
        {
            lostPacketsWindow +=
                (packet.sequence - previousSequence - 1);
        }

        previousSequence = packet.sequence;
    }



    //----------------------------------------------------------
    // Store Features
    //----------------------------------------------------------

    featureWindow[windowIndex].sequence = packet.sequence;

    featureWindow[windowIndex].timestamp = packet.timestamp;

    featureWindow[windowIndex].rssi =
        dataRadio.getRSSI();

    featureWindow[windowIndex].snr =
        dataRadio.getSNR();

    featureWindow[windowIndex].cfo =
        readFrequencyError();

    featureWindow[windowIndex].toa =
        calculateToA();

    featureWindow[windowIndex].crcOK = true;



    //----------------------------------------------------------
    // Debug Output
    //----------------------------------------------------------

    Serial.println("--------------------------------");

    Serial.print("Sequence : ");

    Serial.println(packet.sequence);

    Serial.print("RSSI : ");

    Serial.println(featureWindow[windowIndex].rssi);

    Serial.print("SNR : ");

    Serial.println(featureWindow[windowIndex].snr);

    Serial.print("Sensor : ");

    Serial.println(packet.sensorValue);

    Serial.println("--------------------------------");



    //----------------------------------------------------------
    // Advance Window
    //----------------------------------------------------------

    windowIndex++;

    packetsInWindow++;



    //----------------------------------------------------------
    // Window Complete?
    //----------------------------------------------------------

    if(windowIndex >= FEATURE_WINDOW_SIZE)
    {
        processFeatureWindow();

        windowIndex = 0;

        packetsInWindow = 0;

        lostPacketsWindow = 0;

        crcFailuresWindow = 0;
    }



    digitalWrite(LED_ACTIVITY, LOW);



    //----------------------------------------------------------
    // Return receiver back into RX mode
    //----------------------------------------------------------

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



    //----------------------------------------------------------
    // TinyML / Random Forest
    //----------------------------------------------------------

   Serial.println("Feature Vector Ready");

    Serial.print("Prediction = ");

    Serial.println(prediction);

    Serial.println();
}






//======================================================================
// FEATURE EXTRACTION
//======================================================================

void extractFeatures()
{

    float sumRSSI = 0;

    float sumSNR = 0;

    float sumToA = 0;

    float sumCFO = 0;



    //----------------------------------------------------------
    // Calculate Means
    //----------------------------------------------------------

    for(int i=0;i<FEATURE_WINDOW_SIZE;i++)
    {
        sumRSSI += featureWindow[i].rssi;

        sumSNR += featureWindow[i].snr;

        sumToA += featureWindow[i].toa;

        sumCFO += featureWindow[i].cfo;
    }



    features.meanRSSI =
        sumRSSI / FEATURE_WINDOW_SIZE;

    features.meanSNR =
        sumSNR / FEATURE_WINDOW_SIZE;

    features.meanToA =
        sumToA / FEATURE_WINDOW_SIZE;

    features.CFO =
        sumCFO / FEATURE_WINDOW_SIZE;



    //----------------------------------------------------------
    // Calculate Variance
    //----------------------------------------------------------

    float rssiVariance = 0;

    float snrVariance = 0;

    for(int i=0;i<FEATURE_WINDOW_SIZE;i++)
    {

        float rssiDifference =
            featureWindow[i].rssi -
            features.meanRSSI;

        rssiVariance +=
            rssiDifference *
            rssiDifference;



        float snrDifference =
            featureWindow[i].snr -
            features.meanSNR;

        snrVariance +=
            snrDifference *
            snrDifference;

    }



    features.varRSSI =
        rssiVariance / FEATURE_WINDOW_SIZE;

    features.varSNR =
        snrVariance / FEATURE_WINDOW_SIZE;



    //----------------------------------------------------------
    // Packet Loss Rate
    //----------------------------------------------------------

    features.PLR =
        (float)lostPacketsWindow /
        (FEATURE_WINDOW_SIZE + lostPacketsWindow);



    //----------------------------------------------------------
    // CRC
    //----------------------------------------------------------

    features.consecutiveCRCFailures =
        consecutiveCRCFailures;



    //----------------------------------------------------------
    // Current Radio State
    //----------------------------------------------------------

    features.currentSF = currentSF;

    features.currentCR = currentCR;

}
//======================================================================
// RUN AI INFERENCE
//
// Replace this function later with:
//
//  Random Forest
//  TinyML
//  TensorFlow Lite
//  Edge Impulse
//
// Input:
//      FeatureVector features;
//
// Output:
//      Class ID
//
//          0 = Normal
//          1 = Jammer
//          2 = Weak Link
//          3 = Excellent Link
//======================================================================

int runInference()
{
    //----------------------------------------------------------
    // Temporary decision logic
    //----------------------------------------------------------

    if(features.meanSNR < -8)
        return 1;

    if(features.meanRSSI < -105)
        return 2;

    return 0;
}

void applyReceiverConfiguration(
        uint8_t frequency,
        uint8_t sf,
        uint8_t cr)
{
  float freqMHz = command.frequencyKHz / 1000.0f
    dataRadio.setFrequency(freqMHz);

    dataRadio.setSpreadingFactor(sf);

    dataRadio.setCodingRate(cr);

    dataRadio.startReceive();

    Serial.println("Receiver Reconfigured");
}

//======================================================================
// SEND CONTROL PACKET
//======================================================================

void sendControlPacket(
        uint8_t command,
        uint8_t frequency,
        uint8_t sf,
        uint8_t cr)
{

    ControlPacket packet;

    packet.commandID = ++commandCounter;

    packet.command = command;

    packet.newFrequency = frequency;

    packet.newSF = sf;

    packet.newCR = cr;



    Serial.println();
    Serial.println("Sending Control Command...");



    int state =
        controlRadio.transmit(
            (uint8_t*)&packet,
            sizeof(packet)
        );



    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("Control TX Failed : ");

        Serial.println(state);

        return;
    }



    Serial.println("Waiting for ACK...");



    if(waitForControlAck())
    {
        Serial.println("ACK Received");

        currentFrequency = frequency;

        currentSF = sf;

        currentCR = cr;

        applyReceiverConfiguration(
        currentFrequency,
        currentSF,
        currentCR
    );
    }
    else
    {
        Serial.println("ACK Timeout");
    }
}



//======================================================================
// WAIT FOR CONTROL ACK
//======================================================================

bool waitForControlAck()
{

    controlRadio.startReceive();

    uint32_t startTime = millis();

    while(millis() - startTime < 1500)
    {

        if(controlRadio.isPacketReceived())
        {

            ControlAck ack;

            int state =
                controlRadio.readData(
                    (uint8_t*)&ack,
                    sizeof(ack)
                );

            if(state == RADIOLIB_ERR_NONE)
            {

                if(ack.commandID == commandCounter)
                {
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
//
// Maps ML class -> Radio Adaptation Policy
//======================================================================

void executeDecision(int prediction)
{

    switch(prediction)
    {

        //------------------------------------------------------
        // CLASS 0
        // Normal Link
        //------------------------------------------------------

        case 0:

            Serial.println();
            Serial.println("[AI] Normal Channel");

            // No action required

            break;



        //------------------------------------------------------
        // CLASS 1
        // Jammer Detected
        //------------------------------------------------------

        case 1:

            Serial.println();
            Serial.println("[AI] Jammer Detected");

            Serial.println("Channel Hopping...");

            sendControlPacket(

                CMD_CHANNEL_HOP,

                434000,          // Example hop frequency

                currentSF,

                currentCR

            );

            break;



        //------------------------------------------------------
        // CLASS 2
        // Weak Link
        //------------------------------------------------------

        case 2:

            Serial.println();
            Serial.println("[AI] Weak Link");

            Serial.println("Increasing Robustness");

            sendControlPacket(

                CMD_LINK_ADAPT,

                currentFrequency,

                10,             // Increase SF

                8               // Increase Coding Rate

            );

            break;



        //------------------------------------------------------
        // CLASS 3
        // Excellent Link
        //------------------------------------------------------

        case 3:

            Serial.println();
            Serial.println("[AI] Excellent Link");

            Serial.println("Optimizing Throughput");

            sendControlPacket(

                CMD_LINK_ADAPT,

                currentFrequency,

                7,

                5

            );

            break;

       //------------------------------------------------------
        // Unknown
        //------------------------------------------------------

        default:

            Serial.println();

            Serial.println("Unknown AI Prediction");

            break;

    }

}




//======================================================================
// LOOP
//======================================================================

void loop()
{

    //----------------------------------------------------------
    // Always listen for DATA packets
    //----------------------------------------------------------

    receiveDataPacket();



    //----------------------------------------------------------
    // If a complete feature window is available,
    // run AI and execute policy.
    //----------------------------------------------------------

    if(windowIndex == 0 && packetsInWindow == 0)
    {

        static bool processed = false;

        if(!processed)
        {

            processed = true;

            int prediction = runInference();
            executeDecision(prediction);

        }

    }
    else
    {
        processed = false;
    }

}