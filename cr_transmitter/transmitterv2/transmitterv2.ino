/***********************************************************************
 * AI-Assisted Cognitive Radio LPWAN
 * TRANSMITTER NODE (Fixed & Synchronized)
 *
 * Hardware:
 * ----------
 * ESP32
 * SX1278 #1 -> DATA PLANE (433 MHz)
 * SX1278 #2 -> CONTROL PLANE (445 MHz)
 *
 * DATA RADIO
 * - Continuously transmits application data.
 *
 * CONTROL RADIO
 * - Always listens for AI decisions.
 * - Receives:
 * • Channel hopping commands
 * • SF adaptation
 * • Coding rate adaptation
 * - Sends acknowledgement ONLY for control packets.
 *
 * NOTE:
 * DATA packets are NEVER acknowledged.
 *
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
// DATA RADIO (433 MHz)
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

#define DATA_PACKET_MAGIC 0xDEADBEEF
//======================================================================
// CREATE SHARED SPI BUS
//======================================================================
SPIClass sharedSPI(VSPI);

//======================================================================
// CREATE BOTH LORA OBJECTS
//======================================================================
SX1278 dataRadio = new Module(
    DATA_NSS,
    DATA_DIO0,
    DATA_RST,
    RADIOLIB_NC,
    sharedSPI
);

SX1278 controlRadio = new Module(
    CTRL_NSS,
    CTRL_DIO0,
    CTRL_RST,
    RADIOLIB_NC,
    sharedSPI
);
//======================================================================
// RADIO HEALTH / FAULT-HALT STATE
//======================================================================
const uint8_t RADIO_FAULT_THRESHOLD = 5;

uint8_t  dataRadioFailStreak    = 0;
uint8_t  controlRadioFailStreak = 0;

uint32_t lastGoodTxSequence     = 0;
uint32_t lastGoodAckCommandID   = 0;

bool systemHalted = false;
//======================================================================
// CURRENT RADIO CONFIGURATION
//======================================================================
// FIX: Changed from float (433.0) to uint32_t (433000) to synchronize 
// tracking in kHz with the receiver node and avoid precision loss.
uint32_t currentFrequencyKHz = 433000; 
uint8_t currentSF = 8;
uint8_t currentCR = 5;

//======================================================================
// CONTROL CHANNEL CONFIGURATION
//======================================================================
const float CONTROL_FREQUENCY = 445.00;
const uint8_t CONTROL_SF = 12;
const uint8_t CONTROL_CR = 8;

//======================================================================
// PACKET COUNTER
//======================================================================
uint32_t sequenceNumber = 0;


volatile bool ctrlPacketFlag = false;

void IRAM_ATTR onCtrlDio0() {
    ctrlPacketFlag = true;
}

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
//======================================================================
struct DataPacket
{
    uint32_t magic;  
    uint32_t sequence;
    uint32_t timestamp;
    uint16_t sensorValue;
};

//======================================================================
// CONTROL PACKET
//======================================================================
struct ControlPacket
{
    uint32_t commandID;
    uint8_t command;
    
    // FIX: Changed from 'unit8_t newFrequency' to 'uint32_t newFrequencyKHz'.
    // This fixes the 'unit8_t' typo and expands the size to 32 bits to prevent 
    // large frequency integer values (e.g., 434000) from truncating/overflowing.
    uint32_t newFrequencyKHz; 
    
    uint8_t newSF;
    uint8_t newCR;
};

//======================================================================
// CONTROL ACK
//======================================================================
struct ControlAck
{
    uint32_t commandID;
    bool applied;
};

//======================================================================
// FUNCTION DECLARATIONS
//======================================================================
bool initializeDataRadio();
bool initializeControlRadio();

// FIX: Updated signature parameter from uint8_t to uint32_t to match layout.
void applyRadioConfiguration(
    uint32_t frequencyKHz,
    uint8_t sf,
    uint8_t cr
);

void sendDataPacket();
void checkControlPlane();
void sendControlAck(uint32_t commandID);
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

    controlRadio.startReceive();
    controlRadio.setDio0Action(onCtrlDio0,RISING);
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

    // Convert kHz integer to MHz float for RadioLib setup
    float freqMHz = currentFrequencyKHz / 1000.0f;
    Serial.println("-----------------------------------------");
  Serial.println("-----------------------------------------");
 Serial.printf("DATA Radio Frequency %.2f MHz\n", freqMHz); 

    int state = dataRadio.begin(
                    freqMHz,              // Frequency (MHz)
                    125.0,                // Bandwidth (kHz)
                    currentSF,            // Spreading Factor
                    currentCR,            // Coding Rate (5 = 4/5)
                    RADIOLIB_SX127X_SYNC_WORD,
                    17,                   // TX Power (dBm)
                    8,
                    0                     // Preamble Length
                );

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("DATA Radio Init Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("DATA Radio Initialized");

    Serial.print("Frequency (kHz) : ");
    Serial.println(currentFrequencyKHz);

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
    Serial.println("-----------------------------------------");
    Serial.println("Initializing CONTROL Radio...");
    Serial.println("-----------------------------------------");
    int state = controlRadio.begin(
                    CONTROL_FREQUENCY,
                    125.0,
                    CONTROL_SF,
                    CONTROL_CR,
                    RADIOLIB_SX127X_SYNC_WORD,
                    10,             // Lower TX power is enough
                    8,
                    0
                );

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("CONTROL Radio Init Failed : ");
        Serial.println(state);
        return false;
    }

    Serial.println("CONTROL Radio Initialized");

    Serial.print("Frequency : ");
    Serial.println(CONTROL_FREQUENCY);

    Serial.print("SF : ");
    Serial.println(CONTROL_SF);

    Serial.print("CR : 4/");
    Serial.println(CONTROL_CR);
    Serial.println();

    return true;
}

//======================================================================
// APPLY NEW RADIO CONFIGURATION
//======================================================================
void applyRadioConfiguration(
        uint32_t frequencyKHz,
        uint8_t sf,
        uint8_t cr)
{
    // FIX: Changed from 'command.frequencyKHz' to 'frequencyKHz'.
    // The previous code called an out-of-scope variable 'command' which 
    // caused compilation to fail. Now it uses the passed function argument.
    float freqMHz = frequencyKHz / 1000.0f;
    
    Serial.println();
    Serial.println("========== APPLYING NEW CONFIGURATION ==========");

    Serial.print("Old Frequency (kHz) : ");
    Serial.println(currentFrequencyKHz);

    Serial.print("Old SF : ");
    Serial.println(currentSF);

    Serial.print("Old CR : 4/");
    Serial.println(currentCR);

    currentFrequencyKHz = frequencyKHz;
    currentSF = sf;
    currentCR = cr;
    
    dataRadio.setFrequency(freqMHz);
    dataRadio.setSpreadingFactor(currentSF);
    dataRadio.setCodingRate(currentCR);

    Serial.println();

    Serial.print("New Frequency (kHz) : ");
    Serial.println(currentFrequencyKHz);
    Serial.print("New SF : ");
    Serial.println(currentSF);

    Serial.print("New CR : 4/");
    Serial.println(currentCR);

    Serial.println("Configuration Updated");
    Serial.println("===============================================");
    Serial.println();
}

//======================================================================
// READ SENSOR
//======================================================================
uint16_t readSensor()
{
    // Generates a random value matching ESP32's 12-bit ADC range (0 to 4095)
    return random(0, 4096);
}

void haltSystem(const char* radioName, uint32_t lastKnownGoodValue) {
    if (systemHalted) return;
    systemHalted = true;

    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_ERROR, HIGH);

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
//======================================================================
// SEND DATA PACKET
//======================================================================
void sendDataPacket()
{
    DataPacket packet;
    packet.magic     = DATA_PACKET_MAGIC;
    packet.sequence = sequenceNumber++;
    packet.timestamp = millis();
    packet.sensorValue = readSensor();

    digitalWrite(LED_TX, HIGH);

    int state = dataRadio.transmit(
                    (uint8_t*)&packet,
                    sizeof(packet)
                );

    digitalWrite(LED_TX, LOW);

    if(state == RADIOLIB_ERR_NONE)
    {
        dataRadioFailStreak = 0;
        lastGoodTxSequence = packet.sequence;
        Serial.print("[DATA] Seq=");
        Serial.print(packet.sequence);
        Serial.print("  Sensor=");
        Serial.print(packet.sensorValue);

        Serial.print("  Time=");
        Serial.println(packet.timestamp);
    }
else
    {
        Serial.print("[DATA] Transmission Failed : ");
        Serial.println(state);
        dataRadioFailStreak++;
        if (dataRadioFailStreak >= RADIO_FAULT_THRESHOLD) {
            haltSystem("DATA radio (TX)", lastGoodTxSequence);
            return;
        }
        digitalWrite(LED_ERROR, HIGH);
        delay(100);
        digitalWrite(LED_ERROR, LOW);
    }
}

//======================================================================
// SEND CONTROL ACK
//======================================================================
void sendControlAck(uint32_t commandID)
{
    ControlAck ack;
    ack.commandID = commandID;
    ack.applied = true;

    int state = controlRadio.transmit(
                    (uint8_t*)&ack,
                    sizeof(ack)
                );
    ctrlPacketFlag = false;   // NEW: discard the self-triggered TxDone interrupt from this transmit
    if(state == RADIOLIB_ERR_NONE)
    {
         controlRadioFailStreak = 0;
        lastGoodAckCommandID = commandID;
        Serial.print("[CTRL] ACK Sent : ");
        Serial.println(commandID);
    }
  else
    {
        Serial.print("[CTRL] ACK Failed : ");
        Serial.println(state);
        controlRadioFailStreak++;
        if (controlRadioFailStreak >= RADIO_FAULT_THRESHOLD) {
            haltSystem("CONTROL radio (TX)", lastGoodAckCommandID);
            return;
        }
    }

    // Immediately return to receive mode.
    controlRadio.startReceive();
}

//======================================================================
// CHECK CONTROL CHANNEL
//======================================================================
void checkControlPlane()
{
if (!ctrlPacketFlag) {
    return;
}
ctrlPacketFlag = false;
 Serial.println("[CTRL] DIO0 fired - packet detected on control radio");   // NEW - diagnostic only
    ControlPacket command;

    int state = controlRadio.readData(
                    (uint8_t*)&command,
                    sizeof(command)
                );

    if(state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        Serial.println("[CTRL] CRC Error");
        controlRadio.startReceive();
        return;
    }

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[CTRL] Receive Error : ");
        Serial.println(state);
        controlRadio.startReceive();
        return;
    }

    Serial.println();
    Serial.println("====================================");
    Serial.println("AI CONTROL COMMAND RECEIVED");
    Serial.println("====================================");

    Serial.print("Command ID : ");
    Serial.println(command.commandID);
    Serial.print("Command Type : ");
    Serial.println(command.command);

    Serial.print("Frequency (kHz) : ");
    Serial.println(command.newFrequencyKHz);

    Serial.print("SF : ");
    Serial.println(command.newSF);

    Serial.print("CR : 4/");
    Serial.println(command.newCR);
    Serial.println();

    switch(command.command)
    {
        //----------------------------------------------------------
        // CHANNEL HOP
        //----------------------------------------------------------
        case CMD_CHANNEL_HOP:
            Serial.println("Executing Channel Hop...");
            sendControlAck(command.commandID);
            applyRadioConfiguration(
                    command.newFrequencyKHz,
                    currentSF,
                    currentCR
            );
            break;

        //----------------------------------------------------------
        // LINK ADAPTATION
        //----------------------------------------------------------
        case CMD_LINK_ADAPT:
            Serial.println("Executing Link Adaptation...");
            sendControlAck(command.commandID);
            applyRadioConfiguration(
                    command.newFrequencyKHz,
                    command.newSF,
                    command.newCR
            );
            break;

        //----------------------------------------------------------
        // UNKNOWN COMMAND
        //----------------------------------------------------------
        default:
            Serial.println("Unknown Command");
            controlRadio.startReceive();
            return;
    }



    Serial.println("Configuration Applied Successfully");
    Serial.println("====================================");
    Serial.println();
}

//======================================================================
// TRANSMISSION INTERVAL (milliseconds)
//======================================================================
const uint32_t DATA_INTERVAL = 500;

//======================================================================
// SOFTWARE TIMER
//======================================================================
uint32_t lastTransmissionTime = 0;

//======================================================================
// MAIN LOOP
//======================================================================
void loop()
{
       if (systemHalted) {
        return;
    }
    // 1. Always check whether an AI control packet has arrived.
    checkControlPlane();

    // 2. Periodically transmit application data.
    if (millis() - lastTransmissionTime >= DATA_INTERVAL)
    {
        lastTransmissionTime = millis();
        sendDataPacket();
    }
}
