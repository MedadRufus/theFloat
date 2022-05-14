/*
  Software for Zachtek WSPR Transmitter products
  For Arduino Pro Mini ATMega 328 8MHz, 3.3V boards or ATMega328P-AU chips


  Hardware connections:
  --------------------------
  pin 2 and 3 is Sofware serial port to GPS module
  pin 4 is Status LED, except on Mini and Pico models- they use pin A2. This Led is Used as StatusIndicator to display what state the software is currently in, Yellow LED on all models except Pico that has a white LED
  pin 5,6 and 7 are Relay control pins on the Desktop and LP1 products
  pin 8 is Red TX LED next to RF out SMA connector on the Desktop and LP1 products. Used as Indicator to display when there is RF out.
  pin A1 Hardware Sleep signal of the GPS on the WSPR-TX Pico, The Mini is using Serial command to put GPS to sleep.
  pin A3 Power-down the Si5351 from this pin on the WSPR-TX Mini

*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <defines.hpp>
#include <i2c.hpp>
#include "string_operations.hpp"
#include "wspr_packet_formatting.hpp"
#include "datatypes.hpp"
#include "geofence.hpp"
#include "Si5351.hpp"
#include "state_machine.hpp"
#include "print_operations.hpp"
#include "eeprom.hpp"
#include "filter_management.hpp"
#include "adc.hpp"
#include "wspr_utils.hpp"
#include "gps.hpp"
#include "power.hpp"
#include "led.hpp"

NMEAGPS gps; // This parses the GPS characters
gps_fix fix; // This holds on to the latest values

boolean Si5351I2C_found = false;

// Data structures
enum E_Band
{
    LF2190m = 0,
    LF630m = 1,
    HF160m = 2,
    HF80m = 3,
    HF40m = 4,
    HF30m = 5,
    HF20m = 6,
    HF17m = 7,
    HF15m = 8,
    HF12m = 9,
    HF10m = 10,
    HF6m = 11,
    VHF4m = 12,
    VHF2m = 13,
    UHF70cm = 14,
    UHF23cm = 15
};

// declarations

int StatusLED; // LED that indicates current status. Yellow on LP1, Desktop and Mini models, white on Pico
// Global Variables
S_GadgetData GadgetData;   // Create a datastructure that holds all relevant data for a WSPR Beacon
S_FactoryData FactoryData; // Create a datastructure that holds information of the hardware
E_Mode CurrentMode;        // What mode are we in, WSPR, signal generator or nothing

uint8_t CurrentBand = 0;         // Keeps track on what band we are currently tranmitting on
uint8_t CurrentLP = 0;           // Keep track on what Low Pass filter is currently switched in
const uint8_t SerCMDLength = 50; // Max number of char on a command in the SerialAPI

uint64_t freq; // Holds the Output frequency when we are in signal generator mode or in WSPR mode
int GPSH;      // GPS Hours
int GPSM;      // GPS Minutes
int GPSS;      // GPS Seconds
int fixstate;  // GPS Fix state-machine. 0=Init, 1=wating for fix,2=fix accuired
boolean PCConnected;
uint16_t LoopGPSNoReceiveCount; // If GPS stops working while in ídle mode this will increment
char LastMaidenHead6[7];        // Holds the Maidenhead position from last transmission, used when GadgetData.WSPRData.TimeSlotCode=17 to determine if the transmitter has moved since last TX
// The serial connection to the GPS device
SoftwareSerial GPSSerial(2, 3); // GPS Serial port, RX on pin 2, TX on pin 3

// function declarations

void DoSerialHandling();

unsigned long RandomSeed(void);

void DriveLPFilters();

uint8_t EncodeChar(char Character);

// Implementation of functions

// Parts from NickGammon Serial Input example
// http://www.gammon.com.au/serial
void DoSerialHandling()
{
    static char SerialLine[SerCMDLength]; // A single line of incoming serial command and data
    static uint8_t input_pos = 0;
    char InChar;
    PCConnected = true;
    while (Serial.available() > 0)
    {
        InChar = Serial.read();
        switch (InChar)
        {
        case '\n': // end of text
        {
            SerialLine[input_pos] = 0; // terminating null byte
            // terminator reached, process Command
            DecodeSerialCMD(SerialLine, GadgetData);
            // reset buffer for next time
            input_pos = 0;
            break;
        }

        case '\r': // discard carriage return
        {
            break;
        }

        default:
        {
            // keep adding if not full ... allow for terminating null byte
            if (input_pos < (SerCMDLength - 1))
            {
                SerialLine[input_pos++] = InChar;
            }
            break;
        }

        } // end of switch
    }     // end of processIncomingByte
}

// Create a random seed by doing CRC32 on 100 analog values from port A0
// CRC calculation from Christopher Andrews : https://www.arduino.cc/en/Tutorial/EEPROMCrc
unsigned long RandomSeed(void)
{

    const unsigned long crc_table[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};

    uint8_t ByteVal;
    unsigned long crc = ~0L;

    for (int index = 0; index < 100; ++index)
    {
        ByteVal = analogRead(A0);
        crc = crc_table[(crc ^ ByteVal) & 0x0f] ^ (crc >> 4);
        crc = crc_table[(crc ^ (ByteVal >> 4)) & 0x0f] ^ (crc >> 4);
        crc = ~crc;
    }
    return crc;
}

void setup()
{
    // bool i2c_found;
    i2cInit();
    PCConnected = false;
    fixstate = 0; // GPS fixstate=No location fix
    // Initialize the serial ports, The hardware port is used for communicating with a PC.
    // The Soft Serial is for communcating with the GPS
    Serial.begin(9600); // USB Serial port
    Serial.setTimeout(2000);
    GPSSerial.begin(9600); // Init software serial port to communicate with the on-board GPS module
    // Read all the Factory data from EEPROM at position 400
    if (LoadFromEPROM(FactorySpace)) // Read all Factory data from EEPROM
    {
    }
    else // No factory data was found in EEPROM, set some defaults
    {
        Serial.println(F("{MIN} No factory data found !"));
        Serial.println(F("{MIN} You need to run factory setup to complete the configuration, guessing on calibration values for now"));
        FactoryData.HW_Version = 1;     // Hardware version
        FactoryData.RefFreq = 24999980; // Reference Oscillator frequency
        if (Product_Model == 1011)      // LP1 Model, set some defaults
        {
            FactoryData.HW_Revision = 17;  // Hardware revision
            FactoryData.LP_A_BandNum = 98; // Low Pass filter A is Link
            FactoryData.LP_B_BandNum = 99; // Low Pass filter B is Nothing
            FactoryData.LP_C_BandNum = 99; // Low Pass filter C is Nothing
            FactoryData.LP_D_BandNum = 99; // Low Pass filter D is Nothing
        }

        if (Product_Model == 1012) // Desktop Model, set default version
        {
            FactoryData.HW_Revision = 21; // Hardware revision

            // 80To10
            FactoryData.LP_A_BandNum = 10; // Low Pass filter A is 10m (+17m + 15m and 12m)
            FactoryData.LP_B_BandNum = 3;  // Low Pass filter B is 80m
            FactoryData.LP_C_BandNum = 4;  // Low Pass filter C is 40m
            FactoryData.LP_D_BandNum = 6;  // Low Pass filter D is 20m (+30m)
            /*
                     //80To10
                     FactoryData.LP_A_BandNum = 10;//Low Pass filter A is 10m (+17m + 15m and 12m)
                     FactoryData.LP_B_BandNum = 3; //Low Pass filter B is 80m
                     FactoryData.LP_C_BandNum = 4; //Low Pass filter C is 40m
                     FactoryData.LP_D_BandNum = 6; //Low Pass filter D is 20m (+30m)

                     //Low Model
                     FactoryData.LP_A_BandNum = 0;  //Low Pass filter A is 2190m
                     FactoryData.LP_B_BandNum = 1;  //Low Pass filter B is 630m
                     FactoryData.LP_C_BandNum = 99; //Low Pass filter C is open circuit
                     FactoryData.LP_D_BandNum = 99; //Low Pass filter D is open circuit

                     //MidPlus
                     FactoryData.LP_A_BandNum = 2; //Low Pass filter A is 160m
                     FactoryData.LP_B_BandNum = 3; //Low Pass filter B is 80m
                     FactoryData.LP_C_BandNum = 4; //Low Pass filter C is 40m
                     FactoryData.LP_D_BandNum = 6; //Low Pass filter D is 20m (+30m)

                     //HighPlus
                     FactoryData.LP_A_BandNum = 7;  //Low Pass filter A is 17m
                     FactoryData.LP_B_BandNum = 10; //Low Pass filter B is 10m (+ 15 and 12m)
                     FactoryData.LP_C_BandNum = 11; //Low Pass filter C is 6m
                     FactoryData.LP_D_BandNum = 99; //Low Pass filter D is open circuit

               */
        }
        if (Product_Model == 1029) // LP1 Model with Mezzanine BLP4 addon, set default LP as a MidPlus version
        {
            FactoryData.HW_Revision = 17; // Hardware revision
            FactoryData.LP_A_BandNum = 2; // Low Pass filter A is 160m
            FactoryData.LP_B_BandNum = 3; // Low Pass filter B is 80m
            FactoryData.LP_C_BandNum = 4; // Low Pass filter C is 40m
            FactoryData.LP_D_BandNum = 6; // Low Pass filter D is 20m
        }

        if (Product_Model == 1028) // Pico Model, set default LP to 20m
        {
            FactoryData.HW_Revision = 5;   // Hardware revision
            FactoryData.LP_A_BandNum = 6;  // Low Pass filter A is 20m
            FactoryData.LP_B_BandNum = 99; // Low Pass filter B is open circuit
            FactoryData.LP_C_BandNum = 99; // Low Pass filter C is open circuit
            FactoryData.LP_D_BandNum = 99; // Low Pass filter D is open circuit
        }
    }

    if (LoadFromEPROM(UserSpace)) // Read all UserSpace data from EEPROM at position 0
    {
        CurrentMode = GadgetData.StartMode;
        GadgetData.WSPRData.CallSign[6] = 0;    // make sure Call sign is null terminated in case of incomplete data saved
        GadgetData.WSPRData.MaidenHead4[4] = 0; // make sure Maidenhead locator is null terminated in case of incomplete data saved
        GadgetData.WSPRData.MaidenHead6[6] = 0; // make sure Maidenhead locator is null terminated in case of incomplete data saved
    }
    else // No user data was found in EEPROM, set some defaults
    {

        CurrentMode = SignalGen;
        GadgetData.Name[0] = 'W';
        GadgetData.Name[1] = 'S';
        GadgetData.Name[2] = 'P';
        GadgetData.Name[3] = 'R';
        GadgetData.Name[4] = ' ';
        GadgetData.Name[5] = 'T';
        GadgetData.Name[6] = 'X';
        GadgetData.Name[7] = 0;
        GadgetData.StartMode = Idle;
        GadgetData.WSPRData.CallSign[0] = 'A';
        GadgetData.WSPRData.CallSign[1] = 'A';
        GadgetData.WSPRData.CallSign[2] = '0';
        GadgetData.WSPRData.CallSign[3] = 'A';
        GadgetData.WSPRData.CallSign[4] = 'A';
        GadgetData.WSPRData.CallSign[5] = 'A';
        GadgetData.WSPRData.CallSign[6] = 0;
        GadgetData.WSPRData.LocatorOption = GPS;
        GadgetData.WSPRData.MaidenHead4[0] = 'A';
        GadgetData.WSPRData.MaidenHead4[1] = 'A';
        GadgetData.WSPRData.MaidenHead4[2] = '0';
        GadgetData.WSPRData.MaidenHead4[3] = '0';
        GadgetData.WSPRData.MaidenHead4[4] = 0; // Null termination
        GadgetData.WSPRData.MaidenHead6[0] = 'A';
        GadgetData.WSPRData.MaidenHead6[1] = 'A';
        GadgetData.WSPRData.MaidenHead6[2] = '0';
        GadgetData.WSPRData.MaidenHead6[3] = '0';
        GadgetData.WSPRData.MaidenHead6[4] = 'A';
        GadgetData.WSPRData.MaidenHead6[5] = 'A';
        GadgetData.WSPRData.MaidenHead6[6] = 0; // Null termination
        GadgetData.WSPRData.LocationPrecision = 4;
        GadgetData.WSPRData.PowerOption = Normal; // Use the Power encoding for normal power reporting
        GadgetData.WSPRData.TXPowerdBm = 23;      // Set deafult power to 0.2W
        GadgetData.WSPRData.TimeSlotCode = 16;    // TX on any even minute (no scheduling)
        GadgetData.WSPRData.SuPreFixOption = None;
        if (Product_Model == 1017) // The WSPR mini
        {
            GadgetData.WSPRData.TXPowerdBm = 13; // WSPR Mini has 20mW output power
        }
        if (Product_Model == 1028) // The WSPR Pico
        {
            GadgetData.WSPRData.TXPowerdBm = 10;       // WSPR Pico has 10mW output power
            GadgetData.WSPRData.CallSign[5] = 'B';     // Set other than default Callsign so it will start WSPR automatically even if not configured, helps in the testing of new devices
            GadgetData.WSPRData.LocationPrecision = 6; // Use six letter Maidnhead postion reports by transmitting Type3 messages
        }
        for (int i = 0; i < 16; i++)
        {
            GadgetData.TXOnBand[i] = false; // Disable TX on all bands.
        }
        GadgetData.TXOnBand[5] = true; // enable TX on 30m
        GadgetData.TXOnBand[6] = true; // enable TX on 20m
        GadgetData.TXPause = 480;      // Number of seconds to pause after transmisson
        GadgetData.GeneratorFreq = 1000000000;
        Serial.println(F("{MIN} No user data was found, setting default values"));
    }

    // Set staus LED to  pin 4. This is the case for most hardware versions but some model use a different pinout and vill owerride this value below
    StatusLED = 4;
    switch (Product_Model)
    {
    case 1011:
        Serial.println(F("{MIN} ZachTek WSPR-TX_LP1 transmitter"));
        // De-energize any relays connected to option port
        pinMode(Relay2, OUTPUT);
        pinMode(Relay3, OUTPUT);
        digitalWrite(Relay2, LOW);
        digitalWrite(Relay3, LOW);
        break;

    case 1012:
        Serial.println(F("{MIN} ZachTek WSPR Desktop transmitter"));
        if ((FactoryData.HW_Version == 1) & (FactoryData.HW_Revision == 4)) // Early WSPR Desktop hardware had different Relay driving electronics
        {
            // De-energize all relays
            pinMode(Relay1, INPUT);
            pinMode(Relay2, INPUT);
            pinMode(Relay3, INPUT);
        }
        else
        {
            // De-energize all relays
            pinMode(Relay1, OUTPUT);
            pinMode(Relay2, OUTPUT);
            pinMode(Relay3, OUTPUT);
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, LOW);
            digitalWrite(Relay3, LOW);
        }
        break;

    case 1024:
        Serial.println(F("{MIN} ZachTek Super Simple Signal Generator"));
        StatusLED = 10; // Status LED
        pinMode(SiPower, OUTPUT);
        digitalWrite(SiPower, LOW); // Turn on power to the Si5351
        break;

    case 1017:
        Serial.println(F("{MIN} ZachTek WSPR Mini transmitter"));
        StatusLED = A2; // Status LED uses a different output on the Mini
        pinMode(SiPower, OUTPUT);
        digitalWrite(SiPower, LOW); // Turn on power to the Si5351
        break;

    case 1020:
        Serial.println(F("{MIN} ZachTek WSPR-TX_LP1 transmitter with Mezzanine LP4 board"));
        // De-energize all relays
        pinMode(Relay2, OUTPUT);
        pinMode(Relay3, OUTPUT);
        digitalWrite(Relay2, LOW);
        digitalWrite(Relay3, LOW);
        break;

    case 1028:
        Serial.println(F("{MIN} ZachTek WSPR Pico transmitter"));
        StatusLED = A2; // Status LED uses a different output on the Pico
        // The Pico is assumed to never be used as a stationary transmitter,
        // it will most likely fly in a ballon beacon so set some settings to avoid a user releasing a ballon with a missconfigured beacon
        GadgetData.WSPRData.LocatorOption = GPS;    // Always set the Locator option to GPS calculated as a failsafe
        GadgetData.WSPRData.PowerOption = Altitude; // Always encode Altitude in the power field as a failsafe
        CurrentMode = WSPRBeacon;                   // Always boot the WSPR Pico in to beacon mode as a failsafe
        break;

    case 1029:
        Serial.println(F("{MIN} ZachTek WSPR-TX_LP1 transmitter with Mezzanine BLP4 board"));
        // De-energize all relays
        pinMode(Relay2, OUTPUT);
        pinMode(Relay3, OUTPUT);
        digitalWrite(Relay2, LOW);
        digitalWrite(Relay3, LOW);
        break;
    }

    // Use the Red LED as a Transmitt indicator and the Yellow LED as Status indicator
    pinMode(StatusLED, OUTPUT);
    pinMode(TransmitLED, OUTPUT);

    Serial.print(F("{MIN} Firmware version "));
    Serial.print(SoftwareVersion);
    Serial.print((":"));
    Serial.println(SoftwareRevision);

    // Blink StatusLED to indicate Reboot
    LEDBlink(16);
    random(RandomSeed());
    PowerSaveOFF();

    Si5351I2C_found = DetectSi5351I2CAddress();

    // wspr_encode(GadgetData.WSPRData.CallSign, GadgetData.WSPRData.MaidenHead4, GadgetData.WSPRData.TXPowerdBm, tx_buffer, 3);

    switch (CurrentMode)
    {
    case SignalGen:
        DoSignalGen();
        break;

    case WSPRBeacon:
        CurrentBand = 0;
        DoWSPR();
        break;

    case Idle:
        DoIdle();
        break;
    }
}

void loop()
{

    if (Serial.available())
    { // Handle  Serial API request from the PC
        DoSerialHandling();
    }
    if (CurrentMode == WSPRBeacon)
    {
        DoWSPR(); // If in WSPR beacon mode but it broke out of beacon loop to handle a Serial data from the PC then go back to the WSPR routine
    }
    while (gps.available(GPSSerial))
    { // Handle Serial data from the GPS as they arrive
        fix = gps.read();
        SendAPIUpdate(UMesTime);
        LoopGPSNoReceiveCount = 0;
        if ((GPSS % 4) == 0) // Send some nice-to-have info every 4 seconds, this is a lot of data so we dont want to send it to often to risk choke the Serial output buffer
        {
            SendSatData();                  // Send Satellite position and SNR information to the PC GUI
            SendAPIUpdate(UMesVCC);         // Send power supply voltage at the MCU to the PC GUI
            SendAPIUpdate(UMesCurrentMode); // Send info of what routine is running to the PC GUI
            if (fix.valid.location && fix.valid.time)
            {
                SendAPIUpdate(UMesGPSLock);
                if (GadgetData.WSPRData.LocatorOption == GPS)
                { // If GPS should update the Maidenhead locator
                    calcLocator(fix.latitude(), fix.longitude(), &GadgetData.WSPRData);
                }
                SendAPIUpdate(UMesLocator);
            }
            else
            {
                SendAPIUpdate(UMesNoGPSLock);
            }
        }
        gps_blocking_wait(200);
    }
    LoopGPSNoReceiveCount++;
    if (LoopGPSNoReceiveCount > 60000) // GPS have not sent anything for a long time, GPS is possible in sleep mode or has not started up correctly. This can happen if a brown-out/reboot happens while the GPS was sleeping
    {
        LoopGPSNoReceiveCount = 0;
        Serial.println(F("{MIN} Resetting GPS"));
        GPSWakeUp(); // Try to get GPS going again by sending wake up command
        gps_blocking_wait(2000);
    }
}