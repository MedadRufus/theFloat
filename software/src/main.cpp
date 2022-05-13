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
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h> //NeoGps by SlashDevin"
#include <defines.hpp>
#include <i2c.hpp>
#include "string_operations.hpp"
#include "wspr_packet_formatting.hpp"
#include "datatypes.hpp"
#include "geofence.hpp"
#include "Si5351.hpp"
#include "api_update.hpp"

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

struct S_FactoryData
{
    uint8_t HW_Version;   // Hardware version
    uint8_t HW_Revision;  // Hardware revision
    uint32_t RefFreq;     // The frequency of the Reference Oscillator in Hz, usually 26000000
    uint8_t LP_A_BandNum; // Low Pass filter A Band number (0-15) Ham Band as defined by E_Band  Eg. if a 20m LowPass filter is fitted on LP_A then LP_A_BandNum will be set to 6 by factory config software
    uint8_t LP_B_BandNum; // Low Pass filter B Band number (0-15)
    uint8_t LP_C_BandNum; // Low Pass filter C Band number (0-15)
    uint8_t LP_D_BandNum; // Low Pass filter D Band number (0-15)
};

// declarations

const uint8_t LP_A = 0;
const uint8_t LP_B = 1;
const uint8_t LP_C = 2;
const uint8_t LP_D = 3;

int StatusLED; // LED that indicates current status. Yellow on LP1, Desktop and Mini models, white on Pico
// Global Variables
S_GadgetData GadgetData;   // Create a datastructure that holds all relevant data for a WSPR Beacon
S_FactoryData FactoryData; // Create a datastructure that holds information of the hardware
extern E_Mode CurrentMode; // What mode are we in, WSPR, signal generator or nothing

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

uint8_t BandNumOfHigestLP();
int GetVCC();
void NextFreq(void);
void PickLP(uint8_t TXBand);
void calcLocator(double lat, double lon);
boolean CorrectTimeslot();

void DoSerialHandling();

// silabs related
void DoSignalGen();
void DoIdle();
void DoWSPR();

// wspr related
int SendWSPRMessage(uint8_t WSPRMessageType);

void calcLocator(double lat, double lon);
boolean NewPosition();
void StorePosition();

static void smartdelay(unsigned long delay_ms);

unsigned long RandomSeed(void);
boolean NoBandEnabled(void);
void NextFreq(void);
boolean LastFreq(void);

// eeprom related
unsigned long GetEEPROM_CRC(boolean EEPROMSpace);
bool LoadFromEPROM(boolean EEPROMSpace);
void SaveToEEPROM(boolean EEPROMSpace);

void LEDBlink(int Blinks);

void DriveLPFilters();

uint8_t FreqToBand();
void PickLP(uint8_t TXBand);
uint8_t BandNumOfHigestLP();

int GetVCC();

void PowerSaveOFF();
void PowerSaveON();

void GPSGoToSleep();
void GPSWakeUp();
void GPSReset();

void SerialPrintZero();
void SendSatData();
uint8_t EncodeChar(char Character);

boolean CorrectTimeslot();

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

void DoSignalGen()
{
    if (Si5351I2C_found == false)
    {
        Serial.println(F("{MIN}Hardware ERROR! No Si5351 PLL device found on the I2C buss!"));
    }
    else
    {
        CurrentMode = SignalGen;
        freq = GadgetData.GeneratorFreq;
        PickLP(FreqToBand()); // Use the correct low pass filter
        si5351aSetFrequency(freq, FactoryData.RefFreq);
        digitalWrite(StatusLED, HIGH);
        SendAPIUpdate(UMesCurrentMode);
        SendAPIUpdate(UMesFreq);
    }
}

void DoIdle()
{
    PowerSaveOFF();
    CurrentMode = Idle;
    digitalWrite(StatusLED, LOW);
    si5351aOutputOff(SI_CLK0_CONTROL);
    SendAPIUpdate(UMesCurrentMode);
}

void DoWSPR()
{
    uint8_t pwr1, pwr2; // Used in Altitude to power reporting (balloon coding)
    uint32_t AltitudeInMeter;
    boolean ConfigError;
    // uint32_t GPSNoReceiveCount; //If GPS stops working in WSPR Beacon mode this will increment
    int WSPRMessageTypeToUse;

    if (GadgetData.WSPRData.SuPreFixOption == None) // if standard Call Sign with no Sufix then send a Standards Type 1 message, else Send a Type 2 Message to include the Sufix
    {
        WSPRMessageTypeToUse = 1;
    }
    else
    {
        WSPRMessageTypeToUse = 2;
    }
    if (Si5351I2C_found == false)
    {
        Serial.println(F("{MIN}Hardware ERROR! No Si5351 PLL device found on the I2C buss!"));
    }
    else
    {
        CurrentMode = WSPRBeacon;
        ConfigError = false;

        // Make sure at least one band is enabled for tranmission
        if (NoBandEnabled())
        {
            Serial.println(F("{MIN}Tranmission is not enabled on any band"));
            ConfigError = true;
        }

        // Make sure  call sign is set
        if ((GadgetData.WSPRData.CallSign[0] == 'A') && (GadgetData.WSPRData.CallSign[1] == 'A') && (GadgetData.WSPRData.CallSign[2] == '0') && (GadgetData.WSPRData.CallSign[3] == 'A') && (GadgetData.WSPRData.CallSign[4] == 'A') && (GadgetData.WSPRData.CallSign[5] == 'A')) // Do not actually key the transmitter if the callsign has not been changed from the default one AA0AAA
        {
            Serial.println(F("{MIN}Call Sign not set"));
            ConfigError = true;
        }

        if (ConfigError)
        {
            Serial.println(F("{MIN}Can not start WSPR Beacon"));
            DoIdle(); // Go back to ideling
        }
        else
        {
            CurrentBand = 0;
            NextFreq();                                 // Cycle to next enabled band to transmit on
            freq = freq + (100ULL * random(-100, 100)); // modify TX frequency with a random value beween -100 and +100 Hz
            si5351aOutputOff(SI_CLK0_CONTROL);
            SendAPIUpdate(UMesCurrentMode);

            // LOOP HERE FOREVER OR UNTIL INTERRUPTED BY A SERIAL COMMAND
            while (!Serial.available())
            { // Do until incoming serial command
                while (gps.available(GPSSerial))
                { // If GPS data is available - process it
                    LoopGPSNoReceiveCount = 0;
                    fix = gps.read();
                    SendAPIUpdate(UMesTime);
                    if (Serial.available())
                    { // If serialdata was received on control port then handle command
                        return;
                    }
                    if (fix.valid.location && fix.valid.time)
                    {
                        GPSH = fix.dateTime.hours;
                        GPSM = fix.dateTime.minutes;
                        GPSS = fix.dateTime.seconds;
                        if (GadgetData.WSPRData.LocatorOption == GPS)
                        { // If GPS should update the Maidenhead locator
                            calcLocator(fix.latitude(), fix.longitude());
                        }
                        if ((GPSS == 00) && (CorrectTimeslot())) // If second is zero at even minute then start WSPR transmission. The function CorrectTimeSlot can hold of transmision depending on several user settings. The GadgetData.WSPRData.TimeSlotCode value will influense the behaviour
                        {
                            if ((PCConnected) || (Product_Model != 1028) || ((Product_Model == 1028) && OutsideGeoFence())) // On the WSPR-TX Pico make sure were are outside the territory of UK, Yemen and North Korea before the transmitter is started but allow tranmissions inside the Geo-Fence if a PC is connected so UK users can make test tranmissions on the ground before relase of Picos
                            {
                                GPSGoToSleep(); // Put GPS to sleep to save power
                                // -------------------- Altitude coding to Power ------------------------------------
                                if (GadgetData.WSPRData.PowerOption == Altitude) // If Power field should be used for Altitude coding
                                {
                                    AltitudeInMeter = (uint32_t)fix.altitude();
                                    pwr1 = ValiddBmValue(AltitudeInMeter / 300);                 // Max 18km altitude, every dBm count as 300m and max dBm that can be reported is 60
                                    pwr2 = ValiddBmValue((AltitudeInMeter - (pwr1 * 300)) / 20); // Finer calculations for the second power transmission (if any - depends on user setting) every dBm in this report is 20m. The two reports will be added on the receive side
                                    GadgetData.WSPRData.TXPowerdBm = pwr1;
                                }

                                if (SendWSPRMessage(WSPRMessageTypeToUse) != 0) // Send a WSPR Type 1 or Type 2 message for 1 minute and 50 seconds
                                {
                                    // there was a serial command that interrupted the WSPR Block so go and handle it
                                    return;
                                }
                                if (GadgetData.WSPRData.LocationPrecision == 6) // If higher position precision is set then start a new WSPR tranmission of Type 3
                                {
                                    delay(9000);                                     // wait 9 seconds so we are at the top of an even minute again
                                    if (GadgetData.WSPRData.PowerOption == Altitude) // If Power field should be used for Altitude coding
                                    {
                                        GadgetData.WSPRData.TXPowerdBm = pwr2;
                                    }
                                    if (SendWSPRMessage(3) != 0) // Send a WSPR Type 3 message for 1 minute and 50 seconds
                                    {
                                        // there was a serial command that interrupted the WSPR Block so go and handle it
                                        return;
                                    }
                                }
                                StorePosition(); // Save the current position;
                                if (LastFreq())  // If all bands have been transmitted on then pause for user defined time and after that start over on the first band again
                                {
                                    if ((GadgetData.TXPause > 60) && ((Product_Model == 1017) || (Product_Model == 1028)) && (!PCConnected)) // If the PC is not connected and the TXdelay is longer than a 60 sec then put the MCU to sleep to save current during this long pause (Mini and Pico models only)
                                    {
                                        delay(600);       // Let the serial port send data from its buffer before we go to sleep
                                        Si5351PowerOff(); // Turn off the PLL to save power (Mini Only)
                                        // MCUGoToSleep (GadgetData.TXPause - 10);        //Set MCU in sleep mode until there is 10 seconds left of delay
                                        PowerSaveOFF();   // We are back from sleep - turn on GPS and PLL again
                                        smartdelay(2000); // let the smartdelay routine read a few GPS lines so we can get the new GPS time after our sleep
                                    }
                                    else
                                    {                                            // Regular pause if we did not go to sleep then do a regular pause and send updates to the GUI for the duration
                                        smartdelay(GadgetData.TXPause * 1000UL); // Pause for the time set by the user
                                    }
                                    SendAPIUpdate(UMesWSPRBandCycleComplete); // Inform PC that we have transmitted on the last enabled WSPR band and will start over
                                }
                                GPSWakeUp();
                                NextFreq();                                 // get the frequency for the next HAM band that we will transmit on
                                freq = freq + (100ULL * random(-100, 100)); // modify the TX frequency with a random value beween -100 and +100 Hz to avoid possible lengthy colisions with other users on the band
                                smartdelay(3000);
                            }
                        }
                        else // We have GPS fix but it is not top of even minute so dubble-blink to indicate waiting for top of minute
                        {
                            // SendAPIUpdate(UMesTime);
                            if (GPSS < 57) // Send some nice-to-have info only if the WSPR start is at least 3 seconds away. The last 3 seconds we want to do as little as possible so we can time the start of transmission exactly on the mark
                            {
                                SendAPIUpdate(UMesGPSLock); // Send Locked status
                                SendAPIUpdate(UMesLocator); // Send position
                                SendSatData();              // Send Satellite postion and SNR information to the PC GUI
                            }
                            LEDBlink(2);
                            smartdelay(100);
                        }
                    }
                    else
                    {                                 // Waiting for GPS location fix
                        SendSatData();                // Send Satellite postion and SNR information to the PC GUI while we wait for the GPS location fix
                        LEDBlink(1);                  // singleblink to indicate waiting for GPS Lock
                        SendAPIUpdate(UMesNoGPSLock); // Send No lock status
                        smartdelay(400);
                    }
                } // GPS serial data loop
                LoopGPSNoReceiveCount++;
                if (LoopGPSNoReceiveCount > 60000) // GPS have not sent anything for a long time, GPS is possible in sleep mode or has not started up correctly. This can happen if a brown-out/reboot happens while the GPS was sleeping
                {
                    LoopGPSNoReceiveCount = 0;
                    Serial.println(F("{MIN} Resetting GPS"));
                    GPSReset(); // Try to get GPS going again
                    smartdelay(2000);
                }
            } // Incoming serial command
        }
    }
}

// Transmitt a WSPR message for 1 minute 50 seconds on frequency freq
int SendWSPRMessage(uint8_t WSPRMessageType)
{
    uint8_t i;
    uint8_t Indicator;
    unsigned long startmillis;
    unsigned long endmillis;
    boolean TXEnabled = true;
    int errcode;
    errcode = 0;
    boolean blinked;

    uint8_t *tx_buffer = get_tx_buffer_ptr();
    memset(tx_buffer, 0, get_tx_buffer_size());                                                                                                         // clear WSPR symbol buffer
    wspr_encode(GadgetData.WSPRData.CallSign, GadgetData.WSPRData.MaidenHead4, GadgetData.WSPRData.TXPowerdBm, tx_buffer, WSPRMessageType, GadgetData); // Send a WSPR message for 2 minutes
    // PrintBuffer ('B');
    //  Send WSPR for two minutes
    digitalWrite(StatusLED, HIGH);
    startmillis = millis();
    for (i = 0; i < 162; i++) // 162 WSPR symbols to transmit
    {
        blinked = false;
        endmillis = startmillis + ((i + 1) * (unsigned long)683); // intersymbol delay in WSPR is 682.687 milliseconds (1.4648 baud)
        uint64_t tonefreq;
        tonefreq = freq + ((tx_buffer[i] * 146)); // 146 centiHz (Tone spacing is 1.4648Hz in WSPR)
        if (TXEnabled)
            si5351aSetFrequency(tonefreq, FactoryData.RefFreq);
        // wait untill tone is transmitted for the correct amount of time
        while ((millis() < endmillis) && (!Serial.available()))
            ; // Until time is up or there is serial data received on the control Serial port
        {
            if (!blinked)
            { // do pulsing blinks on Status LED every WSPR symbol to indicate WSPR Beacon transmission
                // Send Status updates to the PC
                Indicator = i;
                Serial.print(F("{TWS} "));
                if (CurrentBand < 10)
                {
                    SerialPrintZero();
                }
                Serial.print(CurrentBand);
                Serial.print(" ");
                if (GadgetData.WSPRData.LocationPrecision == 6)
                {
                    Indicator = Indicator / 2; // If four minutes TX time then halve the indicator value so it will be full after four minutes instead of 2 minutes
                }
                if (WSPRMessageType == 3)
                {
                    Indicator = Indicator + 81; // If this is the second 2 minute transmission then start to from 50%
                }
                if (Indicator < 10)
                {
                    SerialPrintZero();
                }
                if (Indicator < 100)
                {
                    SerialPrintZero();
                }
                Serial.println(Indicator);
                for (int BlinkCount = 0; BlinkCount < 6; BlinkCount++)
                {
                    digitalWrite(StatusLED, HIGH);
                    delay(5);
                    digitalWrite(StatusLED, LOW);
                    delay(50);
                }
                blinked = true;
            }
        }
        if (Serial.available()) // If serialdata was received on Control port then abort and handle command
        {
            errcode = 1;
            break;
        }
    }
    // Switches off Si5351a output
    si5351aOutputOff(SI_CLK0_CONTROL);
    digitalWrite(StatusLED, LOW);

    return errcode;
}

// Maidenhead code from Ossi Väänänen https://ham.stackexchange.com/questions/221/how-can-one-convert-from-lat-long-to-grid-square
void calcLocator(double lat, double lon)
{
    int o1, o2, o3;
    int a1, a2, a3;
    double remainder;
    // longitude
    remainder = lon + 180.0;
    o1 = (int)(remainder / 20.0);
    remainder = remainder - (double)o1 * 20.0;
    o2 = (int)(remainder / 2.0);
    remainder = remainder - 2.0 * (double)o2;
    o3 = (int)(12.0 * remainder);

    // latitude
    remainder = lat + 90.0;
    a1 = (int)(remainder / 10.0);
    remainder = remainder - (double)a1 * 10.0;
    a2 = (int)(remainder);
    remainder = remainder - (double)a2;
    a3 = (int)(24.0 * remainder);
    GadgetData.WSPRData.MaidenHead4[0] = (char)o1 + 'A';
    GadgetData.WSPRData.MaidenHead4[1] = (char)a1 + 'A';
    GadgetData.WSPRData.MaidenHead4[2] = (char)o2 + '0';
    GadgetData.WSPRData.MaidenHead4[3] = (char)a2 + '0';
    GadgetData.WSPRData.MaidenHead4[4] = 0;

    GadgetData.WSPRData.MaidenHead6[0] = (char)o1 + 'A';
    GadgetData.WSPRData.MaidenHead6[1] = (char)a1 + 'A';
    GadgetData.WSPRData.MaidenHead6[2] = (char)o2 + '0';
    GadgetData.WSPRData.MaidenHead6[3] = (char)a2 + '0';
    GadgetData.WSPRData.MaidenHead6[4] = (char)o3 + 'A';
    GadgetData.WSPRData.MaidenHead6[5] = (char)a3 + 'A';
    GadgetData.WSPRData.MaidenHead6[6] = 0;
}

boolean NewPosition() // Returns true if the postion has changed since the last transmission
{
    boolean NewPos = false;
    for (int i = 0; i < GadgetData.WSPRData.LocationPrecision; i++) // Check if the position has changed, test it using either four or six letter Maidenhead precision based on user setting
    {
        if (GadgetData.WSPRData.MaidenHead6[i] != LastMaidenHead6[i])
            NewPos = true;
    }
    return NewPos;
}

void StorePosition() // Saves the current position
{
    for (int i = 0; i < 7; i++)
    {
        LastMaidenHead6[i] = GadgetData.WSPRData.MaidenHead6[i];
    }
}

// Part of the code from the TinyGPS example but here used for the NeoGPS
// Delay loop that checks if the GPS serial port is sending data and in that case passes it of to the GPS object
static void smartdelay(unsigned long delay_ms)
{
    boolean Blink;
    int BlinkCount = 0;

    Blink = (delay_ms > 10000); // If longer than 10 seconds of delay then Blink StatusLED once in a while
    // This custom version of delay() ensures that the gps object
    // is being "fed".
    long TimeLeft;
    unsigned long EndTime = delay_ms + millis();

    do
    {
        while (gps.available(GPSSerial))
            fix = gps.read(); // If GPS data available - process it
        TimeLeft = EndTime - millis();

        if ((TimeLeft > 4000))
        {
            // Send API update
            Serial.print(F("{MPS} "));
            Serial.println(TimeLeft / 1000);
            delay(1000);
            if (Blink)
            {
                BlinkCount++;
                if (BlinkCount > 4) // Blink every 5 seconds
                {
                    LEDBlink(1);
                    BlinkCount = 0;
                }
            }
        }
    } while ((TimeLeft > 0) && (!Serial.available())); // Until time is up or there is serial data received from the computer, in that case end early
    if (delay_ms > 4000)
        Serial.println(F("{MPS} 0")); // When pause is complete send Pause 0 to the GUI so it looks neater. But only if it was at least a four second delay
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

// Returns true if the user has not enabled any bands for TX
boolean NoBandEnabled(void)
{
    boolean NoOne = true;
    for (int FreqLoop = 0; FreqLoop < 13; FreqLoop++)
    {
        if (GadgetData.TXOnBand[FreqLoop])
            NoOne = false;
    }
    return NoOne;
}

// Determine what band to transmit on, cycles upward in the TX enabled bands, e.g if band 2,5,6 and 11 is enbled for TX then the cycle will be 2-5-6-11-2-5-6-11-...
void NextFreq(void)
{
    if (NoBandEnabled())
    {
        freq = 0;
    }
    else
    {
        do
        {
            CurrentBand++;
            if (CurrentBand > 12)
                CurrentBand = 0;
        } while (!GadgetData.TXOnBand[CurrentBand]);

        switch (CurrentBand)
        {
        case 0:
            freq = WSPR_FREQ2190m;
            break;
        case 1:
            freq = WSPR_FREQ630m;
            break;
        case 2:
            freq = WSPR_FREQ160m;
            break;
        case 3:
            freq = WSPR_FREQ80m;
            break;
        case 4:
            freq = WSPR_FREQ40m;
            break;
        case 5:
            freq = WSPR_FREQ30m;
            break;
        case 6:
            freq = WSPR_FREQ20m;
            break;
        case 7:
            freq = WSPR_FREQ17m;
            break;
        case 8:
            freq = WSPR_FREQ15m;
            break;
        case 9:
            freq = WSPR_FREQ12m;
            break;
        case 10:
            freq = WSPR_FREQ10m;
            break;
        case 11:
            freq = WSPR_FREQ6m;
            break;
        case 12:
            freq = WSPR_FREQ4m;
        }
        Serial.print("{TBN} "); // Send API update to inform what band we are using at the moment
        if (CurrentBand < 10)
        {
            SerialPrintZero();
        }
        Serial.println(CurrentBand);
        // We have found what band to use, now pick the right low pass filter for this band
        PickLP(CurrentBand);
    }
}

// Function returns True if the band we just transmitted on was the highest band the user want to transmit on.
boolean LastFreq(void)
{
    boolean Last = true;
    int TestBand;

    TestBand = CurrentBand;
    if (TestBand == 12)
    {
        Last = true;
    }
    else
    {
        do
        {
            TestBand++;
            if (GadgetData.TXOnBand[TestBand])
                Last = false;
        } while (TestBand < 12);
    }
    return Last;
}

// CRC calculation from Christopher Andrews : https://www.arduino.cc/en/Tutorial/EEPROMCrc
// Calculate CRC on either Factory data or Userspace data
unsigned long GetEEPROM_CRC(boolean EEPROMSpace)
{

    const unsigned long crc_table[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};

    unsigned long crc = ~0L;
    int Start;
    int Length;

    if (EEPROMSpace == FactorySpace)
    {
        Start = 400;
        Length = sizeof(FactoryData);
    }
    else
    {
        Start = 0;
        Length = sizeof(GadgetData);
    }
    for (int index = Start; index < (Start + Length); ++index)
    {
        crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
        crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
        crc = ~crc;
    }
    return crc;
}

// Load FactoryData or UserSpace Data from ATMega EEPROM
bool LoadFromEPROM(boolean EEPROMSpace)
{
    int Start;
    int Length;
    unsigned long CRCFromEEPROM, CalculatedCRC;

    if (EEPROMSpace == FactorySpace) // Factory data
    {
        Start = 400;
        Length = sizeof(FactoryData);
        EEPROM.get(Start, FactoryData);              // Load all the data from EEPROM
        CalculatedCRC = GetEEPROM_CRC(FactorySpace); // Calculate the CRC of the data
    }
    else // User data
    {
        Start = 0;
        Length = sizeof(GadgetData);
        EEPROM.get(Start, GadgetData);            // Load all the data from EEPROM
        CalculatedCRC = GetEEPROM_CRC(UserSpace); // Calculate the CRC of the data
    }
    EEPROM.get(Start + Length, CRCFromEEPROM); // Load the saved CRC at the end of the data
    return (CRCFromEEPROM == CalculatedCRC);   // If  Stored and Calculated CRC are the same return true
}

// Save FactoryData or UserSpace Data to Arduino EEPROM
void SaveToEEPROM(boolean EEPROMSpace)
{
    int Start;
    int Length;
    unsigned long CRCFromEEPROM;
    if (EEPROMSpace == FactorySpace)
    {
        Start = 400;
        Length = sizeof(FactoryData);
        EEPROM.put(Start, FactoryData); // Save all the Factory data to EEPROM at adress400
    }
    else // UserSpace
    {
        Start = 0;
        Length = sizeof(GadgetData);
        EEPROM.put(Start, GadgetData); // Save all the User data to EEPROM at adress0
    }
    CRCFromEEPROM = GetEEPROM_CRC(EEPROMSpace); // Calculate CRC on the saved data
    EEPROM.put(Start + Length, CRCFromEEPROM);  // Save the CRC after the data
}

// Brief flash on the Status LED 'Blinks'" number of time
void LEDBlink(int Blinks)
{
    for (int i = 0; i < Blinks; i++)
    {
        digitalWrite(StatusLED, HIGH);
        smartdelay(50);
        digitalWrite(StatusLED, LOW);
        smartdelay(50);
    }
}

// Pulls the correct relays to choose LP filter A,B,C or D
void DriveLPFilters()
{
    if ((Product_Model == 1017) || (Product_Model == 1028))
    {
        // If its the WSPR-TX Mini or Pico then do nothing as they dont have any relays
    }
    else
    {
        SendAPIUpdate(UMesLPF);
        // Product model 1011 E.g WSPR-TX LP1, this will drive the relays on the optional Mezzanine LP4 and Mezzanine BLP4 cards
        if ((Product_Model == 1011) || (Product_Model == 1020) || (Product_Model == 1029))
        {
            switch (CurrentLP)
            {
            case LP_A:
                // all relays are at rest
                digitalWrite(Relay2, LOW);
                digitalWrite(Relay3, LOW);
                break;

            case LP_B:
                digitalWrite(Relay2, HIGH);
                digitalWrite(Relay3, LOW);
                break;

            case LP_C:
                digitalWrite(Relay2, LOW);
                digitalWrite(Relay3, HIGH);
                break;

            case LP_D:
                digitalWrite(Relay2, HIGH);
                digitalWrite(Relay3, HIGH);
                break;

            } // Case
        }     // If Product_Model == 1011
        else
        {
            // is not Product Model 1011 and is Hardware version 1.4 E.g en early model of the Desktop transmitter
            if ((FactoryData.HW_Version == 1) && (FactoryData.HW_Revision == 4)) // Early Hardware has different relay driving
            {
                switch (CurrentLP)
                {
                case LP_A:
                    // all relays are at rest
                    pinMode(Relay1, INPUT); // Set Relay1 as Input to deactivate the relay
                    pinMode(Relay2, INPUT); // Set Relay2 as Input to deactivate the relay
                    pinMode(Relay3, INPUT); // Set Relay3 as Input to deactivate the relay
                    break;

                case LP_B:
                    pinMode(Relay1, OUTPUT); // Set Relay1 as Output so it can be pulled low
                    digitalWrite(Relay1, LOW);
                    pinMode(Relay2, INPUT); // Set Relay2 as Input to deactivate the relay
                    pinMode(Relay3, INPUT); // Set Relay3 as Input to deactivate the relay
                    break;

                case LP_C:
                    pinMode(Relay1, INPUT);  // Set Relay1 as Input to deactivate the relay
                    pinMode(Relay2, INPUT);  // Set Relay2 as Input to deactivate the relay
                    pinMode(Relay3, OUTPUT); // Set Relay3 as Output so it can be pulled low
                    digitalWrite(Relay3, LOW);
                    break;

                case LP_D:
                    pinMode(Relay1, INPUT);  // Set Relay1 as Input to deactivate the relay
                    pinMode(Relay2, OUTPUT); // Set Relay2 as Output so it can be pulled low
                    digitalWrite(Relay2, LOW);
                    pinMode(Relay3, OUTPUT); // Set Relay3 as Output so it can be pulled low
                    digitalWrite(Relay3, LOW);
                    break;
                }
            }
            else
            {
                // Not Product Model 1011 and not Hardvare version 1.4 E.g later model of the Desktop transmitter
                switch (CurrentLP)
                {
                case LP_A:
                    // all relays are at rest
                    digitalWrite(Relay1, LOW);
                    digitalWrite(Relay2, LOW);
                    digitalWrite(Relay3, LOW);
                    break;

                case LP_B:
                    digitalWrite(Relay1, HIGH);
                    digitalWrite(Relay2, LOW);
                    digitalWrite(Relay3, LOW);
                    break;

                case LP_C:
                    digitalWrite(Relay1, LOW);
                    digitalWrite(Relay2, LOW);
                    digitalWrite(Relay3, HIGH);
                    break;

                case LP_D:
                    digitalWrite(Relay1, LOW);
                    digitalWrite(Relay2, HIGH);
                    digitalWrite(Relay3, HIGH);
                    break;
                }
            }
        }
    }
}

// Convert a frequency to a Ham band. Frequency is stored in global variable freq
uint8_t FreqToBand()
{
    uint8_t BandReturn = 15;

    if (freq < (WSPR_FREQ70cm * 1.2))
        BandReturn = 14;
    if (freq < (WSPR_FREQ2m * 1.2))
        BandReturn = 13;
    if (freq < (WSPR_FREQ4m * 1.2))
        BandReturn = 12;
    if (freq < (WSPR_FREQ6m * 1.2))
        BandReturn = 11;
    if (freq < (WSPR_FREQ10m * 1.2))
        BandReturn = 10;
    if (freq < (WSPR_FREQ12m * 1.2))
        BandReturn = 9;
    if (freq < (WSPR_FREQ15m * 1.2))
        BandReturn = 8;
    if (freq < (WSPR_FREQ17m * 1.1))
        BandReturn = 7;
    if (freq < (WSPR_FREQ20m * 1.2))
        BandReturn = 6;
    if (freq < (WSPR_FREQ30m * 1.2))
        BandReturn = 5;
    if (freq < (WSPR_FREQ40m * 1.2))
        BandReturn = 4;
    if (freq < (WSPR_FREQ80m * 1.2))
        BandReturn = 3;
    if (freq < (WSPR_FREQ160m * 1.2))
        BandReturn = 2;
    if (freq < (WSPR_FREQ630m * 1.2))
        BandReturn = 1;
    if (freq < (WSPR_FREQ2190m * 1.2))
        BandReturn = 0;

    return BandReturn;
}

// Out of the four possible LP filters fitted - find the one that is best for Transmission on TXBand
void PickLP(uint8_t TXBand)
{
    boolean ExactMatch = false;
    uint8_t BandLoop;

    // Check if some of the four low pass filters is an exact match for the TXBand
    if (FactoryData.LP_A_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_A;
    }
    if (FactoryData.LP_B_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_B;
    }
    if (FactoryData.LP_C_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_C;
    }
    if (FactoryData.LP_D_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_D;
    }

    // If we did not find a perfect match then use a low pass filter that is higher in frequency.
    if (!ExactMatch)
    {
        for (BandLoop = TXBand; BandLoop < 99; BandLoop++) // Test all higher bands to find a a possible LP filter in one of the four LP banks
        {
            if (FactoryData.LP_A_BandNum == BandLoop) // The LP filter in Bank A is a match for this band
            {
                CurrentLP = LP_A;
                break;
            }
            if (FactoryData.LP_B_BandNum == BandLoop) // The LP filter in Bank B is a match for this band
            {
                CurrentLP = LP_B;
                break;
            }
            if (FactoryData.LP_C_BandNum == BandLoop) // The LP filter in Bank C is a match for this band
            {
                CurrentLP = LP_C;
                break;
            }
            if (FactoryData.LP_D_BandNum == BandLoop) // The LP filter in Bank D is a match for this band
            {
                CurrentLP = LP_D;
                break;
            }
        }
        // If there is no LP that is higher than TXBand then use the highest one, (not ideal as output will be attenuated but best we can do)
        if (BandLoop == 99)
        {
            TXBand = BandNumOfHigestLP();
            if (FactoryData.LP_A_BandNum == TXBand)
            {
                CurrentLP = LP_A;
            }
            if (FactoryData.LP_B_BandNum == TXBand)
            {
                CurrentLP = LP_B;
            }
            if (FactoryData.LP_C_BandNum == TXBand)
            {
                CurrentLP = LP_C;
            }
            if (FactoryData.LP_D_BandNum == TXBand)
            {
                CurrentLP = LP_D;
            }
        }
    }
    DriveLPFilters();
}

// Returns a band that is the highest band that has a LP filter fitted onboard.
// Low pass filter numbering corresponds to Bands or two special cases
// The special cases are: 98=just a link between input and output, 99=Nothing fitted (open circut) the firmware will never use this
// These numbers are set by the factory Configuration program and stored in EEPROM
uint8_t BandNumOfHigestLP()
{
    uint8_t BandLoop, Result;
    Result = FactoryData.LP_A_BandNum; // Use this filter if nothing else is a match.
    // Find the highest band that has a Low Pass filter fitted in one of the four LP banks
    for (BandLoop = 98; BandLoop > 0; BandLoop--)
    {
        if (FactoryData.LP_A_BandNum == BandLoop) // The LP filter in Bank A is a match for this band
        {
            Result = FactoryData.LP_A_BandNum;
            break;
        }
        if (FactoryData.LP_B_BandNum == BandLoop) // The LP filter in Bank B is a match for this band
        {
            Result = FactoryData.LP_B_BandNum;
            break;
        }
        if (FactoryData.LP_C_BandNum == BandLoop) // The LP filter in Bank C is a match for this band
        {
            Result = FactoryData.LP_C_BandNum;
            break;
        }
        if (FactoryData.LP_D_BandNum == BandLoop) // The LP filter in Bank D is a match for this band
        {
            Result = FactoryData.LP_D_BandNum;
            break;
        }
    }
    return Result;
}

// Arduino voltmeter from TINKER : https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
// Retun VCC voltage measured in milliVolt
// So 5000 is 5V, 3300 is 3.3V.
int GetVCC()
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2);            // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC))
        ; // measuring

    uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result;              // Vcc in millivolts
}

void PowerSaveOFF()
{
    GPSWakeUp();
    Si5351PowerOn();
}

void PowerSaveON()
{
    GPSGoToSleep();
    Si5351PowerOff();
}

void GPSGoToSleep()
{
    switch (Product_Model)
    {
    case 1017: // Mini
        // If its the WSPR-TX Mini, send the Sleep string to it
        GPSSerial.println(F("$PMTK161,0*28"));
        // GPSSleep = true;
        break;

    case 1028: // Pico
        // If it is the WSPR-TX Pico it has a hardware line for sleep/wake
        pinMode(GPSPower, OUTPUT);
        digitalWrite(GPSPower, LOW);
        break;
    }
}

void GPSWakeUp()
{
    switch (Product_Model)
    {
    case 1017: // Mini
        // Send anything on the GPS serial line to wake it up
        GPSSerial.println(" ");
        // GPSSleep = false;
        delay(100); // Give the GPS some time to wake up and send its serial data back to us
        break;

    case 1028: // Pico
        // If it is the WSPR-TX Pico it has a hardware line for sleep/wake
        pinMode(GPSPower, OUTPUT);
        digitalWrite(GPSPower, HIGH);
        delay(200);
        pinMode(GPSPower, INPUT);
        delay(200);
        // Send GPS reset string
        // GPSSerial.println(F("$PCAS10,3*1F"));
        // Airborne Mode
        // GPSSerial.println(F("$PCAS11,5*18"));
        break;
    }
}

void GPSReset()
{
    GPSWakeUp();
    // Send GPS reset string
    GPSSerial.println(F("$PCAS10,3*1F"));
}

/*
  //Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
  void MCUGoToSleep( int SleepTime)//Sleep time in seconds, accurate to the nearest 8 seconds
  {
  int SleepLoop;
  SleepLoop = SleepTime / 8.8 ; // every sleep period is 8.8 seconds
  GPSSerial.end();//Must turn off software serialport or sleep will not work
  //Serial.end(); //Turn off Hardware serial port as well as we will temporary change all ports to outputs
  AllIOtoLow ();  //Set all IO pins to outputs to save power
  DisableADC ();  //Turn off ADC to save power

  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1 << 6); //enable interrupt mode

  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
  for (int i = 0; i < SleepLoop; i++)//sleep for eight second intervals untill SleepTime is reached
  {
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
    //Just woke upp after 8 seconds of sleep, do a short blink to indicate that I'm still running
    digitalWrite(StatusLED, HIGH);
    delay (30);
    digitalWrite(StatusLED, LOW);
  }
  //Restore everything
  EnableADC ();
  GPSSerial.begin(9600); //Init software serial port to communicate with the on-board GPS module
  }

  //Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
  void AllIOtoLow ()
  {
  //  Save Power by setting all IO pins to outputs and setting them either low or high
  // (for some odd reason the ATMEga328 takes less power when this is done instead of having IO pins as inputs during sleep, see more in Kevin Darrahs YouTube Videos)
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);

  pinMode(A6, OUTPUT);
  digitalWrite(A6, LOW);

  pinMode(A7, OUTPUT);
  digitalWrite(A7, LOW);

  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);

  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

  }

  void DisableADC ()
  {
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);
  }

  void EnableADC ()
  {
  //Enable ADC again
  ADCSRA |= (1 << 7);
  }
*/

void SerialPrintZero()
{
    Serial.print("0");
}

// Sends the Sattelite data like Elevation, Azimuth SNR and ID using the Serial API {GSI} format
void SendSatData()
{
    uint8_t SNR;
    for (uint8_t i = 0; i < gps.sat_count; i++)
    {
        Serial.print(F("{GSI} "));
        if (gps.satellites[i].id < 10)
        {
            SerialPrintZero();
        }
        Serial.print(gps.satellites[i].id);
        Serial.print(" ");
        if (gps.satellites[i].azimuth < 100)
        {
            SerialPrintZero();
        }
        if (gps.satellites[i].azimuth < 10)
        {
            SerialPrintZero();
        }
        Serial.print(gps.satellites[i].azimuth);
        Serial.print(" ");
        if (gps.satellites[i].elevation < 10)
        {
            SerialPrintZero();
        }
        Serial.print(gps.satellites[i].elevation);
        Serial.print((" "));
        SNR = 0;
        if (gps.satellites[i].tracked)
        {
            SNR = gps.satellites[i].snr;
        }
        else
        {
            SNR = 0;
        }
        if (SNR < 10)
        {
            SerialPrintZero();
        }
        Serial.println(SNR);
    }
    Serial.println();
} // displaySatellitesInView

/*
  //Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
  ISR(WDT_vect) {
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
  }// watchdog interrupt


*/
// Original WSPR code by NT7S - Jason Milldrum https://github.com/etherkit/JTEncode and Bo Hansen - OZ2M RFZero https://rfzero.net
// Modifed for Type2 and Type3 messages by SM7PNV Harry Zachrisson https://github.com/HarrydeBug

/*
   wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)

   Takes an arbitrary message of up to 13 allowable characters and returns

   call - Callsign (6 characters maximum).
   loc - Maidenhead grid locator (4 charcters maximum).
   dbm - Output power in dBm.
   symbols - Array of channel symbols to transmit retunred by the method.
   Ensure that you pass a uint8_t array of size WSPR_SYMBOL_COUNT to the method.

*/

// Only transmit on specific times
boolean CorrectTimeslot()
{
    boolean CorrectSlot = false;
    uint8_t TestMinute;
    uint8_t ScheduleLenght;
    uint8_t SlotCode;
    TestMinute = GPSM;         // Test the Minute variable from the GPS time
    if ((TestMinute % 2) == 0) // First check that it an even minute as WSPR transmissions only start on even minute
    {
        if (GadgetData.WSPRData.TimeSlotCode == 17) // Tracker mode, only transmit when the transmitter is moving
        {
            CorrectSlot = (NewPosition() || (TestMinute == 0)); // Transmit only if the tracker has moved since last transmisson or at top of an Hour
        }
        else if (GadgetData.WSPRData.TimeSlotCode == 16) // No scheduling
        {
            CorrectSlot = true;
        }
        else if (GadgetData.WSPRData.TimeSlotCode == 15) // Band coordinated scheduling
        {
            switch (CurrentBand)
            {
            case 2: // 160m band
                CorrectSlot = (TestMinute == 0 || TestMinute == 20 || TestMinute == 40);
                break;
            case 3: // 80m band
                CorrectSlot = (TestMinute == 2 || TestMinute == 22 || TestMinute == 42);
                break;
            case 4: // 40m band
                CorrectSlot = (TestMinute == 6 || TestMinute == 26 || TestMinute == 46);
                break;
            case 5: // 30m band
                CorrectSlot = (TestMinute == 8 || TestMinute == 28 || TestMinute == 48);
                break;
            case 6: // 20m band
                CorrectSlot = (TestMinute == 10 || TestMinute == 30 || TestMinute == 50);
                break;
            case 7: // 17m band
                CorrectSlot = (TestMinute == 12 || TestMinute == 32 || TestMinute == 52);
                break;
            case 8: // 15m band
                CorrectSlot = (TestMinute == 14 || TestMinute == 34 || TestMinute == 54);
                break;
            case 9: // 12m band
                CorrectSlot = (TestMinute == 16 || TestMinute == 36 || TestMinute == 56);
                break;
            case 10: // 10m band
                CorrectSlot = (TestMinute == 18 || TestMinute == 38 || TestMinute == 58);
                break;
            default:
                CorrectSlot = true; // band does not have schedule, allow it to transmit right now, this applies to 1290m,630m and all bands above 10m
                break;
            }                                           // switch
        }                                               // else if
        else if (GadgetData.WSPRData.TimeSlotCode < 15) // Schedule is on the minute set on Timeslotcode * 2  E.g if Timeslotcode is 3 then minute 06,16,26,36,46 and 56 is used for transmissions.
        {
            if (GadgetData.WSPRData.TimeSlotCode < 5)
            {
                ScheduleLenght = 10;
                SlotCode = GadgetData.WSPRData.TimeSlotCode;
            }
            else
            {
                ScheduleLenght = 20;
                SlotCode = GadgetData.WSPRData.TimeSlotCode - 5;
            }  // if TimeSlotCode <5
            do // Remove the ten minute digit and just leave the minute digit
            {
                if (TestMinute > ScheduleLenght - 1)
                    TestMinute = TestMinute - ScheduleLenght;
            } while (TestMinute > ScheduleLenght - 1);
            CorrectSlot = (TestMinute == (SlotCode * 2)); // if the TimeSlotcode multiplied with 2 (only even minutes) is matching the current minute digit then transmit
        }                                                 // else if
    }
    return CorrectSlot;
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
                    calcLocator(fix.latitude(), fix.longitude());
                }
                SendAPIUpdate(UMesLocator);
            }
            else
            {
                SendAPIUpdate(UMesNoGPSLock);
            }
        }
        smartdelay(200);
    }
    LoopGPSNoReceiveCount++;
    if (LoopGPSNoReceiveCount > 60000) // GPS have not sent anything for a long time, GPS is possible in sleep mode or has not started up correctly. This can happen if a brown-out/reboot happens while the GPS was sleeping
    {
        LoopGPSNoReceiveCount = 0;
        Serial.println(F("{MIN} Resetting GPS"));
        GPSWakeUp(); // Try to get GPS going again by sending wake up command
        smartdelay(2000);
    }
}