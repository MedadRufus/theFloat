#ifndef __datatypes__
#define __datatypes__

#include "Arduino.h"

enum E_SufixPreFixOption
{
    Sufix,
    Prefix,
    None
};

enum E_PowerOption
{
    Normal,
    Altitude
};

enum E_LocatorOption
{
    Manual,
    GPS
};

struct S_WSPRData
{
    char CallSign[7];                   // Radio amateur Call Sign, zero terminated string can be four to six char in length + zero termination
    E_SufixPreFixOption SuPreFixOption; // If a suffix or Prefix to the Callsign is used or not
    char Prefix[4];                     // Prefix three chars max and a zero termination
    uint8_t Sufix;                      // Sufix code in WSPR format, e.g single digit is 0-9, single char (A to Z) is 10-35, double digits (10-99) is 36-125
    E_LocatorOption LocatorOption;      // If transmitted Maidenhead locator is based of GPS location or if it is using MaidneHead4 variable.
    uint8_t LocationPrecision;          // Determines if a second Type 3 transmission will be sent to increase the postiton precision.
    char MaidenHead4[5];                // Maidenhead locator with four characters and a zero termination
    char MaidenHead6[7];                // Maidenhead locator with six characters and a zero termination
    E_PowerOption PowerOption;          // If transmitted Power is based on TXPowerdBm field or is calculated from GPS Altitude.
    uint8_t TXPowerdBm;                 // Power data in dBm min=0 max=60
    uint8_t TimeSlotCode;               // Determine on what time slot a tranmsission will be made. If TimeslotCode is 0 to 4 a ten minute scheduled transmission will be used.
    // 0=TX om minute 0,10,20.. 1=TX on minute 2,12,22..  ..4=TX om minute 08,18,28 etc. If Timeslotcode is 5 to 14 a twenty minute schedule code will be used.
    // 5=TX on minute 0,20,40.  6=TX on minute 2,22,42.  ..14=TX on minute 18,38,58.
    // if the TimeslotCode is 15 a special band coordinated schedule is used.
    // If the TimeslotCode is 16 then no schedule is used, e.g transmission can occur at any time
    // If the TimeslotCode is 17 then transmisson will only occur if GPS derived Maidenhead position has been updated since last transmission. E.g it becomes a tracker that only transmits position updates.
};

enum E_Mode
{
    WSPRBeacon,
    SignalGen,
    Idle
};

struct S_GadgetData
{
    char Name[40];          // Optional Name of the device.
    E_Mode StartMode;       // What mode the Gadget should go to after boot.
    S_WSPRData WSPRData;    // Data needed to transmit a WSPR packet.
    bool TXOnBand[16];      // Arraycount corresponds to the Enum E_Band, True =Transmitt Enabled, False = Transmitt disabled on this band
    unsigned long TXPause;  // Number of seconds to pause after having transmitted on all enabled bands.
    uint64_t GeneratorFreq; // Frequency for when in signal Generator mode. Freq in centiHertz.
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

#endif
