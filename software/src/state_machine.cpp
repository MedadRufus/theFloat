#include "state_machine.hpp"
#include "string_operations.hpp"
#include "defines.hpp"
#include "datatypes.hpp"
#include "print_operations.hpp"
#include "eeprom.hpp"
#include <NMEAGPS.h>

extern E_Mode CurrentMode;        // TODO: replace with getters and setters
extern S_FactoryData FactoryData; // TODO: replace with getters and setters
extern S_GadgetData GadgetData;   // TODO: replace with getters and setters

extern int GPSH;          // GPS Hours
extern int GPSM;          // GPS Minutes
extern int GPSS;          // GPS Seconds
extern int fixstate;      // GPS Fix state-machine. 0=Init, 1=wating for fix,2=fix accuired
extern uint8_t CurrentLP; // Keep track on what Low Pass filter is currently switched in
extern uint64_t freq;     // Holds the Output frequency when we are in signal generator mode or in WSPR mode
extern gps_fix fix;       // This holds on to the latest values

void SendAPIUpdate(uint8_t UpdateType)
{
    switch (UpdateType)
    {
    case UMesCurrentMode:

        Serial.print(F("{CCM} "));
        switch (CurrentMode)
        {
        case Idle:
            Serial.println(("N"));
            Serial.println(F("{TON} F")); // Also Send TX Off info
            break;
        case WSPRBeacon:
            Serial.println(("W"));
            Serial.println(F("{TON} F")); // Also send TX Off info, WSPR routine will change this if it currently transmitting
            break;
        case SignalGen:
            Serial.println(("S"));
            Serial.println(F("{TON} T")); // Also Send TX ON info
            break;
        }
        break;

    case UMesLocator:
        Serial.print(F("{GL4} "));
        Serial.println(GadgetData.WSPRData.MaidenHead4);
        Serial.print(F("{GL6} "));
        Serial.println(GadgetData.WSPRData.MaidenHead6);
        /* Serial.print("{MIN} MH6=");
         Serial.println (GadgetData.WSPRData.MaidenHead6);
         Serial.print("{MIN} StoredMH6=");
         Serial.println (LastMaidenHead6);
         Serial.print("{MIN} NewPos=");
         if (NewPosition())
         {
           Serial.println("True");
         }
         else
          {
           Serial.println("False");
         }
         */
        break;

    case UMesTime:
        GPSH = fix.dateTime.hours;
        GPSM = fix.dateTime.minutes;
        GPSS = fix.dateTime.seconds;
        Serial.print(F("{GTM} "));
        if (GPSH < 10)
        {
            SerialPrintZero();
        }
        Serial.print(GPSH);
        Serial.print(":");
        if (GPSM < 10)
        {
            SerialPrintZero();
        }
        Serial.print(GPSM);
        Serial.print(":");
        if (GPSS < 10)
        {
            SerialPrintZero();
        }
        Serial.println(GPSS);
        break;

    case UMesGPSLock:
        Serial.println(F("{GLC} T"));
        break;

    case UMesNoGPSLock:
        Serial.println(F("{GLC} F"));
        break;

    case UMesFreq:
        Serial.print(F("{TFQ} "));
        Serial.println(uint64ToStr(freq, false));
        break;

    case UMesTXOn:
        Serial.println(F("{TON} T"));
        break;

    case UMesTXOff:
        Serial.println(F("{TON} F"));
        break;

    case UMesWSPRBandCycleComplete:
        Serial.println(F("{TCC}"));
        break;

    case UMesVCC:
        Serial.print(F("{MVC} "));
        Serial.println(GetVCC());
        break;

    case UMesLPF:
        Serial.print(F("{LPI} "));
        if (CurrentLP == LP_A)
        {
            Serial.println("A");
        }
        if (CurrentLP == LP_B)
        {
            Serial.println("B");
        }
        if (CurrentLP == LP_C)
        {
            Serial.println("C");
        }
        if (CurrentLP == LP_D)
        {
            Serial.println("D");
        }
    }
}

// Serial API commands and data decoding
void DecodeSerialCMD(const char *InputCMD, S_GadgetData GadgetData)
{
    char CharInt[13];
    bool EnabDisab;
    uint32_t i;
    if ((InputCMD[0] == '[') && (InputCMD[4] == ']'))
    { // A Command,Option or Data input

        if (InputCMD[1] == 'C')
        { // Commmand

            // Current Mode
            if ((InputCMD[2] == 'C') && (InputCMD[3] == 'M'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    if (InputCMD[8] == 'S')
                    {
                        DoSignalGen();
                    }
                    if (InputCMD[8] == 'W')
                    {
                        // CurrentBand = 0;
                        DoWSPR();
                    }
                    if (InputCMD[8] == 'N')
                    {
                        DoIdle();
                    }
                }    // Set Current Mode
                else // Get
                {
                    SendAPIUpdate(UMesCurrentMode);
                } // Get Current Mode
            }     // CurrentMode

            // Store Current configuration data to EEPROM
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    SaveToEEPROM(UserSpace);
                    Serial.println(F("{MIN} Configuration saved"));
                }
            }

            // Set Low Pass filter (LP filters are automatically set by the WSPR Beacon and Signal Gen. routines but can be temporarily overrided by this command for testing purposes)
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'L'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    if (InputCMD[8] == 'A')
                    {
                        CurrentLP = 0;
                    }
                    if (InputCMD[8] == 'B')
                    {
                        CurrentLP = 1;
                    }
                    if (InputCMD[8] == 'C')
                    {
                        CurrentLP = 2;
                    }
                    if (InputCMD[8] == 'D')
                    {
                        CurrentLP = 3;
                    }
                    DriveLPFilters();
                }
            }
        }

        if (InputCMD[1] == 'O')
        { // Option

            // TX Pause
            if ((InputCMD[2] == 'T') && (InputCMD[3] == 'P'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = InputCMD[10];
                    CharInt[3] = InputCMD[11];
                    CharInt[4] = InputCMD[12];
                    CharInt[5] = 0;
                    // GadgetData.TXPause = atoi(CharInt);
                    GadgetData.TXPause = StrTouint64_t(CharInt);
                }
                else // Get Option
                {
                    Serial.print(F("{OTP} "));
                    if (GadgetData.TXPause < 10000)
                    {
                        SerialPrintZero();
                    }
                    if (GadgetData.TXPause < 1000)
                    {
                        SerialPrintZero();
                    }
                    if (GadgetData.TXPause < 100)
                    {
                        SerialPrintZero();
                    }
                    if (GadgetData.TXPause < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(GadgetData.TXPause);
                }
            } // TX Pause

            // StartMode [OSM]
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'M'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    if (InputCMD[8] == 'S')
                    {
                        GadgetData.StartMode = SignalGen;
                    }
                    if (InputCMD[8] == 'W')
                    {
                        GadgetData.StartMode = WSPRBeacon;
                    }
                    if (InputCMD[8] == 'N')
                    {
                        GadgetData.StartMode = Idle;
                    }
                }    // Set Start Mode
                else // Get
                {
                    Serial.print(F("{OSM} "));
                    switch (GadgetData.StartMode)
                    {
                    case Idle:
                        Serial.println(("N"));
                        break;
                    case WSPRBeacon:
                        Serial.println(("W"));
                        break;
                    case SignalGen:
                        Serial.println(("S"));
                        break;
                    }
                } // Get Start Mode
            }     // StartMode

            // Band TX enable
            if ((InputCMD[2] == 'B') && (InputCMD[3] == 'D'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = 0;
                    CharInt[3] = 0; // What band to set/clear
                    EnabDisab = false;
                    if (InputCMD[11] == 'E')
                        EnabDisab = true;
                    GadgetData.TXOnBand[atoi(CharInt)] = EnabDisab; // Enable or disable on this band
                }                                                   // Set Band TX enable
                else                                                // Get
                {
                    // Get Option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = 0;
                    CharInt[3] = 0; // What band is requested
                    Serial.print(F("{OBD} "));
                    i = atoi(CharInt);
                    if (i < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.print(i);
                    if (GadgetData.TXOnBand[i])
                    {
                        Serial.println((" E"));
                    }
                    else
                    {
                        Serial.println((" D"));
                    }
                } // Get Band TX enable
            }     // Band TX enable

            // Location Option
            if ((InputCMD[2] == 'L') && (InputCMD[3] == 'C'))
            {
                if (InputCMD[6] == 'S')
                { // Set Location Option
                    if (InputCMD[8] == 'G')
                    {
                        GadgetData.WSPRData.LocatorOption = GPS;
                        Serial.println(F("{OLC G} "));            // Echo back setting
                        if (fix.valid.location && fix.valid.time) // If position is known then send it to the PC
                        {
                            GPSH = fix.dateTime.hours;
                            GPSM = fix.dateTime.minutes;
                            GPSS = fix.dateTime.seconds;
                            calcLocator(fix.latitude(), fix.longitude(), &GadgetData.WSPRData);
                            Serial.print(F("{DL4} "));
                            Serial.println(GadgetData.WSPRData.MaidenHead4);
                            Serial.print(F("{DL6} "));
                            Serial.println(GadgetData.WSPRData.MaidenHead6);
                        }
                    }
                    if (InputCMD[8] == 'M')
                    {
                        GadgetData.WSPRData.LocatorOption = Manual;
                    }
                }    // Set Location Option
                else // Get Location Option
                {
                    Serial.print(F("{OLC} "));
                    if (GadgetData.WSPRData.LocatorOption == GPS)
                    {
                        Serial.println(("G"));
                    }
                    else
                    {
                        Serial.println(("M"));
                    }
                } // Get Location Option
            }     // Location Option

            // Locator Precision [OLP]
            if ((InputCMD[2] == 'L') && (InputCMD[3] == 'P'))
            {
                if (InputCMD[6] == 'S')
                { // Set Locator Precision
                    if (InputCMD[8] == '6')
                    {
                        GadgetData.WSPRData.LocationPrecision = 6;
                    }
                    else
                    {
                        GadgetData.WSPRData.LocationPrecision = 4;
                    }
                    // Echo back setting
                    Serial.print(F("{OLP} "));
                    Serial.println(GadgetData.WSPRData.LocationPrecision);
                }    // Set Locator Precision
                else // Get Locator Precision
                {
                    Serial.print(F("{OLP} "));
                    Serial.println(GadgetData.WSPRData.LocationPrecision);
                } // Get Locator Precision
            }     // Locator Precision

            // Power encoding Option
            if ((InputCMD[2] == 'P') && (InputCMD[3] == 'W'))
            {
                if (InputCMD[6] == 'S')
                { // Set Location Option
                    if (InputCMD[8] == 'N')
                    {
                        GadgetData.WSPRData.PowerOption = Normal;
                    }
                    if (InputCMD[8] == 'A')
                    {
                        GadgetData.WSPRData.PowerOption = Altitude;
                    }
                }    // Set Power Encoding Option
                else // Get Location Option
                {
                    Serial.print(F("{OPW} "));
                    if (GadgetData.WSPRData.PowerOption == Normal)
                    {
                        Serial.println(("N"));
                    }
                    else
                    {
                        Serial.println(("A"));
                    }
                } // Get Power Encoding Option
            }     // Power encoding Option

            // Time slot  [OTS]
            if ((InputCMD[2] == 'T') && (InputCMD[3] == 'S'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = 0;
                    CharInt[3] = 0;
                    GadgetData.WSPRData.TimeSlotCode = atoi(CharInt);
                }
                else // Get
                {
                    Serial.print(F("{OTS} "));
                    if (GadgetData.WSPRData.TimeSlotCode < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(GadgetData.WSPRData.TimeSlotCode);
                }
            } // Time slot

            // PreFix/Sufix [OPS]
            if ((InputCMD[2] == 'P') && (InputCMD[3] == 'S'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    if (InputCMD[8] == 'P')
                    {
                        GadgetData.WSPRData.SuPreFixOption = Prefix;
                    }
                    if (InputCMD[8] == 'S')
                    {
                        GadgetData.WSPRData.SuPreFixOption = Sufix;
                    }
                    if (InputCMD[8] == 'N')
                    {
                        GadgetData.WSPRData.SuPreFixOption = None;
                    }
                }    // Set Start Mode
                else // Get
                {
                    Serial.print(F("{OPS} "));
                    switch (GadgetData.WSPRData.SuPreFixOption)
                    {
                    case Prefix:
                        Serial.println(("P"));
                        break;
                    case Sufix:
                        Serial.println(("S"));
                        break;
                    case None:
                        Serial.println(("N"));
                        break;
                    }
                } // Get Start Mode
            }     // StartMode

        } // All Options

        // Data
        if (InputCMD[1] == 'D')
        {

            // Callsign [DCS]
            if ((InputCMD[2] == 'C') && (InputCMD[3] == 'S'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 5; i++)
                    {
                        GadgetData.WSPRData.CallSign[i] = InputCMD[i + 8];
                    }
                    GadgetData.WSPRData.CallSign[6] = 0;
                }
                else // Get
                {
                    Serial.print(F("{DCS} "));
                    Serial.println(GadgetData.WSPRData.CallSign);
                }
            } // Callsign

            // Callsign Sufix [DSF]
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'F'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = InputCMD[10];
                    CharInt[3] = 0;
                    GadgetData.WSPRData.Sufix = atoi(CharInt);
                }
                else // Get
                {
                    Serial.print(F("{DSF} "));
                    if (GadgetData.WSPRData.Sufix < 100)
                    {
                        SerialPrintZero();
                    }
                    if (GadgetData.WSPRData.Sufix < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(GadgetData.WSPRData.Sufix);
                }
            } // Callsign Sufix

            // Callsing Prefix  [DPF]
            if ((InputCMD[2] == 'P') && (InputCMD[3] == 'F'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 2; i++)
                    {
                        GadgetData.WSPRData.Prefix[i] = InputCMD[i + 8];
                    }
                    GadgetData.WSPRData.Prefix[3] = 0;
                }
                else // Get
                {
                    Serial.print(F("{DPF} "));
                    Serial.println(GadgetData.WSPRData.Prefix);
                }
            } // Callsign Prefix

            // Locator 4
            if ((InputCMD[2] == 'L') && (InputCMD[3] == '4'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 3; i++)
                    {
                        GadgetData.WSPRData.MaidenHead4[i] = InputCMD[i + 8];
                    }
                    GadgetData.WSPRData.MaidenHead4[4] = 0;
                }
                else // Get
                {
                    Serial.print(F("{DL4} "));
                    Serial.println(GadgetData.WSPRData.MaidenHead4);
                }
            } // Locator 4

            // Locator 6
            if ((InputCMD[2] == 'L') && (InputCMD[3] == '6'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 5; i++)
                    {
                        GadgetData.WSPRData.MaidenHead6[i] = InputCMD[i + 8];
                    }
                    GadgetData.WSPRData.MaidenHead6[6] = 0;
                }
                else // Get
                {
                    Serial.print(F("{DL6} "));
                    Serial.println(GadgetData.WSPRData.MaidenHead6);
                }
            } // Locator 6

            // Name
            if ((InputCMD[2] == 'N') && (InputCMD[3] == 'M'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 38; i++)
                    {
                        GadgetData.Name[i] = InputCMD[i + 8];
                    }
                    GadgetData.Name[39] = 0;
                }
                else // Get
                {
                    Serial.print(F("{DNM} "));
                    Serial.println(GadgetData.Name);
                }
            } // Name

            // Power data
            if ((InputCMD[2] == 'P') && (InputCMD[3] == 'D'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = 0;
                    CharInt[3] = 0;
                    GadgetData.WSPRData.TXPowerdBm = atoi(CharInt);
                }
                else // Get
                {
                    Serial.print(F("{DPD} "));
                    if (GadgetData.WSPRData.TXPowerdBm < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(GadgetData.WSPRData.TXPowerdBm);
                }
            } // Power Data

            // Generator Frequency
            if ((InputCMD[2] == 'G') && (InputCMD[3] == 'F'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 11; i++)
                    {
                        CharInt[i] = InputCMD[i + 8];
                    }
                    CharInt[12] = 0;
                    GadgetData.GeneratorFreq = StrTouint64_t(CharInt);
                    if (CurrentMode == SignalGen)
                    {
                        DoSignalGen();
                    }
                }
                else // Get
                {
                    Serial.print(F("{DGF} "));
                    Serial.println(uint64ToStr(GadgetData.GeneratorFreq, true));
                }
            } // Generator Frequency

        } // Data

        // Factory data
        if (InputCMD[1] == 'F')
        {

            // Product model Number
            if ((InputCMD[2] == 'P') && (InputCMD[3] == 'N'))
            {
                if (InputCMD[6] == 'G')
                { // Get option
                    Serial.print(F("{FPN} "));
                    if (Product_Model < 10000)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(Product_Model);
                }
            } // Product model Number

            // Hardware Version
            if ((InputCMD[2] == 'H') && (InputCMD[3] == 'V'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = InputCMD[10];
                    CharInt[3] = 0;
                    FactoryData.HW_Version = atoi(CharInt);
                }    // Set
                else // Get Option
                {
                    Serial.print(F("{FHV} "));
                    if (FactoryData.HW_Version < 100)
                    {
                        SerialPrintZero();
                    }
                    if (FactoryData.HW_Version < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(FactoryData.HW_Version);
                }
            } // Hardware Version

            // Hardware Revision
            if ((InputCMD[2] == 'H') && (InputCMD[3] == 'R'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[8];
                    CharInt[1] = InputCMD[9];
                    CharInt[2] = InputCMD[10];
                    CharInt[3] = 0;
                    FactoryData.HW_Revision = atoi(CharInt);
                    Serial.println(' ');
                }    // Set
                else // Get Option
                {
                    Serial.print(F("{FHR} "));
                    if (FactoryData.HW_Revision < 100)
                    {
                        SerialPrintZero();
                    }
                    if (FactoryData.HW_Revision < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(FactoryData.HW_Revision);
                }
            } // Hardware Revision

            // Software Version
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'V'))
            {
                if (InputCMD[6] == 'G')
                { // Get option
                    Serial.print(F("{FSV} "));
                    if (SoftwareVersion < 100)
                    {
                        SerialPrintZero();
                    }
                    if (SoftwareVersion < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(SoftwareVersion);
                }
            } // Software Version

            // Software Revision
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'R'))
            {
                if (InputCMD[6] == 'G')
                { // Get option
                    Serial.print(F("{FSR} "));
                    if (SoftwareRevision < 100)
                    {
                        SerialPrintZero();
                    }
                    if (SoftwareRevision < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(SoftwareRevision);
                }
            } // Software Revision

            // Low pass filter config
            if ((InputCMD[2] == 'L') && (InputCMD[3] == 'P'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    CharInt[0] = InputCMD[10];
                    CharInt[1] = InputCMD[11];
                    CharInt[2] = 0;
                    switch (InputCMD[8])
                    {
                    case 'A':
                        FactoryData.LP_A_BandNum = atoi(CharInt);
                        break;
                    case 'B':
                        FactoryData.LP_B_BandNum = atoi(CharInt);
                        break;
                    case 'C':
                        FactoryData.LP_C_BandNum = atoi(CharInt);
                        break;
                    case 'D':
                        FactoryData.LP_D_BandNum = atoi(CharInt);
                        break;
                    }

                }    // Set
                else // Get Option
                {
                    // If Hardvare is V1 R10 and higher on LP1 and Desktop it has some filters that can do more than one band, indicate by sending out these extra bands to the PC config software
                    // The same goes for the Pico and the LP1 with Mezzanine BLP4 regardless of hardware version
                    // The PC will indicate these bands with the little green square in the GUI
                    if (((Product_Model == 1012) & (FactoryData.HW_Version == 1) & (FactoryData.HW_Revision > 9)) || (Product_Model == 1028) || (Product_Model == 1029))
                    {
                        // If 10m LP filter is fitted then indicate it can do 15m and 12m as well
                        if ((FactoryData.LP_A_BandNum == 10) || (FactoryData.LP_B_BandNum == 10) || (FactoryData.LP_C_BandNum == 10) || (FactoryData.LP_D_BandNum == 10))
                        {
                            Serial.println(F("{FLP} A 09")); // Indicate 12m band
                            Serial.println(F("{FLP} A 08")); // Indicate 15m band
                            Serial.println(F("{FLP} A 07")); // Indicate 17m band
                        }
                        // If 20m LP filter is fitted then indicate it can do 30m as well
                        if ((FactoryData.LP_A_BandNum == 6) || (FactoryData.LP_B_BandNum == 6) || (FactoryData.LP_C_BandNum == 6) || (FactoryData.LP_D_BandNum == 6))
                        {
                            Serial.println(F("{FLP} A 05")); // Indicate 30m band
                        }
                    }
                    Serial.print(F("{FLP} A "));
                    if (FactoryData.LP_A_BandNum < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(FactoryData.LP_A_BandNum);
                    Serial.print(F("{FLP} B "));
                    if (FactoryData.LP_B_BandNum < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(FactoryData.LP_B_BandNum);
                    Serial.print(F("{FLP} C "));
                    if (FactoryData.LP_C_BandNum < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(FactoryData.LP_C_BandNum);
                    Serial.print(F("{FLP} D "));
                    if (FactoryData.LP_D_BandNum < 10)
                    {
                        SerialPrintZero();
                    }
                    Serial.println(FactoryData.LP_D_BandNum);
                }
            } // Low pass filter config

            // Reference Oscillator Frequency
            if ((InputCMD[2] == 'R') && (InputCMD[3] == 'F'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    for (int i = 0; i <= 8; i++)
                    {
                        CharInt[i] = InputCMD[i + 8];
                    }
                    CharInt[9] = 0;
                    FactoryData.RefFreq = StrTouint64_t(CharInt);
                }
                else // Get
                {
                    Serial.print(F("{FRF} "));
                    Serial.println(uint64ToStr(FactoryData.RefFreq, true));
                }
            } // Reference Oscillator Frequency

            // Store Current Factory configuration data to EEPROM
            if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E'))
            {
                if (InputCMD[6] == 'S')
                { // Set option
                    SaveToEEPROM(FactorySpace);
                    Serial.println(F("{MIN} Factory data saved"));
                }
            }

        } // Factory
    }
}