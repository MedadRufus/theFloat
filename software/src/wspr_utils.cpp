#include "wspr_utils.hpp"
#include "print_operations.hpp"
#include "datatypes.hpp"
#include "filter_management.hpp"
#include "wspr_packet_formatting.hpp"
#include <NMEAGPS.h>
#include "gps.hpp"
#include "power.hpp"
#include "defines.hpp"
#include "Si5351.hpp"
#include <SoftwareSerial.h>
#include "state_machine.hpp"
#include "geofence.hpp"
#include "led.hpp"

#define WSPR_FREQ23cm 129650150000ULL // 23cm 1296.501,500MHz (Overtone, not implemented)
#define WSPR_FREQ70cm 43230150000ULL  // 70cm  432.301,500MHz (Overtone, not implemented)
#define WSPR_FREQ2m 14449500000ULL    // 2m    144.490,000MHz //Not working. No decode in bench test with WSJT-X decoding Software
#define WSPR_FREQ4m 7009250000ULL     // 4m     70.092,500MHz //Slightly lower output power
#define WSPR_FREQ6m 5029450000ULL     // 6m     50.294,500MHz //Slightly lower output power
#define WSPR_FREQ10m 2812610000ULL    // 10m    28.126,100MHz
#define WSPR_FREQ12m 2492610000ULL    // 12m    24.926,100MHz
#define WSPR_FREQ15m 2109610000ULL    // 15m    21.096.100MHz
#define WSPR_FREQ17m 1810610000ULL    // 17m    18.106,100MHz
#define WSPR_FREQ20m 1409710000ULL    // 20m    14.097,100MHz
#define WSPR_FREQ30m 1014020000ULL    // 30m    10.140,200MHz
#define WSPR_FREQ40m 704010000ULL     // 40m     7.040,100MHz
#define WSPR_FREQ80m 357010000ULL     // 80m     3.570,100MHz
#define WSPR_FREQ160m 183810000ULL    // 160m    1.838,100MHz
#define WSPR_FREQ630m 47570000ULL     // 630m      475.700kHz
#define WSPR_FREQ2190m 13750000ULL    // 2190m     137.500kHz

extern uint64_t freq;             // Holds the Output frequency when we are in signal generator mode or in WSPR mode
extern S_GadgetData GadgetData;   // TODO: replace with getters and setters
extern S_FactoryData FactoryData; // Create a datastructure that holds information of the hardware
extern uint8_t CurrentBand;       // Keeps track on what band we are currently tranmitting on
extern boolean Si5351I2C_found;
extern NMEAGPS gps;                    // This parses the GPS characters
extern SoftwareSerial GPSSerial;       // GPS Serial port, RX on pin 2, TX on pin 3
extern E_Mode CurrentMode;             // TODO: replace with getters and setters
extern uint16_t LoopGPSNoReceiveCount; // If GPS stops working while in ídle mode this will increment
extern boolean PCConnected;
extern gps_fix fix;             // This holds on to the latest values
extern int GPSH;                // GPS Hours
extern int GPSM;                // GPS Minutes
extern int GPSS;                // GPS Seconds
extern int StatusLED;           // LED that indicates current status. Yellow on LP1, Desktop and Mini models, white on Pico
extern char LastMaidenHead6[7]; // Holds the Maidenhead position from last transmission, used when GadgetData.WSPRData.TimeSlotCode=17 to determine if the transmitter has moved since last TX

boolean getCorrectTimeslot();
boolean getLastFreq(void);
boolean NewPosition();

// Convert a frequency to a Ham band. Frequency is stored in global variable freq
uint8_t FreqToBand()
{
    uint8_t BandReturn = 15;

    if (freq < (WSPR_FREQ70cm * 1.2))
    {
        BandReturn = 14;
    }
    if (freq < (WSPR_FREQ2m * 1.2))
    {
        BandReturn = 13;
    }
    if (freq < (WSPR_FREQ4m * 1.2))
    {
        BandReturn = 12;
    }
    if (freq < (WSPR_FREQ6m * 1.2))
    {
        BandReturn = 11;
    }
    if (freq < (WSPR_FREQ10m * 1.2))
    {
        BandReturn = 10;
    }
    if (freq < (WSPR_FREQ12m * 1.2))
    {
        BandReturn = 9;
    }
    if (freq < (WSPR_FREQ15m * 1.2))
    {
        BandReturn = 8;
    }
    if (freq < (WSPR_FREQ17m * 1.1))
    {
        BandReturn = 7;
    }
    if (freq < (WSPR_FREQ20m * 1.2))
    {
        BandReturn = 6;
    }
    if (freq < (WSPR_FREQ30m * 1.2))
    {
        BandReturn = 5;
    }
    if (freq < (WSPR_FREQ40m * 1.2))
    {
        BandReturn = 4;
    }
    if (freq < (WSPR_FREQ80m * 1.2))
    {
        BandReturn = 3;
    }
    if (freq < (WSPR_FREQ160m * 1.2))
    {
        BandReturn = 2;
    }
    if (freq < (WSPR_FREQ630m * 1.2))
    {
        BandReturn = 1;
    }
    if (freq < (WSPR_FREQ2190m * 1.2))
    {
        BandReturn = 0;
    }
    return BandReturn;
}

// Determine what band to transmit on, cycles upward in the TX enabled bands, e.g if band 2,5,6 and 11 is enbled for TX then the cycle will be 2-5-6-11-2-5-6-11-...
void NextFreq(void)
{
    if (isNoBandEnable())
    {
        freq = 0;
    }
    else
    {
        do
        {
            CurrentBand++;
            if (CurrentBand > 12)
            {
                CurrentBand = 0;
            }
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

// Returns true if the user has not enabled any bands for TX
boolean isNoBandEnable(void)
{
    boolean NoOne = true;
    for (int FreqLoop = 0; FreqLoop < 13; FreqLoop++)
    {
        if (GadgetData.TXOnBand[FreqLoop])
        {
            NoOne = false;
        }
    }
    return NoOne;
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
        if (isNoBandEnable())
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
                            calcLocator(fix.latitude(), fix.longitude(), &GadgetData.WSPRData);
                        }
                        if ((GPSS == 00) && (getCorrectTimeslot())) // If second is zero at even minute then start WSPR transmission. The function CorrectTimeSlot can hold of transmision depending on several user settings. The GadgetData.WSPRData.TimeSlotCode value will influense the behaviour
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
                                StorePosition();   // Save the current position;
                                if (getLastFreq()) // If all bands have been transmitted on then pause for user defined time and after that start over on the first band again
                                {
                                    if ((GadgetData.TXPause > 60) && ((Product_Model == 1017) || (Product_Model == 1028)) && (!PCConnected)) // If the PC is not connected and the TXdelay is longer than a 60 sec then put the MCU to sleep to save current during this long pause (Mini and Pico models only)
                                    {
                                        delay(600);       // Let the serial port send data from its buffer before we go to sleep
                                        Si5351PowerOff(); // Turn off the PLL to save power (Mini Only)
                                        // MCUGoToSleep (GadgetData.TXPause - 10);        //Set MCU in sleep mode until there is 10 seconds left of delay
                                        PowerSaveOFF();          // We are back from sleep - turn on GPS and PLL again
                                        gps_blocking_wait(2000); // let the gps_blocking_wait routine read a few GPS lines so we can get the new GPS time after our sleep
                                    }
                                    else
                                    {                                                   // Regular pause if we did not go to sleep then do a regular pause and send updates to the GUI for the duration
                                        gps_blocking_wait(GadgetData.TXPause * 1000UL); // Pause for the time set by the user
                                    }
                                    SendAPIUpdate(UMesWSPRBandCycleComplete); // Inform PC that we have transmitted on the last enabled WSPR band and will start over
                                }
                                GPSWakeUp();
                                NextFreq();                                 // get the frequency for the next HAM band that we will transmit on
                                freq = freq + (100ULL * random(-100, 100)); // modify the TX frequency with a random value beween -100 and +100 Hz to avoid possible lengthy colisions with other users on the band
                                gps_blocking_wait(3000);
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
                            gps_blocking_wait(100);
                        }
                    }
                    else
                    {                                 // Waiting for GPS location fix
                        SendSatData();                // Send Satellite postion and SNR information to the PC GUI while we wait for the GPS location fix
                        LEDBlink(1);                  // singleblink to indicate waiting for GPS Lock
                        SendAPIUpdate(UMesNoGPSLock); // Send No lock status
                        gps_blocking_wait(400);
                    }
                } // GPS serial data loop
                LoopGPSNoReceiveCount++;
                if (LoopGPSNoReceiveCount > 60000) // GPS have not sent anything for a long time, GPS is possible in sleep mode or has not started up correctly. This can happen if a brown-out/reboot happens while the GPS was sleeping
                {
                    LoopGPSNoReceiveCount = 0;
                    Serial.println(F("{MIN} Resetting GPS"));
                    GPSReset(); // Try to get GPS going again
                    gps_blocking_wait(2000);
                }
            } // Incoming serial command
        }
    }
}

void StorePosition() // Saves the current position
{
    for (int i = 0; i < 7; i++)
    {
        LastMaidenHead6[i] = GadgetData.WSPRData.MaidenHead6[i];
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

// Only transmit on specific times
boolean getCorrectTimeslot()
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

// Function returns True if the band we just transmitted on was the highest band the user want to transmit on.
boolean getLastFreq(void)
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
            {
                Last = false;
            }
        } while (TestBand < 12);
    }
    return Last;
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
}

boolean NewPosition() // Returns true if the postion has changed since the last transmission
{
    boolean NewPos = false;
    for (int i = 0; i < GadgetData.WSPRData.LocationPrecision; i++) // Check if the position has changed, test it using either four or six letter Maidenhead precision based on user setting
    {
        if (GadgetData.WSPRData.MaidenHead6[i] != LastMaidenHead6[i])
        {
            NewPos = true;
        }
    }
    return NewPos;
}
