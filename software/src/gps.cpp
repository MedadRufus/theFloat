#include "gps.hpp"
#include "defines.hpp"
#include "SoftwareSerial.h"
#include <NMEAGPS.h>
#include "led.hpp"

#define GPSPower A1 // Sleep-Wake signal of the GPS on the WSPR-TX Pico

extern SoftwareSerial GPSSerial; // GPS Serial port, RX on pin 2, TX on pin 3
extern NMEAGPS gps;              // This parses the GPS characters
extern gps_fix fix;              // This holds on to the latest values

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

// Part of the code from the TinyGPS example but here used for the NeoGPS
// Delay loop that checks if the GPS serial port is sending data and in that case passes it of to the GPS object
void gps_blocking_wait(unsigned long delay_ms)
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
        {
            fix = gps.read(); // If GPS data available - process it
        }
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