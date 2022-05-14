#include "gps.hpp"
#include "defines.hpp"
#include "SoftwareSerial.h"

#define GPSPower A1 // Sleep-Wake signal of the GPS on the WSPR-TX Pico

extern SoftwareSerial GPSSerial; // GPS Serial port, RX on pin 2, TX on pin 3

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
