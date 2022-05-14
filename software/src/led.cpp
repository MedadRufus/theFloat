#include "led.hpp"
#include "gps.hpp"

extern int StatusLED; // LED that indicates current status. Yellow on LP1, Desktop and Mini models, white on Pico

// Brief flash on the Status LED 'Blinks'" number of time
void LEDBlink(int Blinks)
{
    for (int i = 0; i < Blinks; i++)
    {
        digitalWrite(StatusLED, HIGH);
        gps_blocking_wait(50);
        digitalWrite(StatusLED, LOW);
        gps_blocking_wait(50);
    }
}