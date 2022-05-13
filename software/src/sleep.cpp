#include "sleep.hpp"

// Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
ISR(WDT_vect)
{
    // DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
} // watchdog interrupt

// Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
void MCUGoToSleep(int SleepTime) // Sleep time in seconds, accurate to the nearest 8 seconds
{
    int SleepLoop;
    SleepLoop = SleepTime / 8.8; // every sleep period is 8.8 seconds
    GPSSerial.end();             // Must turn off software serialport or sleep will not work
    // Serial.end(); //Turn off Hardware serial port as well as we will temporary change all ports to outputs
    AllIOtoLow(); // Set all IO pins to outputs to save power
    DisableADC(); // Turn off ADC to save power

    // SETUP WATCHDOG TIMER
    WDTCSR = (24);      // change enable and WDE - also resets
    WDTCSR = (33);      // prescalers only - get rid of the WDE and WDCE bit
    WDTCSR |= (1 << 6); // enable interrupt mode

    // ENABLE SLEEP - this enables the sleep mode
    SMCR |= (1 << 2);                   // power down mode
    SMCR |= 1;                          // enable sleep
    for (int i = 0; i < SleepLoop; i++) // sleep for eight second intervals untill SleepTime is reached
    {
        // BOD DISABLE - this must be called right before the __asm__ sleep instruction
        MCUCR |= (3 << 5);                      // set both BODS and BODSE at the same time
        MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); // then set the BODS bit and clear the BODSE bit at the same time
        __asm__ __volatile__("sleep");          // in line assembler to go to sleep
        // Just woke upp after 8 seconds of sleep, do a short blink to indicate that I'm still running
        digitalWrite(StatusLED, HIGH);
        delay(30);
        digitalWrite(StatusLED, LOW);
    }
    // Restore everything
    EnableADC();
    GPSSerial.begin(9600); // Init software serial port to communicate with the on-board GPS module
}

// Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
void AllIOtoLow()
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

void DisableADC()
{
    // Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
    ADCSRA &= ~(1 << 7);
}

void EnableADC()
{
    // Enable ADC again
    ADCSRA |= (1 << 7);
}