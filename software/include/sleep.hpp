#include "Arduino.h"

// Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
void MCUGoToSleep(int SleepTime); // Sleep time in seconds, accurate to the nearest 8 seconds
void AllIOtoLow();
void DisableADC();
void EnableADC();