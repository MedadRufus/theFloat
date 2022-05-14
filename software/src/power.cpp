#include "power.hpp"
#include "gps.hpp"
#include "Si5351.hpp"

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
