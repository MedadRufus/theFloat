#include "Arduino.h"

// eeprom related
unsigned long GetEEPROM_CRC(boolean EEPROMSpace);
bool LoadFromEPROM(boolean EEPROMSpace);
void SaveToEEPROM(boolean EEPROMSpace);
