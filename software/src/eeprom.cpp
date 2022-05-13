#include "eeprom.hpp"
#include "defines.hpp"
#include "datatypes.hpp"
#include <EEPROM.h>

extern S_FactoryData FactoryData; // TODO: replace with getters and setters
extern S_GadgetData GadgetData;   // TODO: replace with getters and setters

// CRC calculation from Christopher Andrews : https://www.arduino.cc/en/Tutorial/EEPROMCrc
// Calculate CRC on either Factory data or Userspace data
unsigned long GetEEPROM_CRC(boolean EEPROMSpace)
{

    const unsigned long crc_table[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};

    unsigned long crc = ~0L;
    int Start;
    int Length;

    if (EEPROMSpace == FactorySpace)
    {
        Start = 400;
        Length = sizeof(FactoryData);
    }
    else
    {
        Start = 0;
        Length = sizeof(GadgetData);
    }
    for (int index = Start; index < (Start + Length); ++index)
    {
        crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
        crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
        crc = ~crc;
    }
    return crc;
}

// Load FactoryData or UserSpace Data from ATMega EEPROM
bool LoadFromEPROM(boolean EEPROMSpace)
{
    int Start;
    int Length;
    unsigned long CRCFromEEPROM, CalculatedCRC;

    if (EEPROMSpace == FactorySpace) // Factory data
    {
        Start = 400;
        Length = sizeof(FactoryData);
        EEPROM.get(Start, FactoryData);              // Load all the data from EEPROM
        CalculatedCRC = GetEEPROM_CRC(FactorySpace); // Calculate the CRC of the data
    }
    else // User data
    {
        Start = 0;
        Length = sizeof(GadgetData);
        EEPROM.get(Start, GadgetData);            // Load all the data from EEPROM
        CalculatedCRC = GetEEPROM_CRC(UserSpace); // Calculate the CRC of the data
    }
    EEPROM.get(Start + Length, CRCFromEEPROM); // Load the saved CRC at the end of the data
    return (CRCFromEEPROM == CalculatedCRC);   // If  Stored and Calculated CRC are the same return true
}

// Save FactoryData or UserSpace Data to Arduino EEPROM
void SaveToEEPROM(boolean EEPROMSpace)
{
    int Start;
    int Length;
    unsigned long CRCFromEEPROM;
    if (EEPROMSpace == FactorySpace)
    {
        Start = 400;
        Length = sizeof(FactoryData);
        EEPROM.put(Start, FactoryData); // Save all the Factory data to EEPROM at adress400
    }
    else // UserSpace
    {
        Start = 0;
        Length = sizeof(GadgetData);
        EEPROM.put(Start, GadgetData); // Save all the User data to EEPROM at adress0
    }
    CRCFromEEPROM = GetEEPROM_CRC(EEPROMSpace); // Calculate CRC on the saved data
    EEPROM.put(Start + Length, CRCFromEEPROM);  // Save the CRC after the data
}
