#include "geofence.hpp"

// GeoFence grids by Matt Downs - 2E1GYP and Harry Zachrisson - SM7PNV , save some RAM by putting the string in program memmory
const char NoTXGrids[] PROGMEM = {"IO78 IO88 IO77 IO87 IO76 IO86 IO75 IO85 IO84 IO94 IO83 IO93 IO82 IO92 JO02 IO81 IO91 JO01 IO70 IO80 IO90 IO64 PN31 PN41 PN20 PN30 PN40 PM29 PM39 PM28 PM38 LK16 LK15 LK14 LK13 LK23 LK24 LK25 LK26 LK36 LK35 LK34 LK33 LK44 LK45 LK46 LK47 LK48 LK58 LK57 LK56 LK55"}; // Airborne transmissions of this sort are not legal over the UK, North Korea, or Yemen.

// GeoFence, do not transmit over Yemen, North Korea and the UK
// GeoFence code by Matt Downs - 2E1GYP and Harry Zachrisson - SM7PNV
// Defined by the NoTXGrids that holds all the Maidehead grids for these locations
boolean OutsideGeoFence(S_WSPRData WSPRData)
{
    char TestGrid[4];
    boolean Outside;

    Outside = true;
    for (uint16_t GridLoop = 0; GridLoop < strlen_P(NoTXGrids); GridLoop = GridLoop + 5)
    { // Itterate between Geo-Fenced grids
        for (uint16_t CharLoop = 0; CharLoop < 4; CharLoop++)
        {
            TestGrid[CharLoop] = pgm_read_byte_near(NoTXGrids + CharLoop + GridLoop); // Copy a Grid string from program memory to RAM variable.
        }
        if ((WSPRData.MaidenHead4[0] == TestGrid[0]) && (WSPRData.MaidenHead4[1] == TestGrid[1]) && (WSPRData.MaidenHead4[2] == TestGrid[2]) && (WSPRData.MaidenHead4[3] == TestGrid[3]))
        {
            Outside = false; // We found a match between the current location and a Geo-Fenced Grid
        }
    }

    return Outside;
}