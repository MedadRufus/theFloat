#include "wspr_utils.hpp"

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

extern uint64_t freq; // Holds the Output frequency when we are in signal generator mode or in WSPR mode

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
                CurrentBand = 0;
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