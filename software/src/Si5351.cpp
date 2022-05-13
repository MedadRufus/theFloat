#include "Si5351.hpp"
#include "defines.hpp"
#include "i2c.hpp"
#include "string_operations.hpp"

uint8_t Si5351I2CAddress; // The I2C address on the Si5351 as detected on startup

void Si5351PowerOff()
{
    if (Product_Model == 1017 || Product_Model == 1028) // If its the WSPR-TX Mini it has a control line that can cut power to the Si5351
    {
        // Power off the Si5351
        digitalWrite(SiPower, HIGH);
    }
}

void Si5351PowerOn()
{
    if (Product_Model == 1017) // If its the WSPR-TX Mini it has a control line that can cut power to the Si5351
    {
        // Power on the Si5351
        digitalWrite(SiPower, LOW);
        // Give it some time to stabilize voltage before init
        delay(100);
        // re-initialize the Si5351
        i2cInit();
        si5351aOutputOff(SI_CLK0_CONTROL);
    }
}

// I2C and PLL routines from Han Summer demo code https://www.qrp-labs.com/images/uarduino/uard_demo.ino
//
// Set up MultiSynth with integer Divider and R Divider
// R Divider is the bit value which is OR'ed onto the appropriate
// register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t Divider, uint8_t rDiv)
{
    uint32_t P1; // Synth config register P1
    uint32_t P2; // Synth config register P2
    uint32_t P3; // Synth config register P3

    P1 = 128 * Divider - 512;
    P2 = 0; // P2 = 0, P3 = 1 forces an integer value for the Divider
    P3 = 1;

    i2cSendRegister(synth + 0, (P3 & 0x0000FF00) >> 8, Si5351I2CAddress);
    i2cSendRegister(synth + 1, (P3 & 0x000000FF), Si5351I2CAddress);
    i2cSendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv, Si5351I2CAddress);
    i2cSendRegister(synth + 3, (P1 & 0x0000FF00) >> 8, Si5351I2CAddress);
    i2cSendRegister(synth + 4, (P1 & 0x000000FF), Si5351I2CAddress);
    i2cSendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16), Si5351I2CAddress);
    i2cSendRegister(synth + 6, (P2 & 0x0000FF00) >> 8, Si5351I2CAddress);
    i2cSendRegister(synth + 7, (P2 & 0x000000FF), Si5351I2CAddress);
}

// Switches off Si5351a output
void si5351aOutputOff(uint8_t clk)
{
    i2cSendRegister(clk, 0x80, Si5351I2CAddress); // Refer to SiLabs AN619 to see
    // bit values - 0x80 turns off the output stage
    digitalWrite(TransmitLED, LOW);
    SendAPIUpdate(UMesTXOff);
}

// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 10kHz to 150MHz and given in centiHertz (hundreds of Hertz)
// Example: si5351aSetFrequency(1000000200);
// will set output CLK0 to 10.000,002MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
//
void si5351aSetFrequency(uint64_t frequency, uint32_t RefFreq) // Frequency is in centiHz
{
    static uint64_t oldFreq;
    int32_t FreqChange;
    uint64_t pllFreq;
    // uint32_t xtalFreq = XTAL_FREQ;
    uint32_t l;
    float f;
    uint8_t mult;
    uint32_t num;
    uint32_t denom;
    uint32_t Divider;
    uint8_t rDiv;

    if (frequency > 100000000ULL)
    { // If higher than 1MHz then set R output divider to 1
        rDiv = SI_R_DIV_1;
        Divider = 90000000000ULL / frequency; // Calculate the division ratio. 900MHz is the maximum VCO freq (expressed as deciHz)
        pllFreq = Divider * frequency;        // Calculate the pllFrequency:
        mult = pllFreq / (RefFreq * 100UL);   // Determine the multiplier to
        l = pllFreq % (RefFreq * 100UL);      // It has three parts:
        f = l;                                // mult is an integer that must be in the range 15..90
        f *= 1048575;                         // num and denom are the fractional parts, the numerator and denominator
        f /= RefFreq;                         // each is 20 bits (range 0..1048575)
        num = f;                              // the actual multiplier is mult + num / denom
        denom = 1048575;                      // For simplicity we set the denominator to the maximum 1048575
        num = num / 100;
    }
    else // lower freq than 1MHz - use output Divider set to 128
    {
        rDiv = SI_R_DIV_128;
        // frequency = frequency * 128ULL; //Set base freq 128 times higher as we are dividing with 128 in the last output stage
        Divider = 90000000000ULL / (frequency * 128ULL); // Calculate the division ratio. 900MHz is the maximum VCO freq

        pllFreq = Divider * frequency * 128ULL; // Calculate the pllFrequency:
        // the Divider * desired output frequency
        mult = pllFreq / (RefFreq * 100UL); // Determine the multiplier to
        // get to the required pllFrequency
        l = pllFreq % (RefFreq * 100UL); // It has three parts:
        f = l;                           // mult is an integer that must be in the range 15..90
        f *= 1048575;                    // num and denom are the fractional parts, the numerator and denominator
        f /= RefFreq;                    // each is 20 bits (range 0..1048575)
        num = f;                         // the actual multiplier is mult + num / denom
        denom = 1048575;                 // For simplicity we set the denominator to the maximum 1048575
        num = num / 100;
    }

    // Set up PLL A with the calculated  multiplication ratio
    setupPLL(SI_SYNTH_PLL_A, mult, num, denom);

    // Set up MultiSynth Divider 0, with the calculated Divider.
    // The final R division stage can divide by a power of two, from 1..128.
    // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
    // If you want to output frequencies below 1MHz, you have to use the
    // final R division stage
    setupMultisynth(SI_SYNTH_MS_0, Divider, rDiv);

    // Reset the PLL. This causes a glitch in the output. For small changes to
    // the parameters, you don't need to reset the PLL, and there is no glitch
    FreqChange = frequency - oldFreq;

    if (abs(FreqChange) > 100000) // If changed more than 1kHz then reset PLL (completely arbitrary choosen)
    {
        i2cSendRegister(SI_PLL_RESET, 0xA0, Si5351I2CAddress);
    }

    // Finally switch on the CLK0 output (0x4F)
    // and set the MultiSynth0 input to be PLL A
    i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A, Si5351I2CAddress);
    oldFreq = frequency;
    digitalWrite(TransmitLED, HIGH);
    Serial.print(F("{TFQ} "));
    Serial.println(uint64ToStr(frequency, false));
    SendAPIUpdate(UMesTXOn);
}

boolean DetectSi5351I2CAddress()
{
    uint8_t I2CResult;
    boolean Result;
    Si5351I2CAddress = 96; // Try with the normal adress of 96
    i2cStart();
    I2CResult = i2cByteSend((Si5351I2CAddress << 1));
    i2cStop();
    if (I2CResult == I2C_SLA_W_ACK)
    {
        // We found it
        // Serial.println("Detected at adress 96");
        Result = true;
    }
    else
    {
        // Serial.println("Not Detected at adress 96");
        Si5351I2CAddress = 98; // Try the alternative address of 98
        i2cStart();
        I2CResult = i2cByteSend((Si5351I2CAddress << 1));
        i2cStop();
        if (I2CResult == I2C_SLA_W_ACK)
        {
            // Serial.println("Detected at adress 98");
            Result = true;
        }
        else
        {
            // Serial.println("Not Detected at adress 98 either, no Si5351!");
            Result = false;
            Si5351I2CAddress = 0;
        }
    }
    return Result;
}

// I2C and PLL routines from Hans Summer demo code https://www.qrp-labs.com/images/uarduino/uard_demo.ino
//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
    uint32_t P1; // PLL config register P1
    uint32_t P2; // PLL config register P2
    uint32_t P3; // PLL config register P3

    P1 = (uint32_t)(128 * ((float)num / (float)denom));
    P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
    P2 = (uint32_t)(128 * ((float)num / (float)denom));
    P2 = (uint32_t)(128 * num - denom * P2);
    P3 = denom;

    i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8, Si5351I2CAddress);
    i2cSendRegister(pll + 1, (P3 & 0x000000FF), Si5351I2CAddress);
    i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16, Si5351I2CAddress);
    i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8, Si5351I2CAddress);
    i2cSendRegister(pll + 4, (P1 & 0x000000FF), Si5351I2CAddress);
    i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16), Si5351I2CAddress);
    i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8, Si5351I2CAddress);
    i2cSendRegister(pll + 7, (P2 & 0x000000FF), Si5351I2CAddress);
}
