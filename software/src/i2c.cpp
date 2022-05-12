#include "i2c.hpp"
#include "defines.hpp"

uint8_t Si5351I2CAddress; // The I2C address on the Si5351 as detected on startup

// Init TWI (I2C)
//
void i2cInit()
{
    TWBR = 92;
    TWSR = 0;
    TWDR = 0xFF;
    PRR = 0;
}

uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
    uint8_t stts;

    stts = i2cStart();
    if (stts != I2C_START)
        return 1;

    stts = i2cByteSend(Si5351I2CAddress << 1);
    if (stts != I2C_SLA_W_ACK)
        return 2;

    stts = i2cByteSend(reg);
    if (stts != I2C_DATA_ACK)
        return 3;

    stts = i2cByteSend(data);
    if (stts != I2C_DATA_ACK)
        return 4;

    i2cStop();

    return 0;
}

uint8_t i2cReadRegister(uint8_t reg, uint8_t *data)
{
    uint8_t stts;

    stts = i2cStart();
    if (stts != I2C_START)
        return 1;

    stts = i2cByteSend((Si5351I2CAddress << 1));
    if (stts != I2C_SLA_W_ACK)
        return 2;

    stts = i2cByteSend(reg);
    if (stts != I2C_DATA_ACK)
        return 3;

    stts = i2cStart();
    if (stts != I2C_START_RPT)
        return 4;

    stts = i2cByteSend((Si5351I2CAddress << 1) + 1);
    if (stts != I2C_SLA_R_ACK)
        return 5;

    *data = i2cByteRead();

    i2cStop();

    return 0;
}

// I2C and PLL routines from Hans Summer demo code https://www.qrp-labs.com/images/uarduino/uard_demo.ino
uint8_t i2cStart()
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)))
        ;

    return (TWSR & 0xF8);
}

void i2cStop()
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

    while ((TWCR & (1 << TWSTO)))
        ;
}

uint8_t i2cByteSend(uint8_t data)
{
    TWDR = data;

    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)))
        ;

    return (TWSR & 0xF8);
}

uint8_t i2cByteRead()
{
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)))
        ;

    return (TWDR);
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
