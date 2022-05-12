
#include <Arduino.h>

uint8_t i2cStart();
void i2cStop();
uint8_t i2cByteSend(uint8_t data);
uint8_t i2cByteRead();
uint8_t i2cSendRegister(uint8_t reg, uint8_t data);
uint8_t i2cReadRegister(uint8_t reg, uint8_t *data);
void i2cInit();
boolean DetectSi5351I2CAddress();
