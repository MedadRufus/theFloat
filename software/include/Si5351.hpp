#include "Arduino.h"

void Si5351PowerOff();
void Si5351PowerOn();
void si5351aOutputOff(uint8_t clk);
void setupMultisynth(uint8_t synth, uint32_t Divider, uint8_t rDiv);
void si5351aSetFrequency(uint64_t frequency, uint32_t RefFreq);
boolean DetectSi5351I2CAddress();
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom);
void DoSignalGen();
void DoIdle();
