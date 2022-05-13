#include "Arduino.h"
#include "datatypes.hpp"

void SendAPIUpdate(uint8_t UpdateType);
void DecodeSerialCMD(const char *InputCMD, S_GadgetData GadgetData);
