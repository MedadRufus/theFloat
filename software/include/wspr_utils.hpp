#include "Arduino.h"

uint8_t FreqToBand();
void NextFreq(void);
boolean isNoBandEnable(void);
void DoWSPR();
void StorePosition();
int SendWSPRMessage(uint8_t WSPRMessageType);
void SendSatData();
