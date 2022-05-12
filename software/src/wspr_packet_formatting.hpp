#include "Arduino.h"
#include "datatypes.hpp"

void wspr_encode(const char *call, const char *loc, const uint8_t dbm, uint8_t *symbols, uint8_t WSPRMessageType, S_GadgetData GadgetData);
void wspr_message_prep(char *call, char *loc, uint8_t dbm);
uint8_t ValiddBmValue(uint8_t dBmIn);
void convolve(uint8_t *c, uint8_t *s, uint8_t message_size, uint8_t bit_size);
void wspr_interleave(uint8_t *s);
void wspr_merge_sync_vector(uint8_t *g, uint8_t *symbols);
uint8_t wspr_code(char c);

uint8_t *get_tx_buffer_ptr();
uint8_t get_tx_buffer_size();

uint32_t WSPRCallHash(const char *call, S_GadgetData GadgetData);
