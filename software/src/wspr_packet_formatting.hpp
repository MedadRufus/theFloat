/**
 * @file wspr_packet_formatting.hpp
 * @author Medad Newman (medad@medadnewman.co.uk)
 * @brief Original WSPR code by NT7S - Jason Milldrum https://github.com/etherkit/JTEncode and Bo Hansen - OZ2M RFZero https://rfzero.net
 * Modifed for Type2 and Type3 messages by SM7PNV Harry Zachrisson https://github.com/HarrydeBug
 * @version 0.1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Arduino.h"
#include "datatypes.hpp"

/**
 * @brief Takes an arbitrary message of up to 13 allowable characters and returns.
 *
 * @param call Callsign (6 characters maximum).
 * @param loc Maidenhead grid locator (4 charcters maximum).
 * @param dbm Output power in dBm.
 * @param symbols Array of channel symbols to transmit returned by the method. Ensure that you pass a uint8_t array of
 * size WSPR_SYMBOL_COUNT to the method.
 * @param WSPRMessageType
 * @param GadgetData
 */
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
