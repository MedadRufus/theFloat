#include "wspr_packet_formatting.hpp"
#include "datatypes.hpp"

#define WSPR_SYMBOL_COUNT 162

// macro functions
#define rot(x, k) ((x << k) | (x >> (32 - k)))

// from Jason Mildrums JTEncode class
char callsign[7];
char locator[5];
uint8_t power;

uint8_t symbolSequence[WSPR_SYMBOL_COUNT];
uint8_t tx_buffer[WSPR_SYMBOL_COUNT];

uint8_t *get_tx_buffer_ptr()
{
    return tx_buffer;
}

uint8_t get_tx_buffer_size()
{
    return sizeof(tx_buffer);
}
// Converts a letter (A-Z) or digit (0-9)to a special format used in the encoding of WSPR messages
uint8_t EncodeChar(char Character)
{
    uint8_t ConvertedNumber;
    if (Character == ' ')
    {
        ConvertedNumber = 36;
    }
    else
    {
        if (isdigit(Character))
        {
            ConvertedNumber = Character - '0';
        }
        else
        {
            ConvertedNumber = 10 + (Character - 'A');
        }
    }
    return ConvertedNumber;
}

void wspr_encode(const char *call, const char *loc, const uint8_t dbm, uint8_t *symbols, uint8_t WSPRMessageType, S_GadgetData GadgetData)
{
    char call_[7];
    char loc_[5];
    uint8_t dbm_ = dbm;
    strcpy(call_, call);
    strcpy(loc_, loc);
    uint32_t n, m;

    // Ensure that the message text conforms to standards
    // --------------------------------------------------
    wspr_message_prep(call_, loc_, dbm_);

    // Bit packing
    // -----------
    uint8_t c[11];

    switch (WSPRMessageType)
    {
    case 1: // Normal coding with callsign, 4letter Maidenhead postion and power
        n = wspr_code(callsign[0]);
        n = n * 36 + wspr_code(callsign[1]);
        n = n * 10 + wspr_code(callsign[2]);
        n = n * 27 + (wspr_code(callsign[3]) - 10);
        n = n * 27 + (wspr_code(callsign[4]) - 10);
        n = n * 27 + (wspr_code(callsign[5]) - 10);

        m = ((179 - 10 * (locator[0] - 'A') - (locator[2] - '0')) * 180) +
            (10 * (locator[1] - 'A')) + (locator[3] - '0');
        m = (m * 128) + power + 64;
        break;

    case 2: // Call sign and Prefix or suffix for it and power, no Maidenhead position
        n = wspr_code(callsign[0]);
        n = n * 36 + wspr_code(callsign[1]);
        n = n * 10 + wspr_code(callsign[2]);
        n = n * 27 + (wspr_code(callsign[3]) - 10);
        n = n * 27 + (wspr_code(callsign[4]) - 10);
        n = n * 27 + (wspr_code(callsign[5]) - 10);

        if (GadgetData.WSPRData.SuPreFixOption == Sufix)
        {
            // Single number or letter suffix from 0 to 35, 0-9= 0-9. 10-35=A-Z.
            // Or double number suffix from 36 to 125, 36-125=10-99
            m = (27232 + GadgetData.WSPRData.Sufix);
            m = (m * 128) + power + 2 + 64;
        }
        else
        {
            // Three character prefix. Numbers, letters or space
            // 0 to 9=0-9, A to Z=10-35, space=36
            m = EncodeChar(GadgetData.WSPRData.Prefix[0]);          // Left Character
            m = 37 * m + EncodeChar(GadgetData.WSPRData.Prefix[1]); // Mid character
            m = 37 * m + EncodeChar(GadgetData.WSPRData.Prefix[2]); // Right character
            // m = (m * 128) + power +1+ 64;

            if (m > 32767)
            {
                m = m - 32768;
                m = (m * 128) + power + 66;
            }
            else
            {
                m = (m * 128) + power + 65;
            }
        }
        break;

    case 3: // Hashed Callsign, six letter maidenhead position and power
        // encode the six letter Maidenhear postion in to n that is usually used for callsign coding, reshuffle the character order to conform to the callsign rules
        n = wspr_code(GadgetData.WSPRData.MaidenHead6[1]);
        n = n * 36 + wspr_code(GadgetData.WSPRData.MaidenHead6[2]);
        n = n * 10 + wspr_code(GadgetData.WSPRData.MaidenHead6[3]);
        n = n * 27 + (wspr_code(GadgetData.WSPRData.MaidenHead6[4]) - 10);
        n = n * 27 + (wspr_code(GadgetData.WSPRData.MaidenHead6[5]) - 10);
        n = n * 27 + (wspr_code(GadgetData.WSPRData.MaidenHead6[0]) - 10);
        m = 128 * WSPRCallHash(call, GadgetData) - power - 1 + 64;
        break;

    } // switch

    // Callsign is 28 bits, locator/power is 22 bits.
    // A little less work to start with the least-significant bits
    c[3] = (uint8_t)((n & 0x0f) << 4);
    n = n >> 4;
    c[2] = (uint8_t)(n & 0xff);
    n = n >> 8;
    c[1] = (uint8_t)(n & 0xff);
    n = n >> 8;
    c[0] = (uint8_t)(n & 0xff);

    c[6] = (uint8_t)((m & 0x03) << 6);
    m = m >> 2;
    c[5] = (uint8_t)(m & 0xff);
    m = m >> 8;
    c[4] = (uint8_t)(m & 0xff);
    m = m >> 8;
    c[3] |= (uint8_t)(m & 0x0f);
    c[7] = 0;
    c[8] = 0;
    c[9] = 0;
    c[10] = 0;

    // Convolutional Encoding
    // ---------------------
    uint8_t s[WSPR_SYMBOL_COUNT];
    convolve(c, s, 11, WSPR_SYMBOL_COUNT);

    // Interleaving
    // ------------
    wspr_interleave(s);

    // Merge with sync vector
    // ----------------------
    wspr_merge_sync_vector(s, symbols);
}

void wspr_message_prep(char *call, char *loc, uint8_t dbm)
{
    // PrintCallSign ('2');
    // If only the 2nd character is a digit, then pad with a space.
    // If this happens, then the callsign will be truncated if it is
    // longer than 6 characters.
    if (isdigit(call[1]) && isupper(call[2]))
    {
        call[5] = call[4];
        call[4] = call[3];
        call[3] = call[2];
        call[2] = call[1];
        call[1] = call[0];
        call[0] = ' ';
    }

    // Ensure that the only allowed characters are digits and uppercase letters
    uint8_t i;
    for (i = 0; i < 6; i++)
    {
        call[i] = toupper(call[i]);
        if (!(isdigit(call[i]) || isupper(call[i])))
        {
            call[i] = ' ';
            if (i == 4)
            {
                call[5] = ' '; // If char 4 is a space then also set the last character to a space
            }
        }
    }

    memcpy(callsign, call, 6);

    // Grid locator validation
    for (i = 0; i < 4; i++)
    {
        loc[i] = toupper(loc[i]);
        if (!(isdigit(loc[i]) || (loc[i] >= 'A' && loc[i] <= 'R')))
        {
            memcpy(loc, "AA00", 5); // loc = "AA00";
        }
    }
    memcpy(locator, loc, 4);
    power = ValiddBmValue(dbm);
}

// Power level validation
uint8_t ValiddBmValue(uint8_t dBmIn)
{
    uint8_t i;
    uint8_t validateddBmValue;
    const uint8_t valid_dbm[19] =
        {0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40,
         43, 47, 50, 53, 57, 60};
    validateddBmValue = dBmIn;
    if (validateddBmValue > 60)
    {
        validateddBmValue = 60;
    }

    for (i = 0; i < 19; i++)
    {
        if (dBmIn >= valid_dbm[i])
        {
            validateddBmValue = valid_dbm[i];
        }
    }
    return validateddBmValue;
}

void convolve(uint8_t *c, uint8_t *s, uint8_t message_size, uint8_t bit_size)
{
    uint32_t reg_0 = 0;
    uint32_t reg_1 = 0;
    uint32_t reg_temp = 0;
    uint8_t input_bit, parity_bit;
    uint8_t bit_count = 0;
    uint8_t i, j, k;

    for (i = 0; i < message_size; i++)
    {
        for (j = 0; j < 8; j++)
        {
            // Set input bit according the MSB of current element
            input_bit = (((c[i] << j) & 0x80) == 0x80) ? 1 : 0;

            // Shift both registers and put in the new input bit
            reg_0 = reg_0 << 1;
            reg_1 = reg_1 << 1;
            reg_0 |= (uint32_t)input_bit;
            reg_1 |= (uint32_t)input_bit;

            // AND Register 0 with feedback taps, calculate parity
            reg_temp = reg_0 & 0xf2d05351;
            parity_bit = 0;
            for (k = 0; k < 32; k++)
            {
                parity_bit = parity_bit ^ (reg_temp & 0x01);
                reg_temp = reg_temp >> 1;
            }
            s[bit_count] = parity_bit;
            bit_count++;

            // AND Register 1 with feedback taps, calculate parity
            reg_temp = reg_1 & 0xe4613c47;
            parity_bit = 0;
            for (k = 0; k < 32; k++)
            {
                parity_bit = parity_bit ^ (reg_temp & 0x01);
                reg_temp = reg_temp >> 1;
            }
            s[bit_count] = parity_bit;
            bit_count++;
            if (bit_count >= bit_size)
            {
                break;
            }
        }
    }
}

void wspr_interleave(uint8_t *s)
{
    uint8_t d[WSPR_SYMBOL_COUNT];
    uint8_t rev, index_temp, i, j, k;

    i = 0;

    for (j = 0; j < 255; j++)
    {
        // Bit reverse the index
        index_temp = j;
        rev = 0;

        for (k = 0; k < 8; k++)
        {
            if (index_temp & 0x01)
            {
                rev = rev | (1 << (7 - k));
            }
            index_temp = index_temp >> 1;
        }

        if (rev < WSPR_SYMBOL_COUNT)
        {
            d[rev] = s[i];
            i++;
        }

        if (i >= WSPR_SYMBOL_COUNT)
        {
            break;
        }
    }

    memcpy(s, d, WSPR_SYMBOL_COUNT);
}

void wspr_merge_sync_vector(uint8_t *g, uint8_t *symbols)
{
    uint8_t i;
    const uint8_t sync_vector[WSPR_SYMBOL_COUNT] =
        {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0,
         1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
         0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0,
         1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
         0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
         1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0};

    for (i = 0; i < WSPR_SYMBOL_COUNT; i++)
    {
        symbols[i] = sync_vector[i] + (2 * g[i]);
    }
}

uint8_t wspr_code(char c)
{
    // Validate the input then return the proper integer code.
    // Return 255 as an error code if the char is not allowed.

    if (isdigit(c))
    {
        return (uint8_t)(c - 48);
    }
    else if (c == ' ')
    {
        return 36;
    }
    else if (c >= 'A' && c <= 'Z')
    {
        return (uint8_t)(c - 55);
    }
    else
    {
        return 255;
    }
}

// Type 3 call sign hash by RFZero www.rfzero.net modified by SM7PNV
uint32_t WSPRCallHash(const char *call, S_GadgetData GadgetData)
{

    uint32_t a, b, c;
    char CallWithSuPrefix[11];
    uint8_t Length = strlen(call);
    uint8_t CharLoop;
    Serial.print("Length ");
    Serial.print(Length);
    strcpy(CallWithSuPrefix, call);
    if (GadgetData.WSPRData.SuPreFixOption == Sufix)
    {
        CallWithSuPrefix[Length] = '/';     // Add slash at the end
        if (GadgetData.WSPRData.Sufix < 36) // Single digit or letter
        {
            CallWithSuPrefix[Length + 2] = 0; // Zero terminate
            if (GadgetData.WSPRData.Sufix < 10)
            {
                CallWithSuPrefix[Length + 1] = '0' + GadgetData.WSPRData.Sufix; // Add a single digit
            }
            else
            {
                CallWithSuPrefix[Length + 1] = 'A' + (GadgetData.WSPRData.Sufix - 10); // Add a single letter
            }
        }
        else // Suffix is double digits
        {
            /* Seems the Type 3 decodes are not correct in case of two suffix numbers so this code is commented out for now as it will not be used by the Configurtion software
              Number=GadgetData.WSPRData.Sufix-36;
              while (Number>9)
              {
              ++TenDigit;
              Number -= 10;
              }
              CallWithSuPrefix[Length+1]='0'+TenDigit; //Add the Ten Digit
              CallWithSuPrefix[Length+2]='0'+Number; //Add the One Digit
              CallWithSuPrefix[Length+3]=0; //Zero terminate
            */
        }
    } // if Sufix
    else if (GadgetData.WSPRData.SuPreFixOption == Prefix)
    {
        CallWithSuPrefix[0] = GadgetData.WSPRData.Prefix[0];
        CallWithSuPrefix[1] = GadgetData.WSPRData.Prefix[1];
        CallWithSuPrefix[2] = GadgetData.WSPRData.Prefix[2];
        CallWithSuPrefix[3] = '/';

        for (CharLoop = 0; CharLoop < Length; CharLoop++)
        {
            CallWithSuPrefix[CharLoop + 4] = call[CharLoop];
        }
    } // else if Prefix

    Length = strlen(CallWithSuPrefix);
    // Serial.print(" : ");
    // Serial.println(Length);
    // Serial.print("{MIN} Call with Sufix=");
    // Serial.println(CallWithSuPrefix);

    a = b = c = 0xdeadbeef + Length + 146;

    const uint32_t *k = (const uint32_t *)CallWithSuPrefix;

    switch (Length) // Length 3-10 chars, thus 0, 1, 2, 11 and 12 omitted
    {
    case 10:
        c += k[2] & 0xffff;
        b += k[1];
        a += k[0];
        break;
    case 9:
        c += k[2] & 0xff;
        b += k[1];
        a += k[0];
        break;
    case 8:
        b += k[1];
        a += k[0];
        break;
    case 7:
        b += k[1] & 0xffffff;
        a += k[0];
        break;
    case 6:
        b += k[1] & 0xffff;
        a += k[0];
        break;
    case 5:
        b += k[1] & 0xff;
        a += k[0];
        break;
    case 4:
        a += k[0];
        break;
    case 3:
        a += k[0] & 0xffffff;
        break;
    }

    c ^= b;
    c -= rot(b, 14);
    a ^= c;
    a -= rot(c, 11);
    b ^= a;
    b -= rot(a, 25);
    c ^= b;
    c -= rot(b, 16);
    a ^= c;
    a -= rot(c, 4);
    b ^= a;
    b -= rot(a, 14);
    c ^= b;
    c -= rot(b, 24);

    c &= 0xFFFF; // 15 bits mask

    return c;
}

// Maidenhead code from Ossi Väänänen https://ham.stackexchange.com/questions/221/how-can-one-convert-from-lat-long-to-grid-square
void calcLocator(double lat, double lon, S_WSPRData *WSPRData)
{
    int o1, o2, o3;
    int a1, a2, a3;
    double remainder;
    // longitude
    remainder = lon + 180.0;
    o1 = (int)(remainder / 20.0);
    remainder = remainder - (double)o1 * 20.0;
    o2 = (int)(remainder / 2.0);
    remainder = remainder - 2.0 * (double)o2;
    o3 = (int)(12.0 * remainder);

    // latitude
    remainder = lat + 90.0;
    a1 = (int)(remainder / 10.0);
    remainder = remainder - (double)a1 * 10.0;
    a2 = (int)(remainder);
    remainder = remainder - (double)a2;
    a3 = (int)(24.0 * remainder);

    WSPRData->MaidenHead4[0] = (char)o1 + 'A';
    WSPRData->MaidenHead4[1] = (char)a1 + 'A';
    WSPRData->MaidenHead4[2] = (char)o2 + '0';
    WSPRData->MaidenHead4[3] = (char)a2 + '0';
    WSPRData->MaidenHead4[4] = 0;
    WSPRData->MaidenHead6[0] = (char)o1 + 'A';
    WSPRData->MaidenHead6[1] = (char)a1 + 'A';
    WSPRData->MaidenHead6[2] = (char)o2 + '0';
    WSPRData->MaidenHead6[3] = (char)a2 + '0';
    WSPRData->MaidenHead6[4] = (char)o3 + 'A';
    WSPRData->MaidenHead6[5] = (char)a3 + 'A';
    WSPRData->MaidenHead6[6] = 0;
}
