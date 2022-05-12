#include "string_operations.hpp"

uint64_t StrTouint64_t(String InString)
{
    uint64_t y = 0;

    for (uint16_t i = 0; i < InString.length(); i++)
    {
        char c = InString.charAt(i);
        if (c < '0' || c > '9')
            break;
        y *= 10;
        y += (c - '0');
    }
    return y;
}

String uint64ToStr(uint64_t p_InNumber, boolean p_LeadingZeros)
{
    char l_HighBuffer[7];    // 6 digits + null terminator char
    char l_LowBuffer[7];     // 6 digits + null terminator char
    char l_ResultBuffer[13]; // 12 digits + null terminator char
    String l_ResultString = "";
    uint8_t l_Digit;

    sprintf(l_HighBuffer, "%06llu", p_InNumber / 1000000L); // Convert high part of 64bit unsigned integer to char array
    sprintf(l_LowBuffer, "%06llu", p_InNumber % 1000000L);  // Convert low part of 64bit unsigned integer to char array
    l_ResultString = l_HighBuffer;
    l_ResultString = l_ResultString + l_LowBuffer; // Copy the 2 part result to a string

    if (!p_LeadingZeros) // If leading zeros should be removed
    {
        l_ResultString.toCharArray(l_ResultBuffer, 13);
        for (l_Digit = 0; l_Digit < 12; l_Digit++)
        {
            if (l_ResultBuffer[l_Digit] == '0')
            {
                l_ResultBuffer[l_Digit] = ' '; // replace zero with a space character
            }
            else
            {
                break; // We have found all the leading Zeros, exit loop
            }
        }
        l_ResultString = l_ResultBuffer;
        l_ResultString.trim(); // Remove all leading spaces
    }
    return l_ResultString;
}