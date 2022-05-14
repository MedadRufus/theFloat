#include "filter_management.hpp"
#include "defines.hpp"
#include "state_machine.hpp"

extern uint8_t CurrentLP;         // Keep track on what Low Pass filter is currently switched in
extern S_FactoryData FactoryData; // TODO: replace with getters and setters

uint8_t BandNumOfHigestLP();

// Pulls the correct relays to choose LP filter A,B,C or D
void DriveLPFilters()
{
    if ((Product_Model == 1017) || (Product_Model == 1028))
    {
        // If its the WSPR-TX Mini or Pico then do nothing as they dont have any relays
    }
    else
    {
        SendAPIUpdate(UMesLPF);
        // Product model 1011 E.g WSPR-TX LP1, this will drive the relays on the optional Mezzanine LP4 and Mezzanine BLP4 cards
        if ((Product_Model == 1011) || (Product_Model == 1020) || (Product_Model == 1029))
        {
            switch (CurrentLP)
            {
            case LP_A:
                // all relays are at rest
                digitalWrite(Relay2, LOW);
                digitalWrite(Relay3, LOW);
                break;

            case LP_B:
                digitalWrite(Relay2, HIGH);
                digitalWrite(Relay3, LOW);
                break;

            case LP_C:
                digitalWrite(Relay2, LOW);
                digitalWrite(Relay3, HIGH);
                break;

            case LP_D:
                digitalWrite(Relay2, HIGH);
                digitalWrite(Relay3, HIGH);
                break;

            } // Case
        }     // If Product_Model == 1011
        else
        {
            // is not Product Model 1011 and is Hardware version 1.4 E.g en early model of the Desktop transmitter
            if ((FactoryData.HW_Version == 1) && (FactoryData.HW_Revision == 4)) // Early Hardware has different relay driving
            {
                switch (CurrentLP)
                {
                case LP_A:
                    // all relays are at rest
                    pinMode(Relay1, INPUT); // Set Relay1 as Input to deactivate the relay
                    pinMode(Relay2, INPUT); // Set Relay2 as Input to deactivate the relay
                    pinMode(Relay3, INPUT); // Set Relay3 as Input to deactivate the relay
                    break;

                case LP_B:
                    pinMode(Relay1, OUTPUT); // Set Relay1 as Output so it can be pulled low
                    digitalWrite(Relay1, LOW);
                    pinMode(Relay2, INPUT); // Set Relay2 as Input to deactivate the relay
                    pinMode(Relay3, INPUT); // Set Relay3 as Input to deactivate the relay
                    break;

                case LP_C:
                    pinMode(Relay1, INPUT);  // Set Relay1 as Input to deactivate the relay
                    pinMode(Relay2, INPUT);  // Set Relay2 as Input to deactivate the relay
                    pinMode(Relay3, OUTPUT); // Set Relay3 as Output so it can be pulled low
                    digitalWrite(Relay3, LOW);
                    break;

                case LP_D:
                    pinMode(Relay1, INPUT);  // Set Relay1 as Input to deactivate the relay
                    pinMode(Relay2, OUTPUT); // Set Relay2 as Output so it can be pulled low
                    digitalWrite(Relay2, LOW);
                    pinMode(Relay3, OUTPUT); // Set Relay3 as Output so it can be pulled low
                    digitalWrite(Relay3, LOW);
                    break;
                }
            }
            else
            {
                // Not Product Model 1011 and not Hardvare version 1.4 E.g later model of the Desktop transmitter
                switch (CurrentLP)
                {
                case LP_A:
                    // all relays are at rest
                    digitalWrite(Relay1, LOW);
                    digitalWrite(Relay2, LOW);
                    digitalWrite(Relay3, LOW);
                    break;

                case LP_B:
                    digitalWrite(Relay1, HIGH);
                    digitalWrite(Relay2, LOW);
                    digitalWrite(Relay3, LOW);
                    break;

                case LP_C:
                    digitalWrite(Relay1, LOW);
                    digitalWrite(Relay2, LOW);
                    digitalWrite(Relay3, HIGH);
                    break;

                case LP_D:
                    digitalWrite(Relay1, LOW);
                    digitalWrite(Relay2, HIGH);
                    digitalWrite(Relay3, HIGH);
                    break;
                }
            }
        }
    }
}

// Out of the four possible LP filters fitted - find the one that is best for Transmission on TXBand
void PickLP(uint8_t TXBand)
{
    boolean ExactMatch = false;
    uint8_t BandLoop;

    // Check if some of the four low pass filters is an exact match for the TXBand
    if (FactoryData.LP_A_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_A;
    }
    if (FactoryData.LP_B_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_B;
    }
    if (FactoryData.LP_C_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_C;
    }
    if (FactoryData.LP_D_BandNum == TXBand)
    {
        ExactMatch = true;
        CurrentLP = LP_D;
    }

    // If we did not find a perfect match then use a low pass filter that is higher in frequency.
    if (!ExactMatch)
    {
        for (BandLoop = TXBand; BandLoop < 99; BandLoop++) // Test all higher bands to find a a possible LP filter in one of the four LP banks
        {
            if (FactoryData.LP_A_BandNum == BandLoop) // The LP filter in Bank A is a match for this band
            {
                CurrentLP = LP_A;
                break;
            }
            if (FactoryData.LP_B_BandNum == BandLoop) // The LP filter in Bank B is a match for this band
            {
                CurrentLP = LP_B;
                break;
            }
            if (FactoryData.LP_C_BandNum == BandLoop) // The LP filter in Bank C is a match for this band
            {
                CurrentLP = LP_C;
                break;
            }
            if (FactoryData.LP_D_BandNum == BandLoop) // The LP filter in Bank D is a match for this band
            {
                CurrentLP = LP_D;
                break;
            }
        }
        // If there is no LP that is higher than TXBand then use the highest one, (not ideal as output will be attenuated but best we can do)
        if (BandLoop == 99)
        {
            TXBand = BandNumOfHigestLP();
            if (FactoryData.LP_A_BandNum == TXBand)
            {
                CurrentLP = LP_A;
            }
            if (FactoryData.LP_B_BandNum == TXBand)
            {
                CurrentLP = LP_B;
            }
            if (FactoryData.LP_C_BandNum == TXBand)
            {
                CurrentLP = LP_C;
            }
            if (FactoryData.LP_D_BandNum == TXBand)
            {
                CurrentLP = LP_D;
            }
        }
    }
    DriveLPFilters();
}

// Returns a band that is the highest band that has a LP filter fitted onboard.
// Low pass filter numbering corresponds to Bands or two special cases
// The special cases are: 98=just a link between input and output, 99=Nothing fitted (open circut) the firmware will never use this
// These numbers are set by the factory Configuration program and stored in EEPROM
uint8_t BandNumOfHigestLP()
{
    uint8_t BandLoop, Result;
    Result = FactoryData.LP_A_BandNum; // Use this filter if nothing else is a match.
    // Find the highest band that has a Low Pass filter fitted in one of the four LP banks
    for (BandLoop = 98; BandLoop > 0; BandLoop--)
    {
        if (FactoryData.LP_A_BandNum == BandLoop) // The LP filter in Bank A is a match for this band
        {
            Result = FactoryData.LP_A_BandNum;
            break;
        }
        if (FactoryData.LP_B_BandNum == BandLoop) // The LP filter in Bank B is a match for this band
        {
            Result = FactoryData.LP_B_BandNum;
            break;
        }
        if (FactoryData.LP_C_BandNum == BandLoop) // The LP filter in Bank C is a match for this band
        {
            Result = FactoryData.LP_C_BandNum;
            break;
        }
        if (FactoryData.LP_D_BandNum == BandLoop) // The LP filter in Bank D is a match for this band
        {
            Result = FactoryData.LP_D_BandNum;
            break;
        }
    }
    return Result;
}