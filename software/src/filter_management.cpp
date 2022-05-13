#include "filter_management.hpp"
#include "defines.hpp"
#include "state_machine.hpp"

extern uint8_t CurrentLP;         // Keep track on what Low Pass filter is currently switched in
extern S_FactoryData FactoryData; // TODO: replace with getters and setters

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
