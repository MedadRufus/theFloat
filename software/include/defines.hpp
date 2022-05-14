// Constants
#define I2C_START 0x08
#define I2C_START_RPT 0x10
#define I2C_SLA_W_ACK 0x18
#define I2C_SLA_R_ACK 0x40
#define I2C_DATA_ACK 0x28
#define SI5351A_H

#define SI_CLK0_CONTROL 16 // Register definitions
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_PLL_RESET 177

#define SI_R_DIV_1 0b00000000 // R-division ratio definitions
#define SI_R_DIV_2 0b00010000
#define SI_R_DIV_4 0b00100000
#define SI_R_DIV_8 0b00110000
#define SI_R_DIV_16 0b01000000
#define SI_R_DIV_32 0b01010000
#define SI_R_DIV_64 0b01100000
#define SI_R_DIV_128 0b01110000

#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000

#define FactorySpace true
#define UserSpace false

#define UMesCurrentMode 1
#define UMesLocator 2
#define UMesTime 3
#define UMesGPSLock 4
#define UMesNoGPSLock 5
#define UMesFreq 6
#define UMesTXOn 7
#define UMesTXOff 8
#define UMesLPF 9
#define UMesVCC 10
#define UMesWSPRBandCycleComplete 11

// Hardware defines

#define Relay1 5
#define Relay2 6
#define Relay3 7
#define TransmitLED 8 // Red LED next to RF out SMA that will turn on when Transmitting (Pico model do not have a TX LED)
#define SiPower A3    // Power the Si5351 from this pin on the WSPR-TX Mini

// Product model. WSPR-TX_LP1                             =1011
// Product model. WSPR-TX Desktop                         =1012
// Product model. WSPR-TX Mini                            =1017
// Product model. WSPR-TX_LP1 with Mezzanine LP4 card     =1020
// Product model. SSG                                     =1024
// Product model. WSPR-TX Pico                            =1028
// Product model. WSPR-TX_LP1 with Mezzanine BLP4 card    =1029
#define Product_Model 1012

#define SoftwareVersion 1   // 0 to 255. 0=Beta
#define SoftwareRevision 12 // 0 to 255

#define LP_A 0
#define LP_B 1
#define LP_C 2
#define LP_D 3
