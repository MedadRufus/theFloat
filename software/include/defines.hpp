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

#define WSPR_FREQ23cm 129650150000ULL // 23cm 1296.501,500MHz (Overtone, not implemented)
#define WSPR_FREQ70cm 43230150000ULL  // 70cm  432.301,500MHz (Overtone, not implemented)
#define WSPR_FREQ2m 14449500000ULL    // 2m    144.490,000MHz //Not working. No decode in bench test with WSJT-X decoding Software
#define WSPR_FREQ4m 7009250000ULL     // 4m     70.092,500MHz //Slightly lower output power
#define WSPR_FREQ6m 5029450000ULL     // 6m     50.294,500MHz //Slightly lower output power
#define WSPR_FREQ10m 2812610000ULL    // 10m    28.126,100MHz
#define WSPR_FREQ12m 2492610000ULL    // 12m    24.926,100MHz
#define WSPR_FREQ15m 2109610000ULL    // 15m    21.096.100MHz
#define WSPR_FREQ17m 1810610000ULL    // 17m    18.106,100MHz
#define WSPR_FREQ20m 1409710000ULL    // 20m    14.097,100MHz
#define WSPR_FREQ30m 1014020000ULL    // 30m    10.140,200MHz
#define WSPR_FREQ40m 704010000ULL     // 40m     7.040,100MHz
#define WSPR_FREQ80m 357010000ULL     // 80m     3.570,100MHz
#define WSPR_FREQ160m 183810000ULL    // 160m    1.838,100MHz
#define WSPR_FREQ630m 47570000ULL     // 630m      475.700kHz
#define WSPR_FREQ2190m 13750000ULL    // 2190m     137.500kHz

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
#define GPSPower A1   // Sleep-Wake signal of the GPS on the WSPR-TX Pico
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
