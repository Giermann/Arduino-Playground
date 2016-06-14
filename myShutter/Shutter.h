#ifndef __SHUTTER_H
#define __SHUTTER_H
/*
#define COMMAND_GET_STATUS   0xA0
#define COMMAND_GET_POSITION 0xA1
#define COMMAND_GET_LEN_DOWN 0xA2
#define COMMAND_GET_LEN_UP   0xA3
#define COMMAND_SET_LEN_DOWN 0xA4
#define COMMAND_SET_LEN_UP   0xA5
#define COMMAND_MOVE_DOWN    0xB0
#define COMMAND_MOVE_UP      0xB1
*/
#ifdef _WINDOWS
#define LED_ON HIGH
#define LED_OFF LOW

#define relayPin1      4      // the pin numbers for all 4 relays
#define _DEBUG
//#define _BLINK
#ifdef _BLINK
#define led_PIN        3
#define relayPin2      1
#define relayPin3      2
#else
#define relayPin2      3
#define relayPin3      1
#endif
#define relayPin4      2
#define buttonAPin     0      // the number of the pin for push button(s) ADC (ADC0 = 14 .. ADC5 = 19)
#define buttonDPin     5      // the number of the pin for push button(s) digital state
#define onewirePin     0      // the number of the pin for 1-Wire Slave
#define onewireClnt 0x0C
#elif defined( ARDUINO_attiny )
//#if defined( __AVR_ATtiny25__ ) | \
//     defined( __AVR_ATtiny45__ ) | \
//     defined( __AVR_ATtiny85__ )
//#define _OW_SLAVE
//#define _I2C_SLAVE
//#define _BS_SLAVE

// The led is connected so that the tiny sinks current
#define LED_ON LOW
#define LED_OFF HIGH

#define relayPin1      4      // the pin numbers for all 4 relays
//#define _BLINK
#ifdef _BLINK
#define led_PIN        3
#define relayPin2      1
#define relayPin3      2
#else
#define relayPin2      3
#define relayPin3      1
#endif
#define relayPin4      2
#define buttonAPin     0      // the number of the pin for push button(s) ADC (ADC0 = 6, ADC1 = 2, ADC2 = 4, ADC3 = 3)
#define buttonDPin     5      // the number of the pin for push button(s) digital state
#define onewirePin     0      // the number of the pin for 1-Wire Slave
#define onewireClnt 0x0C
#else
#define _DEBUG
#define _T(x) (x)
//  #define _OW_SLAVE
//#define _I2C_SLAVE
//#define _BS_SLAVE

#define LED_ON HIGH
#define LED_OFF LOW

#define _BLINK
#define led_PIN       13
#define relayPin1      4      // the pin numbers for all 4 relays
#define relayPin2      3
#define relayPin3      1
#define relayPin4      2
#define buttonAPin     1      // the number of the pin for push button(s) ADC (ADC0 = 14 .. ADC5 = 19)
#define buttonDPin    15      // the number of the pin for push button(s) digital state
#define onewirePin    18      // the number of the pin for 1-Wire Slave (2 is working)
#define onewireClnt 0x0D
#endif

// only used locally, so keep separated from bae910
uint8_t  tempButton;
uint8_t  newButton;

#define S_IDLE            0x00
#define S_MOVE_OPEN       0x01
#define S_MOVE_CLOSE      0x02

#define crcEEPROM       bae910.memory.field.reserved    // [uint16_t] used for EEPROM saving
#define clientID        bae910.memory.field.alarmc      // [uint8_t]  1 wire client ID (maybe duty4?)
#define controlEEPROM   bae910.memory.field.maxcps      // [uint16_t] used to (re)store EEPROM data
#define CTRL_Save2EEPROM    4321                        //  = store config to EEPROM
#define CTRL_RestoreEEPROM  4711                        //  = reload config from EEPROM
#define CTRL_ResetConfig     815                        //  = restore default values (no EEPROM change)
//#define CTRL_Save2EEPROM    0xBCBC                      //  = store config to EEPROM
//#define CTRL_RestoreEEPROM  0xBEEF                      //  = reload config from EEPROM
//#define CTRL_ResetConfig    0xDEAD                      //  = restore default values (no EEPROM change)

#define currentMillis   bae910.memory.field.rtc         // [uint32_t]
#define internalState1  bae910.memory.field.userc       // [uint8_t]
#define internalState2  bae910.memory.field.userd
#define currentButton   bae910.memory.field.pio         // [uint8_t]  1 or 2; 3 = 1+2
#define lastButtonADC   bae910.memory.field.adc10       // [uint16_t] last ADC value read

#define lastTempButton  bae910.memory.field.count       // [uint32_t]
#define lastPosChange1  bae910.memory.field.adctotp
#define lastPosChange2  bae910.memory.field.adctotn
#define lastStateChg1   bae910.memory.field.alct
#define lastStateChg2   bae910.memory.field.alrt
#define lastBtnEvent1   bae910.memory.field.userm       // [uint32_t]
#define lastBtnEvent2   bae910.memory.field.usern
#define lastLongpress1  bae910.memory.field.usero
#define lastLongpress2  bae910.memory.field.userp
#define lastMoveLength1 bae910.memory.field.useri       // [uint16_t] duration of last movement
#define lastMoveLength2 bae910.memory.field.userj
#define lastMoveState1  bae910.memory.field.usere       // [uint8_t]  last direction moved
#define lastMoveState2  bae910.memory.field.userf
//#ifdef _WINDOWS
//  #define currentPos1   myGlobalWnd->Position1
//  #define currentPos2   myGlobalWnd->Position2
//#else
  #define currentPos1   bae910.memory.field.adc         // [uint8_t]  0 = opened, 100 = closed
  #define currentPos2   bae910.memory.field.cnt
//#endif
#define stopPos1        bae910.memory.field.usera       // [uint8_t]  0..100 | 101..200 = move to one end and back to (200 - stopPos)
#define stopPos2        bae910.memory.field.userb

#define defaultDirOpen  bae910.memory.field.outc        // [uint8_t]  single button mode: default to up direction if position >70%
#define positionFactor1 bae910.memory.field.period1     // [uint16_t] time in ms for 1% movement
#define positionFactor2 bae910.memory.field.period2

#define btnOpenPos1     bae910.memory.field.adcc        // [uint8_t]  move to this position on CLOSE button (move full and back if 0x80 bit set)
#define btnOpenPos2     bae910.memory.field.cntc
#define btnClosePos1    bae910.memory.field.tpm1c
#define btnClosePos2    bae910.memory.field.tpm2c
// obsolete!
//#define postrunLength   bae910.memory.field.duty4       // [uint16_t] keep on moving for 4s, if we think the end is reached

#define buttonSettle    bae910.memory.field.rtcc        // [uint8_t]  state must not change in this time to be treated as valid

#define buttonLongpress bae910.memory.field.duty1       // [uint16_t] minimum length for long button press
#define holdLongpress   bae910.memory.field.duty2       // [uint16_t] hold long press mode after button release for ...ms
#define directionReset  bae910.memory.field.duty3       // [uint16_t] single button mode: move up on second request, reset to down after 15s
#define minADCidle      bae910.memory.field.ovruncnt    // [uint16_t] ADC above this value is being treated as idle
#define minADCbutton1   bae910.memory.field.maxap       // [uint16_t] ADC above is button 1, below is button 2
#define minADCbutton2   bae910.memory.field.maxan       // [uint16_t] ADC above is button 2, below is both buttons pressed

#define relayState      bae910.memory.field.out         // [uint8_t]
#define relayAssignment bae910.memory.field.stalledcnt  // [uint16_t]

/* unused fields left:
    uint32_t unused0x52;
    uint32_t unused0x4e;
    uint16_t userl;
    uint16_t userk;
    uint8_t  userh;
    uint8_t  userg;
//    uint8_t  userf;
//    uint8_t  usere;
    uint16_t selectcnt;
    uint16_t resetcnt;
    uint16_t alcps;
    uint16_t alan;
    uint16_t alap;
    //
    uint16_t pc3;
    uint16_t pc2;
    uint16_t pc1;
    uint16_t pc0;
    uint8_t  unused0x35;
    uint8_t  alarm;
    uint16_t cps;   // read-only
    uint16_t adcan; // read-only
    uint16_t adcap; // read-only
    uint16_t duty4; // in (copied) EEPROM!
    uint8_t  pioc   // in (copied) EEPROM!
*/

#define timeDiff(previous) (unsigned long)(currentMillis - previous)

#endif // __SHUTTER_H
