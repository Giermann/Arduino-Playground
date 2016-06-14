#ifndef __SHUTTER_H
#define __SHUTTER_H

#ifdef _WINDOWS
  #define LED_ON HIGH
  #define LED_OFF LOW

  #define relayPin1      4      // the pin numbers for all 4 relays
  #define _DEBUG
  //#define _BLINK
  #ifdef _BLINK
    #define led_PIN      3
    #define relayPin2    2
  #else
    #define relayPin2    3
  #endif
  #define relayPin3      1
  #define relayPin4      2
  // below are dummies for Windows
  #define buttonAPin     0
  #define buttonDPin     5
  #define onewirePin     0
  #define onewireClnt 0x0C
//#elif defined( __AVR_ATtiny25__ ) | \
//     defined( __AVR_ATtiny45__ ) | \
//     defined( __AVR_ATtiny85__ )
#elif defined( ARDUINO_attiny )
  // The led is connected so that the tiny sinks current
  #define LED_ON  LOW
  #define LED_OFF HIGH

  #define relayPin1      4      // the pin numbers for all 4 relays
  //#define _BLINK
  #ifdef _BLINK
    #define led_PIN      3
    #define relayPin2    2
  #else
    #define relayPin2    3
  #endif
  #define relayPin3      1
  #define relayPin4      2
  #define buttonAPin     0      // the number of the pin for push button(s) ADC (ADC0 = 6, ADC1 = 2, ADC2 = 4, ADC3 = 3)
  #define buttonDPin     5      // the number of the pin for push button(s) digital state
  #define onewirePin     0      // the number of the pin for 1-Wire Slave
  #define onewireClnt 0x0C
#else // Arduino Nano
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


#define S_IDLE            0x00
#define S_MOVE_OPEN       0x01
#define S_MOVE_CLOSE      0x02

// restore EEPROM to default and set client ID:
// # ID=85
// # owwrite /FC.0C0000000000/910/alarm 113 ; owwrite /FC.0C0000000000/910/alarmc $ID ; owwrite /FC.0C0000000000/910/alarm 111 ; sleep 5 ; owdir /uncached ; ID=$(expr $ID + 1) ; echo next ID = $ID

#define crcEEPROM       bae910.memory.field.reserved    // [uint16_t] used for EEPROM saving
#define clientID        bae910.memory.field.alarmc      // [uint8_t]  1 wire client ID (maybe duty4?)
#define controlEEPROM   bae910.memory.field.alarm       // [uint8_t] used to (re)store EEPROM data
#define CTRL_LockButtonMode   66                        //  = lock buttons, only remote control
#define CTRL_SimulateMode     77                        //  = don't act on relays, but update states and positions
#define CTRL_RestorePos       78                        //  = restore/set currentPos via 1-Wire after reset condition
#define CTRL_TestMode         80                        //  = test relay mode (lower 4 bits for relay state: 80..95)
#define CTRL_SomfySetupMode   96                        //  = enter setup mode (both relays for setupLength, lower 2 bits for window: 97 = 1, 98 = 2, 99 = both)
#define CTRL_SomfyDefaultRst 100                        //  = reset default config (both relays for 2x setupLength, lower 2 bits for window: 101 = 1, 102 = 2, 103 = both)
#define CTRL_SomfyAutoSetup  104                        //  = auto end setup (lower 2 bits for window: 105 = 1, 106 = 2, 107 = both)
#define CTRL_Save2EEPROM     111                        //  = store config to EEPROM
#define CTRL_ReloadEEPROM    112                        //  = reload config from EEPROM
#define CTRL_RestoreDefaults 113                        //  = restore default values (no EEPROM change)

#define currentMillis   bae910.memory.field.rtc         // [uint32_t]
#define internalState1  bae910.memory.field.userc       // [uint8_t]
#define internalState2  bae910.memory.field.userd
#define currentButton   bae910.memory.field.pio         // [uint8_t]  1 or 2; 3 = 1+2
#define tempButton      bae910.memory.field.unused0x35
#define lastButtonADC   bae910.memory.field.adc10       // [uint16_t, read-only] last ADC value read

#define lastTempButton  bae910.memory.field.count       // [uint32_t]
#define lastPosChange1  bae910.memory.field.adctotp
#define lastPosChange2  bae910.memory.field.adctotn
#define lastStateChg1   bae910.memory.field.alct
#define lastStateChg2   bae910.memory.field.alrt
#define lastBtnEvent1   bae910.memory.field.userm       // [uint32_t]
#define lastBtnEvent2   bae910.memory.field.usern
#define lastLongpress1  bae910.memory.field.usero
#define lastLongpress2  bae910.memory.field.userp
#define lastMoveState1  bae910.memory.field.usere       // [uint8_t]  last direction moved
#define lastMoveState2  bae910.memory.field.userf
#define lastMoveLength1 bae910.memory.field.adcap       // [uint16_t, read-only] duration of last movement
#define lastMoveLength2 bae910.memory.field.adcan

#define invalidPos      101
#define switchStopPos   150                             // 100 < 'full open first' < switchStopPos < 'full close first'
#ifdef _WINDOWS
  #define currentPos1   myGlobalWnd->Position1
  #define currentPos2   myGlobalWnd->Position2
#else
  #define currentPos1   bae910.memory.field.adc         // [uint8_t, read-only]  0 = opened, 100 = closed
  #define currentPos2   bae910.memory.field.cnt
#endif
#define stopPos1        bae910.memory.field.usera       // [uint8_t]  0..100 | 101..200 = move to one end and back to (stopPos - 100)
#define stopPos2        bae910.memory.field.userb

#define defaultDirOpen  bae910.memory.field.outc        // [uint8_t]  single button mode: default to up direction if position >50%
#define positionFactor1 bae910.memory.field.period1     // [uint16_t] time in ms for 1% movement
#define positionFactor2 bae910.memory.field.period2

#define btnOpenPos1     bae910.memory.field.adcc        // [uint8_t]  move to this position on CLOSE button (move full and back if 0x80 bit set)
#define btnOpenPos2     bae910.memory.field.cntc
#define btnClosePos1    bae910.memory.field.tpm1c
#define btnClosePos2    bae910.memory.field.tpm2c

#define buttonSettle    bae910.memory.field.rtcc        // [uint8_t]  state must not change in this time to be treated as valid

#define buttonLongpress bae910.memory.field.duty1       // [uint16_t] minimum length for long button press
#define holdLongpress   bae910.memory.field.duty2       // [uint16_t] hold long press mode after button release for ...ms
#define directionReset  bae910.memory.field.duty3       // [uint16_t] single button mode: move up on second request, reset to down after 15s (0 = never)
#define setupLength     bae910.memory.field.duty4       // [uint16_t] duration for both relays ON to enter setup mode
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
    uint16_t userj;
    uint16_t useri;
    uint8_t  userh;
    uint8_t  userg;
    uint16_t maxcps;
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
    uint16_t cps;   // read-only
    uint8_t  pioc   // in (copied) EEPROM!
*/

#endif // __SHUTTER_H
