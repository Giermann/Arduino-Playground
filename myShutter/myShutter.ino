/*
 *    Example-Code that emulates various Sensor - mostly for development
 *    --> attach sensors as needed
 *    Tested with https://github.com/PaulStoffregen/OneWire on the other side as Master
 */

#include "Shutter.h"

#ifndef _WINDOWS
  #include <EEPROM.h>
  #include "OneWireHub.h"
#else
  class EEPROMC {
  public:
    static uint8_t read(uint16_t addr) { return 0; };
    static void update(uint16_t addr, uint8_t val) {};
  } EEPROM;
  class OneWireHub {
  public:
    OneWireHub(...) {};

    void attach(...) {};
    void detach(...) {};

    void poll(void) {};
    void printError(void) {};
    bool getError(void) { return false; };
  };
#endif // _WINDOWS

#include "BAE910.h"  // 3rd party device

OneWireHub hub  = OneWireHub(onewirePin);
BAE910 bae910   = BAE910(BAE910::family_code, onewireClnt, 0x00, 0x00, 0x00, 0x00, 0x00);
//auto hub      = OneWireHub(onewirePin);
//auto bae910   = BAE910(BAE910::family_code, onewireClnt, 0x00, 0x00, 0x00, 0x00, 0x00);

#ifdef _BLINK
/*
#ifdef ARDUINO_attiny
  #define onewireClnt 0x0C
  // ds2413:  /uncached/3A.0C0204010300
  const uint8_t led_PIN       = 3;         // the number of the LED pin
  const uint8_t onewirePin   = 0;
#else
  #define onewireClnt 0x0D
  // ds2413:  /uncached/3A.0D0204010300
  #define _BLINK
  const uint8_t led_PIN       = 13;         // the number of the LED pin
  const uint8_t onewirePin   = 18;         // 18 = A4
#endif
*/
static uint8_t cntBlink = 0;
#ifdef _HEARTBEAT
  uint32_t lastHB = 0;
#endif
static uint8_t ledState = LOW;      // ledState used to set the LED
static uint32_t nextMillis  = millis();     // will store next time LED will updated
short numBlink = 0;
#endif



//
// check old and new button state
// TODO: is buttonInterval obsolete since buttonSettle ?
// ISSUE: stop moving with button press causes holdLongpress to begin...
// ISSUE: pressing OPEN in S_MOVE_CLOSE only stops without moving up...
//
void handleButton(uint8_t internalState, uint8_t currentPos, bool currentBtn, bool newBtn, uint32_t &lastBtnEvent, uint8_t &stopPos, uint8_t btnOpenPos, uint8_t btnClosePos = 0, uint32_t lastMoveOpen = 0, uint32_t lastMoveClose = 0)
{
    if (newBtn && !currentBtn && (timeDiff(lastBtnEvent) > buttonInterval)) {
        // button down event
        if (timeDiff(lastBtnEvent) > holdLongpress) {
            lastBtnEvent = currentMillis;
        } else {
#ifdef _DEBUG
            Serial.print(_T("fake long press inside hold timespan: "));
            Serial.println(timeDiff(lastBtnEvent));
#endif
            // fake long press
            lastBtnEvent = currentMillis - buttonLongpress;
        }

        // guess desired direction
        if (internalState != S_IDLE) {
            stopPos = currentPos;
        } else if (currentPos < defaultDirOpen) {
            stopPos = ((lastMoveClose > lastMoveOpen) && (timeDiff(lastMoveClose) < directionReset)) ? btnOpenPos : btnClosePos;
        } else {
            stopPos = ((lastMoveOpen > lastMoveClose) && (timeDiff(lastMoveOpen) < directionReset)) ? btnClosePos : btnOpenPos;
        }
#ifdef _DEBUG
        Serial.print(_T("button down event, internalState (old) = "));
        Serial.print(internalState);
        Serial.print(_T(" - (new) stopPos = "));
        Serial.println(stopPos);
#endif
    }
    else if (!newBtn && currentBtn && (timeDiff(lastBtnEvent) > buttonInterval)) {
        // button up event, keep moving on short press
        if (timeDiff(lastBtnEvent) > buttonLongpress) {
#ifdef _DEBUG
           Serial.print(_T("button up after long press: "));
           Serial.println(timeDiff(lastBtnEvent));
#endif
           stopPos = currentPos;
           // TODO: remember last long press to avoid issue after press for stop
        }
        lastBtnEvent = currentMillis;
#ifdef _DEBUG
        Serial.print(_T("button up event, internalState (old) = "));
        Serial.print(internalState);
        Serial.print(_T(" - (new) stopPos = "));
        Serial.println(stopPos);
#endif
    }
}

//
// calculate new position and decide what to do next
//
void updateShutterPos(uint8_t internalState, uint8_t &currentPos, uint8_t &stopPos, uint32_t &lastPosChange, uint16_t positionFactor)
{
    //
    // TODO: change internalState on direction change?
    //
    if (internalState == S_MOVE_OPEN) {
        while ((currentPos > 0) && (timeDiff(lastPosChange) > positionFactor)) {
            lastPosChange += positionFactor;
            currentPos--;
#ifdef _DEBUG
            Serial.print(_T("position: "));
            Serial.println(currentPos);
#endif
        }
        if (stopPos > 100) {
            // now move to the opposite direction, if not in longpress mode
            // TODO: do we need to switch direction here, if checked below?
//            if (currentPos < 1) stopPos = currentBtn ? currentPos : (stopPos - 100);
        } else if (currentPos < stopPos) {
            // moved too far, but then stop now
            stopPos = currentPos;
        }
    } else if (internalState == S_MOVE_CLOSE) {
        while ((currentPos < 100) && (timeDiff(lastPosChange) > positionFactor)) {
            lastPosChange += positionFactor;
            currentPos++;
#ifdef _DEBUG
            Serial.print(_T("position: "));
            Serial.println(currentPos);
#endif
        }
        if (stopPos > 100) {
            // now move to the opposite direction, if not in longpress mode
            // TODO: do we need to switch direction here, if checked below?
//            if (currentPos > 99) stopPos = currentBtn ? currentPos : (stopPos - 100);
        } else if (currentPos > stopPos) {
            // moved too far, but then stop now
            stopPos = currentPos;
        }
    }
}

//
// calculate new internalState, trace timings
//
void updateState(uint8_t &internalState, uint8_t currentPos, uint8_t &stopPos, bool currentBtn, uint32_t &lastPosChange, uint32_t &lastStateChg, uint16_t &lastMoveLength)
{
    uint8_t newState = internalState;

    // keep state if button still pressed
    if (currentBtn && (internalState != S_IDLE)) return;

    // determine new state from stopPos
    if (stopPos == currentPos) {
        newState = S_IDLE;
    } else if (stopPos > 100) {
        if (internalState == S_IDLE) {
            newState = (stopPos > 150) ? S_MOVE_CLOSE : S_MOVE_OPEN;
            if (stopPos > 150) {
                // first CLOSE to 100, don't move back when inside this range
                newState = S_MOVE_CLOSE;
                if (currentPos >= (stopPos - 100)) stopPos = 100;
            } else {
                // first OPEN to 0, don't move back when inside this range
                newState = S_MOVE_OPEN;
                if (currentPos <= (stopPos - 100)) stopPos = 0;
            }
        } else if ((internalState == S_MOVE_CLOSE) && (currentPos > 99)) {
            newState = S_MOVE_OPEN;
            stopPos -= 100;
        } else if ((internalState == S_MOVE_OPEN) && (currentPos < 1)) {
            newState = S_MOVE_CLOSE;
            stopPos -= 100;
        }
    } else if (stopPos < currentPos) {
        newState = S_MOVE_OPEN;
    } else {
        newState = S_MOVE_CLOSE;
    }

    // trace state change timings
    if (internalState != newState) {
        if (internalState != S_IDLE) {
            // stop moving
            lastMoveLength = timeDiff(lastStateChg);
        } else {
            // start moving
            lastPosChange = currentMillis;
#ifdef _DEBUG
            Serial.print(_T("currentPos = "));
            Serial.print(currentPos);
            Serial.print(_T(", stopPos = "));
            Serial.print(stopPos);
            Serial.println(_T(" - start moving..."));
#endif
        }
        lastStateChg = currentMillis;
        internalState = newState;
    }
}

//
// main shutter function
//
void checkShutter()
{
    currentMillis = millis();
    lastButtonADC = analogRead(buttonAPin);
    newButton     = (lastButtonADC < minADCbutton2 ? 0x03 : (lastButtonADC < minADCbutton1 ? 0x02 : (lastButtonADC < minADCidle ? 0x01 : 0x00)));

/*
#ifdef _BLINK
    if (cntBlink < 1) {
		cntBlink = (lastButtonADC / ((numBlink == 0) ? 512 : ((numBlink == 1) ? 64 : 8))) % 8;
        if (cntBlink == 0) cntBlink = 8;
#ifdef _DEBUG
        Serial.print(_T("ADC: "));
        Serial.print(lastButtonADC);
        Serial.print(_T(", 1/8 in Oct: "));
        Serial.print((int)(lastButtonADC / 8), OCT);
        Serial.print(_T(" bit#"));
        Serial.print(numBlink);
        Serial.print(_T(" = "));
        Serial.println(cntBlink);
#endif
		numBlink++;
		numBlink%=3;

		nextMillis  = millis() + 5000;
        ledState = HIGH; // LED off
        digitalWrite(led_PIN, ledState);
    }
#endif
*/
    //
    // update shutter position first, depending on the active time of current state
    //
    updateShutterPos(internalState1, currentPos1, stopPos1, lastPosChange1, positionFactor1);
    updateShutterPos(internalState2, currentPos2, stopPos2, lastPosChange2, positionFactor2);

    //
    // SG, 08.03.2016 - try to minimize faults, ...wait for ADC to settle
    //
    if (newButton != tempButton) {
        // store last read value for next comparison
        tempButton = newButton;
        lastTempButton = currentMillis;
    } else if (timeDiff(lastTempButton) > buttonSettle) {
        // waited long enough to accept the new button state
        if (defaultDirOpen == 0) {
            //
            // 2-button, single window
            //
            handleButton(internalState1, currentPos1, (currentButton & 1), (newButton & 1), lastBtnEvent1, stopPos1, btnClosePos1);
            handleButton(internalState1, currentPos1, (currentButton & 2), (newButton & 2), lastBtnEvent2, stopPos1, btnOpenPos1);

            // do nothing when both buttons are pressed
            if ((newButton != currentButton) && ((newButton & 3) == 3)) {
                stopPos1 = currentPos1;
                //Serial.println(_T("both buttons pressed"));
            }
        } else {
            //
            // 1-button, both windows
            //
            handleButton(internalState1, currentPos1, (currentButton & 1), (newButton & 1), lastBtnEvent1, stopPos1, btnOpenPos1, btnClosePos1, lastMoveOpen1, lastMoveClose1);
            handleButton(internalState2, currentPos2, (currentButton & 2), (newButton & 2), lastBtnEvent2, stopPos2, btnOpenPos2, btnClosePos2, lastMoveOpen2, lastMoveClose2);
        }
        currentButton = newButton;
    }

    //
    // set newState according desired stopPos
    //
    updateState(internalState1, currentPos1, stopPos1, (defaultDirOpen == 0) ? (currentButton > 0) : (currentButton & 1), lastPosChange1, lastStateChg1, lastMoveLength1);
    updateState(internalState2, currentPos2, stopPos2, (defaultDirOpen == 0) ? false               : (currentButton & 2), lastPosChange2, lastStateChg2, lastMoveLength2);

    if (internalState1 == S_MOVE_OPEN)  lastMoveOpen1  = currentMillis;
    if (internalState1 == S_MOVE_CLOSE) lastMoveClose1 = currentMillis;
    if (internalState2 == S_MOVE_OPEN)  lastMoveOpen2  = currentMillis;
    if (internalState2 == S_MOVE_CLOSE) lastMoveClose2 = currentMillis;

    relayState  = ((internalState1 == S_MOVE_OPEN)  ? ( relayAssignment      & 0x0F) : 0x00);
    relayState |= ((internalState1 == S_MOVE_CLOSE) ? ((relayAssignment>>4)  & 0x0F) : 0x00);
    relayState |= ((internalState2 == S_MOVE_OPEN)  ? ((relayAssignment>>8)  & 0x0F) : 0x00);
    relayState |= ((internalState2 == S_MOVE_CLOSE) ? ((relayAssignment>>12) & 0x0F) : 0x00);

    digitalWrite(relayPin1, (relayState & 1) ? LED_ON : LED_OFF);
    digitalWrite(relayPin2, (relayState & 2) ? LED_ON : LED_OFF);
    digitalWrite(relayPin3, (relayState & 4) ? LED_ON : LED_OFF);
    digitalWrite(relayPin4, (relayState & 8) ? LED_ON : LED_OFF);
}

bool EEPROMget()
{
    crcEEPROM = 0;

    // stalledcnt, ovruncnt = relayAssignment, ADClowIdle   0x5A..0x5D
    for (uint8_t i = 0x1D; i > 19; --i) {
        bae910.memory.bytes[0x3F - i] = EEPROM.read(i);
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x3F - i], crcEEPROM);
    }

    // maxan, maxap = ADClowButton1, ADClowButton2   0x1A..0x1D
    for (uint8_t i = 0x19; i > 15; --i) {
        bae910.memory.bytes[0x7B - i] = EEPROM.read(i);
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x7B - i], crcEEPROM);
    }

    // all others 0x00..0x15
    for (uint8_t i = 0x15; i > 1; --i) {
        bae910.memory.bytes[0x7F - i] = EEPROM.read(i);
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x7F - i], crcEEPROM);
    }

    // make data version dependent ?
    //crcEEPROM = BAE910::crc16(BAE910::BAE910_SW_VER, crcEEPROM);
    //crcEEPROM = BAE910::crc16(BAE910::BAE910_BOOTSTRAP_VER, crcEEPROM);

#ifdef _DEBUG
    Serial.print(_T("Calculated checksum: "));
    Serial.print(reinterpret_cast<uint8_t *>(&crcEEPROM)[1], HEX);
    Serial.print(_T(","));
    Serial.print(reinterpret_cast<uint8_t *>(&crcEEPROM)[0], HEX);
    Serial.print(_T("   stored checksum: "));
    Serial.print(EEPROM.read(1), HEX);
    Serial.print(_T(","));
    Serial.println(EEPROM.read(0), HEX);
#endif
    // check crcEEPROM or force to restore default values
    if ((reinterpret_cast<uint8_t *>(&crcEEPROM)[1] == EEPROM.read(1)) &&
        (reinterpret_cast<uint8_t *>(&crcEEPROM)[0] == EEPROM.read(0)) &&
        (controlEEPROM != 0xDEAD)) {
        return true;
    }

    // invalid CRC, use hard coded defaults
    // TODO: change to defined values!
    bae910.memory.field.alarmc  = 0;      // 1 wire client ID
    bae910.memory.field.outc    = 0;      // default OPEN position (0x00 = 2 button mode)
                                          // (OPEN on first press when closed more than ...%)
    bae910.memory.field.duty3   = 15000;  // direction reset in ms
                                          // (default to opposite direction within this time range)
// obsolete!
//    bae910.memory.field.duty4   = 2000;   // postrun duration in ms
//                                          // (keep moving for ...ms after reaching top/bottom)

    bae910.memory.field.rtcc    = 20;     // button settle (must stay for at least ...ms)
    bae910.memory.field.pioc    = 100;    // button interval (can change after ...ms) [obsolete with button settle?]
    bae910.memory.field.duty1   = 1500;   // long press length in ms
    bae910.memory.field.duty2   = 3000;   // hold longpress mode for ...ms

    bae910.memory.field.outc    = 70;     // single button mode: default to up direction if position >70%
//    bae910.memory.field.outc    = 0 ;     // two button (single window) mode
    bae910.memory.field.adcc    = 0;      // button OPEN position A
    bae910.memory.field.cntc    = 0;      // button OPEN position B
    bae910.memory.field.tpm1c   = 194;    // button CLOSE position A  (100 and back to 94)
    bae910.memory.field.tpm2c   = 194;    // button CLOSE position B

    bae910.memory.field.period1 = 300;    // position factor A
    bae910.memory.field.period2 = 300;    // position factor B

    bae910.memory.field.maxap      = 785;    // ADClowButton1
    bae910.memory.field.maxan      = 680;    // ADClowButton2
    bae910.memory.field.ovruncnt   = 950;    // ADClowIdle

    bae910.memory.field.stalledcnt = 0x8421; // relayAssignment
    return false;
}

void EEPROMput()
{
    crcEEPROM = 0;

    // stalledcnt, ovruncnt = relayAssignment, ADClowIdle   0x5A..0x5D
    for (uint8_t i = 0x1D; i > 19; --i) {
        EEPROM.update(i, bae910.memory.bytes[0x3F - i]);
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x3F - i], crcEEPROM);
    }

    // maxan, maxap = ADClowButton1, ADClowButton2   0x1A..0x1D
    for (uint8_t i = 0x19; i > 15; --i) {
        EEPROM.update(i, bae910.memory.bytes[0x7B - i]);
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x7B - i], crcEEPROM);
    }

    // all others 0x00..0x15
    for (uint8_t i = 0x15; i > 1; --i) {
        EEPROM.update(i, bae910.memory.bytes[0x7F - i]);
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x7F - i], crcEEPROM);
    }

    // make data version dependent ?
    //crcEEPROM = BAE910::crc16(BAE910::BAE910_SW_VER, crcEEPROM);
    //crcEEPROM = BAE910::crc16(BAE910::BAE910_BOOTSTRAP_VER, crcEEPROM);

    EEPROM.update(1, reinterpret_cast<uint8_t *>(&crcEEPROM)[1]);
    EEPROM.update(0, reinterpret_cast<uint8_t *>(&crcEEPROM)[0]);
#ifdef _DEBUG
    Serial.print(_T("Calculated checksum: "));
    Serial.print(reinterpret_cast<uint8_t *>(&crcEEPROM)[1], HEX);
    Serial.print(_T(","));
    Serial.print(reinterpret_cast<uint8_t *>(&crcEEPROM)[0], HEX);
    Serial.print(_T("   stored checksum: "));
    Serial.print(EEPROM.read(1), HEX);
    Serial.print(_T(","));
    Serial.println(EEPROM.read(0), HEX);
#endif
}


void setup()
{
#ifndef ARDUINO_attiny
    Serial.begin(115200);
    Serial.println("OneWire-Hub Test with various Sensors");
#endif

    EEPROMget();

    // Setup OneWire
    bae910.ID[6] = clientID;
    hub.attach(bae910);

    // avoid holdLongpress after boot
    lastBtnEvent1 = -holdLongpress;
    lastBtnEvent2 = -holdLongpress;

    // initialize the push button pin as an input:
    pinMode(buttonDPin, INPUT_PULLUP);

    // initialize all relays as OUTPUT and set to OFF
    pinMode(relayPin1, OUTPUT);
    digitalWrite(relayPin1, LED_OFF);
    pinMode(relayPin2, OUTPUT);
    digitalWrite(relayPin2, LED_OFF);
    pinMode(relayPin3, OUTPUT);
    digitalWrite(relayPin3, LED_OFF);
    pinMode(relayPin4, OUTPUT);
    digitalWrite(relayPin4, LED_OFF);


#ifdef _BLINK
    pinMode(led_PIN, OUTPUT);

    // debug LEDs
    digitalWrite(relayPin1, LED_ON);
    delay(500);
    digitalWrite(relayPin1, LED_OFF);
    delay(500);
    digitalWrite(relayPin2, LED_ON);
    delay(500);
    digitalWrite(relayPin2, LED_OFF);
    delay(500);
    digitalWrite(relayPin3, LED_ON);
    delay(500);
    digitalWrite(relayPin3, LED_OFF);
    delay(500);
    digitalWrite(relayPin4, LED_ON);
    delay(500);
    digitalWrite(relayPin4, LED_OFF);
    delay(500);
    digitalWrite(relayPin1, LED_OFF);
    digitalWrite(relayPin2, LED_OFF);
    digitalWrite(relayPin3, LED_OFF);
    digitalWrite(relayPin4, LED_OFF);
#endif

#ifndef ARDUINO_attiny
    Serial.println("config done");
#endif
}

#ifdef _BLINK
bool blinking()
{
    const  uint32_t interval    = 2000;          // interval at which to blink (milliseconds)

    if (millis() > nextMillis)
    {

        if (cntBlink > 0) {
            nextMillis += interval/5;
            if (ledState == LOW) cntBlink--;
#ifdef _HEARTBEAT
            lastHB = millis()>>15;
#endif
        } else {
            nextMillis += interval;             // save the next time you blinked the LED
        }
        if (ledState == LOW)    ledState = HIGH;
        else                    ledState = LOW;
        digitalWrite(led_PIN, ledState);
        return 1;
    }
    return 0;
}
#endif

void loop()
{
    // following function must be called periodically
//#ifndef _WINDOWS
    hub.poll();

  #ifndef ARDUINO_attiny
    // this part is just for debugging (dbg_HINT in OneWire.h must be enables for output)
    if (hub.getError()) hub.printError();
  #endif
//#endif
#ifdef _BLINK
    if (cntBlink < 1) {
  #ifndef _WINDOWS
        cntBlink = hub.getError();
  #endif
        if (cntBlink > 0) {
            nextMillis  = millis() + 5000;
            ledState = HIGH; // LED off
            digitalWrite(led_PIN, ledState);
        }
    }
#endif

    //
    // check data for validity
    // TODO: add more checks!
    //
    if (stopPos1 > 200) stopPos1 = currentPos1;
    if (stopPos2 > 200) stopPos2 = currentPos2;

    //
    // check if we have to copy to EEPROM
    //
    if (controlEEPROM == 0xBCBC) {
        EEPROMput();
        controlEEPROM = 0;
        // re-attach device on ID change
        if (bae910.ID[6] != clientID) {
            hub.detach(bae910);
            bae910.ID[6] = clientID;
            hub.attach(bae910);
        }
    } else if ((controlEEPROM == 0xDEAD) || (controlEEPROM == 0xBEEF)) {
        EEPROMget();
        controlEEPROM = 0;
        // re-attach device on ID change
        if (bae910.ID[6] != clientID) {
            hub.detach(bae910);
            bae910.ID[6] = clientID;
            hub.attach(bae910);
        }
    }

    // main routine
    checkShutter();

#ifdef _BLINK
    // Blink triggers the state-change
    if (blinking())
    {
  #ifdef _HEARTBEAT
        if ((cntBlink < 1) && (millis()>>15 > lastHB)) {
            cntBlink = 15;
            nextMillis  = millis() + 5000;
            ledState = HIGH; // LED off
            digitalWrite(led_PIN, ledState);
        }
  #endif
    }
#endif
}
