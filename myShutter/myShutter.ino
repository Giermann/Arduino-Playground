/*
 *    Shutter controller
 *    (c) 2016 Sven Giermann
 *    
 *    ISSUES:
 *    - press different direction while moving stops instead of switching direction
 *    - stopping with same direction starts holdLongpress (2 button mode; WORKAROUND: set holdLongpress to 0)
 *    - longPress does not start moving on ends (2 button mode)
 *
 *    - very different move lengths
 *      a) skip 1-Wire communication while moving --> also solves some issues from above
 *      b) switch relay offset (lastPosChange = currentMillis + offset/positionfactor;)
 *
 *    - move length = 0 after moving up :(
 *
 *    HISTORY:
 *      02.06.2016 - add directionReset==0 (never), implement CTRL_SimulateMode
 *      07.06.2016 - implement CTRL_SimulateMode, reassigned some variables, fix stopPos from 1-Wire
 *      13.06.2016 - reworked stopPos with S_REVERSE_REQUEST, optimized code size
 *      21.06.2016 - skip 1-Wire poll while moving, add relay offset (keep 0 for now)
 *
 *    CODE SIZE:    7.680 Bytes [1.6.9] + 254 SRAM
 */

#include "Shutter.h"

#ifndef _WINDOWS
  #include <EEPROM.h>
  #include "OneWireHub.h"
#endif // _WINDOWS

#include "BAE910.h"  // 3rd party device

OneWireHub hub  = OneWireHub(onewirePin);
BAE910 bae910   = BAE910(BAE910::family_code, onewireClnt, 0x00, 0x00, 0x00, 0x00, 0x00);

#ifdef _BLINK
  static uint8_t cntBlink = 0;
  #ifdef _HEARTBEAT
    uint32_t lastHB = 0;
  #endif
  static uint8_t  ledState   = LED_OFF;     // ledState used to set the LED
  static uint32_t nextMillis = millis();    // will store next time LED will updated
  short numBlink = 0;
#endif


//
// compare code size:  define [7.504 Bytes], function [7.502 Bytes]
//   13.06.2016 - swapped: define is 4 Bytes less than function
//
//#define timeDiff(previous) (unsigned long)(currentMillis - previous)
#define timeDiff(previous) (uint32_t)(currentMillis - previous)
//uint32_t timeDiff(uint32_t previous) { return (currentMillis - previous); }


//
// (0) check submitted 1-Wire stopPos for validity (104 Bytes)
//
void checkStopPos(uint8_t &internalState, uint8_t &currentPos, uint8_t &stopPos)
{
    if (stopPos > 200) {
        stopPos = currentPos;
    } else if (controlEEPROM == CTRL_RestorePos) {  // && (currentPos == invalidPos)
        //
        // TODO: when only ONE change from invalidPos allowed, set stopPos = invalidPos in setup()!
        //
        currentPos = stopPos;
/*
    } /*else if (internalState == S_IDLE) {
        //
        // TODO: temporarily workaround updateState() - remove, if corrected there
        //
        if (stopPos > 100) {
            // skip moving to end, directly move to desired position when in those ranges
            if (stopPos > switchStopPos) {
                if (currentPos >= (stopPos%100)) stopPos -= 100;
            } else {
                if (currentPos <= (stopPos%100)) stopPos -= 100;
            }
        }
*/
/* SG, 21.06.2016 - not possible any more
    } else if (internalState & S_MOVE_OPEN) {
        // move full open and then back to desired stopPos
        if ((stopPos > currentPos) && (stopPos <= 100))
            internalState |= S_REVERSE_REQUEST;
        //if ((stopPos > currentPos) && (stopPos <= 100)) stopPos += 100;
    } else if (internalState & S_MOVE_CLOSE) {
        // move full close and then back to desired stopPos
        if (stopPos < currentPos)
            internalState |= S_REVERSE_REQUEST;
        //if (stopPos < currentPos) stopPos += 100;
*/
    }
    // IDLE:
    // - normal action
    // - BUT: when inside range (0..openPos | closePos..100), decrease by 100 (only WORKAROUND button=full close/open in this case)
    //
    // MOVE_OPEN:
    // - if ((stopPos <= 100) && (stopPos > currentPos)) newState = MOVE_CLOSE
    // - if ((stopPos > 100) && (stopPos > switchStopPos)) newState = MOVE_CLOSE
    //
    // MOVE_CLOSE:
    // - if ((stopPos <= 100) && (stopPos < currentPos)) newState = MOVE_OPEN
    // - if ((stopPos > 100) && (stopPos <= switchStopPos)) newState = MOVE_OPEN
    //
    // TODO: recode updateState() to first check internalState instead of stopPos > 100 ??
    //
}

//
// (1) calculate new shutter position
//
void updateShutterPos(uint8_t &internalState, uint8_t &currentPos, uint8_t &stopPos, uint32_t &lastPosChange, uint16_t positionFactor)
{
    if ((internalState & S_RELAY_OFFSET) && (timeDiff(lastPosChange) > relayOffset)) {
#ifdef _DEBUG
        Serial.print(_T("relayOffset reached @ "));
        Serial.println(timeDiff(lastPosChange));
#endif
        internalState -= S_RELAY_OFFSET;
        lastPosChange += relayOffset;
    }
    if (internalState & S_RELAY_OFFSET) return;

    if (internalState & S_MOVE_OPEN) {
        while ((timeDiff(lastPosChange) > positionFactor) && (currentPos > 0)) {
            lastPosChange += positionFactor;
            if (stopPos == currentPos) stopPos--; // do not move back later, when only moved too far
            currentPos--;
        }
        // OLD: moved too far, but then stop now
        //if (currentPos < stopPos) stopPos = currentPos;
        //if ((currentPos < stopPos) && (stopPos <= 100)) stopPos = currentPos;
    } else if (internalState & S_MOVE_CLOSE) {
        while ((timeDiff(lastPosChange) > positionFactor) && (currentPos < 100)) {
            lastPosChange += positionFactor;
            if (stopPos == currentPos) stopPos++; // do not move back later, when only moved too far
            currentPos++;
        }
        // OLD: moved too far, but then stop now
        //if (currentPos > stopPos) stopPos = currentPos;
    }
//#ifdef _DEBUG
//    Serial.print(_T("position: "));
//    Serial.println(currentPos);
//#endif
}

//
// (2+3) check old and new button state and set status accordingly
//
void handleButtonStatus(uint8_t &internalState, uint8_t currentPos, bool currentBtn, bool newBtn, uint32_t &lastBtnEvent, uint32_t &lastLongpress, uint8_t &stopPos, uint8_t btnClosePos, uint8_t btnOpenPos, uint32_t &lastPosChange, uint32_t &lastStateChg, uint16_t &lastMoveLength, uint8_t desiredState)
//void handleButtonStatus(uint8_t &internalState, uint8_t currentPos, bool currentBtn, bool newBtn, uint32_t &lastBtnEvent, uint32_t &lastLongpress, uint8_t &stopPos, uint8_t btnMovePos, uint32_t &lastPosChange, uint32_t &lastStateChg, uint16_t &lastMoveLength, uint8_t desiredState)
{
//    uint8_t newState = internalState & ~S_REVERSE_REQUEST;
//
// TODO: S_REVERSE_REQUEST is obsolete! (only keep S_RELAY_OFFSET)
//
    uint8_t newState = internalState & (S_MOVE_OPEN | S_MOVE_CLOSE);

    if (newBtn) {
        // button down event or long pressed
        newState = desiredState;
        if (!currentButton) {
            lastBtnEvent = currentMillis;
            if ((timeDiff(lastLongpress) <= holdLongpress) || (internalState == desiredState)) {
//            if ((timeDiff(lastLongpress) <= holdLongpress) || (stopPos == btnMovePos)) {
#ifdef _DEBUG
                Serial.print(_T("fake long press inside hold timespan: "));
                Serial.println(timeDiff(lastLongpress));
#endif
                // same button when already moving, or within holdLongpress: fake long press
                lastBtnEvent -= buttonLongpress;
            }
        }
        // TODO: move to button up case ?
        //
        // TODO: prevent setting wrong direction when already moving in range 85..100
        //
        stopPos = (desiredState == S_MOVE_CLOSE) ?
            (( (btnClosePos <= 100) || (currentPos == invalidPos) || (currentPos < (btnClosePos%100)) ) ? btnClosePos : 100) :
            (( (btnOpenPos <= 100) || (currentPos == invalidPos) || (currentPos > (btnOpenPos%100)) ) ? btnOpenPos : 0);
    } else if (currentBtn) {
        // button up event, keep moving on short press
        // TODO: if we use lastStateChg here, there's no need for lastBtnEvent anymore (but then fake lastStateChg above!)
        if (timeDiff(lastBtnEvent) > buttonLongpress) {
#ifdef _DEBUG
            Serial.print(_T("button up after long press: "));
            Serial.println(timeDiff(lastBtnEvent));
#endif
            stopPos = currentPos;
            newState = S_IDLE;
            // remember last long press for holdLongpress
            lastLongpress = currentMillis;
#ifdef _DEBUG
        } else {
            Serial.print(_T("keep moving to (new) stopPos: "));
            Serial.println(stopPos);
#endif
        }
    } else if (currentPos == stopPos) {
        newState = S_IDLE;
//    } else if (internalState == S_IDLE) {
    } else if (newState == S_IDLE) {
        // externally set stopPos -> guess direction and start moving
        // get first direction from currentPos (instead of switchStopPos)
        // TODO: handle stopPos > 100
        newState = (stopPos > currentPos) ? S_MOVE_CLOSE : S_MOVE_OPEN;
    } else {
        // check if we have to turn after reaching an end
        if (stopPos > 100) {
            if ((internalState & S_MOVE_CLOSE) && (currentPos > 99)) {
                newState = S_MOVE_OPEN;
                stopPos -= 100;
            } else if ((internalState & S_MOVE_OPEN) && (currentPos < 1)) {
                newState = S_MOVE_CLOSE;
                stopPos -= 100;
            }
        }
    }

    // trace state change timings
    if (internalState != newState) {
        if (internalState != S_IDLE) {
            // stop moving
            lastMoveLength = timeDiff(lastStateChg);
        }
        if (newState != S_IDLE) {
            // assume shutter opened, if requested to close on invalidPos
            if (currentPos == invalidPos) currentPos = (newState & S_MOVE_OPEN) ? 100 : 0;
            // start moving
#ifdef _DEBUG
            // TODO: does write incorrect debug for second window
            //Serial.print((lastMoveState1 & S_MOVE_OPEN) ? _T("lastMoveOpen: ") : _T("lastMoveClose: "));
            //Serial.print(timeDiff(lastPosChange));
            Serial.print(_T("  stopPos = "));
            Serial.println(stopPos);
            Serial.print(_T(" - start "));
            Serial.print((newState & S_MOVE_OPEN) ? _T("OPENING: ") : _T("CLOSING: "));
            Serial.print(internalState, BIN);
            Serial.print(_T(" -> "));
            Serial.println(newState, BIN);
#endif
            lastPosChange = currentMillis;
            newState |= S_RELAY_OFFSET;
        }
        lastStateChg = currentMillis;
        internalState = newState;
    }
}


// helper functions (reduce code size)
uint8_t getOpenPos(uint8_t currentPos, uint8_t btnOpenPos)
{
    return (( (btnOpenPos <= 100) || (currentPos == invalidPos) || (currentPos > (btnOpenPos%100)) ) ? btnOpenPos : 0);
}
uint8_t getClosePos(uint8_t currentPos, uint8_t btnClosePos)
{
    return (( (btnClosePos <= 100) || (currentPos == invalidPos) || (currentPos < (btnClosePos%100)) ) ? btnClosePos : 100);
}
uint8_t guessMoveDir(uint32_t lastPosChange, uint8_t lastMoveState, uint8_t currentPos)
{
    return
                (((timeDiff(lastPosChange) < directionReset) || (directionReset == 0)) ?
                 (lastMoveState & S_MOVE_OPEN) : (currentPos < defaultDirOpen)) ?
//                (((timeDiff(lastPosChange) > directionReset) && (directionReset > 0)) ?
//                 (currentPos < defaultDirOpen) : (lastMoveState & S_MOVE_OPEN)) ?
                S_MOVE_CLOSE : S_MOVE_OPEN
                ;
/*
    bool doClose = (currentPos < defaultDirOpen);
    if ((timeDiff(lastPosChange) < directionReset) || (directionReset == 0)) {
        doClose = (lastMoveState == S_MOVE_OPEN);
    }
    return doClose ? S_MOVE_CLOSE : S_MOVE_OPEN;
*/
}

//
// main shutter function
//
void checkShutter()
{
    currentMillis     = millis();
    lastButtonADC     = analogRead(buttonAPin);
    uint8_t newButton =
        (controlEEPROM == CTRL_LockButtonMode) ? 0 :
        (lastButtonADC < minADCbutton2)        ? 3 :
        (lastButtonADC < minADCbutton1)        ? 2 :
        (lastButtonADC < minADCidle)           ? 1 :
        0;

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
        ledState = LED_OFF;
        digitalWrite(led_PIN, ledState);
    }
#endif
*/
    //
    // update shutter position first, depending on the active time of current state
    //
    updateShutterPos(internalState1, currentPos1, stopPos1, lastPosChange1, positionFactor1);
    updateShutterPos(internalState2, currentPos2, stopPos2, lastPosChange2, positionFactor2);

    // SG, 08.03.2016 - wait for ADC to settle, try to minimize faults
    if (newButton != tempButton) {
        // store last read value for next comparison
        tempButton = newButton;
        lastTempButton = currentMillis;
    } else if (timeDiff(lastTempButton) > buttonSettle) {
/*
    // SG, 09.06.2016 - handle only button changes
//    } else if ((timeDiff(lastTempButton) > buttonSettle) && (newButton != currentButton)) {     // 4 Bytes more!!
    } else if (timeDiff(lastTempButton) > buttonSettle) if (newButton != currentButton) {
//    } else if (newButton != currentButton) if (timeDiff(lastTempButton) > buttonSettle) {     // 16 Bytes more!!
*/
        // waited long enough to accept the new button state (if not in lock button mode)
#ifdef _DEBUG
        if (newButton != currentButton) {
            Serial.print(_T("Accepted button state: "));
            Serial.print(newButton);
            Serial.print(_T(" - previous: "));
            Serial.println(currentButton);
        }
#endif
        if (defaultDirOpen == 0) {
            //
            // 2 button, single window
            //

            // do nothing when both buttons are pressed
            if ((newButton & 3) == 3) {
                stopPos1 = currentPos1;
                internalState1 = S_IDLE;
                newButton = 0; // stop long press moves as well!
                //Serial.println(_T("both buttons pressed"));
            } else {
                //
                // 1 = CLOSE
                // 2 = OPEN
                //
                // TODO:
                //  * maybe integrate desiredState to internalState ( S_MOVE_CLOSE<<2 == S_SHOULD_CLOSE )
                //  * minimize overhead: determine desiredState and StopPos on button press here, then update status outside if()
                //
                if ((newButton & 1) || ((newButton == 0) && (currentButton & 1))) {
                    handleButtonStatus(internalState1, currentPos1, (currentButton & 1), (newButton & 1), lastBtnEvent1, lastLongpress1, stopPos1,
                        btnClosePos1, btnOpenPos1, lastPosChange1, lastStateChg1, lastMoveLength1, S_MOVE_CLOSE
//                        getClosePos(currentPos1, btnClosePos1), lastPosChange1, lastStateChg1, lastMoveLength1, S_MOVE_CLOSE
                    );
                } else /*if (newButton & 2)*/ {
                    handleButtonStatus(internalState1, currentPos1, (currentButton & 2), (newButton & 2), lastBtnEvent2, lastLongpress2, stopPos1,
                        btnClosePos1, btnOpenPos1, lastPosChange1, lastStateChg1, lastMoveLength1, S_MOVE_OPEN
//                        getOpenPos(currentPos1, btnOpenPos1), lastPosChange1, lastStateChg1, lastMoveLength1, S_MOVE_OPEN
                    );
                }
            }
        } else {
            //
            // 1 button, 2 window mode
            //
            if (internalState1 == S_IDLE) {
//                bool doClose = (guessMoveDir(lastPosChange1, lastMoveState1, currentPos1) == S_MOVE_CLOSE);
                handleButtonStatus(internalState1, currentPos1, (currentButton & 1), (newButton & 1), lastBtnEvent1, lastLongpress1, stopPos1,
//                    doClose ? getClosePos(currentPos1, btnClosePos1) : getOpenPos(currentPos1, btnOpenPos1),
                    btnClosePos1, btnOpenPos1,
                    lastPosChange1, lastStateChg1, lastMoveLength1,
                    guessMoveDir(lastPosChange1, lastMoveState1, currentPos1)
//                    doClose ? S_MOVE_CLOSE : S_MOVE_OPEN
                );
            } else {
                handleButtonStatus(internalState1, currentPos1, (currentButton & 1), (newButton & 1), lastBtnEvent1, lastLongpress1, stopPos1,
                    stopPos1, stopPos1, lastPosChange1, lastStateChg1, lastMoveLength1, internalState1
                );
            }
            if (internalState2 == S_IDLE) {
//                bool doClose = (guessMoveDir(lastPosChange2, lastMoveState2, currentPos2) == S_MOVE_CLOSE);
                handleButtonStatus(internalState2, currentPos2, (currentButton & 2), (newButton & 2), lastBtnEvent2, lastLongpress2, stopPos2,
//                    doClose ? getClosePos(currentPos2, btnClosePos2) : getOpenPos(currentPos2, btnOpenPos2),
                    btnClosePos2, btnOpenPos2,
                    lastPosChange2, lastStateChg2, lastMoveLength2,
                    guessMoveDir(lastPosChange2, lastMoveState2, currentPos2)
//                    doClose ? S_MOVE_CLOSE : S_MOVE_OPEN
                );
            } else {
                handleButtonStatus(internalState2, currentPos2, (currentButton & 2), (newButton & 2), lastBtnEvent2, lastLongpress2, stopPos2,
                    stopPos2, stopPos2, lastPosChange2, lastStateChg2, lastMoveLength2, internalState2
                );
            }
        }
        currentButton = newButton;
    }

    if (internalState1 != S_IDLE) lastMoveState1 = internalState1;
    if (internalState2 != S_IDLE) lastMoveState2 = internalState2;
}

void setRelayState(bool bClose1, bool bOpen1, bool bClose2, bool bOpen2)
{
    relayState  = (bClose1 ? ( relayAssignment      & 0x0F) : 0x00);
    relayState |= (bOpen1  ? ((relayAssignment>>4)  & 0x0F) : 0x00);
    relayState |= (bClose2 ? ((relayAssignment>>8)  & 0x0F) : 0x00);
    relayState |= (bOpen2  ? ((relayAssignment>>12) & 0x0F) : 0x00);

    digitalWrite(relayPin1, ((relayState & 1) && (controlEEPROM != CTRL_SimulateMode)) ? LED_ON : LED_OFF);
    digitalWrite(relayPin2, ((relayState & 2) && (controlEEPROM != CTRL_SimulateMode)) ? LED_ON : LED_OFF);
    digitalWrite(relayPin3, ((relayState & 4) && (controlEEPROM != CTRL_SimulateMode)) ? LED_ON : LED_OFF);
    digitalWrite(relayPin4, ((relayState & 8) && (controlEEPROM != CTRL_SimulateMode)) ? LED_ON : LED_OFF);
}

bool EEPROMget()
{
    crcEEPROM = 0;

// always use default values on windows (wrong byte order)
#ifndef _WINDOWS
    // stalledcnt, ovruncnt = relayAssignment, minADCidle   0x5A..0x5D
    for (uint8_t i = 0x1D; i > 0x19; --i) {
        bae910.memory.bytes[0x3F - i] = EEPROM.read(i); // (i - 0x40) for real address
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x3F - i], crcEEPROM);
    }

    // maxan, maxap = minADCbutton1, minADCbutton2   0x1A..0x1D
    for (uint8_t i = 0x19; i > 0x15; --i) {
        bae910.memory.bytes[0x7B - i] = EEPROM.read(i); // (i - 4) for real address
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
        (controlEEPROM != CTRL_RestoreDefaults)) {
        return true;
    }
#endif // _WINDOWS

    //
    // invalid CRC, use hard coded defaults
    //
    clientID = bae910.ID[6];    // 1 wire client ID
    directionReset  = 15000;    // direction reset in ms
                                // (default to opposite direction within this time range)
    setupLength     = 4000;     // duration in ms for both relays ON to enter setup mode

    buttonSettle    = 50;       // button settle (must stay for at least ...ms); 20 sometimes flickers, 200 is slow in response!
    buttonLongpress = 1500;     // long press length in ms
    holdLongpress   = 4000;     // hold longpress mode for ...ms

    defaultDirOpen  = 30;       // single button mode: default to up direction if position >30%
//    defaultDirOpen  = 0;        // two button (single window) mode
    btnOpenPos1     = 0;        // button OPEN position A
    btnOpenPos2     = 0;        // button OPEN position B
    btnClosePos1    = 185;      // button CLOSE position A  (100 and back to 85)
    btnClosePos2    = 185;      // button CLOSE position B

    positionFactor1 = 300;      // position factor A
    positionFactor2 = 300;      // position factor B
    relayOffset     = 0;        // additional time from switching on a relay until motor moves

    // measured:  both=620, 2=712, 1=838..839, idle=1023
    minADCbutton2   = 666;      // ADClowButton2 (ex 680)
    minADCbutton1   = 775;      // ADClowButton1 (ex 785)
    minADCidle      = 931;      // ADClowIdle    (ex 950)

    relayAssignment = 0x8421;   // relayAssignment
    return false;
}

void EEPROMput()
{
    crcEEPROM = 0;

    // stalledcnt, ovruncnt = relayAssignment, minADCidle   0x5A..0x5D
    for (uint8_t i = 0x1D; i > 0x19; --i) {
        EEPROM.update(i, bae910.memory.bytes[0x3F - i]); // (i - 0x40) for real address
        crcEEPROM = BAE910::crc16(bae910.memory.bytes[0x3F - i], crcEEPROM);
    }

    // maxan, maxap = minADCbutton1, minADCbutton2   0x1A..0x1D
    for (uint8_t i = 0x19; i > 0x15; --i) {
        EEPROM.update(i, bae910.memory.bytes[0x7B - i]); // (i - 4) for real address
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
    Serial.println(_T("Shutter controller with OneWire-Hub"));
#endif

    EEPROMget();
    controlEEPROM = CTRL_ReloadEEPROM; // attach to hub in main loop

    // initialize invalid position; open on first move (18 Bytes)
    currentPos1 = invalidPos;
    currentPos2 = invalidPos;
    stopPos1 = invalidPos;
    stopPos2 = invalidPos;

    // avoid wrong assumptions after boot (104 Bytes)
    lastBtnEvent1 = initLast;
    lastBtnEvent2 = initLast;
    lastLongpress1 = initLast;
    lastLongpress2 = initLast;
    lastPosChange1 = initLast;
    lastPosChange2 = initLast;

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
#endif

#ifndef ARDUINO_attiny
    Serial.println(_T("config done"));
#endif
}

#ifdef _BLINK
bool blinking()
{
    const  uint32_t interval    = 2000;          // interval at which to blink (milliseconds)

    if (millis() > nextMillis) {

        if (cntBlink > 0) {
            nextMillis += interval/5;
            if (ledState == LED_OFF) cntBlink--;
#ifdef _HEARTBEAT
            lastHB = millis()>>15;
#endif
        } else {
            nextMillis += interval;             // save the next time you blinked the LED
        }
        if (ledState == LED_ON) ledState = LED_OFF;
        else                    ledState = LED_ON;
        digitalWrite(led_PIN, ledState);
        return 1;
    }
    return 0;
}
#endif

void loop()
{
    if ((internalState1 == S_IDLE) && (internalState2 == S_IDLE)) {
        // following function must be called periodically
        hub.poll();

#ifndef ARDUINO_attiny
        // this part is just for debugging (dbg_HINT in OneWire.h must be enabled for output)
        if (hub.getError()) hub.printError();
#endif
#ifdef _BLINK
        if (cntBlink < 1) {
            cntBlink = hub.getError();
            if (cntBlink > 0) {
                nextMillis  = millis() + 5000;
                ledState = LED_OFF;
                digitalWrite(led_PIN, ledState);
            }
        }
#endif

        //
        // check data for validity
        // TODO: add more checks!
        //
        checkStopPos(internalState1, currentPos1, stopPos1);
        checkStopPos(internalState2, currentPos2, stopPos2);


        //
        // check if we have to copy to EEPROM
        //
        if ((controlEEPROM == CTRL_Save2EEPROM) || (controlEEPROM == CTRL_ReloadEEPROM) || (controlEEPROM == CTRL_RestoreDefaults)) {
            if (controlEEPROM == CTRL_Save2EEPROM)
                EEPROMput();
            else
                EEPROMget();

            // SG, 03.05.2016 - always re-attach on those commands
            hub.detach(bae910);
            bae910.ID[6] = clientID;
            bae910.ID[7] = bae910.crc8(bae910.ID, 7);
            hub.attach(bae910);
#ifndef ARDUINO_attiny
            if (hub.getError()) hub.printError();
#endif
            controlEEPROM = 0;
        }
    }

    //
    // main routine
    //
    if (controlEEPROM < CTRL_TestMode) {
        checkShutter();
        //
        // check for simulate here and remove from setRelayState saves 12 Bytes code, but does not update relayState
        //
        //if (controlEEPROM == CTRL_SimulateMode)
        //    setRelayState(false, false, false, false);
        //else
        setRelayState((internalState1 & S_MOVE_CLOSE),
                      (internalState1 & S_MOVE_OPEN),
                      (internalState2 & S_MOVE_CLOSE),
                      (internalState2 & S_MOVE_OPEN));
    } else if ((controlEEPROM < (CTRL_SomfyAutoSetup + 4)) && (internalState1 == S_IDLE) && (internalState2 == S_IDLE)  ) {
        //
        // special operation modes only allowed in IDLE mode
        //
        if (controlEEPROM > CTRL_SomfyDefaultRst) {
            // always start with 1 setup period
            setRelayState(controlEEPROM & 1, controlEEPROM & 1, controlEEPROM & 2, controlEEPROM & 2);
            delay(setupLength);
        }
        if (controlEEPROM > CTRL_SomfyAutoSetup) {
            // reset also for auto setup mode, so delay another 3s
            delay(setupLength);
            setRelayState(false, false, false, false);
            delay(buttonLongpress);
            // start auto setup with entering setup mode
            setRelayState(controlEEPROM & 1, controlEEPROM & 1, controlEEPROM & 2, controlEEPROM & 2);
            delay(setupLength);
            setRelayState(false, false, false, false);
            delay(buttonLongpress);
            // setup auto ending by pressing UP for 3s
            setRelayState(false, controlEEPROM & 1, false, controlEEPROM & 2);
            delay(setupLength);
            setRelayState(false, false, false, false);
            delay(buttonLongpress);
        }
        if (controlEEPROM > CTRL_SomfySetupMode) {
            // finish auto setup with one last setup mode
            setRelayState(controlEEPROM & 1, controlEEPROM & 1, controlEEPROM & 2, controlEEPROM & 2);
            delay(setupLength);
            setRelayState(false, false, false, false);
            controlEEPROM = 0;
        } else {
            // "normal" relay test mode, simply set relays
            setRelayState(controlEEPROM & 1, controlEEPROM & 2, controlEEPROM & 4, controlEEPROM & 8);
        }
    } else {
        // invalid setting, reset
        controlEEPROM = 0;
    }

#ifdef _BLINK
    // Blink triggers the state-change
    if (blinking())
    {
  #ifdef _HEARTBEAT
        if ((cntBlink < 1) && (millis()>>15 > lastHB)) {
            cntBlink = 15;
            nextMillis  = millis() + 5000;
            ledState = LED_OFF;
            digitalWrite(led_PIN, ledState);
        }
  #endif
    }
#endif
}
