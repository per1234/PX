/*
 * PX.cpp
 *
 *  Created on: 08.10.2016
 *  Changed on: 
 *  Version:    1.0
 *
 *  Copyright:  Michael Blank, Reinhard Thamm
 *
 *  interface hardware needed ! see www.oscale.net/SX

 Read SX Signal from the track - SX1 and SX2 are the pins which receive
 the 2 track signals (2 and 3 on an Atmega328).
 (based on code from SX22b lib)

 No writing or decoder programming implemented, address is set via main
 arduino program.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */


#include <Arduino.h>

#include "PX.h"


PX::PX() {
    _scopeFlag = 0;
}

void PX::init() {
     // initialize function
     // initialize pins and variables
     // and start looking for a SYNC signal

    pinMode(SX1, INPUT);     // track 1 input
    pinMode(SX2, INPUT);      // track 2 input

    for (int i=0; i<112;i++) {
          // reset sx variable to zero
        _sx[i]=0;
    }
     _toggle=0;
    _adrCount =0;

    // start always with search for SYNC
    _state = SYNC;
    _zeroCount = 0;
}

void PX::init(uint8_t tba) {
    // special init which enable a scope trigger
    // at a Selectrix address tba (must be 0..112)
    // scope will go high at bit 12 of this address!!
    // i.e. interesting data are shortly BEFORE
    // this trigger
    _scopeFlag=1;
     pinMode(SCOPE,OUTPUT);
    _triggerAdr = tba;
    init();
}


void PX::switchAdr() {
     // a SYNC signal was received, now look for a valid
     // base address

    switch(_adrCount) {
    case 0:   // this is the GLEISSPANNUNG bit        
        break;                                               
    case 1:         
    case 4:
        break; // ignore, should be checked to get less sensitive to
               // interruptions
    case 2:  // B3
        bitWrite(_baseAdr,3,_bit);
        break;
    case 3:  // B2
        bitWrite(_baseAdr,2,_bit);
        break;
    case 5:  // B1
        bitWrite(_baseAdr,1,_bit);
        break;
    case 6:  // B0
        bitWrite(_baseAdr,0,_bit);
        break;
    case 7: // last "1"
        // _baseAdr is complete !

        // advance to next state - next we are looking
        // for the 7 data bytes (i.e. 7 SX Channels)
        _state = DATA;
        _dataFrameCount = 0;
        _dataBitCount = 0;
        _data=0;
        break;
    }
}

void PX::switchData() {
    // continue reading _data
    // a total of 7 DATA blocks will be received
    // for a certain base-address
    // calc sx channel from baseAdr and dataFrameCount
    _channel = (15-_baseAdr) + ((6-_dataFrameCount)<<4);

    switch(_dataBitCount) {
    case 2:  // "Trenn_bits"
    case 5:
    case 8:
        if (_bit == 1) {
           _dataBitCount++;
        } else {
           // ERROR => reset to SYNC state
            _dataFrameCount=0;
            _state =SYNC;
            _zeroCount = 0;
            _dataBitCount=0;
        }
        break; // ignore
    case 0:  // D0
        _data=0;
        bitWrite(_data,0,_bit);
        _dataBitCount++;

        if (_scopeFlag) {                                           //Tha: also watching _scopeFlag..
            if (_channel ==  _triggerAdr) {                         //     (bugfix 30.01.2015)
                bitWrite(SCOPE_PORT, SCOPE_PORTPIN, HIGH);          //Tha: digitalWrite(SCOPE,HIGH);
            }
            else {
                bitWrite(SCOPE_PORT, SCOPE_PORTPIN, LOW);           //Tha: digitalWrite(SCOPE,LOW);
            }
        }

        break;
    case 1:  // D1   
        bitWrite(_data,1,_bit);
        _dataBitCount++;
        break;
    case 3:  // D2 
        bitWrite(_data,2,_bit);
        _dataBitCount++;
        break;
    case 4:  // D3                               
        bitWrite(_data,3,_bit);
        _dataBitCount++;
        break;
    case 6:  // D4
        bitWrite(_data,4,_bit);
        _dataBitCount++;
        break;
    case 7:  // D5
        bitWrite(_data,5,_bit);
        _dataBitCount++;
        break;
    case 9:  // D6
        bitWrite(_data,6,_bit);
        _dataBitCount++;
        break;
    case 10: // D7
         bitWrite(_data,7,_bit);
        _dataBitCount++;
        break;
    case 11:  // == MAX_DATABITCOUNT
        // _bit value should always equal HIGH, not tested here.
        // copy _data byte to SX _channel
        _sx[_channel] = _data;

        // increment dataFrameCount to move on the next DATA byte
        // check, if we already reached the last DATA block - in this
        // case move on to the next SX-Datenpaket, i.e. look for SYNC
        _dataFrameCount ++;
        if (_dataFrameCount == MAX_DATACOUNT) {
            // and move on to SYNC _state
            _dataFrameCount=0;
            _state =SYNC;
            _zeroCount = 0;
            _dataBitCount=0;
        } else {
            _dataBitCount = 0;  // reset _bit counter
            _data = 0;
            // continue with reading next _data uint8_t
        }
    }  //end switch/case _dataBitCount
}

uint8_t PX::get(uint8_t channel) {
     // returns the value of a SX channel
    if (channel < MAX_CHANNEL_NUMBER)
       return _sx[channel];
    else
       return 0;
}

void PX::isr() {

    // interrupt service routine 
    // driven by RISING signal INT0 and INT1 (pins 2 and 3 on ATmega328)

     // 3 different states are distinguished
     //     1. SNYC = looking for a SYNC signal
     //     2. ADDR = (after SYNC received) look for base address (0..15)
     //     3. DATA = (after ADDR decoded) decode the 7 data-bytes
    
    // is SX2 = high ??
    //_inbit = (SX2_PINREG & ( _BV(SX2_PORTPIN) ) ) > 0;

    //_inbit = (SX2_PINREG & _BV(SX2_PORTPIN)) > 0;  
    // (if not, SX1 must have caused this interrupt)

    // sx bit is high, when the polarity has changed
    // sx bit is low (0), when the polarity has not changed

    //_bit = ~(_inbit == _lastInbit);  
    //ncount = _bit;
    //_lastInbit = _inbit;         

    uint8_t _inputd = PIND & ( (1 << SX1) | (1 << SX2) );
    uint8_t _in;

    
    if  (_inputd == (1 << SX1)) {
        // SX1 toggled and is high
        _in = 1;
        //	PORTA |= (1 << PA1);

        } else if  (_inputd == (1 << SX2)) {
        // SX2 toggled and is high
        _in = 0;
        // 	PORTA |= (1 << PA1);
        
        } else {
        //	PORTA &= ~(1 << PA1);
        return;
    }
    
    //_lastinputb = _inputb;
    if (_lastInbit == _in) {
        _bit = LOW;
        } else {
        _bit = HIGH;
    }
    _lastInbit = _in;


    switch(_state) {
    case SYNC:

        if (_bit == LOW)  {
            _zeroCount++;
        } else {
            if (_zeroCount == 3)  {        // sync bits 0 0 0 1 found
                _state = ADDR;         // advance to next state
                _baseAdr = 0;   //init
                _adrCount = 0;  //init

            } else {  // no valid sync, try again ...
                _zeroCount = 0;       // reset _zeroCounter
            } // endif _zeroCount
        }  // endif _bit==LOW
        break;
    case ADDR:
        switchAdr();
        _adrCount++;
        break;
    case DATA:
        switchData();
    }
}



