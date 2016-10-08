/*
 * DecodeTrackSig.ino
 *
 *  Created on: 08.10.2016
 *  Changed on: 
 *  
 *  Author: Michael Blank
 *  
 *  Example program for the Selectrix(TM) Library
 *  to decode the track signal - the results are sent to
 *  the serial port
 *  
 *  needs special hardware, see http://opensx.net/locodecoder 
 *  
 */


#include "PX.h"   // this is the Selectrix library

#define LED_PIN  13   // on most Arduinos there is an LED at pin 13

#define LED_97_1 9    // read sx channel 97 and switch on/off LEDs 
#define LED_97_2 10   // on pins 9 and 10

PX sx;                // library


static int ledState = LOW;
static byte oldSx[MAX_CHANNEL_NUMBER];

void printSXValue(int i,int data) {
    // send data for 1 SX Channel on serial port
    Serial.print("V ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(data);
}

void toggleLed() {
    // to notify a change
	if (ledState == LOW)  {
		ledState = HIGH;
	} else {
		ledState = LOW;
	}
	digitalWrite(LED_PIN, ledState);
}

void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    // sx.isr() is a member function (i.e. it's part of the PX class)
    // but attachInterrupt() expects a non-member function (i.e. a 
    // function which is static and/or not part of a class at all).
    sx.isr();
} 

void setup() {

    pinMode(LED_PIN,OUTPUT);
    pinMode(LED_97_1,OUTPUT);
    pinMode(LED_97_2,OUTPUT);
    pinMode(SCOPE,OUTPUT);
    
    digitalWrite(LED_PIN, ledState);
    Serial.begin(115200);      // open the serial port
    for (int i=0; i<112;i++) {
        oldSx[i]=0;   // initialize to zero
        printSXValue(i,0);
    }

    // initialize interrupt routine
    sx.init(97);   // scope triggered at channel 97

    // RISING slope on INT0/INT1 trigger the interrupt routine sxisr (see above)
    attachInterrupt(digitalPinToInterrupt(SX1), sxisr, RISING);
    attachInterrupt(digitalPinToInterrupt(SX2), sxisr, RISING);
} 


void loop() {
  
	// check selectrix channels for changes
    for (int i=0; i<112; i++) {
        byte d=sx.get(i);
        if (oldSx[i] != d) {   // data have changed on SX bus
            oldSx[i] = d;
            printSXValue(i,d);   // send new value to serial port
            toggleLed();  // indicate change
        }
        if (i == 97) {
           digitalWrite(LED_97_1,bitRead(d,1));
           digitalWrite(LED_97_2,bitRead(d,2));
        }
    }
    
}




