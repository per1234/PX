/*

 Lok-Dekoder ATmega328P
 (sx-lok-dec2-328)
 "Simple" Version - ohne Motorregelung
 
 Hardware siehe:
 https://opensx.net/projekte/funktionsdekoder/arduino-lokdecoder/
 
 */
#include "Arduino.h"
#include "PX.h"

//#include <Adafruit_SleepyDog.h>

#define HW  "HW sx-loc-dec2-328"
#define SW  "SW Simple-12-jan-2017"

#define PXADDR  16   // SX(PX) Lok-Adresse
#define NSPEED   4   // Max speed (=Fahrstufe *NSPEED => PWM Output
#define V0SPEED  16  // Anfahrt

#define FIN    9     // Motor PWM pin1 
#define RIN   10     // Motor PWM pin2 

//#define DEBUG      // voller DEBUG output
//#define DEBUG_7SEG   // DEBUG mit sparkfun COM-11629 

PX px;

uint8_t horn, lastPxvalue, dir, light;
volatile uint32_t n_int = 0, last_n_int = -100;
uint8_t errorCount = 0;
uint8_t measure = 0;
uint16_t feedback = 0;
uint32_t lastRun = 0;   // for loop timing
uint8_t resetNint = 0;

// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);
#ifdef DEBUG
    Serial.begin(115200);
    Serial.println(HW);
    Serial.println(SW);
#endif
#ifdef DEBUG_7SEG
    Serial.begin(9600);
#endif
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;   // 32kHz PWM Frequenz

    pinMode(FIN, OUTPUT);
    pinMode(RIN, OUTPUT);
    digitalWrite(FIN, HIGH);  // brake
    digitalWrite(RIN, HIGH);  // brake

    digitalWrite(5, LOW);
    pinMode(5, OUTPUT);
    digitalWrite(6, LOW);
    pinMode(6, OUTPUT);
    digitalWrite(4, LOW);
    pinMode(4, OUTPUT);

    // initialize interrupt routine
    px.init();

    // RISING slope on INT0/INT1 trigger the interrupt routine sxisr (see above)
    attachInterrupt(digitalPinToInterrupt(SX1), sxisr, RISING);   //INT0
    attachInterrupt(digitalPinToInterrupt(SX2), sxisr2, RISING);   //INT1

}

void sxisr(void) {
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    // sx.isr() is a member function (i.e. it's part of the PX class)
    // but attachInterrupt() expects a non-member function (i.e. a 
    // function which is static and/or not part of a class at all).
    px.isr();
}

void sxisr2(void) {
    px.isr2();
}

void printSXValue(int i, int data) {
    // send data for 1 SX Channel on serial port
    Serial.print("V ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(data);
}

void loop() {

    // run only every 11msecs
    if ((millis() - lastRun) <= 10)
        return;

    lastRun = millis();

    volatile uint8_t pxvalue = px.get(PXADDR);

    uint16_t speedin = (pxvalue & 0x1f);   // for later debug output
    uint16_t speed = speedin;

    if (speed != 0) {
        speed = speed * NSPEED + V0SPEED;
    }

    static uint16_t lastSpeed = 0;
    if (lastSpeed == 0) {
        lastSpeed = speed;
    }
    uint16_t actSpeed = (speed + 31 * lastSpeed) >> 5;  //low pass filter
    lastSpeed = actSpeed;

    dir = (pxvalue & B00100000) >> 5;
    uint32_t t1, dt = 0;

    if (dir) {
        digitalWrite(FIN, LOW);
        analogWrite(RIN, actSpeed);

    } else {
        analogWrite(FIN, actSpeed);
        digitalWrite(RIN, LOW);
    }

    horn = (pxvalue & B10000000) >> 7;
    digitalWrite(4, horn);

    light = (pxvalue & B01000000) >> 6;

    if (light) {
        if (dir) {
            digitalWrite(6, HIGH);
            digitalWrite(5, LOW);
        } else {
            digitalWrite(5, HIGH);
            digitalWrite(6, LOW);
        }
    } else {
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
    }
    digitalWrite(4, horn);
    static uint8_t debug_count = 0;
    debug_count++;
    if (debug_count > 8) {
        // send debug output to serial port
        debug_count = 0;
#ifdef DEBUG
        Serial.print(speedin);    // speed directly from SX signal
        Serial.print(" ");
        Serial.print(actSpeed);// filtered speed
        Serial.print(" ");
#endif
#ifdef DEBUG_7SEG
        Serial.write(0x76);   // clear
        char c = '9';
        if (n_int < 9) {
            char c = '0' + n_int;
        }
        Serial.write(c);      // sig strength 0..9
        Serial.write('-');
        Serial.print(speedin);      // speed from sx inpu
#endif
        resetNint++;
        if (resetNint > 9) {
#ifdef DEBUG
            Serial.println(n_int);   // number of received/correct
            // SX frames for a single address in last 8x50msec
#endif
            resetNint = 0;
            n_int = 0;
        }
        digitalWrite(LED_BUILTIN, LOW);
    }
}
