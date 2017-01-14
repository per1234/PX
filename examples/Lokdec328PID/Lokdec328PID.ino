/*

 Test des lokdekoder mit ATmega328P
 (sx-lok-dec2-328)
 
 BETA Version, 14 Jan 2017
 
 Motorregelung mit PID, siehe:
 http://playground.arduino.cc/Code/PIDLibrary
 https://github.com/br3ttb/Arduino-PID-Library
 
 Hardware:
 https://opensx.net/projekte/funktionsdekoder/arduino-lokdecoder/
 
 (C) 2017 Michael Blank, opensx.net
 
 */
#include "Arduino.h"
#include "PX.h"
#include "PID_v1.h"
//#include <Adafruit_SleepyDog.h>

#define HW  "HW sx-loc-dec2-328"
#define SW  "SW PID 14-jan-2017"


#define PXADDR  14
#define NSPEED   4      // Multiplikation Wert für Speed
                        // PWM-Wert = NSPEED * SX-wert (0...31)
#define V0SPEED  16     // Anfahrts PWM Wert
#define PID_ON

#define FIN    9    // Motoranschluss 1
#define RIN   10    // Motoranschluss 2

// declarations for PID
// Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp = 0.15, Ki = 0.2, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//#define DEBUG     // serial out, standard debug
#define DEBUG_7SEG       // sparkfun COM-11629 

PX px;

uint8_t horn, lastPxvalue, dir, light;
volatile uint32_t n_int = 0, last_n_int = -100;
uint8_t errorCount = 0;
uint8_t measure = 0;
uint16_t feedback = 0;
uint32_t lastRunTime = 0;   // for loop timing
uint8_t resetNint = 0;
uint8_t pidSpeed;


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

    // PWM mit 32kHz
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;

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

    //Watchdog.enable(256);

    // initialize interrupt routine
    px.init();

    // RISING slope on INT0/INT1 trigger the interrupt routine sxisr (see above)
    attachInterrupt(digitalPinToInterrupt(SX1), sxisr, RISING);   //INT0
    attachInterrupt(digitalPinToInterrupt(SX2), sxisr2, RISING);   //INT1

    Input = analogRead(A0);
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(50);
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
    if ((millis() - lastRunTime) <= 10)
        return;

    lastRunTime = millis();

    // motor active ~50msecs, then measure
    // then do 1 measurement of the EMK
    measure++;
    if (measure > 4) {
        measure = 0;
    }

    volatile uint8_t pxvalue = px.get(PXADDR);

    uint16_t speedin = (pxvalue & 0x1f);   // for later debug output
    uint16_t speed = speedin;

    if (speed != 0) {
        speed = speed * NSPEED;
    }
#ifndef PID_ON
    speed += V0SPEED;   // offset, added only when PID is not active
#endif
    static uint16_t lastSpeed = 0;
    if (lastSpeed == 0) {
        lastSpeed = speed;
    }
    uint16_t actSpeed = (speed + 31 * lastSpeed) >> 5;  //lowpass filter
    lastSpeed = actSpeed;

    dir = (pxvalue & B00100000) >> 5;
    uint32_t t1, dt = 0;

    if (measure == 0) {
        // measure EMK
        digitalWrite(FIN, LOW);
        digitalWrite(RIN, LOW);
        delay(5);
        feedback = 0;
        for (uint8_t i = 0; i < 8; i++) {   // average 8times
            feedback += analogRead(A0);
        }
        feedback = feedback >> 2;
        Input = (double) feedback;   // feedback 0 ... 330
        Setpoint = (double)actSpeed * 1.6;
        myPID.Compute();
        if (actSpeed == 0) {
            pidSpeed = 0;  //PID does not really work at 0 speed
        } else {
            pidSpeed = Output;
        }

    } else {

        if (dir) {
            digitalWrite(FIN, LOW);
#ifdef PID_ON
            analogWrite(RIN, pidSpeed);
#else
            analogWrite(RIN, actSpeed);
#endif
        } else {
#ifdef PID_ON
            analogWrite(FIN, pidSpeed);
#else
            analogWrite(FIN, actSpeed);
#endif
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
    } //endif measure

    if (measure == 0) {   // debug output
#ifdef DEBUG
            Serial.print(speedin);   // speed from px-signal
            Serial.print(" ");
            Serial.print(actSpeed);  // px-speed * 4 and lp filtered 
            Serial.print(" ");
            Serial.print(pidSpeed);  // output from PID
            Serial.print(" ");
            Serial.println(feedback);  // input from analogRead of EMK
#endif
#ifdef DEBUG_7SEG
        Serial.write(0x76);   // clear
        char c = '9';
        if (n_int < 9) {
            char c = '0' + n_int;
        }
        Serial.write(c);        // sig strength 0..9
        Serial.write('-');
        Serial.print(speedin);  // speed from px-signal
#endif
        resetNint++;
        if (resetNint > 9) {
#ifdef DEBUG
            Serial.println(n_int);   // = number of successful reads of PX-signal
                                     // during last ~ 600msec
#endif
            resetNint = 0;
            n_int = 0;
        }
        digitalWrite(LED_BUILTIN, LOW);
    }
}

