

/*

 Test des lokdekoder mit ATmega328P
 (sx-lok-dec2-328)
 
 
 */
#include "Arduino.h"
#include "PX.h"
//#include <Adafruit_SleepyDog.h>

#define HW  "HW sx-loc-dec2-328"
#define SW  "SW 26-dec-2016"

#define PXADDR  14
#define FIN    9
#define RIN   10   

//#define DEBUG
#define DEBUG_7SEG
//#define LA_F   5
//#define LA_R   6
//#define FUNC1  4

PX px;
uint8_t mydata = 0;
//static byte oldSx[MAX_CHANNEL_NUMBER];
uint8_t horn, lastPxvalue, dir, light;
volatile uint32_t n_int = 0, last_n_int=-100;
uint8_t errorCount = 0;

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

    setPwmFrequency(1);
    uint8_t mode1 = TCCR1B & B00000111;

#ifdef DEBUG
    Serial.print("pwm mode=");
    Serial.println(mode1);
#endif
   
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

    //Just adding that single-bit change to setup() will force 
    // timer 1 into Fast PWM mode as well. The result:
    // bitSet(TCCR1B, WGM12);

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
    // if you want to understand this, see:
    // http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1239522239   
    // sx.isr() is a member function (i.e. it's part of the PX class)
    // but attachInterrupt() expects a non-member function (i.e. a 
    // function which is static and/or not part of a class at all).
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
   /* if ((n_int - last_n_int) > 2 ) { 
      Watchdog.reset();
    } else {
      last_n_int = n_int;
    } */
    volatile uint8_t pxvalue = px.get(PXADDR);

    /*if (pxvalue != lastPxvalue) {
     lastPxvalue = pxvalue;
     delay(20);
     return;
     } */

    mydata = pxvalue;

    uint16_t speedin = (mydata & 0x1f);
    uint16_t speed = speedin;
    if (speed == 0) {
      
    } else {
       speed = speed * 3 + 14 ;
    }
    static uint16_t lastSpeed = 0;
    if (lastSpeed == 0) {
      lastSpeed = speed;
    }
    uint16_t actSpeed = (speed + 7 * lastSpeed) >> 3;  //lp filter
    lastSpeed = actSpeed;

    dir = (pxvalue & B00100000) >> 5;
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

    mydata = mydata & B11100000;
    mydata = mydata >> 1;

    uint16_t feedback = 0;
    for (uint8_t i = 0; i < 15; i++) {
        feedback += analogRead(A0);
        delay(1);
    }
    feedback = feedback >> 4;
    static uint32_t t1 = millis();
    if ((millis() - t1) >= 1000) {

        t1 = millis();
#ifdef DEBUG
        Serial.print(t1);
        Serial.print(" ");
        Serial.print(n_int);
        Serial.print(" ");
        Serial.print(pxvalue);
        Serial.print(" ");
        Serial.print(horn);
        Serial.print(" ");
        Serial.print(dir);
        Serial.print(" ");
        Serial.print(speedin);
        Serial.print(" ");
        Serial.print(actSpeed);
        Serial.print(" ");
        Serial.println(feedback);
#endif
#ifdef DEBUG_7SEG
        Serial.write(0x76);   // clear
        char c = '9';
        if (n_int < 9) {
           char c = '0' + n_int;
        } 
        Serial.write(c);      // sig strength 0..9
        Serial.write('-');
        Serial.print(speedin);  // speed from sx inpu
#endif
  /*      if (n_int < 9) {
         errorCount++;
         if (errorCount >= 5) {
         errorCount = 0;
         }
         } else {
         errorCount = 0;
        } */
        n_int = 0; 
        
        delay(50);
    } else {   
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
    }

}

void setPwmFrequency(int divisor) {
  byte mode;
  
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    TCCR1B = TCCR1B & 0b11111000 | mode;
 
}
