#include <Arduino.h>
#line 1 "/home/alejandrov/Dropbox/GitHub/proMicro-lineFollower/Seguidor_micro.ino"
// Machine God [G4]
#include "Vehiculo.h"

#line 4 "/home/alejandrov/Dropbox/GitHub/proMicro-lineFollower/Seguidor_micro.ino"
void setup(void);
#line 13 "/home/alejandrov/Dropbox/GitHub/proMicro-lineFollower/Seguidor_micro.ino"
void loop(void);
#line 4 "/home/alejandrov/Dropbox/GitHub/proMicro-lineFollower/Seguidor_micro.ino"
void setup(void) {
  setupADC(2, A6, A8, A9, A10, A0, A1, A2, A3);
  ddwmrInit(0, 15, 14, 7, 16, 5, 6);
  // stop, leftFront, leftBack, rightFront, rightBack, leftPWM,  rightPWM
  startADC();
  pinMode(1, OUTPUT);
  pinMode(3, OUTPUT);
}

void loop(void) {
  pidPool();
  if (error == 0) {
    writePin(1, 1);
    writePin(3, 1);
  } else if (error < 0) {
    writePin(1, 1);
    writePin(3, 0);
  } else {
    writePin(1, 0);
    writePin(3, 1);
  }
}

/* Pro micro pinout
 AVR // Analog // Com // PWM // Arduino // Use
 d3       tx          1  [LED_R]
 d2       rx         *0  [STOP]
 d1       SDA        *2  [ENABLE]
 d0       SCL P0b976  3  [LED_L]
 d4  *A6              4  [Sens]
 c6          *P3a488  5  [PWM]
 d7   A7     *P4a488  6  [PWM]
 e6                  *7  [Car]
 b4  *A8              8  [Sens]
 b5  *A9      P1a488  9  [Sens]
 b6  *A10     P1b488 10  [Sens]
 b2       MOSI      *16  [Car]
 b3       MISO      *14  [Car]
 b1       SCLK      *15  [Car]
 f7  *A0             18  [Sens]
 f6  *A1             19  [Sens]
 f5  *A2             20  [Sens]
 f4  *A3             21  [Sens]
           RXLED      17  [RXLED]
//*/

