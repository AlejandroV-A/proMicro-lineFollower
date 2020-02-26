#include <Arduino.h>
#line 1 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
// Machine God [G4]
#include "Vehiculo.h"

// TODO tunning the gains, and setting translational speed
// with speed ups and downs. Do not start any motor until a line is detected.

#line 7 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
void setup(void);
#line 16 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
void loop(void);
#line 7 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
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
/*
I'm a Ph.D. candidate that has been interested in embedded systems for more than
10 years.

My first approach to embedded system development was as a high school student,
where I learned to program Microchip PIC MCUs in assembler language, at this
time starting to participate in robotics contests. In my engineering studies, I
also learned to program AVR MCUs in assembler language, to then advance to
program both Microchip's platforms in C language, continuing to participate and
win in international robotics contest.

As a graduate student, I've been working with embedded systems to tests the
control laws that I propose. The embedded systems that I use are unmanned
vehicles, mainly quadcopters. In this works, I also started using NXP LPC MCUs
in C language.

Nowadays, I'm also lecturing the "embedded systems" subject at UNITEC. In this
university, as the students already own some hardware, particularly Arduino, I
had to fit the program to use that hardware. Arduino is a platform that I had
never used before, but transporting the knowledge from other platforms I
understood its nuances swiftly.

This background shows that the embedded systems my main technological interest
and where I would like to perform for my career, using my high level of
technical understanding as well as the PhD-level abilities in propose, adapt,
and use state-of-the-art technology.

This background makes the embedded systems my main technological interest and
where I would like to perform for my career, using my high level of technical
understanding as well as the many abilities in proposing, adapting, and using
state-of-the-art technology.
*/

