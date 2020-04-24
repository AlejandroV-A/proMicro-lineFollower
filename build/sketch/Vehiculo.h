#ifndef Vehiculo
#define Vehiculo

//#include <EEPROM.h>

#define pGain (-20)    //(20.0)
#define iGain (-0.03)  //(-0.05) //(-0.01) //(0.0) //
#define dGain (-0.125) //(-0.1) //(-0.125) //(0.0) //

#define vBase (100) //(50) //(120) //

#define maxW (255)
#define maxV (255)

// 16 = 104us => 9615.38Hz, always breaks real time boundaries, do not use
// 32 = 208us => 4807.69Hz, max snsFilSz 1
// 64 = 416us => 2403.84Hz, max snsFilSz 2
#define preescalerSet128   // 832us per cycle, ie 1201.92Hz, max snsFilSz 8*
#define RXLED 17           // Pin number with the RXLED
#define calibMinDif (200)  // Min diff to re-calibrate
#define maxSnsInLine (6)   // Max quantity of sensors on line allowed
#define diffMaxCnt (32766) // max signed int value-1, max time to differentiate
#define difStdrGn (2048)   // a gain to make noticeable the diff
#define maxIntErr (-50 / iGain) // Critical value [32760]
#define snsFilSz 8              // Size of the sensor filter
#define recalTrig 100           // Number of cycles to re-calibrate
#define maxWidthColCh 4         // Maximum line width to enable color change
#define minWidthColCh 1         // minimum line width to enable color change
//#define whiteLine // Line is black by default

// Fast non re-setting pin read, set, and clear
#define setPin(b)                                                              \
  (*portOutputRegister(digitalPinToPort(b)) |= digitalPinToBitMask(b))
#define clrPin(b)                                                              \
  (*portOutputRegister(digitalPinToPort(b)) &= ~digitalPinToBitMask(b))
#define writePin(a, b) ((b == 0) ? (clrPin(a)) : (setPin(a)))
#define tstPin(b)                                                              \
  ((*portInputRegister(digitalPinToPort(b)) & digitalPinToBitMask(b)) != 0)

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile bool adcDone = false;
volatile byte snsPntr = 0;
int sensor[8] = {0};
volatile int snsFilReg[snsFilSz][8] = {{0}};
volatile byte snsStkPntr = 0;
volatile int cyclCnt = 0;
volatile bool ready2cal = false;
uint8_t An[8] = {0};
uint8_t enPin;
int calib = 0;
byte digitalSens = 0;
int error = 0;
int derror = 0;
int ierror = 0;
int errorOld = 0;
int diffCntr = 1;
bool linecolor = 0; // 0 - Black line, 1 - White line
int memError = 0;

// Motor direction pins
uint8_t lf, lb, rf, rb;

// Motor PWM pins
uint8_t lp, rp;

// Stop signal pin
uint8_t stp;

// Current speeds
int v = 1, w = 0, v1 = 1, v2 = 1;

// Sign function, hope it is optimized by compiler
static inline int8_t sgn(int val) {
  if (val < 0)
    return -1;
  if (val == 0)
    return 0;
  return 1;
}

// Initializes the pins for the sensor bar
void setupADC(uint8_t enablePin, uint8_t An0, uint8_t An1, uint8_t An2,
              uint8_t An3, uint8_t An4, uint8_t An5, uint8_t An6, uint8_t An7) {
  enPin = enablePin;
  An[0] = An0;
  An[1] = An1;
  An[2] = An2;
  An[3] = An3;
  An[4] = An4;
  An[5] = An5;
  An[6] = An6;
  An[7] = An7;
  pinMode(enPin, OUTPUT);
  pinMode(RXLED, OUTPUT);
  writePin(enPin, 1);
  ADCSRA = bit(ADEN);   // turn ADC on
  ADCSRA |= bit(ADATE); // ADATE to free run
#ifdef preescalerSet16
  ADCSRA |= bit(ADPS2); // Prescaler of  16
#elif defined(preescalerSet32)
  ADCSRA |= bit(ADPS0) | bit(ADPS2); // Prescaler of  32
#elif defined(preescalerSet64)
  ADCSRA |= bit(ADPS1) | bit(ADPS2); // Prescaler of  64
#else // preescalerSet128
  ADCSRA |= bit(ADPS0) | bit(ADPS1) | bit(ADPS2); // Prescaler of 128
#endif
  // ADCSRB |= bit(ADHSM); //Enable high speed adc
  for (byte i = 0; i < 8; i++) {
    switch (An[i]) {
    case A0:
      DIDR0 |= bit(ADC7D); // disable the digital buffer
      break;
    case A1:
      DIDR0 |= bit(ADC6D); // disable the digital buffer
      break;
    case A2:
      DIDR0 |= bit(ADC5D); // disable the digital buffer
      break;
    case A3:
      DIDR0 |= bit(ADC4D); // disable the digital buffer
      break;
    case A6:
      DIDR2 |= bit(ADC8D); // disable the digital buffer
      break;
    case A7:
      DIDR2 |= bit(ADC10D); // disable the digital buffer
      break;
    case A8:
      DIDR2 |= bit(ADC11D); // disable the digital buffer
      break;
    case A9:
      DIDR2 |= bit(ADC12D); // disable the digital buffer
      break;
    case A10:
      DIDR2 |= bit(ADC13D); // disable the digital buffer
      break;
    }
  }
}

// Stablishes the next adc port to be used
void setADCInput(uint8_t adcIn) {
  switch (adcIn) {
  case A0:
    ADMUX = bit(REFS0) | 0b00000111; // AVcc and select input port
    bitClear(ADCSRB, MUX5);
    break;
  case A1:
    ADMUX = bit(REFS0) | 0b00000110; // AVcc and select input port
    bitClear(ADCSRB, MUX5);
    break;
  case A2:
    ADMUX = bit(REFS0) | 0b00000101; // AVcc and select input port
    bitClear(ADCSRB, MUX5);
    break;
  case A3:
    ADMUX = bit(REFS0) | 0b00000100; // AVcc and select input port
    bitClear(ADCSRB, MUX5);
    break;
  case A6:
    ADMUX = bit(REFS0); // AVcc and select input port
    bitSet(ADCSRB, MUX5);
    break;
  case A7:
    ADMUX = bit(REFS0) | 0b00010; // AVcc and select input port
    bitSet(ADCSRB, MUX5);
    break;
  case A8:
    ADMUX = bit(REFS0) | 0b00011; // AVcc and select input port
    bitSet(ADCSRB, MUX5);
    break;
  case A9:
    ADMUX = bit(REFS0) | 0b00100; // AVcc and select input port
    bitSet(ADCSRB, MUX5);
    break;
  case A10:
    ADMUX = bit(REFS0) | 0b00101; // AVcc and select input port
    bitSet(ADCSRB, MUX5);
    break;
  }
}

// Starts the convertion cycle
void startADC(void) {
  setADCInput(An[snsPntr]);
  ADCSRA |= bit(ADSC) | bit(ADIE);
}

void calibration(void) {
  byte maxSens = 0, minSens = 0;
  // Looking for the maximum and minimum
  for (byte i = 1; i < 8; i++) {
    if (sensor[i] > sensor[maxSens])
      maxSens = i;
    if (sensor[i] < sensor[minSens])
      minSens = i;
  }
  // If there are enough difference between max and min calibration is made
  if ((sensor[maxSens] - sensor[minSens]) > calibMinDif) {
    calib = (sensor[maxSens] + sensor[minSens]) / 2;
  }
  ready2cal = false;
}

int getLineWidth(int &first, byte sens, bool color) {
  int width = 0, last = 8;
  if (color) { // white line for inverted sensors, this is, low = line
    for (byte i = 0; i < 8; i++) {
      if (!bitRead(sens, i)) {
        first = i;
        break;
      }
    }
    for (byte i = 0; i < 8; i++) {
      if (!bitRead(sens, 7 - i)) {
        last = 7 - i;
        break;
      }
    }
  } else { // black line, for inverter sensors, means high = line
    for (byte i = 0; i < 8; i++) {
      if (bitRead(sens, i)) {
        first = i;
        break;
      }
    }
    for (byte i = 0; i < 8; i++) {
      if (bitRead(sens, 7 - i)) {
        last = 7 - i;
        break;
      }
    }
  }
  if (first < 8)
    width = last - first;
  return width;
}

// Generate error, derror and ierror, derror and ierror are pretty big
int genError(void) {
  int firstHigh = 8, lastHigh = 8, lineWidth = 0;
  byte dsms = 0;
  int snsMean = 0;
  int snsMnAcc = 0;
  int snsMeasure = 0;
  int dst2Mean = 0;
  static int derror_a = 0;
  // Filtering sensor measures
  for (byte i = 0; i < 8; i++) { // Fundamental cycle, used for each sensor
#if snsFilSz > 2                 // the oddest is dismissed then a mean is get
    snsMnAcc = 0;
    for (byte j = 0; j < snsFilSz; j++) { // Sensor mean value
      snsMnAcc += snsFilReg[j][i];
    }
    snsMean = snsMnAcc / snsFilSz;
    dst2Mean = abs(snsMean - snsFilReg[0][i]); // Initial value distance to mean
    for (byte j = 1; j < snsFilSz; j++) {      // distance from mean
      if (dst2Mean < abs(snsMean - snsFilReg[j][i])) { // Is it more distant
        dst2Mean = abs(snsMean - snsFilReg[j][i]);
        dsms = j; // New oddest measure
      }
    }
    sensor[i] = (snsMnAcc - snsFilReg[dsms][i]) / (snsFilSz - 1); // Kick the
                                                                  // odd
#elif snsFilSz > 1
    sensor[i] = (snsFilReg[0][i] + snsFilReg[1][i]) / 2;
#elif snsFilSz > 0
    sensor[i] = snsFilReg[0][i];
#else
#error snsFilSz must be al least 1
#endif
  }
  adcDone = false; // The lectures from the sensor filter register are done
  // Digital sensing
  digitalSens = 0;
  for (byte i = 0; i < 8; i++) { // High means line
    if (sensor[i] > calib)
      digitalSens |= bit(i); // black line for inverted sensor
  }
  // Error abstraction
  firstHigh = 8;
  lineWidth = getLineWidth(firstHigh, digitalSens, linecolor);
  if ((firstHigh < 8) && (lineWidth < maxSnsInLine)) {
    errorOld = error;
    error = 2 * firstHigh - 7 + lineWidth;
    // Generating memory state
    if (bitRead(digitalSens, 0) != bitRead(digitalSens, 7)) {
      if (bitRead(digitalSens, 0)) {
        if (linecolor) {
          memError = 7;
        } else {
          memError = -7;
        }
      } else {
        if (linecolor) {
          memError = -7;
        } else {
          memError = 7;
        }
      }
    }
  } else if (lineWidth == 7) { // Does this qualify as a color change?
    int lineWidthA = 0, firstHighA = 0, lastHighA = 0;
    lineWidthA = getLineWidth(firstHighA, digitalSens, !linecolor);
    if ((lineWidthA >= minWidthColCh) && (lineWidthA <= maxWidthColCh)) {
      // This qualifies as a color change
      linecolor = !linecolor;
      errorOld = error;
      error = 2 * firstHighA - 7 + lineWidthA;
    }
  } else if (firstHigh == 8) { // Memory state, because line loose
    errorOld = error;
    error = memError;
  }
  // Differential
  if (error == errorOld) {
    if (++diffCntr > diffMaxCnt)
      diffCntr = diffMaxCnt;
    derror = derror_a * difStdrGn / diffCntr;
  } else {
    derror_a = error - errorOld;
    derror = derror_a * difStdrGn / diffCntr;
    diffCntr = 1;
  }
  // Integral
  ierror += error;
  if (ierror > maxIntErr)
    ierror = maxIntErr;
  if (ierror < -maxIntErr)
    ierror = -maxIntErr;
  return error;
}

// Interruption for adc done, storage and sets next
ISR(ADC_vect) {
  snsFilReg[snsStkPntr][snsPntr] = ADC;
  if (++snsPntr > 7) {
    writePin(RXLED, !adcDone); // While RXLED is on, there are real time failure
    adcDone = true;
    snsPntr = 0;
    if (++cyclCnt > recalTrig) {
      cyclCnt = 0;
      ready2cal = true;
    }
    if (++snsStkPntr >= snsFilSz) {
      snsStkPntr = 0;
    }
  }
  // The pointer is set in advance since it is already set when the interrupt is
  // triggered
  if (snsPntr == 7)
    setADCInput(An[0]);
  else
    setADCInput(An[snsPntr + 1]);
}

// Initializes the attached pins to the power stage
void ddwmrInit(uint8_t stop, uint8_t leftFront, uint8_t leftBack,
               uint8_t rightFront, uint8_t rightBack, uint8_t leftPWM,
               uint8_t rightPWM) {
  lf = leftFront;
  lb = leftBack;
  rf = rightFront;
  rb = rightBack;
  lp = leftPWM;
  rp = rightPWM;
  stp = stop;
  pinMode(lf, OUTPUT);
  pinMode(lb, OUTPUT);
  pinMode(rf, OUTPUT);
  pinMode(rb, OUTPUT);
  pinMode(lp, OUTPUT);
  pinMode(rp, OUTPUT);
  pinMode(stp, INPUT);
  writePin(lf, 1);
  writePin(lb, 1);
  writePin(rf, 1);
  writePin(rb, 1);
  analogWrite(lp, v1);
  analogWrite(rp, v2);
}

void pwmWrite(uint8_t pin, int val) {
  switch (digitalPinToTimer(pin)) {
// XXX fix needed for atmega8
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
  case TIMER0A:
    // connect pwm to pin on timer 0
    sbi(TCCR0, COM00);
    OCR0 = val; // set pwm duty
    break;
#endif

#if defined(TCCR0A) && defined(COM0A1)
  case TIMER0A:
    // connect pwm to pin on timer 0, channel A
    sbi(TCCR0A, COM0A1);
    OCR0A = val; // set pwm duty
    break;
#endif

#if defined(TCCR0A) && defined(COM0B1)
  case TIMER0B:
    // connect pwm to pin on timer 0, channel B
    sbi(TCCR0A, COM0B1);
    OCR0B = val; // set pwm duty
    break;
#endif

#if defined(TCCR1A) && defined(COM1A1)
  case TIMER1A:
    // connect pwm to pin on timer 1, channel A
    sbi(TCCR1A, COM1A1);
    OCR1A = val; // set pwm duty
    break;
#endif

#if defined(TCCR1A) && defined(COM1B1)
  case TIMER1B:
    // connect pwm to pin on timer 1, channel B
    sbi(TCCR1A, COM1B1);
    OCR1B = val; // set pwm duty
    break;
#endif

#if defined(TCCR1A) && defined(COM1C1)
  case TIMER1C:
    // connect pwm to pin on timer 1, channel B
    sbi(TCCR1A, COM1C1);
    OCR1C = val; // set pwm duty
    break;
#endif

#if defined(TCCR2) && defined(COM21)
  case TIMER2:
    // connect pwm to pin on timer 2
    sbi(TCCR2, COM21);
    OCR2 = val; // set pwm duty
    break;
#endif

#if defined(TCCR2A) && defined(COM2A1)
  case TIMER2A:
    // connect pwm to pin on timer 2, channel A
    sbi(TCCR2A, COM2A1);
    OCR2A = val; // set pwm duty
    break;
#endif

#if defined(TCCR2A) && defined(COM2B1)
  case TIMER2B:
    // connect pwm to pin on timer 2, channel B
    sbi(TCCR2A, COM2B1);
    OCR2B = val; // set pwm duty
    break;
#endif

#if defined(TCCR3A) && defined(COM3A1)
  case TIMER3A:
    // connect pwm to pin on timer 3, channel A
    sbi(TCCR3A, COM3A1);
    OCR3A = val; // set pwm duty
    break;
#endif

#if defined(TCCR3A) && defined(COM3B1)
  case TIMER3B:
    // connect pwm to pin on timer 3, channel B
    sbi(TCCR3A, COM3B1);
    OCR3B = val; // set pwm duty
    break;
#endif

#if defined(TCCR3A) && defined(COM3C1)
  case TIMER3C:
    // connect pwm to pin on timer 3, channel C
    sbi(TCCR3A, COM3C1);
    OCR3C = val; // set pwm duty
    break;
#endif

#if defined(TCCR4A)
  case TIMER4A:
    // connect pwm to pin on timer 4, channel A
    sbi(TCCR4A, COM4A1);
#if defined(COM4A0) // only used on 32U4
    cbi(TCCR4A, COM4A0);
#endif
    OCR4A = val; // set pwm duty
    break;
#endif

#if defined(TCCR4A) && defined(COM4B1)
  case TIMER4B:
    // connect pwm to pin on timer 4, channel B
    sbi(TCCR4A, COM4B1);
    OCR4B = val; // set pwm duty
    break;
#endif

#if defined(TCCR4A) && defined(COM4C1)
  case TIMER4C:
    // connect pwm to pin on timer 4, channel C
    sbi(TCCR4A, COM4C1);
    OCR4C = val; // set pwm duty
    break;
#endif

#if defined(TCCR4C) && defined(COM4D1)
  case TIMER4D:
    // connect pwm to pin on timer 4, channel D
    sbi(TCCR4C, COM4D1);
#if defined(COM4D0) // only used on 32U4
    cbi(TCCR4C, COM4D0);
#endif
    OCR4D = val; // set pwm duty
    break;
#endif

#if defined(TCCR5A) && defined(COM5A1)
  case TIMER5A:
    // connect pwm to pin on timer 5, channel A
    sbi(TCCR5A, COM5A1);
    OCR5A = val; // set pwm duty
    break;
#endif

#if defined(TCCR5A) && defined(COM5B1)
  case TIMER5B:
    // connect pwm to pin on timer 5, channel B
    sbi(TCCR5A, COM5B1);
    OCR5B = val; // set pwm duty
    break;
#endif

#if defined(TCCR5A) && defined(COM5C1)
  case TIMER5C:
    // connect pwm to pin on timer 5, channel C
    sbi(TCCR5A, COM5C1);
    OCR5C = val; // set pwm duty
    break;
#endif
  }
}

// Sets angular and translational velocities
void setSpeed(int _w, int _v) {
  int w, v, rv, lv;
  if (abs(_w) > maxW) {
    w = sgn(_w) * maxW;
  } else {
    w = _w;
  }
  if (abs(_v) > maxV) {
    v = sgn(_v) * maxV;
  } else {
    v = _v;
  }
  if (abs(v + w) > 255) {
    v = sgn(v) * (255 - abs(w));
  }
  rv = v - w;
  lv = v + w;
  if (rv > 255) {
    rv = 255;
  } else if (rv < -255) {
    rv = -255;
  }
  if (lv > 255) {
    lv = 255;
  } else if (lv < -255) {
    lv = -255;
  }
  if (rv > 0) {
    writePin(rf, 1);
    writePin(rb, 0);
    pwmWrite(rp, rv);
  } else if (rv < 0) {
    writePin(rf, 0);
    writePin(rb, 1);
    pwmWrite(rp, -rv);
  } else {
    writePin(rf, 0);
    writePin(rb, 0);
    pwmWrite(rp, 0);
  }
  if (lv > 0) {
    writePin(lf, 1);
    writePin(lb, 0);
    pwmWrite(lp, lv);
  } else if (lv < 0) {
    writePin(lf, 0);
    writePin(lb, 1);
    pwmWrite(lp, -lv);
  } else {
    writePin(lf, 0);
    writePin(lb, 0);
    pwmWrite(lp, 0);
  }
}

// Returns the state of the stop signal
bool ddwmrStop(void) {
  // return digitalRead(stp);
  return tstPin(stp);
}

void pidPool() {
  if (ready2cal) {
    calibration();
  }
  if (adcDone) {
    genError(); // error, derror, ierror
    setSpeed(pGain * error + dGain * derror + iGain * ierror, vBase);
  }
}

#endif

/* pro micro pinout
AVR // Analog // Com // PWM // Arduino
 d3       tx          1
 d2       rx          0
 d1       SDA         2
 d0       SCL P0b976  3
 d4   A6              4
 c6           P3a488  5
 d7   A7      P4a488  6
 e6                   7
 b4   A8              8
 b5   A9      P1a488  9
 b6   A10     P1b488 10
 b2       MOSI       16
 b3       MISO       14
 b1       SCLK       15
          RXLED      17
 f7   A0             18
 f6   A1             19
 f5   A2             20
 f4   A3             21
//*/
