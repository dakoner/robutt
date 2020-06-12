#include "DRV8835MotorShield_esp32.h"
const unsigned char DRV8835MotorShieldEsp32::_M1PWM = 26;
const unsigned char DRV8835MotorShieldEsp32::_M1DIR = 27;

const unsigned char DRV8835MotorShieldEsp32::_M2PWM = 32;
const unsigned char DRV8835MotorShieldEsp32::_M2DIR = 33 ;

void DRV8835MotorShieldEsp32::initPinsAndMaybeTimer()
{
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high, 
  // even for a short time.
  // It is called after pinMode to handle the case where the boarMd
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  digitalWrite(_M1PWM, LOW);
  pinMode(_M1PWM, OUTPUT);
  digitalWrite(_M1PWM, LOW);
  digitalWrite(_M2PWM, LOW);
  pinMode(_M2PWM, OUTPUT);
  digitalWrite(_M2PWM, LOW);
  digitalWrite(_M1DIR, LOW);
  pinMode(_M1DIR, OUTPUT);
  digitalWrite(_M1DIR, LOW);
  digitalWrite(_M2DIR, LOW);
  pinMode(_M2DIR, OUTPUT);
  digitalWrite(_M2DIR, LOW);
#ifdef DRV8835MOTORSHIELD_USE_20KHZ_PWM
  // timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
#endif
  ledcAttachPin(_M1PWM, 1); // use LED PWM channel 1
  ledcAttachPin(_M2PWM, 2); // use LED PWM channel 2
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);

}

// speed should be a number between -400 and 400
void DRV8835MotorShieldEsp32::setM1Speed(int speed)
{
  init(); // initialize if necessary
      
  if (speed > 255)  // max 
    speed = 255;
if (speed < -255)  // min 
    speed = -255;    
    
#ifdef DRV8835MOTORSHIELD_USE_20KHZ_PWM
  OCR1A = speed;
#else
  ///analogWrite(_M1PWM, abs(speed)); // default to using analogWrite, mapping 400 to 255
    ledcWrite(1, abs(speed));
 


#endif 

  if (speed < 0) 
    digitalWrite(_M1DIR, HIGH);
  else
    digitalWrite(_M1DIR, LOW);
}

// speed should be a number between -400 and 400
void DRV8835MotorShieldEsp32::setM2Speed(int speed)
{
  init(); // initialize if necessary
   
  if (speed > 255)  // max PWM duty cycle
    speed = 255;
     if (speed < -255)  // min PWM duty cycle
    speed = -255;
    
#ifdef DRV8835MOTORSHIELD_USE_20KHZ_PWM
  OCR1B = speed;
#else
  //analogWrite(_M2PWM, abs(speed)); // default to using analogWrite, mapping 400 to 255
    ledcWrite(2, abs(speed));
#endif

  if (speed < 0) 
    digitalWrite(_M2DIR, HIGH);
  else
    digitalWrite(_M2DIR, LOW);
}

// set speed for both motors
// speed should be a number between -400 and 400
void DRV8835MotorShieldEsp32::setSpeeds(int m1Speed, int m2Speed){
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

