#ifndef DRV8835MotorShield_Esp32_h
#define DRV8835MotorShield_Esp32_h

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__)
  #define DRV8835MOTORSHIELD_USE_20KHZ_PWM
#endif

#include <Arduino.h>

class DRV8835MotorShieldEsp32
{
  public:
    static void setM1Speed(int speed);
    static void setM2Speed(int speed);
    static void setSpeeds(int m1Speed, int m2Speed);
  
  private:
    static void initPinsAndMaybeTimer();
    static const unsigned char _M1DIR;
    static const unsigned char _M2DIR;
    static const unsigned char _M1PWM;
    static const unsigned char _M2PWM;
   
    static inline void init()
    {
      static boolean initialized = false;

      if (!initialized)
      {
        initialized = true;
        initPinsAndMaybeTimer();
      }
    }
};
#endif
