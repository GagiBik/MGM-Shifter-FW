
#ifndef PWMSINGLE_H
#define PWMSINGLE_H

#include <arduino.h>

class PWMSingle {
  public:
    PWMSingle();
    //define the gpio pin, frequency, and starting dutycycle
    //valied range expected: 100Hz to 10kHz
    //Minimum duty cycle step is 0.5% 

    //Onetime call to start the PWM
    bool begin(uint8_t pwmPin, uint32_t freq, uint8_t dutyCycle);

    //Runtime adjustment
    void setDutyCycle(uint8_t dutyCycle);     // set duty cycle from 0% to 100%
    void setFrequency(uint32_t frequency);
    void setDirection(bool direction);

  private:
    uint8_t _port;
    uint8_t _pin;
    uint8_t _dirpin;
    uint8_t _wo;
    uint8_t _cc;
    uint32_t _freqHz;
    uint8_t _duty;
    bool _dir;

    // internal helpers
    bool pinToWO (uint8_t pin, uint8_t &wo, uint8_t &cc);
    void updatePeriodAndCC();
    void tcc0_init(uint8_t portGroup, uint8_t pwmPin);
};

#endif