#include "PWMSingleChannel.h"

// PWM using the WO of TCC0, use the available pins connected with the TCC0 WOs
// PA04,PA05,PA08,PA09,PA10,PA11,PA12,PA13,PA14,PA15,PA16,PA17,PA18,PA19,PA20,PA21,PA22,PA23 
// PB10,PB11,PB12,PB13,PB16,PB17,PB30,PB31
// PBxx are not supported by this API 

/*  TCC0 to GPIO Wave output mapping:
  TCC0: 
  WO[0] → CC0 →  	PA04,PA08,PB30
  WO[1] → CC1 →  	PA05,PA09,PB31
  WO[2] → CC2 → 	PA10,PA18
  WO[3] → CC3 → 	PA11,PA19
  WO[4] → CC0 → 	PA14,PA22,PB10,PB16 
  WO[5] → CC1 → 	PA15,PA23,PB11,PB17 
  WO[6] → CC2 → 	PA12,PA16,PA20,PB12 
  WO[7] → CC3 → 	PA13,PA17,PA21,PB13 
*/

PWMSingle::PWMSingle(): _pin(0), _wo(0), _cc(0), _freqHz(1000), _duty(10), _dir(0){}

bool PWMSingle::begin(uint8_t pwmPin, uint32_t freqHz, uint8_t dutyCycle){
  if (!pinToWO(pwmPin, _wo, _cc)) return false;  // invalid pin
  if (freqHz < 100 || freqHz > 10000) return false; // out of range
  if (dutyCycle < 0) dutyCycle = 0;
  if (dutyCycle > 100) dutyCycle = 100;  

  _pin = pwmPin;
  _freqHz = freqHz;
  _duty = dutyCycle;

  tcc0_init(0, _pin);     //Configure GPIO peripheral for PAxx and TCC0
  updatePeriodAndCC();
}

// Convert GPIO pin (not Arduino Pin) → WO[x] + CC[x] mapping
bool PWMSingle::pinToWO (uint8_t pin, uint8_t &wo, uint8_t &cc){
    // TCC0 valid pins for SAMD21G18A PAxx (partial) with WO mapping
    // PA04, PA05, PA08, PA09, PA10, PA11, PA12, PA13, PA14 
    // PA15, PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23 
    // NOTE: These numbers are PAxx, not Arduino pins
    // value of wo (_wo) is not used, as whenever a GPIO pin is configured for type "F"
    // it is automatically connected with the TCC0 WO[7] Table 7-1.

    switch (pin) {
      case  4: wo = 0; cc = 0; break; // PA04, WO[0], CC0
      case  5: wo = 1; cc = 1; break; // PA05, WO[1], CC1
      case  8: wo = 0; cc = 0; break; // PA08, WO[0], CC0
      case  9: wo = 1; cc = 1; break; // PA09, WO[1], CC1
      case 10: wo = 2; cc = 2; break; // PA10, WO[2], CC2
      case 11: wo = 3; cc = 3; break; // PA11, WO[3], CC3
      case 12: wo = 6; cc = 2; break; // PA12, WO[6], CC2
      case 13: wo = 7; cc = 3; break; // PA13, WO[7], CC3
      case 14: wo = 4; cc = 0; break; // PA14, WO[4], CC0
      case 15: wo = 5; cc = 1; break; // PA15, WO[5], CC1
      case 16: wo = 4; cc = 0; break; // PA16, WO[4], CC0
      case 17: wo = 5; cc = 1; break; // PA17, WO[5], CC1
      case 18: wo = 2; cc = 2; break; // PA18, WO[2], CC2
      case 19: wo = 3; cc = 3; break; // PA19, WO[3], CC3
      case 20: wo = 6; cc = 2; break; // PA20, WO[6], CC2
      case 21: wo = 7; cc = 3; break; // PA21, WO[7], CC3
      case 22: wo = 4; cc = 0; break; // PA22, WO[4], CC0
      case 23: wo = 5; cc = 1; break; // PA23, WO[5], CC1
      default:
          return false; // Not supported
    }
    return true;
}

// Configure GPIO pins, and TCC0
void PWMSingle::tcc0_init(uint8_t portGroup, uint8_t pwmPin) {
  // Enable PA18 -> TCC0/WO[2]
  PORT->Group[portGroup].PINCFG[pwmPin].bit.PMUXEN = 1;
  // clear the required nibble based on odd/even
  PORT->Group[portGroup].PMUX[pwmPin >> 1].reg &= ~(0xF << (4 * (pwmPin & 1)));
  
  // set bits for Function F based on odd/even, 
  // and hence assigned to a TCCx WO[y] based on table 7-1
  PORT->Group[portGroup].PMUX[pwmPin >> 1].reg |= 0x5 << (4 * (pwmPin & 1));      
  // Enable TCC0 clock
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

  // Connect GCLK0 (48 MHz) to TCC0/1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC0_TCC1);
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Reset TCC0
  TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
  while (TCC0->SYNCBUSY.bit.SWRST);

  // Prescaler = 8
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV8;

  // Normal PWM (NPWM)
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (TCC0->SYNCBUSY.bit.WAVE);

  // Enable TCC0
  TCC0->CTRLA.bit.ENABLE = 1;
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

void PWMSingle::setDutyCycle(uint8_t dutyCycle){
  if (dutyCycle < 0) dutyCycle = 0;
  if (dutyCycle > 100) dutyCycle = 100;
  _duty = dutyCycle;
  updatePeriodAndCC();
}

void PWMSingle::setFrequency(uint32_t freqHz){
  if (freqHz < 100 || freqHz > 10000) return;
  _freqHz = freqHz;
  updatePeriodAndCC();
}

void PWMSingle::setDirection(bool direction){

}

void PWMSingle::updatePeriodAndCC(){
  uint32_t clk = 48000000 / 8; // 6 MHz after prescaler
  uint32_t top = (clk / _freqHz) - 1;
  if (top > 0xFFFF) top = 0xFFFF;

  // Disable TCC0 while updating
  TCC0->CTRLA.bit.ENABLE = 0;
  while (TCC0->SYNCBUSY.bit.ENABLE);

  // Set TOP
  TCC0->PER.reg = top;
  while (TCC0->SYNCBUSY.bit.PER);

  // Set duty cycle
  uint32_t ccVal = (uint32_t)((_duty / 100.0) * (top + 1));

  TCC0->CC[_cc].reg = ccVal;                       // WO[2] normal PWM  

  // Wait for correct CC sync flag
  while (TCC0->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 << _cc));
  //while (TCC0->SYNCBUSY.bit.CC3);

  // Re-enable TCC0
  TCC0->CTRLA.bit.ENABLE = 1;
  while (TCC0->SYNCBUSY.bit.ENABLE);
}




