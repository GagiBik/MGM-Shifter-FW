#include "GPIOEventCountSAMD21.h"

// Pin → EIC channel map (SAMD21G18A)
struct PinEICMapEntry { uint8_t port; uint8_t pin; uint8_t eic; };
static const PinEICMapEntry PinEICMap[] = {
    {0, 0, 0}, {0,1,1}, {0,2,2}, {0,3,3}, {0,4,4}, {0,5,5}, {0,6,6}, {0,7,7},
    {0,8,8}, {0,9,9}, {0,10,10}, {0,11,11}, {0,12,12}, {0,13,13}, {0,14,14}, {0,15,15},
    {0,16,0}, {0,17,1}, {0,18,2}, {0,19,3},
    {1,0,0}, {1,1,1}, {1,2,2}, {1,3,3}, {1,4,4}, {1,5,5}, {1,6,6}, {1,7,7}
};

EventCounter::EventCounter(uint8_t port, uint8_t pin, uint8_t tcNumber, EIC_SenseMode senseMode)
: _port(port), _pin(pin), _tcNumber(tcNumber), _senseMode(senseMode)
{
  /*  Initialization is done in the begin(), as constructor runs before the MCU starts, 
      which may cause failure to proper configuration of the peripherals.
  */  
  // DEBUG_PRINTLN(DEBUG_INFO, "EventCounter constructor called");  // Keep the MCU inactive
}

bool EventCounter::validateInputs() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Validating inputs");
  if (_port > 1) return false;
  if (_pin > 31) return false;
  if (!(_tcNumber == 3 || _tcNumber == 4 || _tcNumber == 5)) return false;
  return true;
}

bool EventCounter::mapEICChannel() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Mapping EIC channel");
  _eicChannel = 0xFF;
  for (auto &m : PinEICMap) {
      if (m.port == _port && m.pin == _pin) { _eicChannel = m.eic; return true; }
  }
  return false;
}

void EventCounter::begin() {
  // Serial.print("Port:"); Serial.print(_port); Serial.print(" | Counter: "); Serial.print(_tcNumber);
  
  // if (!validateInputs()) return;
  // if (!mapEICChannel()) return;

  DEBUG_PRINTLN(DEBUG_INFO, "EventCounter.begin()");

  if (!validateInputs()) {
      DEBUG_PRINTLN(DEBUG_ERROR, "Invalid port/pin/TC number");
      return;
  }
  if (!mapEICChannel()) {
      DEBUG_PRINTLN(DEBUG_ERROR, "Failed to map EIC channel");
      return;
  }

  configurePM();
  configureGCLK();
  configureGPIO();
  configureEIC();
  configureEVSYS();
  configureTC();

  DEBUG_PRINT(DEBUG_INFO, "Initialized using Port ");
  DEBUG_PRINT(DEBUG_INFO, _port);
  DEBUG_PRINT(DEBUG_INFO, ", Pin ");
  DEBUG_PRINT(DEBUG_INFO, _pin);
  DEBUG_PRINT(DEBUG_INFO, ", TC");
  DEBUG_PRINT(DEBUG_INFO, _tcNumber);
  DEBUG_PRINT(DEBUG_INFO, ", eicChannel ");
  DEBUG_PRINTLN(DEBUG_INFO, _eicChannel);
}

void EventCounter::configurePM() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Configuring PM");

  PM->APBAMASK.reg |= PM_APBAMASK_EIC;
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  if (_tcNumber == 3){
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;
    DEBUG_PRINTLN(DEBUG_DEBUG, "TC3 Selected");
  } 
  else if (_tcNumber == 4){
    PM->APBCMASK.reg |= PM_APBCMASK_TC4;
    DEBUG_PRINTLN(DEBUG_DEBUG, "TC4 Selected");
  } 
  else if (_tcNumber == 5){
    PM->APBCMASK.reg |= PM_APBCMASK_TC5;
    DEBUG_PRINTLN(DEBUG_DEBUG, "TC5 Selected");
  } 
}

void EventCounter::configureGCLK() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Configuring GCLK");
  
  // EIC
  // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EIC_GCLK_ID) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN; 
  while (GCLK->STATUS.bit.SYNCBUSY);

  // TCx
  // uint8_t gclk_id = (_tcNumber == 3) ? GCLK_CLKCTRL_ID_TCC2_TC3 : GCLK_CLKCTRL_ID_TC4_TC5;
  uint8_t gclk_id = (_tcNumber == 3) ? TC3_GCLK_ID :
                    (_tcNumber == 4) ? TC4_GCLK_ID : TC5_GCLK_ID;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(gclk_id) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TC4_TC5 | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
}

void EventCounter::configureGPIO() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Configuring GPIO");

  uint8_t pmuxIndex = _pin >> 1;                  // Divided by 2, faster than using mathmetical division 
  _pmuxFunc = PORT_PMUX_PMUXE_A_Val;              // Always function A = EIC, same value for PMUXO/PMUXE

  DEBUG_PRINT(DEBUG_INFO, "pmuIndex ");
  DEBUG_PRINTLN(DEBUG_INFO, _pmuxFunc);           // pmuIndex does not have global scope to print by DEBUG_PRINT

  PORT->Group[_port].DIRCLR.reg = (1ul << _pin);  // input

  // Enable input buffer and pull resistor
  PORT->Group[_port].PINCFG[_pin].bit.PULLEN = 0; // No pull-up/down,
  PORT->Group[_port].PINCFG[_pin].bit.INEN   = 1; // Enable input

  // Uncomment if pull-up/down is used
  // Set pull-up (OUTSET) or pull-down (OUTCLR)
  // PORT->Group[_port].OUTSET.reg = (1 << _pin); // Pull-up example

  PORT->Group[_port].PINCFG[_pin].bit.PMUXEN = 1; // Connect to a peripheral instead of digital I/O

  if (_pin & 1){
    PORT->Group[_port].PMUX[pmuxIndex].bit.PMUXO = _pmuxFunc;           // Odd pins
  } 
  else{
    PORT->Group[_port].PMUX[pmuxIndex].bit.PMUXE = _pmuxFunc;           // Even pins
  } 
}

void EventCounter::configureEIC() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Configuring EIC");

  EIC->CTRL.bit.ENABLE = 0;
  while (EIC->STATUS.bit.SYNCBUSY);

  uint8_t cfgIndex = _eicChannel / 8;                                   // Select CONFIG register 0 or 1
  uint8_t chan = _eicChannel % 8;                                       // Determine x in SENSEx[2:0] bits 

  EIC->CONFIG[cfgIndex].reg |= ((uint32_t)_senseMode << (chan * 4));    // Set intup signal type, 
                                                                        // SENSEx[2:0] + FILTENx = 4

  EIC->INTENCLR.reg = (1 << _eicChannel);                               // To ensure no interrupt is sent to the CPU
  EIC->EVCTRL.reg |= (1 << _eicChannel);                                // _eicChannel = EXTINTx, x = 0 to 15

  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY);
}

void EventCounter::configureEVSYS() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Configuring EVSYS");

  uint8_t evsysChannel = _tcNumber - 3; // TC3 → 0, TC4 → 1, TC5 → 2
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(evsysChannel) |                                // channel index, 0, 1, 2
                        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_0 + _eicChannel) |      // EVSYS_ID_GEN_EIC_EXTINT_0 = 12
                        EVSYS_CHANNEL_PATH_ASYNCHRONOUS;

  EVSYS->USER.reg = EVSYS_USER_USER(15 + _tcNumber) |                       // EVSYS_ID_USER_TC3_EVU = 18, _TC4_ = 19, _TC5_ = 20
                    EVSYS_USER_CHANNEL(evsysChannel + 1);                   // USER reg uses 1-based channel numbers
}

void EventCounter::configureTC() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Configuring TC");

  Tc *TCx = (_tcNumber == 3) ? TC3 :
            (_tcNumber == 4) ? TC4 : TC5;

  TCx->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);

  TCx->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;
  TCx->COUNT16.EVCTRL.reg = TC_EVCTRL_TCEI |                // Enable event input
                                    TC_EVCTRL_EVACT_COUNT;  // Increment counter on each event
  TCx->COUNT16.COUNT.reg = 0;

  TCx->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

uint32_t EventCounter::read() {
  DEBUG_PRINTLN(DEBUG_VERBOSE, "Reading TC counter value");  
  Tc *TCx = (_tcNumber == 3) ? TC3 :
            (_tcNumber == 4) ? TC4 : TC5;
  
  uint16_t count = TCx->COUNT16.COUNT.reg;
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
  return count;
}

void EventCounter::reset() {
  DEBUG_PRINTLN(DEBUG_DEBUG, "Resetting TC counter");
  Tc *TCx = (_tcNumber == 3) ? TC3 :
            (_tcNumber == 4) ? TC4 : TC5;
  TCx->COUNT16.COUNT.reg = 0;
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}
