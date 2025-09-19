#include "GPIOEventCountSAMD21.h"

EventCounter counter0(0, 6, 4, EIC_SenseMode::EDGERISE); // PA10 → TC4
EventCounter counter1(0, 7, 5, EIC_SenseMode::EDGERISE);   // PA09 → TC5

//#define TC_INSTANCE    TC4

void setup() {
    SerialUSB.begin(115200);
    delay(3000);
    counter0.begin();
    counter1.begin();
}

void loop() {
    static unsigned long lastMillis = 0;
    if (millis() - lastMillis >= 1000) {
        lastMillis = millis();
        SerialUSB.print("Counter0: "); SerialUSB.print(counter0.read());
        SerialUSB.print(" | Counter1: "); SerialUSB.print(counter1.read());
        SerialUSB.println();
        counter0.reset();
        counter1.reset();
    }
}

