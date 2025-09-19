#include "PWMSingleChannel.h"

PWMSingle pwm1;

#define DIR_PIN 6

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(15200);
  delay(3000);

  if(!pwm1.begin(21,800,1)){
    Serial.println("failed to start PWM");
  }
  Serial.println("Probably pwm is running..");
}

void loop() {
  // pwm1.setDutyCycle(10);
  // delay(5000);
  // pwm1.setDutyCycle(50);
  // delay(5000);

  //dir = !dir;
  digitalWrite(DIR_PIN, HIGH);
  for (int d = 10; d <= 90; d += 10) {
    pwm1.setDutyCycle(d);
    delay(2000);
  }

  for (int d = 90; d >= 10; d -= 10) {
    pwm1.setDutyCycle(d);
    delay(2000);
  }  
  digitalWrite(DIR_PIN, LOW);
  for (int d = 10; d <= 90; d += 10) {
    pwm1.setDutyCycle(d);
    delay(2000);
  }

  for (int d = 90; d >= 10; d -= 10) {
    pwm1.setDutyCycle(d);
    delay(2000);
  }  
}
