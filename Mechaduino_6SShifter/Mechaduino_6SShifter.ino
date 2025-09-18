
/*
  -------------------------------------------------------------
  Mechaduino 0.1 & 0.2 Firmware  v0.1.5
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to all contributors!
  --------------------------------------------------------------
  
  Controlled via a SerialUSB terminal at 115200 baud.

  Implemented serial commands are:

 s  -  step
 d  -  dir
 p  -  print [step number] , [encoder reading]

 c  -  calibration routine
 e  -  check encoder diagnositics
 q  -  parameter query

 x  -  position mode
 v  -  velocity mode
 t  -  torque mode

 y  -  enable control loop
 n  -  disable control loop
 r  -  enter new setpoint

 j  -  step response
 k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled
 g  -  generate sine commutation table
 m  -  print main menu


  ...see serialCheck() in Utils for more details

*/

/************** Linear actuator, by Rylan *****************/

#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"


#define limitSwitch1 2
#define limitSwitch2 3
#define led ledPin 


//Gear up positions:
#define gearRatio 5.18    //57/11
#define backLash 2        //barrel shaft backlash in degree



//Parameters
const int aisPin  = A4;
const int numReadings  = 300; //300;
int readings [numReadings];
int readIndex  = 0;
long total  = 0;
int potFactor = 3; 
int maxLinPos = 10; 
int minLinPos = -10; 

//Variables
int aisVal  = 0;
int lastTime = 0; 

long lastBlink = 0;
int secCount = 0;
double dd = 5;
//int adcAverage = 0;
int adcOldAverage = 0;
int countAdcItr = 0;

// Shifting parameters
float r_1_mid = 0;
float r_N_mid = 0;
float r_2_mid = 0;
float r_3_mid = 0;
float r_4_mid = 0;
float r_5_mid = 0;
float r_6_mid = 0;
float r_dn_offset = 0;

void setup()        // This code runs once at startup
{                         
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt
  disengageMotor();             // Same as "N" openloop mode, to reduce motor current

  // pinMode(limitSwitch1, INPUT_PULLUP);
  // pinMode(limitSwitch2, INPUT_PULLUP);

  SerialUSB.begin(115200);          
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  Serial1.begin(115200);          
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  Serial1.println("Servo started");


  serialMenu();                     // Prints menu to serial monitor
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  
  // spot check some of the lookup table to decide if it has been filled in
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0){
    SerialUSB.println("WARNING: Lookup table is empty! Run calibration");
    //calibrate();
  }

  adcAverage = 0;

  mode = 'x';                   // start in position mode
  // r = yw; 
  // double r0 = r;  

  //magneticOffset = read_angle();
  
  // adjust the direction, +/-, if less than 180, then ok, if greater than 180, then put -
  // if (magneticOffset > 180)  magneticOffset = magneticOffset - 360;      
  
  SerialUSB.print("MagOffset: ");
  SerialUSB.println(magneticOffset);

  int newMagOffset = read_angle();
  if (newMagOffset > 180)  newMagOffset = newMagOffset - 360;

  SerialUSB.print("Actual MagOffset: ");
  SerialUSB.println(newMagOffset);

  r = magneticOffset;  

  calculateRvalues();

  delay(1000);

  enableTCInterrupts();         // uncomment this line to start in closed loop 
  delay(1000);
}


void loop()                     // main loop
{
  serialCheck();              //must have this execute in loop for serial commands to function

  // r = gearAngle + magneticOffset;

  // gearChange();
  //readAnalogSmooth();
  // if (abs(adcAverage - adcOldAverage)>1){
  //   SerialUSB.print(F("ais val ")); SerialUSB.println(round(adcAverage/3.4),0);    
  //   adcOldAverage = adcAverage;
  // }
  
  //delay(50);

  //r = -(adcAverage*1.4);        //gear ratio 5 2/11 = 5.182
  // gearTest();
  // gearTest2();

  //delay (5000);

  hbBlink();  
  if (secCount >= 900) {
    secCount = 0;
  }
}


void calculateRvalues(){
  // up shift r values  
  r_1_mid = -(1*gearRatio) + magneticOffset;       //1 mid
  r_N_mid = -(39*gearRatio) + magneticOffset;       //N mid
  r_2_mid = -(74*gearRatio) + magneticOffset;       //2 mid
  r_3_mid = -(131*gearRatio) + magneticOffset;       //3 mid
  r_4_mid = -(198*gearRatio) + magneticOffset;       //4 mid
  r_5_mid = -(259*gearRatio) + magneticOffset;       //5 mid
  r_6_mid = -(313*gearRatio) + magneticOffset;       //6 mid & end

  r_dn_offset = backLash*gearRatio;
}

void disengageMotor(){
  disableTCInterrupts();          //disable closed loop
  analogFastWrite(VREF_2, 0);     //set phase currents to zero
  analogFastWrite(VREF_1, 0);                       
}


void gearTest(){
  int delayMS = 500;
  int delayLong =2000;
  //r = 60;
  //delay(delayMS);
  r = -208 + magneticOffset;       //1 start
  delay(delayMS);
  r = -390 + magneticOffset;       //1 mid
  delay(delayLong);
  r = -539 + magneticOffset;       //1 end  
  delay(delayMS);                        //nutral
  r = -580 + magneticOffset;       //2 start   // 590, has grinding
  delay(delayMS);
  r = -714 + magneticOffset;       //2 mid
  delay(delayLong);
  r = -870 + magneticOffset;       //2 end
  delay(delayMS);
  r = -950 + magneticOffset;       //3 start
  delay(delayMS);
  r = -1075 + magneticOffset;       //3 mid
  delay(delayLong);
  r = -1278 + magneticOffset;       //3 end
  delay(delayMS);
  r = -1310 + magneticOffset;       //4 start
  delay(delayMS);
  r = -1430 + magneticOffset;       //4 mid & end
  delay(delayLong);
  // r = -1530;       //dead end
  // delay(500);

// gear down
  r = -1310 + magneticOffset;       //4 start
  delay(delayMS);
  r = -1278 + magneticOffset;       //3 end
  delay(delayMS);
  r = -1075 + magneticOffset;       //3 mid
  delay(delayLong);
  r = -950 + magneticOffset;       //3 start
  delay(delayMS);
  r = -870 + magneticOffset;       //2 end
  delay(delayMS);
  r = -714 + magneticOffset;       //2 mid
  delay(delayLong);
  r = -590 + magneticOffset;       //2 start
  delay(delayMS);
  r = -539 + magneticOffset;       //1 end
  delay(delayMS);
  r = -390 + magneticOffset;       //1 mid         
  delay(delayLong);
  r = -208 + magneticOffset;       //1 start
  delay(delayMS);
  r = 40 + magneticOffset;           // Nutral
  delay (delayLong);
}


void gearTest2(){
  int delayMS = 500;
  int delayLong =3000;

  //Up shift  
  // r = 0 + magneticOffset;       //1 mid & end
  // delay(delayLong);
  r = r_1_mid - (0.5*gearRatio);       //1 mid
  delay(delayLong);

  r = r_N_mid - (0*gearRatio);       //N mid
  delay(delayLong);

  r = r_2_mid - (-6*gearRatio);       //2 mid
  delay(delayLong);

  r = r_3_mid - (-9*gearRatio);       //3 mid
  delay(delayLong);

  r = r_4_mid - (0*gearRatio);       //4 mid
  delay(delayLong);

  r = r_5_mid - (0*gearRatio);       //5 mid & end
  delay(delayLong);

  r = r_6_mid - (3*gearRatio);       //6 mid & end
  delay(delayLong);

// Down shift
  r =  r_5_mid + r_dn_offset -(-1*gearRatio);       //5 mid
  delay(delayLong);

  r =  r_4_mid + r_dn_offset - (-4*gearRatio);       //4 mid
  delay(delayLong);

  r = r_3_mid + r_dn_offset -(-4*gearRatio);       //3 mid
  delay(delayLong);

  r = r_2_mid + r_dn_offset -(0*gearRatio);       //2 mid
  delay(delayLong);

  r = r_N_mid + r_dn_offset - (0.5*gearRatio);       //N mid
  delay(delayLong);

  r = r_1_mid + r_dn_offset - (-1*gearRatio);         // 1 mid
  delay (delayLong);
}

void hbBlink(){
  long fromReset = millis();
  long now = fromReset - lastBlink;  
  
  if (now < 80) digitalWrite(led, HIGH);      
  if ((now > 80) && (now <= 180)) digitalWrite(led, LOW);
  if ((now > 180) && (now <= 200)) digitalWrite(led, HIGH);
  if ((now > 200) && (now <= 999)) digitalWrite(led, LOW);
  if (now > 999){
    delayMicroseconds(10);
    lastBlink = fromReset;
    secCount = secCount +1; //must be initialized to 0 externally after the expected period
  }
}


void readAnalogSmooth( ) { /* function readAnalogSmooth */
  //Test routine for AnalogSmooth
  aisVal += analogRead(aisPin);
  countAdcItr += 1;

  int itrMax = 20;

  if (countAdcItr>itrMax){
    adcAverage = aisVal/itrMax;
    //SerialUSB.print(F("ais val ")); SerialUSB.println(adcAverage);
    countAdcItr = 0;        
    aisVal = 0;
  }
}


long smooth() { /* function smooth */
  ////Perform average on sensor readings
  long average;
  // subtract the last reading:
  total = total - readings[readIndex];
  // read the sensor:
  readings[readIndex] = map(analogRead(aisPin),0,1023, minLinPos, maxLinPos );
  // add value to total:
  total = total + readings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

 if (average > maxLinPos) {average = maxLinPos;}
 if (average < minLinPos) {average = minLinPos;}

  return average;
}


void homeActuatorForceSence()
{
  while (U < 55)
  {
    r += 1;
    delay(2);
    //SerialUSB.println(U);
  }
  SerialUSB.print("Extended limit found at position: ");
  SerialUSB.println(r);
  maxLinPos = r - 200; 
  SerialUSB.print("Soft max Extension: ");  
  SerialUSB.println(maxLinPos);
  
  r -= 30;
  delay(100);
  while (U < 85)
  {
    r -= 1;
    delay(2);  
    //SerialUSB.println(U);
  }
  
  SerialUSB.print("Retrected limit found at position: ");
  SerialUSB.println(r);
  minLinPos = r + 200; 
  SerialUSB.print("Soft max Retrection: ");  
  SerialUSB.println(minLinPos);

  r += 400; 
  delay(500);
  PA += 0.9;
}


void backAndForthAtSpeed()
{
  //r = smooth() ; 
  while (r < maxLinPos)
  {
    r += 7;
    delay(1);
  }
  
  delay(100);
  while (r > minLinPos)
  {
    r -= 3;
    delay(1);
  }
  delay(100);
}


void backAndForthStep()
{
  //r = smooth() ; 
  r = maxLinPos; 
  delay(2000);
  r = minLinPos; 
  delay(2000);
}
