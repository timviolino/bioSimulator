#define DEBUG

////////////////////////////// Test Parameter definitions ///////////////////////////////
volatile int goalLoad = 200;                    // load set by the user
float deadband = 0.5f;                         // hold load with 1% of load

////////////////////////////// States ///////////////////////////////
enum stateEnum {USER_INPUT, BOOT, RUN_TEST, STOP}; 
uint8_t state = BOOT;

////////////////////////////// Serial definitions ///////////////////////////////
#include <Wire.h>
const unsigned long baudRate = 9600;


////////////////////////////// Linear Actuator definitions ///////////////////////////////
#define RPWM 10                                   // connect Arduino pin 10 to IBT-2 pin RPWM
#define LPWM 11                                   // connect Arduino pin 11 to IBT-2 pin LPWM
const float MIN_SPEED = 43.0f;                    // set min speed for linear actuator
const float MAX_SPEED = 90.0f;                    // set max speed for linear actuator
const float SPEED_RANGE = MAX_SPEED - MIN_SPEED;
byte testSpeed = MIN_SPEED;                       // variable used to store actual speed


//////////////////////////// Scale definitions //////////////////////////////
#include "HX711.h"
#define DAT 3
#define CLK 2
HX711 scale;
const float MIN_LOAD = 50.0;                      // minimum load at which 1% precision is achievable
const float MAX_LOAD = 400.0;                     // maximum load capacity of system [N]
const int CALIBRATION_FACTOR = -7050;             // factor used to calibrate laod cell with known weight
float loadRatio = 0.0f;                           // stores percentage of max load test is operating at


//////// Main Functions ////////
void setup() {
  //delay(3000);
  mySerialBegin();
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(9);
  Wire.onReceive(receiveEvent);
}

void loop() {
  switch(state) {
    case USER_INPUT:
      #ifdef DEBUG
      Serial.println("waiting...");
      #endif
      break;
      
    case BOOT:
      initLinearActuator();
      initLoadCell();
      state = RUN_TEST;
      break;
      
    case RUN_TEST:
      runTest();
      delay(90);
      break;
      
    case STOP:
      state = USER_INPUT;
      break;
  }
}

//////// Helper Functions ////////
void receiveEvent(int bytes) {
  if (state == USER_INPUT) {
    goalLoad = -Wire.read();
    state = BOOT;
  }
  else if (Wire.read() == 1){state = STOP;}
}

void runTest() {
  volatile float load = readLoad();                    // record load measurement from sensor [N]
  float loadOffset = fabs(goalLoad)-fabs(load);
  if (loadOffset < -deadband) {setMotorSpeed(-testSpeed);}
  else if (loadOffset > deadband) {setMotorSpeed(testSpeed);}
  setMotorSpeed(0);

#ifdef DEBUG
  Serial.print("Goal Load = " + String(goalLoad));
  Serial.println(" | Test Speed = " + String(testSpeed) + " | Deadband = " + String(deadband));
  Serial.println("");                                 // clear serial com line
#endif
}

volatile float readLoad() {
  volatile float load = 0.00f;
  if (scale.is_ready()) {
    load = 9.81 * scale.get_units(2);
  }

#ifdef DEBUG
  Serial.print("Load = ");
  Serial.print(load, 2);
  Serial.print(" N | ");
#endif

  return load;
}

void setMotorSpeed(int motorSpeed) {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  if (motorSpeed < 0) {analogWrite(RPWM, abs(motorSpeed));}    // Retract actuator
  else {analogWrite(LPWM, abs(motorSpeed));}                   // Extend actuator
  delay(30);                                                   // Minimum travel time
}

void mySerialBegin() {
  Serial.begin(baudRate);
  while (!Serial); // wait for serial
}

void initLinearActuator() {
#ifdef DEBUG
  Serial.println("");
  Serial.println("Beginning linear actuator program");
#endif

  pinMode(RPWM, OUTPUT); // Configure pin 10 as an output
  pinMode(LPWM, OUTPUT); // Configure pin 11 as an output

#ifdef DEBUG
  Serial.println("Retracting Actuator");
#endif

  // set variables 
  if(goalLoad > MIN_LOAD) {
    loadRatio = (float(goalLoad)-MIN_LOAD)/MAX_LOAD;
    testSpeed = MIN_SPEED + loadRatio*SPEED_RANGE;
    deadband = goalLoad*0.01;
  }
  
  // retract actuator to ensure load cell tares properly
  int retractTime = 500+loadRatio*500;
  setMotorSpeed(-MAX_SPEED);
  delay(retractTime);
  setMotorSpeed(0);
  
  

#ifdef DEBUG
  Serial.println("Retract Time = " + String(retractTime));
#endif
}

void initLoadCell() {
#ifdef DEBUG
  Serial.println("Starting scale");
#endif

  scale.begin(DAT, CLK);
  scale.set_scale(CALIBRATION_FACTOR);  // this value is obtained by calibrating the scale with known weights;

#ifdef DEBUG
  Serial.println("Taring scale");
#endif

  scale.tare(); //Reset the scale to 0

#ifdef DEBUG
  Serial.println("Beginning loop...");
#endif

}
