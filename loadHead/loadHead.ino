#define DEBUG

////////////////////////////// Library Imports ///////////////////////////////
#include <HX711.h>                            // Used for reading load cell amplifier
#include <Wire.h>                             // Used for Arduino networking

////////////////////////////// Constant Value Definitions ///////////////////////////////
#define CLK 2                                     // connect Arduino pin D2 to HX711 CLK
#define DAT 3                                     // connect Arduino pin D3 to HX711 DAT 
#define RPWM 10                                   // connect Arduino pin D10 to IBT-2 pin RPWM
#define LPWM 11                                   // connect Arduino pin D11 to IBT-2 pin LPWM
enum {USER_INPUT, BOOT, RUN_TEST, STOP};          // indices used for accessing machine states
enum {MIN, MAX, RAMP};                            // indices used for accessing physical constants
const uint8_t SPEEDS[3] = {43, 90, 55};           // start up speed used to prevent linear actuator 'sticking'
const uint16_t LOADS[2] = {50, 400};              // load capacities of system [N]
const uint32_t t_RETRACT = 6000;                  // time for linear actuator to retract at beginning of test
const int64_t CALIBRATION_FACTOR = -7050;         // factor used to calibrate laod cell with known weight
const uint64_t BAUD_RATE = 9600;                  // baud rate used for serial communications with IDE

////////////////////////////// Global Variable Declarations ///////////////////////////////
uint8_t state = BOOT;                             // stores the current state of the machine
uint8_t testSpeed = SPEEDS[MIN];                  // variable used to store actual speed
volatile uint16_t goalLoad = 50;                  // load set by the user
float deadband = 0.5f;                            // hold load with 1% of load
float loadRatio = 0.0f;                           // stores percentage of max load test is operating at

//////////////////////////// Object Declarations //////////////////////////////
HX711 scale;

//////// Main Functions ////////
void setup() {
  mySerialBegin();
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
      initVariables();
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
}

volatile float readLoad() {
  volatile float load = 0.00f;
  if (scale.is_ready()) {load = 9.81f * scale.get_units(2);}
  
  #ifdef DEBUG
  Serial.print("Load = ");
  Serial.print(load, 2);
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

void initLinearActuator() {
  pinMode(RPWM, OUTPUT);        // configure pin 10 as an output
  pinMode(LPWM, OUTPUT);        // configure pin 11 as an output
  
#ifdef DEBUG
  Serial.println("");
  Serial.println("Beginning linear actuator program");
  Serial.println("Retracting Actuator");
#endif
  
  setMotorSpeed(-SPEEDS[MAX]);  // retract actuator to ensure load cell tares properly
  delay(t_RETRACT);
  setMotorSpeed(0);
}

void initVariables()
{
  if(goalLoad > LOADS[MIN]) {
    loadRatio = (float(goalLoad)-LOADS[MIN])/LOADS[MAX];
    testSpeed = SPEEDS[MIN] + loadRatio*(SPEEDS[MAX]-SPEEDS[MIN]);
    deadband = goalLoad*0.01;
  }
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

  scale.tare();                         //Reset the scale to 0

#ifdef DEBUG
  Serial.println("Beginning loop...");
#endif
}

void printVariables()
{
  #ifdef DEBUG
    
    Serial.print(" N | ");
    Serial.print("Goal Load = " + String(goalLoad));
    Serial.println(" | Test Speed = " + String(testSpeed) + " | Deadband = " + String(deadband));
    Serial.println("");       // clear serial com line
  #endif
}

/////////////////////////////// Generic Helper Functions ///////////////////////////////
void mySerialBegin() {
  Serial.begin(BAUD_RATE);    // begin serial communications at BAUD_RATE
  while (!Serial);            // wait for serial
}
