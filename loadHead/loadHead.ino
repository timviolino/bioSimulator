#define DEBUG

////////////////////////////// Library Imports ///////////////////////////////
#include <HX711.h>                            // Used for reading load cell amplifier
#include <Wire.h>                             // Used for Arduino networking
#include "LinearActuator.h"                   // custom linear actuator control class

////////////////////////////// Constant Value Definitions ///////////////////////////////
#define CLK 2                               // connect Arduino pin D2 to HX711 CLK
#define DAT 3                               // connect Arduino pin D3 to HX711 DAT 
#define RPWM 10                             // connect Arduino pin D10 to IBT-2 pin RPWM
#define LPWM 11                             // connect Arduino pin D11 to IBT-2 pin LPWM
enum {USER_INPUT, BOOT, RUN_TEST, SHUT_DOWN}; // indices used for accessing machine states
enum {MIN, MAX, RAMP};                      // indices used for accessing physical constants
enum {START, STOP};                         // indices used for i2c codes
const uint8_t ADDY = 11;                    // i2c ADDY
const uint8_t CODES[2] = {245, 255};        // i2c codes
const uint8_t V[3] = {47, 90, 55};          // start up speed used to prevent linear actuator 'sticking'
const uint16_t F[2] = {50, 400};            // load capacities of system [N]
const int64_t LC_FACTOR = -7050;            // factor used to calibrate laod cell with known weight
const uint64_t BAUD_RATE = 115200;          // baud rate used for serial communications with IDE

////////////////////////////// Global Variable Declarations ///////////////////////////////
uint8_t state = USER_INPUT;              // stores the current state of the machine
uint8_t v_run = V[MIN];                  // variable used to store actual speed
volatile uint16_t F_goal = 50;           // load set by the user
float F_range = 0.5f;                    // hold load with 1% of load
float F_ratio = 0.0f;                    // stores percentage of max load test is operating at

//////////////////////////// Object Declarations //////////////////////////////
HX711 scale;
LinearActuator actuator = LinearActuator(RPWM, LPWM);


//////// Main Functions ////////
void setup() {
  mySerialBegin();
  Wire.begin(ADDY);
  Wire.onReceive(receive);
}

void loop() {
  switch(state) {
    case USER_INPUT:
      break;
      
    case BOOT:
      actuator.init();
      initLoadCell();
      initVariables();
      state = RUN_TEST;
      break;
      
    case RUN_TEST:
      runTest();
      printVariables();
      delay(90);
      break;

    case SHUT_DOWN:
      actuator.retract();
      state = USER_INPUT;
      break;
  }
}

//////// Helper Functions ////////
void receive(int b) {
  uint8_t msg = Wire.read();
  if (msg == CODES[STOP]) {state = SHUT_DOWN;}
  else if (msg == CODES[START]) {state = BOOT;}
  else if (state == USER_INPUT) {F_goal = msg*10;}
}

void runTest() {
  volatile float F_read = read_F();                    // record load measurement from sensor [N]
  float del_F = fabs(F_goal)-fabs(F_read);
  if (del_F < -F_range) {actuator.setSpeed(-v_run);}
  else if (del_F > F_range) {actuator.setSpeed(v_run);}
  actuator.setSpeed(0);
}

float read_F() {
  float F_read = 0.00f;
  if (scale.is_ready()) {F_read = 9.81f * scale.get_units(2);}
  
  #ifdef DEBUG
  Serial.print("Load = ");
  Serial.print(F_read, 2);
  #endif
  
  return F_read;
}

void initVariables()
{
  if(F_goal > F[MIN]) {
    F_ratio = (float(F_goal)-F[MIN])/F[MAX];
    v_run = V[MIN] + F_ratio*(V[MAX]-V[MIN]);
    F_range = F_goal*0.01;
  }
}

void initLoadCell() {
#ifdef DEBUG
  Serial.println("Starting scale");
#endif

  scale.begin(DAT, CLK);
  scale.set_scale(LC_FACTOR);  // this value is obtained by calibrating the scale with known weights;

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
    Serial.print(" | Goal Load = " + String(F_goal));
    Serial.print(" | Test Speed = " + String(v_run) + " | F_range = " + String(F_range));
    Serial.println("");       // clear line
  #endif
}

/////////////////////////////// Generic Helper Functions ///////////////////////////////
void mySerialBegin() {
  Serial.begin(BAUD_RATE);    // begin serial communications at BAUD_RATE
  while (!Serial);            // wait for serial
}
