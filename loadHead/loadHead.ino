//#define DEBUG

////////////////////////////// Library Imports ///////////////////////////////
#include <HX711.h>                            // Used for reading load cell amplifier
#include <Wire.h>                             // Used for Arduino networking
#include "LinearActuator.h"                   // custom linear actuator control class

////////////////////////////// Constant Value Definitions ///////////////////////////////
#define CLK 2                               // connect Arduino pin D2 to HX711 CLK
#define DAT 3                               // connect Arduino pin D3 to HX711 DAT 
#define RPWM 10                             // connect Arduino pin D10 to IBT-2 pin RPWM
#define LPWM 11                             // connect Arduino pin D11 to IBT-2 pin LPWM
enum {WAIT, BOOT, RUN_TEST, DOWN};          // machine states, used for main switch case 
enum {MIN, MAX};                            // indices of V and F
enum {START, STOP};                         // indices of CODES
const uint8_t ADDY = 11;                    // i2c address of this arduino
constexpr uint8_t CODES[2] = {245, 255};    // i2c codes, needs 'constexpr' modifier to be used in switch case

const uint8_t t[2] = {18, 36};              // range of step times [ms]
const uint8_t V[2] = {45, 115};             // range of motor speeds [0-255]

const int8_t F_STATIC = -7;                 // dead weight on specimen
const uint16_t F[2] = {50, 400};            // load capacities of system [N]
const int64_t LC_FACTOR = -7050;            // factor used to calibrate laod cell with known weight
const uint64_t BAUD_RATE = 115200;          // baud rate used for serial communications with IDE

////////////////////////////// Global Variable Declarations ///////////////////////////////
uint8_t state = WAIT;                       // stores the current state of the machine
int16_t v_run = V[MIN];                     // stores current speed [0-255]
uint16_t t_step = t[MIN];                   // stores current time step [ms]
volatile uint16_t F_goal = 50;              // load set by the user
float F_range = 0.5f;                       // hold load with 1% of load
float F_ratio = 0.0f;                       // stores percentage of max load test is operating at
float F_over = 0.0f;                        // calculates overshoot

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
  switch (state) {
    case WAIT:
      break;

    case BOOT:
      state = RUN_TEST;     // NOTE - this needs to be at beginning else it will override STOP signal from display
      actuator.init();
      initLoadCell();
      initVariables();
      break;

    case RUN_TEST:
      runTest();
      printVariables();
      break;

    case DOWN:
      actuator.step(-80, 2000);
      state = WAIT;
      break;
  }
}

//////// Helper Functions ////////
void receive(int b) {
  uint8_t msg = Wire.read();
  if (msg == CODES[STOP]) {
    state = DOWN;
  }
  else if (msg == CODES[START]) {
    state = BOOT;
  }
  else if (state == WAIT) {
    F_goal = msg * 10;
  }
}

void receive2(int b) {
  uint8_t msg = Wire.read();
  switch (msg) {
    case CODES[STOP]:
      state = DOWN;
      break;
    case CODES[START]:
      state = BOOT;
      break;
    default:
      F_goal = msg * 10;
      break;
  }
}

void runTest() {
  float F_applied, del_F; int16_t v_temp;
  volatile float F_read = read_F();                     // record load measurement from sensor [N]
  F_applied = fabs(F_read + F_STATIC);                  // calculate true load being applied to specimen
  del_F = F_goal - F_applied;                           // calculate offset from goal load
  v_temp = (fabs(del_F) > F_range) ? v_run : 0;         // set magnitude of approach: v_run if out of range or 0 if within range
  v_temp = copysign(v_temp, del_F);                     // set sign of approach: 
  actuator.step(v_temp, t_step);                        // actuate at speed v for time t
  checkOvershoot(F_applied);                            
}

float read_F() {
  delay(90);        // delay to prevent flooding load cell
  float F_read = 0.00f;
  if (scale.is_ready()) {
    F_read = 9.81f * scale.get_units(2);
  }

#ifdef DEBUG
  Serial.print("F_applied = ");
  Serial.print(fabs(F_read + F_STATIC), 2);
#endif
  return F_read;
}

void initVariables()
{
  F_ratio = float(F_goal)/ F[MAX];                  // calculate goal laod as percent full load scale 
  v_run = interpolate(F_ratio, V);                  // calculate speed via linear extrapolation
  t_step = interpolate(F_ratio, t);                 // calculate step time - - - 
  F_range = F_goal * 0.01;                          // calculate 1% precision bar
  F_over = 0.f;                                     // reset overshoot value (NOT USED IN ACTUAL TEST)
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

void checkOvershoot(float F) {
  F_over = (F > F_over) ? F : F_over;
}

void printVariables()
{
#ifdef DEBUG
  Serial.print(" | F_goal = " + String(F_goal));
  Serial.print(" | v_run = " + String(v_run));
  Serial.print(" | t_step = " + String(t_step));
  Serial.print(" | F_range = " + String(F_range));
  Serial.print(" | overshoot = " + String(F_over - F_goal));
  Serial.println("");       // clear line
#endif
}

/////////////////////////////// Generic Helper Functions ///////////////////////////////
void mySerialBegin() {
  Serial.begin(BAUD_RATE);    // begin serial communications at BAUD_RATE
  while (!Serial);            // wait for serial
}

int16_t interpolate(float k, const uint8_t X[2]) {return X[MIN] + k * (X[MAX] - X[MIN]);}
