////////////////////////////// Libraries //////////////////////////////
#include <SPI.h>                          // Used for packing CAN messages
#include <mcp2515_can.h>                  // Used for packing CAN messages
#include "CubeMarsAK.h"                   // Cube Mars AK Series motor control class 
#include "waveforms.h"                    // array of hardcoded sine wave positions
#include <Wire.h>                         // Used for networking between Arduinos

////////////////////////////// Global Constant Defintions //////////////////////////////
//#define DEBUG
#define CAN_INT_PIN 3                         // attach to arduino pin d3 to connect with can shield int
#define SPI_CS_PIN 10                         // attach to arduino pin d4 MUST BE 9 FOR UNO SHIELD
enum {WAIT, START, RUN_TEST, STOP};           // machine states
enum {FREQUENCY, STROKE, STEP};               // indices of parameters
enum {YAW, PITCH, ROLL};                      // indices of motors
enum {MIN, MAX};                              // indices of ranges
bool ON[3] = {true, true, true};              // code sent to turn selected motors on
bool OFF[3] = {false, false, false};          // code sent to turn all motors off
const uint8_t N_MTR = 3;                      // number of motors
const uint8_t ADDY = 10;                      // i2c address
constexpr uint8_t CODES[4] = {0, 245, 0, 255};// i2c codes, needs 'constexpr' modifier to be used in switch case
const uint8_t t_FSTEP = 250;                  // time per step of ramp
const uint64_t BAUD_RATE = 230400;            // rate of communications over serial bus
const float RANGE[3][2] = {
  {1.f, 5.f},                                 // frequency [Hz]
  {3.f, 7.5f},                                // stroke [deg]
  {1.f, 4.f}                                  // step size for sine wave [0-120]
};
const float RANGE_KD[3][2] = {
  {0.03f, 0.5f},
  {0.1f,  2.5f},
  { .5f,  4.9f}
};
const float RANGE_KP[3] = {215.f, 350.f, 350.f};
const float RANGE_ROLL_OFFSET[2] = {.05f, .13f};

mcp2515_can CAN(SPI_CS_PIN);
CubeMarsAK motors[N_MTR];

////////////////////////////// Test Parameter Variable Declarations //////////////////////////////
volatile float params[3] = {1.0f, 3.0f, 1.f};   // stores parameters: [Hz, deg, 0-120, Kp, Kd] 
float rollOffset = 1.f;                         // function of frequency and stroke 
uint8_t state = WAIT;                           // current machine state
float stroke_rad = 0.0f;                        // motor input equivalent to user specified deg's of rotation
uint8_t i_pos = 0;                              // stores current index used for wave array
uint8_t t_step = 0;                             // time between steps on wave
unsigned long t_lastStep = 0;                   // time at which motors were last actuated
uint8_t i_input = FREQUENCY;                    // index of param for ui upload
float p_max = 0.f;

//////////////////////////////// Setup & Loop ////////////////////////////////
void setup()
{
  mySerialBegin();
  Wire.begin(ADDY);
  Wire.onReceive(receive);
}

void loop()
{
  switch (state)
  {
    case WAIT: break;
    case START:
      state = RUN_TEST;               // NOTE - this needs to be at beginning else it will override STOP signal from display
      myCANBegin();
      initMotors(ON);
      ramp(1, params[FREQUENCY]);
      t_step = get_t_step(10*params[FREQUENCY], STEPS / params[STEP]);
      t_lastStep = millis();
      break;
    case RUN_TEST:
      takeStep(t_step, params[STEP]);
      break;
    case STOP:
      state = WAIT;
      ramp(params[FREQUENCY], 1);
      center();
      initMotors(OFF);
      break;
  }
}

/////////////////// High Level Functions ///////////////////

void takeStep(unsigned long t_step, uint8_t i_step)
{
  uint8_t i, i_new;
  uint64_t del_t = get_del_t(t_lastStep);         // record time since last step
  if (del_t <= t_step) {return;}                  // check if time between steps has elapsed
  t_lastStep += del_t;                            // update time of last step
  float p = getPos(i_pos);                        // record position on sine wave
  i_new = i_pos + i_step;                         // save temporary value for next logic statement
  i_pos = (i_new >= 120) ? 0 : i_new;             // if i+step >= 120, i = 0; else i += step
  for (i = 0; i < N_MTR; i++) {
    if(i == ROLL) {motors[i].setPos(p*rollOffset);}
    else {motors[i].setPos(p);}
    //if (motors[i]._powered) {printMax(radToDeg(motors[i].get(P)), i_pos);}
  }
}

void printMax(float p, uint8_t i)
{
  if (p > p_max) {p_max = p;}
  if (i < params[2]) {                        // end of cycle
    if (p_max > 0) {Serial.println(p_max);}
    p_max = 0;
  }
}

void ramp(uint8_t f0, uint8_t f1)
{
  uint16_t f = f0*10;                             // current frequency [Hz]
  const int8_t dir = copysign(1, f1 - f0);        // direction of ramp (-1:down, +1:up)
  uint8_t i_step = i_step = calcStep(f);          // step size taken through sine wave
  uint64_t t0 = millis(); uint16_t del_t;         // variables used for timing [ms]   
  f1 *= 10;                          
  while (dir * (f1 - f) > 1)
  {
    del_t = get_del_t(t0);                        // get time since last step
    t_step = get_t_step(f, STEPS / i_step);
    takeStep(t_step, i_step);
    if (del_t > t_FSTEP)
    {
      f += dir;
      i_step = calcStep(f);
      t0 += del_t;
    }
  }
}

uint8_t calcStep(uint8_t f) {
  uint8_t step = 1;
  step = (f > 20) ? 2:step;
  step = (f > 30) ? 3:step;
  step = (f > 40) ? 4:step;
  return step;
}

void center()
{
  const uint8_t f = 1; const uint8_t i_step = 1;
  t_step = get_t_step(10*f, STEPS) + 1;              // set time step for centering
  while (i_pos != i_step) {
    takeStep(t_step, i_step);                        // rotate until theta = 0
  }
}

/////////////////// Low Level Functions ///////////////////

float getPos(uint8_t i_pos)
{
  int16_t p_int = SINE[i_pos];                        // read current value out from table
  p_int -= (WAVE_AMP / 2 - 1);                        // zero align position. -u1 for zero indexing
  return p_int * stroke_rad * 2.f / WAVE_AMP;         // convert to motor radians
}

uint64_t get_t_step(uint16_t f, uint16_t n) {
  return (f != 0) ? int(10000/(f*n)) : 1000;
}

void receive(int b) {
  uint8_t msg = Wire.read();
  switch (msg) {
    case CODES[STOP]:
      state = STOP;
      break;
    case CODES[START]:
      state = START;
      break;
    default:
      params[i_input] = msg / 10.0f;
      i_input = (i_input == 1) ? 0 : 1;
      break;
  }
}

void initMotors(bool powered[3])
{
  stroke_rad = degToRad(params[STROKE]);
  params[STEP] = interpolate(params[FREQUENCY], RANGE[FREQUENCY], RANGE[STEP]);
  params[STEP] = round(params[STEP]);
  float factor = interpolate(params[STROKE], RANGE[STROKE], RANGE_ROLL_OFFSET);
  float range_factor[2] = {factor, 0.f};
  factor -= interpolate(params[FREQUENCY], RANGE[FREQUENCY], range_factor);
  rollOffset = 1.f - factor;
  for (uint8_t i = 0; i < N_MTR; i++) {
    motors[i].setID(i+3);
    motors[i].setPower(powered[i]);
    motors[i].init();
    float kp_temp = RANGE_KP[i];
    uint8_t i_param = (i == YAW) ? STROKE : FREQUENCY;
    float kd_temp = interpolate(params[i_param], RANGE[i_param], RANGE_KD[i]);
    motors[i].set(KP, kp_temp);
    motors[i].set(KD, kd_temp);
  }
}

//////////////////////////// Generic Helper Functions //////////////////////////////////
void mySerialBegin() {Serial.begin(BAUD_RATE); while (!Serial);}  // wait for serial

void myCANBegin() {while (CAN_OK != CAN.begin(CAN_1000KBPS)) {delay(100);}}

unsigned long get_del_t(unsigned long t0) {return millis() - t0;}

float interpolate(float x, const float X[2], const float Y[2]) {return Y[0] + (x - X[0]) / (X[1] - X[0]) * (Y[1] - Y[0]);}

// Note, there is a slight offset per radian in the encoder, hence the deviation from 0.01745
const float k = 0.017125f;
float degToRad(float deg) {return deg * k;}

float radToDeg(float rad) {return rad / k;}
