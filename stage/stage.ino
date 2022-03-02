////////////////////////////// Libraries //////////////////////////////
#include <SPI.h>                          // Used for packing CAN messages
#include <mcp2515_can.h>                  // Used for packing CAN messages
#include "CubeMarsAK.h"                   // Cube Mars AK Series motor control class 
#include "waveforms.h"                    // array of hardcoded sine wave positions
#include <Wire.h>                         // Used for networking between Arduinos

////////////////////////////// Global Constant Defintions //////////////////////////////
#define DEBUG
#define CAN_INT_PIN 3                         // attach to arduino pin d3 to connect with can shield int
#define SPI_CS_PIN 10                         // attach to arduino pin d4 MUST BE 9 FOR UNO SHIELD
enum {WAIT, BOOT, RUN_TEST, COMPLETE};        // machine states
enum {FREQUENCY, STROKE};                     // indices of parameters
enum {ROLL, PITCH, YAW};                      // indices of motors
enum {START, STOP};                           // indices of i2c codes
const bool ON[3] = {true, false, false};
const bool OFF[3] = {false, false, false};
const uint8_t N_MTR = 3;                      // number of motors
const uint8_t ADDY = 10;                      // i2c address
constexpr uint8_t CODES[2] = {245, 255};      // i2c codes, needs 'constexpr' modifier to be used in switch case
const uint8_t I_STEP = 4;                     // step size taken through sine wave
const uint8_t t_FSTEP = 250;                  // time per step of ramp
const uint64_t BAUD_RATE = 230400;            // rate of communications over serial bus
const float F_STEP = 0.1f;                    // step at which the frequency is changed

mcp2515_can CAN(SPI_CS_PIN);
CubeMarsAK motors[N_MTR];

////////////////////////////// Test Parameter Variable Declarations //////////////////////////////
// The expected test parameters are as follows:
// frequency in Hz {0.5, 5}
// stroke in deg {1, 7.5}

volatile float params[2] = {5.0f, 2.0f};                // stores parameters for duration of test
float days = 0.0f, hours = 4.0f, mins = 0.0f;           // easy duration set up
uint8_t state = WAIT;                                   // current machine state
float stroke_rad = 0.0f;                                     // motor input equivalent to user specified deg's of rotation
uint8_t i_pos = 0;                                      // stores current index used for wave array
uint8_t t_step = 0;                                     // time between steps on wave
unsigned long t_lastStep = 0;                           // time at which motors were last actuated
uint8_t i_input = FREQUENCY;                            // index of param for ui upload
float p_offset = 3.5f;
uint64_t i_offset = 0;

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
    case WAIT:
      break;
    case BOOT:
      state = RUN_TEST;               // NOTE - this needs to be at beginning else it will override STOP signal from display
      initCANShield();
      initMotors(ON);
      ramp(0, params[FREQUENCY]);
      bootTiming();
      break;
    case RUN_TEST:
      oscillate(t_step, I_STEP);
      break;
    case COMPLETE:
      ramp(params[FREQUENCY], 0);
      center();
      initMotors(OFF);
      state = WAIT;
      break;
  }
}

/////////////////// High Level Functions ///////////////////

void oscillate(unsigned long t_step, uint8_t i_step)
{
  uint8_t i, i_new;
  uint64_t del_t = get_del_t(t_lastStep);           // record time since last step
  if (del_t > t_step)                               // check if time between steps has elapsed
  {
    t_lastStep += del_t;                            // update time of last step
    float p = getPos(i_pos);                        // record position on sine wave
    i_new = i_pos + i_step;                         // save temporary value for next logic statement
    i_pos = (i_new >= 120) ? 0 : i_new;             // if i+step >= 120, i = 0; else i += step
    for (i = 0; i < N_MTR; i++) {
      motors[i].setPos(p);
      if (motors[i]._powered) {
        Serial.print(radToDeg(p));
        Serial.print(" ");
        float out = radToDeg(motors[i].getPos());
        Serial.println(out);
      }
    }
  }
}

void ramp(float f0, float f1)
{
  float f = f0;                                   // current frequency [Hz]
  const float dir = copysign(1, f1 - f0);         // direction of ramp (-1:down, +1:up)
  uint64_t t0, del_t;                             // variables used for timing [ms]
  t0 = millis();
  while (dir * (f1 - f) > F_STEP)
  {
    del_t = get_del_t(t0);
    t_step = get_t_step(f, STEPS / I_STEP) + 1;
    oscillate(t_step, I_STEP);
    if (del_t > t_FSTEP)
    {
      f += dir * F_STEP;
      t0 += del_t;
    }
  }
}

void center()
{
  uint8_t f = 1; uint8_t i_step = 1;
  t_step = get_t_step(f, STEPS) + 1;              // set time step for centering
  while (i_pos != i_step) {
    oscillate(t_step, i_step);                    // rotate until theta = 0
  }
}

void printOffset(float p)
{
  const float p_max = 3.5;
  i_offset ++;
  if (i_offset > 1000) {
    p_offset = p_max;
  }
  float del_p = abs(p - p_max);
  if (del_p < p_offset) {
    p_offset = del_p;
  }
  Serial.print(" ");
  Serial.println(p_offset);
}

/////////////////// Low Level Functions ///////////////////

float getPos(uint8_t i_pos)
{
  int16_t p_int = pgm_read_word_near(SINE + i_pos);   // read current value out from table
  p_int -= (WAVE_AMP / 2 - 1);                        // zero align position. -u1 for zero indexing
  return p_int * stroke_rad * 2.f / WAVE_AMP;         // convert to motor radians
}

uint64_t get_t_step(float f, uint16_t n) {
  return (f != 0) ? int(1000/(f*n)) : 1000;
}

void receive(int b) {
  uint8_t msg = Wire.read();
  switch (msg) {
    case CODES[STOP]:
      state = COMPLETE;
      break;
    case CODES[START]:
      state = BOOT;
      break;
    default:
      params[i_input] = msg / 10.0f;
      i_input = (i_input == 1) ? 0 : 1;
      break;
  }
}

void initCANShield()
{
  while (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
#ifdef DEBUG
    //Serial.println("CAN BUS Shield init fail");
#endif
    delay(100);
  }
#ifdef DEBUG
  //Serial.println("CAN BUS Shield init ok!");
#endif
}

void initMotors(bool powered[3])
{
  for (int i = 0; i < N_MTR; i++) {
    motors[i].setID(i+3);
    motors[i].setPower(powered[i]);
    motors[i].init();
  }
  stroke_rad = degToRad(params[STROKE]);
}

void bootTiming()
{
  t_step = get_t_step(params[FREQUENCY], STEPS / I_STEP);
  t_lastStep = millis();
}

//////////////////////////// Generic Helper Functions //////////////////////////////////
void mySerialBegin()
{
  Serial.begin(BAUD_RATE);
  while (!Serial); // wait for serial
}

float getMillis(float d, float hr, float m) {return 60.f*1000.f*(d * 1440.f + hr * 60.f + m);}

unsigned long get_del_t(unsigned long t0) {return millis() - t0;}

// Note, there is a slight offset per radian in the encoder, hence the deviation from 0.01745
float degToRad(float deg) {return deg * 0.017125f;}

float radToDeg(float rad) {return rad / 0.017125f;}
