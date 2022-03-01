////////////////////////////// Libraries //////////////////////////////
#include <SPI.h>                          // Used for packing CAN messages
#include <mcp2515_can.h>                  // Used for packing CAN messages
#include "CubeMarsAK.h"                   // Cube Mars AK Series motor control class 
#include "waveforms.h"                    // array of hardcoded sine wave positions
#include <Wire.h>                         // Used for networking between Arduinos

////////////////////////////// Global Constant Defintions //////////////////////////////
#define CAN_INT_PIN 3                         // attach to arduino pin d3 to connect with can shield int
#define SPI_CS_PIN 10                         // attach to arduino pin d4 MUST BE 9 FOR UNO SHIELD
enum {USER_INPUT, BOOT, RUN_TEST, COMPLETE};  // machine states
enum {FREQUENCY, STROKE};                     // indices of parameters
enum {ROLL, PITCH, YAW};                      // indices of motors
enum {START, STOP};                           // indices of i2c codes
const uint8_t N_MTR = 3;                      // number of motors
const uint8_t ADDY = 10;                      // i2c address
const uint8_t CODES[2] = {245, 255};          // i2c codes
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

bool powered[3] = {true, true, true};                   // which motors are powered, id needs to be > 2
volatile float params[2] = {5.0f, 2.0f};                // stores parameters for duration of test                                                         
float days = 0.0f, hours = 4.0f, mins = 0.0f;           // easy duration set up 
uint8_t state = USER_INPUT;                             // current machine state 
float k_rad = 0.0f;                                     // motor input equivalent to user specified deg's of rotation
uint8_t i_pos = 0;                                      // stores current index used for wave array
uint8_t t_step = 0;                                     // time between steps on wave
unsigned long t_lastStep = 0;                           // time at which motors were last actuated
uint8_t i_input = FREQUENCY;                            // index of param for ui upload

//////////////////////////////// Setup & Loop ////////////////////////////////
void setup() 
{
  mySerialBegin();
  setupMotors();
  Wire.begin(ADDY);
  Wire.onReceive(receive);
}

void loop() 
{
  switch (state) 
  {
    case USER_INPUT:
      break;
    case BOOT:
      state = RUN_TEST;               // NOTE - this needs to be at beginning else it will override STOP signal from display
      initCANShield();
      boot();
      ramp(0, params[FREQUENCY]);
      bootTiming();
      break;
    case RUN_TEST:
      oscillate(t_step, I_STEP);
      break;
    case COMPLETE:
      ramp(params[FREQUENCY], 0);
      center();
      stopTest();
      setupMotors();
      state = USER_INPUT;
      break;
  }
}

void initCANShield() 
{
  while (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
  #ifdef DEBUG 
  Serial.println("CAN BUS Shield init fail"); 
  #endif
    delay(100);
  }
  #ifdef DEBUG 
  Serial.println("CAN BUS Shield init ok!"); 
  #endif
}

void boot() 
{
  for (int i = 0; i < N_MTR; i++) {motors[i].boot();}
  k_rad = degToRad(params[STROKE]);
}

void bootTiming() 
{
  t_step = get_t_step(params[FREQUENCY], STEPS/I_STEP);
  t_lastStep = millis();
}

uint64_t get_t_step(float f, uint16_t n) {
  float f_step;
  uint64_t t_step = 1000;
  if(f != 0)
  {
    f_step = n * f;                         // [step/s]
    t_step = int(1000.0f/f_step);           // [ms/step], int
  }
  return t_step;
}

void oscillate(unsigned long t_step, uint8_t i_step) 
{
  uint8_t i, i_new;
  uint64_t del_t = get_del_t(t_lastStep);                // record time since last step
  if (del_t > t_step)                                    // check if time between steps has elapsed
  {               
    t_lastStep += del_t;                                 // update time of last step                            
    float p = getPosition(i_pos);                        // record position on sine wave
    i_new = i_pos + i_step;                              // save temporary value for next logic statement
    i_pos = (i_new >= 120) ? 0 : i_new;                  // if i+step >= 120, i = 0; else i += step
    for (i = 0; i < N_MTR; i++) {motors[i].setPos(p);}   // iterate through motors
  }
}

void ramp(float f0, float f1) 
{
  float f = f0;
  const float dir = copysign(1, f1-f0);
  uint64_t t0, del_t; 
  t0 = millis();
  while (dir*(f1-f) > F_STEP)
  {
    del_t = get_del_t(t0);
    t_step = get_t_step(f, STEPS/I_STEP)+1;
    oscillate(t_step, I_STEP);
    if (del_t > t_FSTEP) 
    {
      f += dir*F_STEP;
      t0 += del_t;
    }
  }
}

void center()
{
  t_step = get_t_step(5*F_STEP, STEPS)+1;       // set time step for centering
  while (i_pos != 1) {oscillate(t_step, 1);}    // rotate until theta = 0
}

void stopTest() 
{ 
  for (int i = 0; i < N_MTR; i++) 
  {
    motors[i].setPower(false);
    motors[i].boot();
  }
}

float getPosition(uint8_t i_pos) 
{
  int16_t p_int = float(pgm_read_word_near(waveformsTable + i_pos));   // read current value out from table
  p_int -= (WAVE_AMP/2-1);                                             // zero allign position. -u1 for zero indexing
  float p_float = p_int/float(WAVE_AMP/2);                             // normalize to -1, +1 amplitude
  p_float *= k_rad;                                                    // convert to motor radians
  return p_float;
}

void receive(int b) {
  uint8_t msg = Wire.read();
  if (msg == CODES[STOP]) 
  {
    state = COMPLETE;
    i_input = FREQUENCY;
  }
  else if (msg == CODES[START]) 
  {
    state = BOOT;
    i_input = FREQUENCY;
  }
  else if (state == USER_INPUT) 
  {
    params[i_input] = msg/10.0f;
    i_input++;
    if (i_input > STROKE) {i_input = 0;}
  }
}

void setupMotors() 
{
  for (int i = 0; i < N_MTR; i++) 
  {
    uint8_t id = i+3;
    motors[i].setID(id);
    motors[i].setPower(powered[i]);
  }
}
//////////////////////////// Generic Helper Functions //////////////////////////////////
void mySerialBegin() 
{
  Serial.begin(BAUD_RATE);
  while (!Serial); // wait for serial
}

float getMillis(float days, float hours, float mins) 
{
  return days*86400000.f + hours*3600000.f + mins*60000.f;
}

unsigned long get_del_t(unsigned long t0) 
{
  unsigned long t = millis();
  return t-t0;
}

float degToRad(float deg)
{
  const float K = 0.017125f;      // Note, there is a slight offset per radian in the encoder, hence the deviation from 0.01745 
  return deg*K;
}
