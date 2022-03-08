#include "CubeMarsAK.h"
#define SPI_CS_PIN 10

mcp2515_can CAN1(SPI_CS_PIN);

enum {INIT, MIN, MAX, BIT};       // rows of CMD
enum {ENTER, EXIT, ZERO};         // indices of MODES
const float CMD[4][N_CMDS] = {
  {  0.0f,   0.0f,    0.0f,  115.f,   .5f},
  {-12.5f, -50.00f, -25.0f,   0.0f,  0.0f},
  { 12.5f,  50.00f,  25.0f, 500.0f,  5.0f},
  { pow(2, 16)-1, pow(2, 12)-1, pow(2, 12)-1, pow(2, 12)-1, pow(2, 12)-1}
};
const byte MODES[3] = {0xFC, 0xFD, 0xFE};

////////////////// Function Definitions /////////////////
CubeMarsAK::CubeMarsAK() {}

void CubeMarsAK::init() {
  if(_powered) 
  {
    _setMode(MODES[ENTER]);
    _setMode(MODES[ZERO]);                                       
    for (uint8_t i = 0; i < N_CMDS; i++) {_cmds[i] = CMD[INIT][i];}
  }
  else {_setMode(MODES[EXIT]);}
}

void CubeMarsAK::setPos(float p) {
  if(!_powered) {return;}
  _cmds[P] = p;
  _packCmds();
  _unpackReply();
}

void CubeMarsAK::setID(uint8_t id) {_id = id;}

void CubeMarsAK::setPower(bool powered) {_powered = powered;}

void CubeMarsAK::set(uint8_t i, float x) {_cmds[i] = x;}

float CubeMarsAK::get(uint8_t i) {return _ui2f(_reads[i], CMD[MIN][i], CMD[MAX][i], CMD[BIT][i]);}

void CubeMarsAK::_packCmds() {
  unsigned int _cmdInts[N_CMDS];
  for (uint8_t i = 0; i<N_CMDS; i++) 
  {
    _cmds[i] = constrain(_cmds[i], CMD[MIN][i], CMD[MAX][i]);                    // limit data to be within bounds
    _cmdInts[i] = _f2ui(_cmds[i], CMD[MIN][i], CMD[MAX][i], CMD[BIT][i]);   // convert floats to unsigned ints 
  }

  /// pack ints into the can buffer, bit 0 is LSB ///
  _buf[0] = _cmdInts[P] >> 8;                                   // 0: [position[15-8]]
  _buf[1] = _cmdInts[P] & 0xFF;                                 // 1: [position[7-0]]
  _buf[2] = _cmdInts[V] >> 4;                                   // 2: [velocity[11-4]]
  _buf[3] = ((_cmdInts[V] & 0xF) << 4) | (_cmdInts[KP] >> 8);   // 3: [velocity[3-0], kp[11-8]]
  _buf[4] = _cmdInts[KP] & 0xFF;                                // 4: [kp[7-0]]
  _buf[5] = _cmdInts[KD] >> 4;                                  // 5: [kd[11-4]]
  _buf[6] = ((_cmdInts[KD] & 0xF) << 4) | (_cmdInts[To] >> 8);  // 6: [kd[3-0], torque[11-8]]
  _buf[7] = _cmdInts[To] & 0xFF;                                // 7: [torque[7-0]]
  
  CAN1.sendMsgBuf(_id, 0, 8, _buf);
}

void CubeMarsAK::_unpackReply() {
  if (CAN_MSGAVAIL == CAN1.checkReceive()) 
  {
    CAN1.readMsgBuf(0,_buf);
    _reads[P] = (_buf[1] << 8) | _buf[2];          // 16 bit position, between -4*pi and 4*pi
    _reads[V] = (_buf[3] << 4) | (_buf[4] >> 4);   // 12 bit velocity, between -30 and + 30 rad/s
    _reads[To] = ((_buf[4] & 0xF) << 8) | _buf[5];
  }
}

void CubeMarsAK::_setMode(byte mode){
 if(mode == MODES[ZERO]) // NOTE: must give all zero command before zeroing
 {
    for (uint8_t i = 0; i < N_CMDS; i++) {_cmds[i] = 0.0f;}
    setPos(0.0f);
 }
 
 for (uint8_t i = 0; i < BUF_LENGTH; i++) {_buf[i] = 0xFF;}
 _buf[7] = mode;
 CAN1.sendMsgBuf(_id, 0, BUF_LENGTH, _buf);
 delay(1000);
}

// Conversions between float and unsigned int
unsigned int CubeMarsAK::_f2ui(float x, float x_min, float x_max, float bits){
 float span = x_max - x_min;
 return (uint32_t) ((x-x_min)*bits/span);
}

float CubeMarsAK::_ui2f(unsigned int x_int, float x_min, float x_max, float bits){
 float span = x_max - x_min;
 return ((float)x_int)*span/bits + x_min;
}
