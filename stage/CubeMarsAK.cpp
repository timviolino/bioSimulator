#include "CubeMarsAK.h"
#define SPI_CS_PIN 9


mcp2515_can CAN1(SPI_CS_PIN);


enum {P, V, KP, KD, To};
const float INITS[CMDS_LENGTH] = {0.0f, 0.0f, 30.0f, 0.5f, 0.0f};
const float MINS[CMDS_LENGTH] = {-12.5f, -50.00f, 0.0f, 0.0f, -25.0f};
const float MAXES[CMDS_LENGTH] = {12.5f, 50.00f, 500.0f, 5.0f, 25.0f};
const float BITS[CMDS_LENGTH] = {16, 12, 12, 12, 12};

enum {ENTER, EXIT, ZERO};
byte MODES[3] = {0xFC, 0xFD, 0xFE};

CubeMarsAK::CubeMarsAK() {}

CubeMarsAK::CubeMarsAK(uint8_t id, bool powered)
  : _id(id)
  , _powered(powered)
  {}

void CubeMarsAK::boot() {
  if(_powered) 
  {
    _setMotorMode(MODES[ENTER]);
    for (int i = 0; i < CMDS_LENGTH; i++) {_cmds[i] = 0.0f;}
    setPos(0.0f);
    _setMotorMode(MODES[ZERO]);
    for (int i = 0; i < CMDS_LENGTH; i++) {_cmds[i] = INITS[i];}
  }
  else {_setMotorMode(MODES[EXIT]);}
}

void CubeMarsAK::setPos(float p) {
  if(!_powered) {return;}
  _cmds[P] = p;
  _packCmds();
  _unpackReply();
}

void CubeMarsAK::setID(uint8_t id) {
  _id = id;
}

void CubeMarsAK::setPower(bool powered) {
  _powered = powered;
}

void CubeMarsAK::_setZero() {
  if (CAN_MSGAVAIL == CAN1.checkReceive())
  {
    byte len = 0;
    CAN1.readMsgBuf(&len, _buf);
    unsigned int p_int = (_buf[1] << 8) | _buf[2];          // 16 bit position, between -4*pi and 4*pi
    _zero = _uint_to_float(p_int, MINS[P], MAXES[P], BITS[P]);
  }
  else {_zero = 0.0;}
  
  #ifdef DEBUG
    Serial.println("Motor id " + String(_id) + " Zero Position: " + String(_zero));
  #endif
}

void CubeMarsAK::_packCmds() {
  unsigned int _cmdInts[CMDS_LENGTH];
  for (int i=0; i<CMDS_LENGTH; i++) 
  {
    _cmds[i] = constrain(_cmds[i], MINS[i], MAXES[i]);                    // limit data to be within bounds
    _cmdInts[i] = _float_to_uint(_cmds[i], MINS[i], MAXES[i], BITS[i]);   // convert floats to unsigned ints 
  }
  
#ifdef DEBUG
  Serial.print("id: " + String(_id));
  Serial.print(" |Pck| p: (");
  Serial.print(_cmds[P], 4); 
  Serial.print(", " + String(_cmdInts[P]) + ")");
  Serial.print(" v: " + String(_cmds[V]));
  Serial.print(" t: " + String(_cmds[To]));
#endif

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
    byte len = 0;
    CAN1.readMsgBuf(&len,_buf);

    /// unpack ints from can buffer ///
    unsigned int p_int = (_buf[1] << 8) | _buf[2];          // 16 bit position, between -4*pi and 4*pi
    unsigned int v_int = (_buf[3] << 4) | (_buf[4] >> 4);   // 12 bit velocity, between -30 and + 30 rad/s
    unsigned int t_int = ((_buf[4] & 0xF) << 8) | _buf[5];

    /// convert uints to floats ///
    float p = _uint_to_float(p_int, MINS[P], MAXES[P], 16);
    float v = _uint_to_float(v_int, MINS[V], MAXES[P], 12);
    float t = _uint_to_float(t_int, MINS[To], MAXES[To], 12);

#ifdef DEBUG
    Serial.print(" |Upck| p: (" + String(p_int) + ", " + String(p));
    Serial.print("), v: (" + String(v_int) + ", " + String(v));
    Serial.println("), t: (" + String(t_int) + ", " + String(t) + ")");
#endif
  }
  else 
  {
    #ifdef DEBUG
      Serial.println(" No CAN message received...");
    #endif
  }
}

void CubeMarsAK::_setMotorMode(byte mode){
 for (int i=0;i<BUF_LENGTH;i++) {_buf[i] = 0xFF;}
 _buf[7] = mode;
 CAN1.sendMsgBuf(_id, 0, BUF_LENGTH, _buf);
 
 #ifdef DEBUG
 Serial.println("Motor id " + String(_id) + " Mode: " + String(mode));
 #endif
 delay(1000);
}

unsigned int CubeMarsAK::_float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 unsigned int pgg = 0;
 if (bits == 12){
   pgg = (unsigned int) ((x-offset)*4095.0/span);
 }
 if (bits == 16){
   pgg = (unsigned int) ((x-offset)*65535.0/span);
 }
 return pgg;
}

float CubeMarsAK::_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
 /// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 float pgg = 0;
 if (bits == 12){
   pgg = ((float)x_int)*span/4095.0 + offset;
 }
 if (bits == 16){
   pgg = ((float)x_int)*span/65535.0 + offset;
 }
 return pgg;
}
