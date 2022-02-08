//#define debug

#ifndef _motorPackets_h_
#define _motorPackets_h_

#include "mathOps.h"

#define NUM_CMDS 5
enum CMDENUM {P, V, KP, KD, To};
const float MINS[NUM_CMDS] = {-12.5f, -50.00f, 0.0f, 0.0f, -25.0f};
const float MAXES[NUM_CMDS] = {12.5f, 50.00f, 500.0f, 5.0f, 25.0f};
const float BITS[NUM_CMDS] = {16, 12, 12, 12, 12};

void pack_cmd(uint8_t id, float cmds[5]) {
  byte buf[8];                                        // CAN Packet is 8 8-bit words

  unsigned int cmd_ints[NUM_CMDS] = {};
  for (int i=0; i<NUM_CMDS; i++) 
  {
    cmds[i] = constrain(cmds[i], MINS[i], MAXES[i]);                    // limit data to be within bounds
    cmd_ints[i] = float_to_uint(cmds[i], MINS[i], MAXES[i], BITS[i]);   // convert floats to unsigned ints 
  }
  
  
#ifdef DEBUG
  Serial.print("id: " + String(id));
  Serial.print(" |Pck| p: (" + String(cmds[P]) + ", " + String(cmd_ints[P]) + ")");
  //Serial.print(", v: " + String(v));
  //Serial.print(", t: " + String(t));
  //Serial.print(", v: " + String(v_int));
  //Serial.print(", t: " + String(t_int));
#endif

  /// pack ints into the can buffer, bit 0 is LSB ///
  buf[0] = cmd_ints[P] >> 8;                                   // 0: [position[15-8]]
  buf[1] = cmd_ints[P] & 0xFF;                                 // 1: [position[7-0]]
  buf[2] = cmd_ints[V] >> 4;                                   // 2: [velocity[11-4]]
  buf[3] = ((cmd_ints[V] & 0xF) << 4) | (cmd_ints[KP] >> 8);   // 3: [velocity[3-0], kp[11-8]]
  buf[4] = cmd_ints[KP] & 0xFF;                                // 4: [kp[7-0]]
  buf[5] = cmd_ints[KD] >> 4;                                  // 5: [kd[11-4]]
  buf[6] = ((cmd_ints[KD] & 0xF) << 4) | (cmd_ints[To] >> 8);  // 6: [kd[3-0], torque[11-8]]
  buf[7] = cmd_ints[To] & 0xFF;                                // 7: [torque[7-0]]
  
  CAN.sendMsgBuf(id, 0, 8, buf);
}

float readPosition() {
    float p = 0.0;
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
      byte len = 0;
      byte buf[8];                                          // CAN Packet is 5 8-bit words
      CAN.readMsgBuf(&len, buf);
      unsigned int p_int = (buf[1] << 8) | buf[2];          // 16 bit position, between -4*pi and 4*pi
      p = uint_to_float(p_int, MINS[P], MAXES[P], 16);
    }
    else
    {
      #ifdef DEBUG
        Serial.print(" No CAN message received...");
      #endif
    }
    return p;
}

void unpack_reply() {

  // check if data coming
  if (CAN_MSGAVAIL == CAN.checkReceive()) 
  {
    byte len = 0;
    byte buf[8];                                          // CAN Packet is 5 8-bit words
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    /// unpack ints from can buffer ///
    unsigned int id = buf[0];
    unsigned int p_int = (buf[1] << 8) | buf[2];          // 16 bit position, between -4*pi and 4*pi
    unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);   // 12 bit velocity, between -30 and + 30 rad/s
    unsigned int t_int = ((buf[4] & 0xF) << 8) | buf[5];

    /// convert uints to floats ///
    float p = uint_to_float(p_int, MINS[P], MAXES[P], 16);
    float v = uint_to_float(v_int, MINS[V], MAXES[P], 12);
    float t = uint_to_float(t_int, MINS[To], MAXES[To], 12);

#ifdef DEBUG
    Serial.print(" |Upck| p: (" + String(p_int) + ", " + String(p));
    Serial.print("), v: (" + String(v_int) + ", " + String(v));
    Serial.print("), t: (" + String(t_int) + ", " + String(t) + ")");
#endif
  }
  else 
  {
    #ifdef DEBUG
      Serial.print(" No CAN message received...");
    #endif
  }
}
#endif
