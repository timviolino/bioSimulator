/*#include "CubeMarsAK.h"
#include <mcp2515_can.h>                  // Used for packing CAN messages
#define SPI_CS_PIN 9

mcp2515_can CAN(SPI_CS_PIN);

#define BUF_LENGTH 8

CubeMarsAK::CubeMarsAK()
  {}

void CubeMarsAK::enterMotorMode(uint8_t id){
 // Enter Motor Mode (enable)
 byte buf[BUF_LENGTH];
 buf[0] = 0xFF;
 buf[1] = 0xFF;
 buf[2] = 0xFF;
 buf[3] = 0xFF;
 buf[4] = 0xFF;
 buf[5] = 0xFF;
 buf[6] = 0xFF;
 buf[7] = 0xFC;
 CAN.sendMsgBuf(id, 0, BUF_LENGTH, buf);
 
 #ifdef DEBUG
 Serial.println("Motor Mode Initiated: Motor ID " + String(id));
 #endif
 
 delay(100);
}

void CubeMarsAK::exitMotorMode(uint8_t id){
 // Exit Motor Mode (disable)
 byte buf[BUF_LENGTH];
 buf[0] = 0xFF;
 buf[1] = 0xFF;
 buf[2] = 0xFF;
 buf[3] = 0xFF;
 buf[4] = 0xFF;
 buf[5] = 0xFF;
 buf[6] = 0xFF;
 buf[7] = 0xFD;
 CAN.sendMsgBuf(id, 0, BUF_LENGTH, buf);
 delay(100);
}


// NOTE this is the suggested zeroing method, but it has unpredictable results
// use with caution
void CubeMarsAK::zero(uint8_t id){
 // set zero point for motor
 byte buf[BUF_LENGTH];
 buf[0] = 0xFF;
 buf[1] = 0xFF;
 buf[2] = 0xFF;
 buf[3] = 0xFF;
 buf[4] = 0xFF;
 buf[5] = 0xFF;
 buf[6] = 0xFF;
 buf[7] = 0xFE;
 CAN.sendMsgBuf(id, 0, BUF_LENGTH, buf);

 delay(100);
}

unsigned int CubeMarsAK::float_to_uint(float x, float x_min, float x_max, int bits){
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

float CubeMarsAK::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
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
}*/
