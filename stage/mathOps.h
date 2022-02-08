#ifndef _mathOps_h_
#define _mathOps_h_

unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
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

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
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

#endif
