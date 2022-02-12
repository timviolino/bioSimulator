#ifndef CUBE_MARS_AK_h_
#define CUBE_MARS_AK_h_

//#define DEBUG 

#include <Arduino.h>
#include <mcp2515_can.h>

#define BUF_LENGTH 8
#define CMDS_LENGTH 5


class CubeMarsAK
{
  private:
    // Variables
    uint8_t _id;
    bool _powered;
    float _zero;
    byte _buf[BUF_LENGTH];
    float _cmds[CMDS_LENGTH];

    // Functions
    void _setZero();
    void _packCmds();
    void _unpackReply();
    void _setMotorMode(byte mode);
    float _uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
    unsigned int _float_to_uint(float x, float x_min, float x_max, int bits);
    

  public:
    CubeMarsAK();
    CubeMarsAK(uint8_t id, bool powered);
    void boot();
    void setID(uint8_t id);
    void setPower(bool powered);
    void setPos(float p);
    
};


#endif
