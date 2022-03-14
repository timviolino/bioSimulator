#ifndef CUBE_MARS_AK_h_
#define CUBE_MARS_AK_h_

#include <Arduino.h>
#include <mcp2515_can.h>

#define BUF_LENGTH 8
#define N_CMDS 5
enum {P, V, To, KP, KD};          // cols of CMD

class CubeMarsAK
{
  private:
    // Variables
    uint8_t _id;
    float _zero;
    byte _buf[BUF_LENGTH];
    float _cmds[N_CMDS];
    unsigned int _reads[N_CMDS-2] = {0, 0, 0};

    // Functions
    void _packCmds();
    void _unpackReply();
    void _setMode(byte mode);
    float _ui2f(uint32_t, float, float, uint8_t);
    uint32_t _f2ui(float, float, float, uint8_t);
    
  public:
    bool _powered;
    
    CubeMarsAK();
    void init();
    void setPower(bool powered);
    void setID(uint8_t id);
    void setPos(float p);
    void set(uint8_t, float);
    float get(uint8_t);
};

#endif
