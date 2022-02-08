#ifndef _motorMode_h_
#define _motorMode_h_

#define LENGTH 8

void enterMotorMode(uint8_t id){
 // Enter Motor Mode (enable)
 byte buf[LENGTH];
 buf[0] = 0xFF;
 buf[1] = 0xFF;
 buf[2] = 0xFF;
 buf[3] = 0xFF;
 buf[4] = 0xFF;
 buf[5] = 0xFF;
 buf[6] = 0xFF;
 buf[7] = 0xFC;
 CAN.sendMsgBuf(id, 0, LENGTH, buf);
 
 #ifdef DEBUG
 Serial.println("Motor Mode Initiated: Motor ID " + String(id));
 #endif
 
 delay(100);
}

void exitMotorMode(uint8_t id){
 // Exit Motor Mode (disable)
 byte buf[LENGTH];
 buf[0] = 0xFF;
 buf[1] = 0xFF;
 buf[2] = 0xFF;
 buf[3] = 0xFF;
 buf[4] = 0xFF;
 buf[5] = 0xFF;
 buf[6] = 0xFF;
 buf[7] = 0xFD;
 CAN.sendMsgBuf(id, 0, LENGTH, buf);
 delay(100);
}

// NOTE this is the suggested zeroing method, but it has unpredictable results
// use with caution
void zero(uint8_t id){
 // set zero point for motor
 byte buf[LENGTH];
 buf[0] = 0xFF;
 buf[1] = 0xFF;
 buf[2] = 0xFF;
 buf[3] = 0xFF;
 buf[4] = 0xFF;
 buf[5] = 0xFF;
 buf[6] = 0xFF;
 buf[7] = 0xFE;
 CAN.sendMsgBuf(id, 0, LENGTH, buf);

 delay(100);
}

#endif
