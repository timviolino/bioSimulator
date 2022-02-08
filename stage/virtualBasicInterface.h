#ifndef _motorPackets_h_
#define _motorPackets_h_

void readUserInput() 
{
  while (Serial.available() > 0) 
  {
    String message = Serial.readString();
    unpackMessage(message);
  }
}

void unpackMessage(String message, float & params[4], bool & powered[3]) 
{
  int stringLength = message.length();
  message = message.substring(4, stringLength - 4);
  stringLength -= 8;
  String currentValue;
  int j = 0;
  for (int i = 0; i < stringLength; i++) 
  {
    if (message[i] == ',') 
    {
      if(j < 4) 
      {
        params[j] = currentValue.toFloat();
      }
      else
      {
        if (int(CurrentValue.toFloat()) == 1) 
        {
          powered[j-4] = true;
        }
        else 
        {
          powered[j-4] = false;
        }
      }
      j++;
      currentValue = "";
    }
    else 
    {
      currentValue += message[i];
    }
  }
}

void debugMessage() 
{
#ifdef DEBUG
  String paramOut = String(params[0]) + "," + String(params[1]) + "," + String(params[2]) + "," + String(params[3]) + ",";
  for (int i = 0; i < 3; i++) 
  {
    paramOut += String(powered[i]);
    paramOut += ",";
  }
  Serial.println(paramOut);
#endif
}

#endif
