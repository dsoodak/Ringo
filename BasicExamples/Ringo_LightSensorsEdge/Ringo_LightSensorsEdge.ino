


//Version 1.1 7/2015
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/


#include "RingoHardware.h"


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial();
  RestartTimer();
  //
}

char edge;
signed int i;
void loop(){ 
  
  //delay(10);
  if(GetTime()>200){
    edge=LookForEdge();
    /*for(i=0;i<8;i++){
     Serial.print(LeftEdgeArray[i]);Serial.print("\t");
    }
    Serial.println();*/
    Serial.print(" Edge: ");
    for(i=5;i>=0;i--){if((1<<i)&edge) Serial.print("1"); else Serial.print("0");}//print in binary with leading zeroes
    //Serial.print(edge,BIN);//print in binary
    Serial.print(" Left: ");
    Serial.print(LeftEdgeSensorValue);
    Serial.print("/");
    Serial.print(LeftEdgeSensorAverage);
    Serial.print(" Right: ");
    Serial.print(RightEdgeSensorValue);
    Serial.print("/");
    Serial.print(RightEdgeSensorAverage); 
    Serial.print(" Rear: ");
    Serial.print(RearEdgeSensorValue);
    Serial.print("/");
    Serial.print(RearEdgeSensorAverage); 
    Serial.println();
    RestartTimer();
  }  
}
