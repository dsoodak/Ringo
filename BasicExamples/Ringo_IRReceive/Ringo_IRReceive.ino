


//Version 1.0 7/2015
//Dustin Soodak
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/

//Open the serial monitor to see (in hex values) what is sent
//when you press a button on the remote.

#include "RingoHardware.h"


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial();
  SwitchButtonToPixels();
}

int i;
void loop(){
  RxIRRestart(4);//since remote sends 4 bytes
  while(1){
    if(IsIRDone()){//wait for robot or IR button
      RxIRStop();
      SetPixelRGB(3,0,100,0);//top front LED green
      RefreshPixels();
      Serial.print("Bytes: ");
      for(i=0;i<IRNumOfBytes;i++){//in case less than 4 were received before routine timed out.
        Serial.print(((unsigned char)IRBytes[i]),HEX);
        Serial.print(" ");
      }
      Serial.println();
      delay(10);//so LED has time to come fully on 
      SetPixelRGB(3,0,0,0);//top LED off
      RefreshPixels();      
      break;
    }//end if(IsIRDone())    
  }//end while(1) wait for robot or IR button
  // 
}
