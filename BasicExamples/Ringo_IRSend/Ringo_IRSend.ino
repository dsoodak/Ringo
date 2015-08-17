


//Version 1.0 7/2015
//This example written by Dustin Soodak
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/


#include "RingoHardware.h"


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes

  //
}

unsigned char data[]={0x00,0xFF,0x45,0xBA};
void loop(){ 
  SwitchMotorsToSerial();
  SwitchPixelsToButton();
  while(1){       
    if(ButtonPressed()){//wait for button to be pressed      
      break; 
    }
  }//end while(1) wait for robot button
  Serial.print("go ");
  TxIR(data,4);
  while(ButtonPressed());//wait for button to be released
}
