


//Version 1.0 7/2015
//Dustin Soodak
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/


//Press power button on IR remote or user button on robot to
//activate example. Press reset button to restart.
//This example can also be activated with another robot that
//has Ringo_IRSend.

#include "RingoHardware.h"


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes

  //This code Loops until either IR remote (power) button or robot 
  //user button pressed. Useful to put before code that will move
  //robot so it doesn't try to get away when you are trying to
  //program it.
  //You may also with to put this code into loop() below if you want
  //to activate activate a behavior repeatedly.
  SwitchPixelsToButton();
  RxIRRestart(4);//since remote sends 4 bytes
  while(1){
    if(IsIRDone()){//wait for robot or IR button
      RxIRStop();
      if(IRBytes[2]==0x45 && IRBytes[3]==0xBA){//power button
        IRBytes[2]=0;IRBytes[3]=0;
        break;
      }
      else{
        RxIRRestart(4);
        SwitchPixelsToButton();
      }
    }//end if(IsIRDone())    
    if(ButtonPressed()){
      delay(1000);//so user isn't still moving robot when navigation starts
      break; 
    }
  }//end while(1) wait for robot or IR button
  // 
}


void loop(){
   
  //Example user code:  
  SwitchSerialToMotors();
  RotateAccurate(PresentHeading()+180, 1000);//rotate 180 degrees to right, timeout 1000ms
  while(1);//stop here (goes in endless loop since ARduino is based on c/c++ which interprets "1" as "true")
}
