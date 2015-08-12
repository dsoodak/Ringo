


//Version 1.0 8/2015
//This example written by Dustin Soodak
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/

//1. Enter new values for degrees to turn. LED is white.
//2. Press button. 500ms later it will calibrate the gyroscope and start
//   printing present heading to serial port. LED is off.
//3. Press button. 500ms later it will calibrate the gyroscope and move the
//   set degrees from present heading. LED is green.
//4. When it is done turning, the LED will turn RED. Connect to debugger and
//   press the button to print debug info to screen. LED will turn white and
//   new turn amount can be entered if desired.

#include "RingoHardware.h"

#define ROTATE_TIMEOUT 2500

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  
  Serial.println();
  //
}
char res;
int degr=360,degrfinal,degrinit;
void loop(){ 
  SwitchButtonToPixels();
  SetPixelRGB(0,5,5,5);RefreshPixels();
  SwitchPixelsToButton();
  SwitchMotorsToSerial();
  Serial.print("Turn ");Serial.print(degr);Serial.println(" degrees (type new number to change)");
  while(!ButtonPressed()){//go in endless loop until button pressed
    if(Serial.available()){
      if(isDigit(Serial.peek()) || Serial.peek()=='-'){
        PlayChirp(NOTE_G6,100);delay(100);PlayChirp(NOTE_G6,0); 
        degr=Serial.parseInt();//note: this will run faster if serial terminal is set to send at least one newline character.  
        Serial.print("changed to ");
        Serial.print(degr);
      }
      Serial.println();  
      delay(1);//in case the OS delays any characters  
      while(Serial.available()) Serial.read(); 
    }
  }    
  Serial.println();
  while(ButtonPressed());//wait till button released
  SwitchButtonToPixels();
  SetPixelRGB(0,0,0,0);RefreshPixels();
  SwitchPixelsToButton();
  delay(500);
  NavigationBegin();
  SwitchPixelsToButton();
  SwitchMotorsToSerial();
  RestartTimer();
  while(!ButtonPressed()){
    SimpleGyroNavigation();
    if(GetTime()>200){
      Serial.println(PresentHeading());
      RestartTimer();
    }
  }
  while(ButtonPressed());
  SwitchButtonToPixels();
  SetPixelRGB(0,0,10,0);RefreshPixels();
  SwitchPixelsToButton();
  delay(500);
  degrinit=PresentHeading();
  res=RotateAccurate(degrinit+degr, ROTATE_TIMEOUT);
  degrfinal=PresentHeading();
  SwitchButtonToPixels();
  SetPixelRGB(0,10,0,0);RefreshPixels();
  SwitchPixelsToButton();
  while(!ButtonPressed());
  while(ButtonPressed());
  SwitchMotorsToSerial();
  Serial.print("Function out(1=successful): ");Serial.println(res,DEC);
  Serial.print("final: ");Serial.println(degrfinal);
  Serial.print("target: ");Serial.println(degrinit+degr,DEC);
  
}
