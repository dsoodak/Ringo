
/*

Ringo Robot
Ringo_CalibrateGyroscope_Rev01
Version 1.0 8/2015

The gyroscope on Ringo varies in sensitiviety by about +/- 5% from unit
to unit. For this reason, some navigation functions that use the
gyroscope may be inaccurate by a few degrees. For example, running
the code "RotateAccurate(180, ROTATE_TIMEOUT);" may result in over 
shooting or under shooting the 180 degree mark on your Ringo.

This sketch allows you to calibrate your specific Ringo unit to
account for this inaccuracy.  The calibrated value is stored in
EEPROM in Ringo's processor (so Ringo won't forget the value when
he is turned off). You only need to do this one time for a given
Ringo robot. From that point forward (even if the battery goes dead),
Ringo will automatically read the calibrated value out of his 
memory and use it when navigating.

Instructions for using this sketch:

1) Load this sketch onto your Ringo.

2) Turn on Ringo.  Place him on a smooth surface pointing in any
   specific direction. His rear pixel should be BLUE at this point.
   
3) Press the USER button once and let go. You will hear a quick
   chirp. The rear pixel will then turn GREEN for 1.5 seconds.
   YOU MUST HAVE YOUR HANDS OFF OF RINGO, AND RINGO MUST BE
   COMPLETELY STILL AT THE END OF THIS 1.5 SECONDS. ANY VIBRATION
   ON THE SURFACE WHAT SO EVER WILL EFFECT THE CALIBRATION (including
   fans running on your desk, speakers playing music, etc). You want
   Ringo to be as still as possible at the end of the 1.5 seconds.

4) Ringo's rear light will turn RED and he will begin to spin 
   in 3 consecutive circles. If his gyroscope is perfectly calibrated,
   he will stop heading the exact direction he started at. Chances
   are, he will either over-shoot or under-shoot a bit.
   After he finishes rotating, his rear LED will turn BLUE again.
   Make a mental note of how many degrees he was over or under.

5) Plug Ringo into the programming adaptor and open the serial
   monitor window. Make sure the serial monitor is set to
   "57600 baud" (drop down in the lower right of the serial window).
   Type in the number of degrees Rigno over shot (as a positive
   number) or the number of degrees he under-shot (as a negative
   number).
   
6) Press enter on your computer keyboard. You should hear Ringo beep 
   and see the number you entered echoed back to the screen.
   Once the number is accepted, Ringo does some quick calculations
   in the background and writes the new calibration offset to his
   EEPROM memory.

7) Unplug Ringo from the programming cable and repeat the test.
   The next time he spins he should be more accurate. If Ringo
   is still over or under-shooting, continue the process of plugging
   him back into the programming cable and entering the amount
   of over or under shoot until you are satisfied he is rotating 
   accurately. Note, the rotation function is only accurate 
   to +/- one degree, so he'll never rotate PERFECTLY,
   but he should get pretty close even after 3 full rotations.

8) Press the RESET button (or click the power switch off then
   back on). Without plugging Ringo back into the programmer,
   run the test again and he should still be rotating
   accurately now. If this is true, then you're all set!

   NOTE: The calibration is stored in EEPROM addresses 1020~1024
         which is the very end of the EEPROM address space.
         Be sure to avoid writing to these addresses if you use
         the EEPROM for anything else to avoid over-writing
         the calibration. If the calibration is accidentally
         over-written you can just run this calibration sequence
         again.



This example written by Kevin King, starting from 
Ringo_GyroSenseAndTurn by Dustin Soodak
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/
Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "RingoHardware.h"

#define ROTATE_TIMEOUT 4000
#define ROTATION_DEGREES_FOR_TEST 1080

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  
  Serial.println();
  //
}

char res;
float NewGyroscopeCalibrationMultiplier;
float degr=ROTATION_DEGREES_FOR_TEST;
float overshoot=0;
float runningOvershoot=0;
int degrfinal,degrinit;
float rotationActual=ROTATION_DEGREES_FOR_TEST;

void loop(){ 
  SetPixelRGB(TAIL_TOP,0,0,40);
  SwitchPixelsToButton();
  SwitchMotorsToSerial();
  Serial.print("Present Multiplier: ");Serial.print(GyroscopeCalibrationMultiplier,8);Serial.println("");
  Serial.println("Enter degrees over/undershoot after 3 rotations... ");
  while(!ButtonPressed()){//go in endless loop until button pressed
    if(Serial.available()){
      if(isDigit(Serial.peek()) || Serial.peek()=='-'){
        SetPixelRGB(TAIL_TOP,0,60,0);
        PlayChirp(NOTE_G6,100);delay(100);OffChirp();
        SetPixelRGB(TAIL_TOP,0,0,40); 
        SwitchPixelsToButton();
        overshoot=Serial.parseInt();//note: this will run faster if serial terminal is set to send at least one newline character.
        if(overshoot>180){
          overshoot=180;
        }
        if(overshoot<-180){
          overshoot=-180;
        }
        runningOvershoot+=overshoot;
        Serial.println("");
        Serial.print("Overshoot: ");Serial.print(overshoot,4);Serial.println("");   
        Serial.print("Running Overshoot: ");Serial.print(runningOvershoot,4);Serial.println("");  

        rotationActual = ROTATION_DEGREES_FOR_TEST+runningOvershoot;       
        NewGyroscopeCalibrationMultiplier = degr/rotationActual;

        EEPROM_writeAnything(GYRO_CAL_MULT_ADDRESS, NewGyroscopeCalibrationMultiplier); //write to EEPROM
        GetGyroCalibrationMultiplier();                                                 //go retrieve new value
        Serial.print("New Multiplier from EEPROM: ");Serial.print(GyroscopeCalibrationMultiplier,8);Serial.println("");        
      }
      Serial.println();  
      delay(1);//in case the OS delays any characters  
      while(Serial.available()) Serial.read(); 
    }
  }    
  Serial.println();


  PlayChirp(NOTE_C7,100);delay(100);OffChirp();
  while(ButtonPressed());
  delay(250);
  SetPixelRGB(TAIL_TOP,0,40,0);
  SwitchPixelsToButton();
  delay(1500);
  NavigationBegin();
  degrinit=PresentHeading();
  res=RotateAccurate(degrinit+degr, ROTATE_TIMEOUT);
  degrfinal=PresentHeading();
  SetPixelRGB(TAIL_TOP,20,0,0);
  SwitchPixelsToButton();
  
}


