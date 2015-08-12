


//Version 1.0 7/2015
//This example written by Dustin Soodak
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/

//It will give an idea of how long an analog read takes and
//how long it takes a sensor to react to a change in light level
//from turning an LED on or off
//This example looks at the left side/bottom sensor, but can be easily
//modified to read any of the 6 light sensors.
//Used to determine what delays go into LookAtEdge(), ReadSideSensors(),
//and ReadEdgeLightSensors()(currently "delayMicroseconds(200)").
//

#include "RingoHardware.h"


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial();
  SwitchPixelsToButton();
  RestartTimer();
  //
}


unsigned long TimerValue[10];
unsigned long  MicInit,Mic1,Mic2;
#define SAMPLE_SIZE  30
int SensorOnValue[SAMPLE_SIZE];
int SensorOffValue[SAMPLE_SIZE];
char i;//this is an 8 bit processor so it will go slightly faster if using an 8 bit number
void loop(){ 
  if(ButtonPressed()){
    Serial.println();
    Serial.println("First, see how long one analog reading takes in us (micro-seconds or millionths of a second):");
    MicInit=micros();
    for(i=0;i<10;i++)
      TimerValue[i]=micros();
    Mic1=micros()-MicInit;
    MicInit=micros();
    for(i=0;i<10;i++){      
      ReadLeftLightSensor();//read any light sensor here...all should take the same time
      TimerValue[i]=micros();
    }
    Mic2=micros()-MicInit;
    Serial.print("Averaging 10 readings, we get ");
    Serial.print((Mic2-Mic1)/10,DEC);
    Serial.println(" us each");
    Serial.println("Individual analog reads: ");//note: extra random delay caused by millisecond timer interrupt
    Serial.print(TimerValue[0]-MicInit,DEC);
    Serial.print(" ");
    for(i=1;i<10;i++){
      Serial.print(TimerValue[i]-TimerValue[i-1],DEC);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println();
    Serial.println("Now, turn the IR LED on and increase the interval before measuring:");
    SwitchAmbientToEdge();//looking at edge/bottom rather than side/ambient sensors
    EdgeLightsOff();//"digitalWrite(IR_Send,0)" if reading a side/ambient sensor
    delayMicroseconds(1000);
    for(i=0;i<SAMPLE_SIZE;i++){      
      EdgeLightsOn();//"digitalWrite(IR_Send,1)" if reading a side sensor
      delayMicroseconds(i*10+10);
      SensorOnValue[i]=ReadLeftLightSensor();//"ReadRightLightSensor()" to look at either 
                                             //the right edge/bottom or embient/side sensor.
      delayMicroseconds(1000);
      EdgeLightsOff();
      delayMicroseconds(i*10+10);
      SensorOffValue[i]=ReadLeftLightSensor();
      delayMicroseconds(1000);
    }
    Serial.println("us\ton\toff\tdifference");
    for(i=0;i<SAMPLE_SIZE;i++){
      Serial.print(i*10+10);
      Serial.print("\t");
      Serial.print(SensorOnValue[i],DEC);
      Serial.print("\t");
      Serial.print(SensorOffValue[i],DEC);
      Serial.print("\t");
      Serial.println(SensorOnValue[i]-SensorOffValue[i],DEC);
    }
    while(ButtonPressed());
  }//end if(ButtonPressed())
}
