


//Version 1.0 7/2015
//Dustin Soodak
//This code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/


//Each robot has a number of states represented by LED color.
//When the button is pressed, it transmits its state to other
//robots. Each one that receives it then echoes it after a 
//random interval (and making sure no other signals are detected)
//to minimize overlapping transmissions. One of the bytes is used
//as an error check so garbled signals can be ignored.
//The more robots there are in the herd, the more likely it should
//be that all turn the new color.






#include "RingoHardware.h"

#define BOT_NUM 0;  //not yet used in this sample code
#define DELAY_RANGE 40   //max ms before repeating received message
#define DELAY_RESOLUTION 1 //ms



//**************************************************
//Simple networking functions
//**************************************************
//
//Data format (4 bytes):
//0        1           2        3
//BotNum   MessageID   Data     simple CRC error check
//
//Alogrithm:
//Once a bot transmits a message, each one that receives it will
//wait a random amount of time (and make sure it isn't picking 
//up anything else) before re-transmitting it for bots that haven't
//heard it yet.
//In this version, bot number isn't yet used, so a message is
//uniquely identified by a randomly generated MessageID and the
//Data field.
//This information is stored from the last message so the bot
//doesn't re-echo it until at least 200ms have passed. Otherwise, 
//a message might continue to get passed around indefinitely.
//
//Some simple modifications:
//1. If each bot has a unique BotNum, then can use it instead of
//the Data field, and simply increment MessageID to guaruntee that
//it only repeats every 256 messages.
//2. Another thing you can do with unique IDs is calculate a fixed
//delay for each bot based on its number so there won't ever be 
//overlapping re-transmissions from two that receive a message at
//the same time.

void PrintIRBytes(void){//for debugging
  char i;
  for(i=0;i<IRNumOfBytes;i++){//in case less than 4 were received before routine timed out.
    Serial.print(((unsigned char)IRBytes[i]),HEX);
    Serial.print(" ");
  }
}

unsigned char CalculateCRC(unsigned char *Data){
  char i;
  unsigned char crc=0;
  for(i=0;i<3;i++){
    crc+=Data[i]+1;
  }
  return crc;
}
void AddCRC(unsigned char *Data){
  Data[3]=CalculateCRC(Data);
}
unsigned char CheckCRC(unsigned char *Data){
  return Data[3]==CalculateCRC(Data);
}

unsigned char BotNum=BOT_NUM;
uint32_t LastRxTime=0;
unsigned char LastRxBytes[4]={0,0,0,0};
void IRNetworkStartReceiving(void){
  RxIRRestart(4); 
}
char IRNetworkReceiveAndEcho(void){
  char Completed=0;
  char i;
    if(IsIRDone()){//wait for robot or IR button
      RxIRStop();      
      if(CheckCRC(IRBytes)){//make sure data not corrupted
        if(LastRxBytes[1]!=IRBytes[1]//make sure not immediately repeating it again
           || LastRxBytes[2]!=IRBytes[2]
           || millis()-LastRxTime>200){
          //Store data and Rx time
          LastRxTime=millis();
          PrintIRBytes();
          Serial.println();
          for(i=0;i<4;i++)
            LastRxBytes[i]=IRBytes[i];
          //Wait random time and make sure noone else is sending
          while(1){
            RxIRRestart(4);
            delay((millis()%(DELAY_RANGE/DELAY_RESOLUTION))*DELAY_RESOLUTION);//delay from 0 to 35ms
            if(IRReceiving){
              while(!IsIRDone());
            }
            else{      
              break;
            }
          }
          RxIRStop();
          //re-send received symbol                   
          TxIR(LastRxBytes,4); //would need to use another buffer
                               //such as IRBytes if changing before echoing.
          Completed=1;                   
        }//make sure not a repeat
      }//end if(CheckCRC(IRBytes))    
      if(!Completed)
        IRNetworkStartReceiving();      
    }//end if(IsIRDone())    
    return Completed;
}
char GetIRNetworkData(void){
  return LastRxBytes[2];
}
void IRNetworkSend(char Data){
    IRBytes[0]=BotNum;
    IRBytes[1]=millis()&0xFF;//choose random 8 bit number for unique ID
    IRBytes[2]=Data;
    AddCRC(IRBytes); 
    TxIR(IRBytes,4);
}

//**************************************************
//end Simple networking functions
//**************************************************




char BotMood=0;
char GetRobotMood(void){
  return BotMood; 
}
void SetRobotMood(char newmood){
  BotMood=newmood;
  char r=0,g=0,b=0;
  SwitchButtonToPixels();
  switch(newmood){
  case 0:  
    r=15;g=0;b=0;//red
  break;
  case 1:
    r=0;g=15;b=0;//green
  break;
  case 2:
    r=0;g=0;b=15;//blue
  break;
  case 3:
    r=5;g=5;b=5;//white
  break;
  }
  SetPixelRGB(0,r,g,b);
  SetPixelRGB(1,r,g,b);
  SetPixelRGB(3,r,g,b);
  RefreshPixels();
  SwitchPixelsToButton(); 
}//end SetRobotMood()


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial();
  SetRobotMood(GetRobotMood());//initiate mood display
}

void loop(){
  IRNetworkStartReceiving();
  RestartTimer();
  while(!ButtonPressed()){//exit if IR Rx or button pressed
    if(IRNetworkReceiveAndEcho()){     
      SetRobotMood(GetIRNetworkData());
      break; 
    }
  }//end while(1) wait for IR receive or button pressed
  if(ButtonPressed()){
    while(ButtonPressed());//wait till button release
    //Change mood and send over IR
    SetRobotMood((GetRobotMood()+1)%4);//rotate between the 4 moods
    Serial.print("Bytes: ");
    PrintIRBytes();
    Serial.println();
    Serial.print("new mood");
    Serial.println(GetRobotMood(),DEC);
    IRNetworkSend(GetRobotMood());
  }//end if(ButtonPressed()) 
}//end loop()
