


//Version 1.0 8/2015
//This example written by Dustin Soodak
//LookAtEdge() updates global variables for current, previous, and average values of edge sensors.
//LookForEdge() uses this to decide if you have gone over an edge or not.
//
//Code is licensed under:
//Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
//https://creativecommons.org/licenses/by-sa/2.0/
//
//Changes to RingoHardware.h:
//typedef struct RecordedDataStruct{int left; int rear; int right;uint32_t pos;};//can edit number, names, and data types
//#define RECORDED_DATA_ARRAY_LENGTH 40 //set to 1 if not using
//#define RecordedDataPrintRow() do{Serial.print(RecordedDataRow.left,DEC);Serial.print("\t");Serial.print(RecordedDataRow.rear,DEC);Serial.print("\t");Serial.print(RecordedDataRow.right,DEC);Serial.print("\t");Serial.print(RecordedDataRow.pos);}while(0)

//Instructions: 
//1. Check periodic printout to set absolute limits DARK_MIN, and LIGHT_MAX, in RingoHardware.h. Also see if printed
//   averages are large enough when nothing for light to bounce off of in order to justify non-zero values for
//   REAR_ZERO, RIGHT_ZERO, and LEFT_ZERO  (REAR_ZERO will probably be around 30-40).
//   LEDs will turn red to indicate physical edge or dark tape, and green to indicate white tape. Keep in mind that
//   it will adjust to new levels so after backing away from an edge, it will turn the opposite color briefly.
//   There might also be significant variation depending on how often LookAtEdge() (called by LookForEdge()) runs, 
//   unless TIME_ADJUSTED_AVERAGE is chosen instead of RUNNING_AVERAGE.
//2. Press button to get into mode where motor directions and delay between measurements can be set
//   by typing numbers into the serial port (LED is white).
//3. Press button again to turn LED blue and let it go either a flat unmarked surface, towards an edge, or black or 
//   white tape.  Edit IsOverEdge_Test() below for each of these cases according to comments in the function.
//4. Press button again to print out data for this test to the serial port and copy/paste it into openoffice 
//   spreadsheet EdgeDetectionData.ods. Look at left mult, rear mult, and right mult to see what values should
//   be put into DARK_EDGE_MULT_1, etc. Also see if there are enough false triggers on the least reflective
//   flat surface you can find in order to make it necessary to un-comment CHECK_EDGE_TWICE.
//5. Press button to repeat.

#include "RingoHardware.h"

char edge;
int LeftInit,RightInit,RearInit;
int len;

char IsOverEdge_Test(void){//this is used in the high speed data collection mode but not when periodically printing to screen.
  //
  //Level surface test:  Prevents it from false triggering on a flat surface with no tape.
  //return 0;
  //
  //Dark tape or physical edge test:  This version will generally work and NOT trigger early in nearly all cases where you are looking for an edge or black tape:
  return (LeftEdgeSensorValue<LeftInit/4?LEFT_DARK:0) | (RearEdgeSensorValue<RearInit/4?REAR_DARK:0) | (RightEdgeSensorValue<RightInit/4?RIGHT_DARK:0);
  //
  //White tape test:
  //return (LeftEdgeSensorValue>LeftInit*4?LEFT_BRIGHT:0) | (RearEdgeSensorValue>RearInit*4?REAR_BRIGHT:0) | (RightEdgeSensorValue*RightInit*4?RIGHT_BRIGHT:0);
  //
  //If just want to use IsOverEdge() like LookForEdge() does:
  //return IsOverEdge();
  //
}

//#define SLOW_MOTION_MODE_WHEN_PRINTING   //un-comment if you want to see every reading going into LookForEdge() printed on screen (when in periodic printing mode)






signed int i;

uint32_t us,us2;

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial();
  SwitchPixelsToButton();

  i=BrightEdgeMult1(100);
  Serial.println(i);
  i=MultiplyBy7over8(100);
  Serial.println(i);
  
  //
  i=0;
  us=micros();
  while(micros()-us<100000){
    ReadLeftLightSensor();i++;
  }
  Serial.print("average per reading: ");
  Serial.print((float)100000/i);
  Serial.println(" us");
  //
  //Serial.print("recorded data total time: ");
  RecordedDataReset(0);
  i=0;
  us=micros();  
  while(micros()-us<100000){    
    RecordedDataRefresh();i++;
  }
  //Serial.println(i);
  //Serial.println("data");RecordedDataPrint();
  //Serial.print(RecordedDataTime());
  //Serial.println(" us");
  Serial.print("average time spent per data record: ");
  Serial.print((float)100000/i);
  Serial.println(" us");
  //
  ResetLookAtEdge();
  us=micros();
  for(i=0;i<500;i++){
      LookAtEdge();
  }  
  us=micros()-us;
  Serial.print("average per set of readings: ");
  Serial.print(us/i);
  Serial.println(" us");
  //
}


char inputnum=0;
int Values[]={200,200,1000};
int val;
const char *Names[]={"left motor","right motor","microsecond delay between measuring edge sensors"};
void PrintValue(char Num){
 Serial.print(Names[Num]);Serial.print("(");Serial.print(Values[Num]);Serial.print(")");
}

void loop(){
  RestartTimer();
  while(!ButtonPressed()){
    #ifndef SLOW_MOTION_MODE_WHEN_PRINTING
    edge=LookForEdge();//move to inside "if(GetTime()>200 || edge)" if you only want it to update values when printing (slow motion update)
    #endif
    delayMicroseconds(Values[2]);
    if(GetTime()>200 || edge){
      #ifdef SLOW_MOTION_MODE_WHEN_PRINTING
      edge=LookForEdge();//move out of this if() statement if you want it to update at a more realistic speed
      #endif
      Serial.print(" Edge: ");
      for(i=5;i>=0;i--){if((1<<i)&edge) Serial.print("1"); else Serial.print("0");}//print in binary with leading zeroes
      //Serial.print(edge,BIN);//print in binary
      Serial.print(" Left: ");
      Serial.print(LeftEdgeSensorValue);
      Serial.print("/");
      Serial.print(LeftEdgeSensorAverage);
      Serial.print(" Rear: ");
      Serial.print(RearEdgeSensorValue);
      Serial.print("/");
      Serial.print(RearEdgeSensorAverage); 
      Serial.print(" Right: ");
      Serial.print(RightEdgeSensorValue);
      Serial.print("/");
      Serial.print(RightEdgeSensorAverage);
      Serial.println();
      //Serial.print(" LeftPrev ");Serial.print(LeftEdgeSensorValuePrev());
      //Serial.print(" RearPrev ");Serial.print(RearEdgeSensorValuePrev());
      //Serial.print(" RightPrev ");Serial.print(RightEdgeSensorValuePrev());
      //Serial.println();
      if(LeftDarkDetected(edge))
        SetPixelRGB(EYE_LEFT,20,0,0);
      if(LeftBrightDetected(edge))
        SetPixelRGB(EYE_LEFT,0,20,0);
      if(LeftDetected(edge)==0)
        SetPixelRGB(EYE_LEFT,0,0,0);      
      if(RearDarkDetected(edge))
        SetPixelRGB(TAIL_TOP,20,0,0);
      if(RearBrightDetected(edge))
        SetPixelRGB(TAIL_TOP,0,20,0);
      if(BackEdgeDetected(edge)==0)
        SetPixelRGB(TAIL_TOP,0,0,0);
      if(RightDarkDetected(edge))
        SetPixelRGB(EYE_RIGHT,20,0,0);
      if(RightBrightDetected(edge))
        SetPixelRGB(EYE_RIGHT,0,20,0);
      if(RightDetected(edge)==0)
        SetPixelRGB(EYE_RIGHT,0,0,0);      
      RestartTimer();
      #ifdef SLOW_MOTION_MODE_WHEN_PRINTING
      edge=0;//in case SLOW_MOTION_MODE_WHEN_PRINTING is defined
      #endif
    }//end if(GetTime()>200)
  }//end while(!ButtonPressed())
  SetAllPixelsRGB(0,0,0);
  while(ButtonPressed());

  
  SetPixelRGB(3,5,5,5);////white on 
  SwitchPixelsToButton();
  PrintValue(inputnum);
  //SwitchMotorsToSerial();
  delay(1);//in case the OS delays any characters  
  while(Serial.available()) Serial.read();//since motors may have produced a spurious character
  while(!ButtonPressed()){//go in endless loop until button pressed
    if(Serial.available()){
      if(isDigit(Serial.peek()) || Serial.peek()=='-'){
        PlayChirp(NOTE_G6,100);delay(100);PlayChirp(NOTE_G6,0); 
        val=Serial.parseInt();
        Values[inputnum]=val;
        Serial.print(" changed to ");
        Serial.print(val);
      }
      Serial.println();  
      delay(1);//in case the OS delays any characters  
      while(Serial.available()) Serial.read(); 
      inputnum=(inputnum+1)%3;
      PrintValue(inputnum);
    }
  }    
  Serial.println();
  while(ButtonPressed());//wait till button released
  SwitchButtonToPixels();
  SetPixelRGB(3,0,0,20);//RefreshPixels();//blue on 
  delay(500);//time for user to remove hand from robot
  SwitchSerialToMotors();   
  Motors(Values[0],Values[1]);
  SetPixelRGB(3,0,20,0);//RefreshPixels();//green on
  RestartTimer();
  ResetLookAtEdge();
  LeftInit=LeftEdgeSensorValue;
  RightInit=RightEdgeSensorValue;
  RearInit=RearEdgeSensorValue;
  i=0;
  RestartTimer();
  RecordedDataReset(0);
  us=micros();
  len=0;
  //SwitchMotorsToSerial();Serial.println(LeftEdgeSensorValue);Serial.println();
  edge=0;
  while(!edge){
      delayMicroseconds(Values[2]);
      LookAtEdge();i++;
      edge=IsOverEdge_Test();
      RecordedDataRow.pos=RecordedDataTime();//RecordedDataPosition;
      RecordedDataRow.left=LeftEdgeSensorValue;//(float)LeftEdgeSensorValue/LeftEdgeSensorAverage;
      RecordedDataRow.right=RightEdgeSensorValue;//(float)RightEdgeSensorValue/RightEdgeSensorAverage;
      RecordedDataRow.rear=RearEdgeSensorValue;//RearEdgeSensorValue;//(float)RearEdgeSensorValue/RearEdgeSensorAverage;
      RecordedDataRefresh();
      //timeout if more than half a second (500ms)
      if(GetTime()>500){
        break;        
      }      
  }
  
  us2=RecordedDataTime();
  us=micros()-us;
  Motors(0,0);
  SwitchMotorsToSerial();  
  SetPixelRGB(3,0,0,0);//off
  Serial.print("edge ");Serial.println(edge,BIN);
  if(RightDetected(edge))
    SetPixelRGB(EYE_RIGHT,20,0,0);
  if(LeftDetected(edge))
    SetPixelRGB(EYE_LEFT,20,0,0);
  if(BackEdgeDetected(edge))
    SetPixelRGB(TAIL_TOP,20,0,0);
    
   
  while(!ButtonPressed());while(ButtonPressed());//wait for button

  SetAllPixelsRGB(0,0,0);
  
  /*Serial.println("raw");
  for(i=0;i<RECORDED_DATA_ARRAY_LENGTH;i++){
    RecordedDataRow=RecordedDataArray[i];RecordedDataPrintRow();Serial.println();
  }*/
  Serial.println();
  Serial.print("Total: ");
  Serial.print(us/1000);
  Serial.println(" ms");
  Serial.print("RecordedDataTime() ");Serial.println(us2);
  Serial.print("length ");Serial.println(RecordedDataLength);
  Serial.print("position ");Serial.println(RecordedDataPosition);
  Serial.println("left   \trear   \tright   \tmicroseconds");  
  RecordedDataPrint();
  Serial.print("LeftInit ");Serial.print(LeftInit);Serial.print(" RearInit ");Serial.print(RearInit);Serial.print(" RightInit ");Serial.print(RightInit);
  Serial.println();
  while(!ButtonPressed());while(ButtonPressed());//wait for button
  
}//end loop()
