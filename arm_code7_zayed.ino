#include <AccelStepper.h>
#include <MultiStepper.h>
MultiStepper steppers;
int stPins[] = {0,4,5,6,7,8,9,10,11};
AccelStepper st4(AccelStepper::FULL2WIRE,stPins[1],stPins[2]);
AccelStepper st1(AccelStepper::FULL2WIRE,stPins[3],stPins[4]);
AccelStepper st2(AccelStepper::FULL2WIRE,stPins[5],stPins[6]);
AccelStepper st3(AccelStepper::FULL2WIRE,stPins[7],stPins[8]);

// *>>>>>>>>>>>>   PIN DECLARE <<<<<<<<<<**
//int ch[] = {0,34,36,38,40,42,44,46,48,50,52}; // RADIO CHANNElS
// *>>>>>>>>>>>>   Variables <<<<<<<<<<**
int Max_sp = 6000;
int sbthSp = 0,sbthSafety = 30;
int sbthMaL = 1+sbthSafety,sbthMaM = 64,sbthMaH = 127-sbthSafety;
int sbthMbL = 128+sbthSafety,sbthMbM = 192,sbthMbH = 255-sbthSafety;
int deadZone = 50,minVal=900,maxVal=2020;
int fwd_threshold = 1500+deadZone, bwd_threshold = 1500-deadZone;
int interrupt[] = {0,3,2,18,19,20,21,0,0};
int chVal[] =     {0,0,0,0,0,0,0,0,0};
int globalChng = 100, chngFine = 10;  
long positions[4];
volatile long StartTime[] = {0,0,0,0,0,0,0};
volatile long CurrentTime[] = {0,0,0,0,0,0,0};
volatile long Pulses[] = {0,0,0,0,0,0,0};
int PulseWidth[] = {0,0,0,0,0,0,0};
int mappingDiv = 6;
int RLS = 200, RLst = 80, GS = 700, GSst = 100;

void setup(){
  Serial.begin(9600);
  Serial3.begin(9600);
  
  interruptConfig();
  multStpr();
  setSp(Max_sp);
}
void loop(){
  takeInterruptSignals();
  printChValues();
  operateFunc();
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  
  Serial.println();
}
void operateFunc(){
  runGrip(chVal[1],GS,GSst);
  runJa(3000-chVal[3]);
  runJb(chVal[2]);
  runBase(chVal[4],1000,200);
  rotateGripper(chVal[6],RLS,RLst);
  elevateGripper(chVal[5],RLS,RLst);
  
//  if(900<chVal[5] && chVal[5]<1500)rotateGripper(chVal[6],RLS,RLst);
//  else elevateGripper(chVal[6],RLS,RLst);
}
void runGrip(int chVal,float sp,int localChng){
  //------------- Base Stepper1------------------
  int stNo = 0;
  int dir = -1;
  if(fwd_threshold<chVal && chVal<maxVal){
    positions[stNo] += dir*localChng;
    Serial.print("\tGripping");
    Serial.print(sp);
  }
  else if(minVal<chVal && chVal<bwd_threshold){
    positions[stNo] -= dir*localChng;
    Serial.print("\tReleasing");
    Serial.print(sp);
  }
  st1.setMaxSpeed(sp);
}
void runBase(int chVal,float sp,int Chng){
  //------------- Base Stepper1------------------
  int stNo = 3;
  int localSp = sp, localChng = Chng;
  if(1100<chVal && chVal<1900){
    localSp = sp;
    localChng = Chng-100;
  }
  else {
    localSp = sp+3000;
    localChng = Chng+100;
  }
  if(fwd_threshold<chVal && chVal<maxVal){
    positions[stNo] -= localChng;
    Serial.print("\tBase CLK\t");
    Serial.print(localSp);
  }
  else if(minVal<chVal && chVal<bwd_threshold){
    positions[stNo] += localChng;
    Serial.print("\tBase CCCLK\t");
    Serial.print(localSp);
  }
  st4.setMaxSpeed(localSp);
}
void rotateGripper(int chVal,float sp,int localChng){
//  **  ROTATE THE GRIPPER ***
  int stNo[] = {0,1,2};
  if(fwd_threshold<chVal && chVal<maxVal){
    positions[stNo[1]] += localChng;
    positions[stNo[2]] -= localChng;
    Serial.print("\tGrip CLK");
    Serial.print(sp);
  }
  else if(minVal<chVal && chVal<bwd_threshold){
    positions[stNo[1]] -= localChng;
    positions[stNo[2]] += localChng;
    Serial.print("\tGrip CCCLK");
    Serial.print(sp);
  }
  st2.setMaxSpeed(sp);
  st3.setMaxSpeed(sp);
  
}
void elevateGripper(int chVal, int sp, int localChng){
//  **  ELEVATE THE GRIPPER *** 
  int stNo[] = {0,1,2};
  if(fwd_threshold<chVal && chVal<maxVal){
    positions[stNo[1]] += localChng;
    positions[stNo[2]] += localChng;
    Serial.print("\tGrip elevating");
    Serial.print(sp);
  }
  else if(minVal<chVal && chVal<bwd_threshold){
    positions[stNo[1]] -= localChng;
    positions[stNo[2]] -= localChng;
    Serial.print("\tGrip lowering");
    Serial.print(sp);
  }
  st2.setMaxSpeed(sp);
  st3.setMaxSpeed(sp);
}

void runJa(int ch_val){
  //------------- Joint 1 Actuator------------------
  
  //__** EXPAND *____
  if(ch_val>fwd_threshold && ch_val<2030){
    sbthSp = map(ch_val,fwd_threshold,maxVal,sbthMaM-1,sbthMaL);
    Serial.print("\t Ja Expanding");
    Serial.print(sbthSp);
    Serial3.write(sbthSp);
  }
  //__** SHRINK *____
  else if(ch_val>minVal && ch_val<bwd_threshold){
    sbthSp = map(ch_val,minVal,bwd_threshold,sbthMaH,sbthMaM+1);
    Serial.print("\t Ja Shrinking");
    Serial.print(sbthSp);
    Serial3.write(sbthSp);
  }
  //__** STOP *____
  else{
      Serial3.write(sbthMaM);
  }
}
void runJb(int ch_val){
  //------------- Joint 2 Actuator------------------
  //__** EXPAND *____
  if(ch_val>fwd_threshold+100 && ch_val<maxVal){
    sbthSp = map(ch_val,fwd_threshold,maxVal,sbthMbM-1,sbthMbL);
    Serial.print("\t Jb Expanding");
    Serial.print(sbthSp);
    Serial3.write(sbthSp);
  }
  //__** SHRINK *____
  else if(ch_val>minVal && ch_val<bwd_threshold-100){
    sbthSp = map(ch_val,minVal,bwd_threshold,sbthMbH,sbthMbM+1);
    Serial.print("\t Jb Shrinking");
    Serial.print(sbthSp);
    Serial3.write(sbthSp);
  }
  //__** STOP *____
  else{
      Serial3.write(sbthMbM);
  }
}

void takeInterruptSignals(){
    if(Pulses[1] < 2100)chVal[1] = Pulses[1];
    if(Pulses[2] < 2100)chVal[2] = Pulses[2];
    if(Pulses[3] < 2100)chVal[3] = Pulses[3];
    if(Pulses[4] < 2100)chVal[4] = Pulses[4];
    if(Pulses[5] < 2100)chVal[5] = Pulses[5];
    if(Pulses[6] < 2100)chVal[6] = Pulses[6];
}
void printChValues(){
  Serial.print(chVal[1]);Serial.print(" ");
  Serial.print(chVal[2]);Serial.print(" ");
  Serial.print(3000-chVal[3]);Serial.print(" ");
  Serial.print(chVal[4]);Serial.print(" ");
  Serial.print(chVal[5]);Serial.print(" ");
  Serial.print(chVal[6]);Serial.print(" ");
  //delay(100);
}
void interruptConfig(){
  pinMode(interrupt[1],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt[1]),PulseTimerA,CHANGE);
  pinMode(interrupt[2],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt[2]),PulseTimerB,CHANGE);
  pinMode(interrupt[3],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt[3]),PulseTimerC,CHANGE);
  pinMode(interrupt[4],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt[4]),PulseTimerD,CHANGE);
  pinMode(interrupt[5],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt[5]),PulseTimerE,CHANGE);
  pinMode(interrupt[6],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt[6]),PulseTimerF,CHANGE);
}
void multStpr(){
  steppers.addStepper(st1);
  steppers.addStepper(st2);
  steppers.addStepper(st3);
  steppers.addStepper(st4);
}
void setSp(int sp){
  st1.setMaxSpeed(sp);
  st2.setMaxSpeed(sp);
  st3.setMaxSpeed(sp);
  st4.setMaxSpeed(sp);
}
void PulseTimerA(){
  int i = 1;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerB(){
  int i = 2;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerC(){
  int i = 3;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerD(){
  int i = 4;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerE(){
  int i = 5;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerF(){
  int i = 6;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
