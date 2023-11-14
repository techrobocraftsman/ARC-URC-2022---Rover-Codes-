#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Encoder.h>

Encoder myEnc1(33,34);
Encoder myEnc2(35,36);
Encoder myEnc3(37,38);
Encoder myEnc4(39,40);
//
//Encoder myEnc1(34,33);
//Encoder myEnc2(36,35);
//Encoder myEnc3(38,37);
//Encoder myEnc4(40,39);
int encDir = 2;

int ch[] = {0,14,15,18};
int stPins[] = {0,2,3,4,5,6,7,8,9};
AccelStepper stepper1(AccelStepper::FULL2WIRE,stPins[1],stPins[2]);
AccelStepper stepper2(AccelStepper::FULL2WIRE,stPins[3],stPins[4]);
AccelStepper stepper3(AccelStepper::FULL2WIRE,stPins[5],stPins[6]);
AccelStepper stepper4(AccelStepper::FULL2WIRE,stPins[7],stPins[8]);
MultiStepper steppers;

long enPos[] = {0,0,0,0};
int error[] = {0,0,0,0};
int pos[]={0,0,0,0,0};

volatile long StartTime[] = {0,0,0,0};
volatile long CurrentTime[] = {0,0,0,0};
volatile long Pulses[] = {0,0,0,0};
int PulseWidth[] = {0,0,0,0};
long positions[4];
int lr,swA,swB;
//int minVal=20,maxVal=1050;
int minVal=950,maxVal=2010;
int mid=1500, dead=100;
int bwTh = mid-dead, fwTh = mid+dead;
int chng = 100, chngFine = 10;   //The inc or dec value of position
int tol = 110,tolFine = 15;
int maxSp = 3500, correctionSp = 2000;
int doneA = 0,equalize = false;
float Ka = 10000/90;
float offsetCnst = .85;

void setup() {
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  pinMode(ch[1],INPUT_PULLUP);
  pinMode(ch[2],INPUT_PULLUP);
  pinMode(ch[3],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ch[1]),PulseTimerA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch[2]),PulseTimerB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch[3]),PulseTimerC,CHANGE);
  
  digitalWrite(13,HIGH);
  //pinMode(A0,INPUT);
  // Configure each stepper
  setSp(maxSp);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
}

void loop() {
  if(Pulses[1] < 2100)lr = Pulses[1];
  if(Pulses[2] < 2100)swA = Pulses[2];
  if(Pulses[2] < 2100)swB = Pulses[3];
  printChVal();
  readEnc();
  //Serial.print(lr);
  
            
  if(900<swB && swB<1300){
    Serial.println("A");
    if(900<swA && swA<1300 && doneA != 0){
      Serial.print("Going to 0 deg angle \t");
      changePos(0);
      doneA = 0;
    }
    else if(1300<swA && swA<1700 && doneA != 1){
      //Serial.println("\t Only Front \t");
      pos[0] = 0;
      pos[1] = 0;
      pos[2] = 0;
      pos[3] = 0;
      onlyFrontSn(lr,maxSp-500);
      doneA = 1;
    }
    else if(1700<swA && swA<2100 && doneA != 2){
      Serial.print("Going to 45 deg angle \t");
      threeSD(45);
      doneA = 2;
    }
    //    printPos();
    if(bwTh<lr && lr<fwTh && (doneA==0 || doneA==2)){
      if(doneA==0)correction(correctionSp);
      if(doneA==2)correction(maxSp);
    }
    else{
      if(doneA == 1)onlyFrontSn(lr,maxSp);
      else if(doneA == 0)alignAll(lr,maxSp);
    }
  }
  else if(1300<swB && swB<1700){
    Serial.println("B");
    if(900<swA && swA<1300 && doneA != 0){
      Serial.print("Going to 0 deg angle \t");
      changePos(90);
      doneA = 0;
    }
    else if(1300<swA && swA<1700 && doneA != 1){
      //Serial.println("\t Only Front \t");
      pos[0] = 0;
      pos[1] = 0;
      pos[2] = 0;
      pos[3] = 0;
      onlyBackSn(lr,maxSp-500);
      doneA = 1;
    }
    else if(1700<swA && swA<2100 && doneA != 2){
      Serial.print("Going to 45 deg angle \t");
      threeSD(45);
      doneA = 2;
    }
    //    printPos();
    if(bwTh<lr && lr<fwTh && (doneA==0 || doneA==2)){
      if(doneA==0)correction(correctionSp);
      if(doneA==2)correction(maxSp);
    }
    else{
      if(doneA == 1)onlyBackSn(lr,maxSp);
      else if(doneA == 0)alignAll(lr,maxSp);
    }
    
  }
  else if(1700<swB && swB<2100){
    Serial.println("C");
    
  }
  
  

   // Blocks until all are in position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
    
  //printAngle();
}

void rotateBare(int lr, int sp){
  if(minVal<lr && lr<bwTh){
    positions[0] += chng;
    positions[1] += chng;
    positions[2] += chng;
    positions[3] += chng;
//    Serial.println(positions);
  }
  else if(fwTh<lr && lr<maxVal){
    positions[0] -= chng;
    positions[1] -= chng;
    positions[2] -= chng;
    positions[3] -= chng;
    //Serial.println(positions);
  }
}
void alignAll(int lr, int sp){
  Serial.print(" Align All ");
  sp /= 5;
  if(minVal<lr && lr<bwTh){
    pos[0] += chng;
    pos[1] += chng;
    pos[2] += chng;
    pos[3] += chng;
//    Serial.println(pos);
    sp *= map(lr,minVal,bwTh,5,1);
  }
  else if(fwTh<lr && lr<maxVal){
    pos[0] -= chng;
    pos[1] -= chng;
    pos[2] -= chng;
    pos[3] -= chng;
    //Serial.println(pos);
    sp *= map(lr,fwTh,maxVal,1,5);
  }
  if(lr>1800 && lr<2000)sp = maxSp;
  Serial.println(sp);
  setSp(sp);
  error[0] = pos[0]-enPos[0];
  error[1] = pos[1]-enPos[1];
  error[2] = pos[2]-enPos[2];
  error[3] = pos[3]-enPos[3];

    if(error[0]>tol)positions[0] += chng;
    else if(error[0]<-tol) positions[0] -= chng;
    if(error[1]>tol)positions[1] += chng;
    else if(error[1]<-tol) positions[1] -= chng;
    if(error[2]>tol)positions[2] += chng;
    else if(error[2]<-tol) positions[2] -= chng;
    if(error[3]>tol)positions[3] += chng;
    else if(error[3]<-tol) positions[3] -= chng;
}

void onlyFrontSn(int lr, int sp){
  int midVal = 1500;
  Serial.print(lr);
  Serial.print(" Only front ");
  setSp(sp);
  int POS = map(lr,1170,1830,3500,-3500);
  if(lr>midVal){    //right
    pos[1] = POS*offsetCnst;
    pos[3] = POS/offsetCnst;
  }
  if(lr<midVal){    //left
    pos[1] = POS/offsetCnst;
    pos[3] = POS*offsetCnst;
  }
  Serial.println(POS);
  error[0] = pos[0]-enPos[0];
  error[1] = pos[1]-enPos[1];
  error[2] = pos[2]-enPos[2];
  error[3] = pos[3]-enPos[3];

    if(error[0]>tol)positions[0] += chng;
    else if(error[0]<-tol) positions[0] -= chng;
    if(error[1]>tol)positions[1] += chng;
    else if(error[1]<-tol) positions[1] -= chng;
    if(error[2]>tol)positions[2] += chng;
    else if(error[2]<-tol) positions[2] -= chng;
    if(error[3]>tol)positions[3] += chng;
    else if(error[3]<-tol) positions[3] -= chng;
}
void onlyBackSn(int lr, int sp){
  int midVal = 1500;
  Serial.print(lr);
  Serial.print(" Only Back ");
  setSp(sp);
  int POS = map(lr,1170,1830,3500,-3500);
  if(lr>midVal){    //right
    pos[0] = POS*offsetCnst;
    pos[2] = POS/offsetCnst;
  }
  if(lr<midVal){    //left
    pos[0] = POS/offsetCnst;
    pos[2] = POS*offsetCnst;
  }
  Serial.println(POS);
  error[0] = pos[0]-enPos[0];
  error[1] = pos[1]-enPos[1];
  error[2] = pos[2]-enPos[2];
  error[3] = pos[3]-enPos[3];

    if(error[0]>tol)positions[0] += chng;
    else if(error[0]<-tol) positions[0] -= chng;
    if(error[1]>tol)positions[1] += chng;
    else if(error[1]<-tol) positions[1] -= chng;
    if(error[2]>tol)positions[2] += chng;
    else if(error[2]<-tol) positions[2] -= chng;
    if(error[3]>tol)positions[3] += chng;
    else if(error[3]<-tol) positions[3] -= chng;
}

void correction(int sp){
  setSp(sp);
  //Serial.println(sp);
  error[0] = pos[0]-enPos[0];
  error[1] = pos[1]-enPos[1];
  error[2] = pos[2]-enPos[2];
  error[3] = pos[3]-enPos[3];

    if(error[0]>tolFine)positions[0] += chngFine;
    else if(error[0]<-tolFine) positions[0] -= chngFine;
    if(error[1]>tolFine)positions[1] += chngFine;
    else if(error[1]<-tolFine) positions[1] -= chngFine;
    if(error[2]>tolFine)positions[2] += chngFine;
    else if(error[2]<-tolFine) positions[2] -= chngFine;
    if(error[3]>tolFine)positions[3] += chngFine;
    else if(error[3]<-tolFine) positions[3] -= chngFine;
}

void changePos(float angle){
  setSp(maxSp+500);
  Serial.println(maxSp);
  angle *= Ka;
 // Serial.print(angle);
 // Serial.print(Ka);
  
  pos[0] = angle;
  pos[1] = angle;
  pos[2] = angle;
  pos[3] = angle;
  
  error[0] = pos[0]-enPos[0];
  error[1] = pos[1]-enPos[1];
  error[2] = pos[2]-enPos[2];
  error[3] = pos[3]-enPos[3];

    if(error[0]>tol)positions[0] += chng;
    else if(error[0]<-tol) positions[0] -= chng;
    if(error[1]>tol)positions[1] += chng;
    else if(error[1]<-tol) positions[1] -= chng;
    if(error[2]>tol)positions[2] += chng;
    else if(error[2]<-tol) positions[2] -= chng;
    if(error[3]>tol)positions[3] += chng;
    else if(error[3]<-tol) positions[3] -= chng;
}
void threeSD(float angle){
  setSp(maxSp+500);
  Serial.println(maxSp);
  angle *= Ka;
  //Serial.print(angle);
  //Serial.print(Ka);
  
  pos[0] = angle;
  pos[1] = -angle;
  pos[2] = -angle;
  pos[3] = angle;
  
  error[0] = pos[0]-enPos[0];
  error[1] = pos[1]-enPos[1];
  error[2] = pos[2]-enPos[2];
  error[3] = pos[3]-enPos[3];

    if(error[0]>tol)positions[0] += chng;
    else if(error[0]<-tol) positions[0] -= chng;
    if(error[1]>tol)positions[1] += chng;
    else if(error[1]<-tol) positions[1] -= chng;
    if(error[2]>tol)positions[2] += chng;
    else if(error[2]<-tol) positions[2] -= chng;
    if(error[3]>tol)positions[3] += chng;
    else if(error[3]<-tol) positions[3] -= chng;
}

void printChVal(){
  Serial.print(lr);
  Serial.print("  ");
  Serial.print(swA);
  Serial.print("  ");
  Serial.print(swB);
  Serial.println("  ");
  
}
void printAngle(){
  Serial.print(pos[0]/Ka);
  Serial.print("  ");
  Serial.print(pos[1]/Ka);
  Serial.print("  ");
  Serial.print(pos[2]/Ka);
  Serial.print("  ");
  Serial.print(pos[3]/Ka);
  Serial.println("  ");
  
}

void printAll(){
  Serial.print(enPos[0]);
  Serial.print("  ");
  Serial.print(positions[0]);
  Serial.print(" , ");
  Serial.print(enPos[1]);
  Serial.print("  ");
  Serial.print(positions[1]);
  Serial.print(" , ");
  Serial.print(enPos[2]);
  Serial.print("  ");
  Serial.print(positions[2]);
  Serial.print(" , ");
  Serial.print(enPos[3]);
  Serial.print("  ");
  Serial.print(positions[3]);
}
void printError(){
  Serial.print(error[0]);
  Serial.print(" , ");
  Serial.print(error[1]);
  Serial.print(" , ");
  Serial.print(error[2]);
  Serial.print(" , ");
  Serial.print(error[3]);
}
void printPos(){
  Serial.print(pos[2]);
  Serial.print(" , ");
  Serial.print(enPos[2]);
  Serial.print(" , ");
  Serial.print(error[2]);
  Serial.print(" , ");
  Serial.println(positions[2]);
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
void readEnc(){
  enPos[0] = encDir*myEnc1.read();
  enPos[1] = encDir*myEnc2.read();
  enPos[2] = encDir*myEnc3.read();
  enPos[3] = encDir*myEnc4.read();
}
void setSp(int a){
  stepper1.setMaxSpeed(a);
  stepper2.setMaxSpeed(a);
  stepper3.setMaxSpeed(a);
  stepper4.setMaxSpeed(a);
}
