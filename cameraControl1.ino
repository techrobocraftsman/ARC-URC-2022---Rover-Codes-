#include <Servo.h>
Servo Srvo1;
Servo Srvo2;
int inA[] = {0,2,7,9};
int inB[] = {0,3,4,9};
int srvo1 = 8,srvo2 = 6;

int ch[] = {0,10,11,12,13};
int chVal[] = {0,0,0,0,0,0,0,0,0};
int maxVal = 2010, minVal = 980, tol = 100, bwTh = 1500-tol,fwTh = 1500+tol;
int spA = 180,spB = 200;
int anglA=110, anglB=90;

void setup() {
  // put your setup code here, to run once:
  for(int i=1; i<=3; i++){
    pinMode(inA[i],OUTPUT);
  }
  for(int i=1; i<=4; i++){
    pinMode(ch[i],INPUT);
  }
  Serial.begin(9600);
  Srvo1.attach(srvo1);
  Srvo2.attach(srvo2);
  
  //Srvo1.write(anglA);
}

void loop() {
  chVal[1] = 3000 - pulseIn(ch[1], HIGH);
  chVal[2] = pulseIn(ch[2], HIGH);
  chVal[3] = pulseIn(ch[3], HIGH);
  //chVal[4] = pulseIn(ch[4], HIGH);

  digitalWrite(inA[3], spA);
//  analogWrite(inB[3], spA);
  
  AcameraX(chVal[1]);
  AcameraY(chVal[2]);
  BcameraX(chVal[3]);
  //BcameraY(chVal[3]);
//  printChVal();
  Serial.println();
}
void AcameraX(int ch){
  if(minVal<ch && ch<bwTh){
    digitalWrite(inA[1],1);
    digitalWrite(inA[2],0);
    Serial.print(" mA clk ");
  }
  else if(fwTh<ch && ch<maxVal){
    digitalWrite(inA[1],0);
    digitalWrite(inA[2],1);
    Serial.print(" mA cclk ");
  }
  else{
    digitalWrite(inA[1],0);
    digitalWrite(inA[2],0);
//    Serial.print(" off ");
  }
}
void AcameraY(int ch){
  int del = 5;
  if(minVal<ch && ch<bwTh){
    if(anglA>10)anglA--;
    delay(del);
  }
  else if(fwTh<ch && ch<maxVal){
    if(anglA<170)anglA++;
    delay(del);
  }
  Srvo1.write(anglA);
  Serial.print(anglA);
}
void BcameraX(int ch){
  if(minVal<ch && ch<bwTh){
    analogWrite(inB[1],100);
    digitalWrite(inB[2],0);
    Serial.print(" mB clk ");
  }
  else if(fwTh<ch && ch<maxVal){
    digitalWrite(inB[1],0);
    digitalWrite(inB[2],1);
    Serial.print(" mB cclk ");
  }
  else{
    digitalWrite(inB[1],0);
    digitalWrite(inB[2],0);
//    Serial.print(" off ");
  }
}
void BcameraY(int ch){
  int del = 5;
  if(minVal<ch && ch<bwTh){
    if(anglB>85)anglB--;
    delay(del);
  }
  else if(fwTh<ch && ch<maxVal){
    if(anglB<170)anglB++;
    delay(del);
  }
  Srvo2.write(anglB);
  Serial.print(anglB);
}
void printChVal(){
  Serial.print(chVal[1]);
  Serial.print(" , ");
  Serial.print(chVal[2]);
  Serial.print(" , ");
  Serial.print(chVal[3]);
  Serial.print(" , ");
  Serial.print(chVal[4]);
  Serial.println(" , ");  
}
