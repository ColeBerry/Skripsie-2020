//Author: Cole Berry
//Institution: Stellenbosch University
//Descrition: This program was written to facilitate communication 
//            between a laptop using MATLAB and an Arduino UNO microcontroller.

#include <EEPROM.h> //package to access EEPROM
//Define Vaiables
//Motors
int Pin2 = 2; //Azimuth Step
int Pin3 = 3; //Altitude Step
int Pin4 = 4; //Declination Step
int Pin5 = 5; //Azimuth Direction
int Pin6 = 6; //Altitude Direction
int Pin7 = 7; //Declination Direction
int Pin12 = 12; // Right Ascension Step
int Pin13=13; //Right Ascension Direction
//Switches
int Az_switch = 9; 
int Alt_switch = 10; 
int RA_switch = 11; 
int Dec_switch = A1;

int Enable = 8; //Enable CNC Shield
int del = 5; //delay between steps (microseconds)
int matlabData;

//EEPROM Variables
int vAz; //stored Az position
int vAlt;//stored Alt position
int vRA;//stored RA position
int vDec;//stored Dec position
int Azdir; //Az unwind direction
int Altdir; //Alt unwind direction
int RAdir; //RA unwind direction
int Decdir; //Dec unwind direction

void setup() 
{
//Setup that executes the calibration
//routine before the serial port opens.

//Pin Modes
pinMode(Pin2,OUTPUT);
pinMode(Pin3,OUTPUT);
pinMode(Pin4,OUTPUT);
pinMode(Pin5,OUTPUT);
pinMode(Pin6,OUTPUT);
pinMode(Pin7,OUTPUT);
pinMode(Pin12,OUTPUT);
pinMode(Pin13,OUTPUT);
pinMode(Dec_switch, INPUT_PULLUP);
pinMode(Az_switch, INPUT_PULLUP);
pinMode(Alt_switch, INPUT_PULLUP);
pinMode(RA_switch, INPUT_PULLUP);
digitalWrite(Enable, HIGH);
delay(5);
//Read EEPROM for previous direction values
vAz = EEPROM.read(0);
vAlt = EEPROM.read(1);
vRA = EEPROM.read(2);
vDec = EEPROM.read(3);
//Define calibration step direction for each mount
if(vAz==1)
Azdir=1;
else if(vAz==2)
Azdir=0;
if(vAlt==1)
Altdir=1;
else if(vAlt==2)
Altdir=0;
if(vRA==1)
RAdir=1;
else if(vRA==2)
RAdir=0;
if(vDec==1)
Decdir=1;
else if(vDec==2)
Decdir=0; 

//Azimuth Calibration
while (digitalRead(Az_switch)) 
{ // Do this until the switch is not activated
  digitalWrite(Pin5, Azdir); 
  digitalWrite(Pin2, HIGH);
  delay(del);                    
  digitalWrite(Pin2, LOW);
  delay(del);
}
//Altitude Calibration
 while(digitalRead(Alt_switch))
 {// Do this until the switch is not activated 
  digitalWrite(Pin6, Altdir);    
  digitalWrite(Pin3, HIGH);
  delay(del);                  
  digitalWrite(Pin3, LOW);
  delay(del);
}
//RA Calibration
 while(digitalRead(RA_switch)) 
 {// Do this until the switch is not activated
  digitalWrite(Pin13, RAdir);     
  digitalWrite(Pin12, HIGH);
  delay(del);                      
  digitalWrite(Pin12, LOW);
  delay(del);
}
//DEC Calibration
 while(digitalRead(Dec_switch)) 
 {// Do this until the switch is not activated
  digitalWrite(Pin7,Decdir);     
  digitalWrite(Pin4, HIGH);
  delay(del);                       
  digitalWrite(Pin4, LOW);
  delay(del);
}
// Start Serial communication once the calibration is done
Serial.begin(9600); 
}
 
void loop() 
{
   
 if(Serial.available()>0) // if the serial port is opened
 {
  matlabData=Serial.read(); // read data into a variable
  //stepper motor character instructions
  //designated Az Step Characters
  if (matlabData==3) 
      digitalWrite(Pin2,HIGH); 
  else if(matlabData==4)
    digitalWrite(Pin2,LOW); 
 //designated Alt Step Characters  
  if(matlabData==5)
    digitalWrite(Pin3,HIGH); 
  else if(matlabData==6)
    digitalWrite(Pin3,LOW);
 //designated Dec Step Characters   
    else if (matlabData==7)
      digitalWrite(Pin4,HIGH); 
  else if(matlabData==8)
    digitalWrite(Pin4,LOW);
  //designated Az Direction Characters    
  else  if(matlabData==9)
    digitalWrite(Pin5,HIGH); 
  else if(matlabData==10)
    digitalWrite(Pin5,LOW); 
  //designated Alt Direction Characters    
  else if (matlabData==11)
      digitalWrite(Pin6,HIGH); 
  else if(matlabData==12)
    digitalWrite(Pin6,LOW); 
  //designated Dec Direction Characters   
  if(matlabData==13)
    digitalWrite(Pin7,HIGH); 
  else if(matlabData==14)
    digitalWrite(Pin7,LOW); 
 //designated RA step Characters    
  if(matlabData==23)
    digitalWrite(Pin12,HIGH); 
  else if(matlabData==24)
    digitalWrite(Pin12,LOW); 
 //deignated RA direction characters   
  if(matlabData==25)
    digitalWrite(Pin13,HIGH); 
  else if(matlabData==26)
    digitalWrite(Pin13,LOW); 
 //Read last known motor position  
    else if(matlabData==27){
    vAz=1; 
    EEPROM.write(0, vAz);}
    else if(matlabData==28){
    vAz=2; 
    EEPROM.write(0, vAz);}
    else if(matlabData==29){
    vAlt=1; 
    EEPROM.write(1, vAlt);}
    else if(matlabData==30){
    vAlt=2; 
    EEPROM.write(1, vAlt);}   
     else if(matlabData==31){
    vRA=1; 
    EEPROM.write(2, vRA);}
    else if(matlabData==32){
    vRA=2;
    EEPROM.write(2, vRA);}
     else if(matlabData==33){
    vDec=1; 
    EEPROM.write(3, vDec);}
    else if(matlabData==34){
    vDec=2; 
    EEPROM.write(3, vDec);}
}

}
