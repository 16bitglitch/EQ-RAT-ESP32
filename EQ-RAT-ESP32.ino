/*  
# EQRAT-UNO
## Simple Equatorial Mount Ra Sidereal Tracker for ESP32 Based Boards

By Charles Gershom 
@charlesgershom 

See Readme.md

*/
 
#include <WiFi.h>
#include "BluetoothSerial.h"

//Debug Stuff
const bool DEBUG=true;
String appname="EQ-RAT32";

//Configuration
const int Mount_Worm_Gear_Ratio=130;
const int Motor_Gear_Ratio=3;
const int Steps_Per_Rev=400;
const int Microstep_Setting=128;


//Stuff for timer calc (doing everything as floats until its time to convert to timer)

const float Seconds_Earth_Rotate=86164.09053;
float Earth_Seconds_Per_Degree =Seconds_Earth_Rotate / 360.0;
float MicroSteps_Per_Degree =((float)Mount_Worm_Gear_Ratio * (float)Motor_Gear_Ratio *  (float)Steps_Per_Rev * (float)Microstep_Setting) / 360.0;
float Step_Delay_Microseconds =(Earth_Seconds_Per_Degree / MicroSteps_Per_Degree) * 1000000.0;
float Step_Delay_Timer_Half_Phase=Step_Delay_Microseconds / 2.0;

volatile long lastInterruptTime=0;
volatile long currentInterruptTime=0;

long lastLoopTime=0;
long currentLoopTime=0;

//Ra Stepper Config
const int RAdirPin = 16;   
const int RAstepPin = 26;   


//Dec Stepper Config
const int DECdirPin = 27;   
const int DECstepPin = 25;    

//Ra Stepper State
uint8_t raStepState=LOW;

volatile uint8_t istracking=HIGH;

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Current position in Meade lx200 format, see updateLx200Coords()
String lx200RA  = "00:00:00#";
String lx200DEC = "+90*00:00#";

// Serial Input
char input[20];     // stores serial input
int  in = 0;        // current char in serial input
// Serial Input (New) coords
long inRA    = 0;
long inDEC   = 0;

const long NORTH_DEC   = 324000; // 90°

// Current coords in Secs (default to true north)
long currRA  = 0;     
long currDEC = NORTH_DEC;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void timerCount(){
  lastInterruptTime=currentInterruptTime;
  currentInterruptTime=micros();
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  //get set stuff
  
 
 //Switch the step pin state
  if(raStepState==LOW){
    raStepState=HIGH;
  }
  else{
    raStepState=LOW;
  }
   //Write to the step pin
   if(istracking==HIGH)
     digitalWrite(RAstepPin, raStepState); 

  
   portEXIT_CRITICAL_ISR(&timerMux);
}



// Generated with http://www.arduinoslovakia.eu/application/timer-calculator
void setupTimer1() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, Step_Delay_Timer_Half_Phase, true);
  timerAlarmEnable(timer);
}

void flashLed(){
  digitalWrite(14, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(14, LOW);    // turn the LED off by making the voltage LOW
  delay(100);       
}

void twitchMotors(){
   uint8_t state=LOW;
   digitalWrite(RAdirPin, HIGH);   // invert this (HIGH) if wrong direction    
  for(int s=0;s<5000;s++){
       digitalWrite(RAstepPin, HIGH); 
       delayMicroseconds(10);
       digitalWrite(RAstepPin, LOW); 
       delayMicroseconds(10);
        
   
  } 
  
   digitalWrite(RAdirPin, LOW);   // invert this (HIGH) if wrong direction    
   for(int s=0;s<5000;s++){
       digitalWrite(RAstepPin, HIGH); 
       delayMicroseconds(10);
       digitalWrite(RAstepPin, LOW); 
       delayMicroseconds(10);
        
   
  } 
      digitalWrite(DECdirPin, HIGH);   // invert this (HIGH) if wrong direction    
  for(int s=0;s<5000;s++){
       digitalWrite(DECstepPin, HIGH); 
       delayMicroseconds(10);
       digitalWrite(DECstepPin, LOW); 
       delayMicroseconds(10);
        
   
  } 
  
   digitalWrite(DECdirPin, LOW);   // invert this (HIGH) if wrong direction    
   for(int s=0;s<5000;s++){
       digitalWrite(DECstepPin, HIGH); 
       delayMicroseconds(10);
       digitalWrite(DECstepPin, LOW); 
       delayMicroseconds(10);
        
   
  } 
     
  
  
}


void setup() {  
  
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  Serial.println("Starting EQRAT");
  Serial.println("--------------");
  Serial.println("Timer Calc : " + String(Step_Delay_Microseconds));
  pinMode(RAstepPin, OUTPUT);   
  pinMode(RAdirPin, OUTPUT);    
  digitalWrite(RAdirPin, LOW);   // invert this (HIGH) if wrong direction    
  pinMode(DECstepPin, OUTPUT);   
  pinMode(DECdirPin, OUTPUT);    
  digitalWrite(DECdirPin, LOW);   // invert this (HIGH) if wrong direction  
  
  SerialBT.begin("EQ-RAT"); //Bluetooth device name
  Serial.println("Bluetooth started, now you can pair!");

  twitchMotors();
  
  //Setup and start Timer
  setupTimer1();

  // lx200DEC[3] = char(223); // set correct char in string as per earlier specs - FIXME: this should not be needed anymores
 }   

int slewRaDecBySecs(long raSecs, long decSecs) {
return 1;
}

 /* 
 *  Basic Meade LX200 protocol
 */
void lx200(String s) { // all :.*# commands are passed here 
  if (s.substring(1,3).equals("GR")) { // :GR# 
   // printLog("GR");
    // send current RA to computer
    SerialBT.print(lx200RA);
  } else if (s.substring(1,3).equals("GD")) { // :GD# 
   // printLog("GD");
    // send current DEC to computer
    SerialBT.print(lx200DEC);
  } else if (s.substring(1,3).equals("GV")) { // :GV*# Get Version *
    char c = s.charAt(3); 
    if ( c == 'P') {// GVP - Product name
       SerialBT.print(appname);  
    } else if (c == 'N') { // GVN - firmware version
       SerialBT.print("100");  
    }
    SerialBT.print('#');
  } else if (s.substring(1,3).equals("Sr")) { // :SrHH:MM:SS# or :SrHH:MM.T# // no blanks after :Sr as per Meade specs
   // printLog("Sr");
    // this is INITAL step for setting position (RA)
    long hh = s.substring(3,5).toInt();
    long mi = s.substring(6,8).toInt();
    long ss = 0;
    if (s.charAt(8) == '.') { // :SrHH:MM.T#
      ss = (s.substring(9,10).toInt())*60/10;
    } else {
      ss = s.substring(9,11).toInt();
    }
    inRA = hh*3600+mi*60+ss;
    SerialBT.print(1); // FIXME: input is not validated
  } else if (s.substring(1,3).equals("Sd")) { // :SdsDD*MM:SS# or :SdsDD*MM#
    //printLog("Sd");
    // this is the FINAL step of setting a pos (DEC) 
    long dd = s.substring(4,6).toInt();
    long mi = s.substring(7,9).toInt();
    long ss = 0;
    if (s.charAt(9) == ':') { ss = s.substring(10,12).toInt(); }
    inDEC = (dd*3600+mi*60+ss)*(s.charAt(3)=='-'?-1:1);
    // FIXME: the below should not be needed anymore since :CM# command is honored
    if (currDEC == NORTH_DEC) { // if currDEC is still the initial default position (North)
      // assume this is to sync current position to new input
      currRA  = inRA;
      currDEC = inDEC;
      updateLx200Coords(currRA, currDEC); // recompute strings
    }
    SerialBT.print(1); // FIXME: input is not validated
  } else if (s.charAt(1) == 'M') { // MOVE:  :MS# (slew), :Mx# (slow move)
    if (s.charAt(2) == 'S' ) { // SLEW
         printLog("---MOVE SLEW---");
        printLog(s);
      // assumes Sr and Sd have been processed hence
      // inRA and inDEC have been set, now it's time to move
      long deltaRaSecs  = currRA-inRA;
      long deltaDecSecs = currDEC-inDEC;
      // FIXME: need to implement checks, but can't wait for slewRaDecBySecs
      //        reply since it may takes several seconds:
      SerialBT.print(0); // slew is possible 
       istracking=LOW;
      // slewRaDecBySecs replies to lx200 polling with current position until slew ends:
      if (slewRaDecBySecs(deltaRaSecs, deltaDecSecs) == 1) { // success         
        currRA  = inRA;
        currDEC = inDEC;
        updateLx200Coords(currRA, currDEC); // recompute strings
        
      } else { // failure
        SerialBT.print("1Range_too_big#");
         printLog("---SLEW FAIL - RANGE TOO BIG---");
        printLog(s);
     }
      istracking=true;
    } else {
      printLog("MMovex");
     
    }
  } else if (s.charAt(1) == 'Q') { // :Q# or :Qx# stop Dec Motor and set RA to Tracking
   // moveDecHalt();
    //moveRaTracking();
     printLog("--- DO TRACKING---");
   
    istracking=HIGH;
  } else if (s.substring(1,3).equals("CM")) { // :CM# sync
    printLog("---SYNC POSITION---");
    printLog(s);
    // assumes Sr and Sd have been processed
    // sync current position with input
 
    currRA  = inRA;
    currDEC = inDEC;
    SerialBT.print("Synced#");
    updateLx200Coords(currRA, currDEC); // recompute strings
    istracking=true;
  }
}

/* Update lx200 RA&DEC string coords so polling 
 * (:GR# and :GD#) gets processed faster
 */
void updateLx200Coords(long raSecs, long decSecs) {
  unsigned long pp = raSecs/3600;
  unsigned long mi = (raSecs-pp*3600)/60;
  unsigned long ss = (raSecs-mi*60-pp*3600);
  lx200RA = "";
  if (pp<10) lx200RA.concat('0');
  lx200RA.concat(pp);lx200RA.concat(':');
  if (mi<10) lx200RA.concat('0');
  lx200RA.concat(mi);lx200RA.concat(':');
  if (ss<10) lx200RA.concat('0');
  lx200RA.concat(ss);lx200RA.concat('#');

  pp = abs(decSecs)/3600;
  mi = (abs(decSecs)-pp*3600)/60;
  ss = (abs(decSecs)-mi*60-pp*3600);
  lx200DEC = "";
  lx200DEC.concat(decSecs>0?'+':'-');
  if (pp<10) lx200DEC.concat('0');
  lx200DEC.concat(pp);lx200DEC.concat(char(223)); // FIXME: may be just * nowadays
  if (mi<10) lx200DEC.concat('0'); 
  lx200DEC.concat(mi);lx200DEC.concat(':');
  if (ss<10) lx200DEC.concat('0');
  lx200DEC.concat(ss);lx200DEC.concat('#');
 } 


void doSerial(){
  // Check if message on serial input
  if (SerialBT.available() > 0) {
    input[in] = SerialBT.read(); 

    // discard blanks. Meade LX200 specs states :Sd and :Sr are
    // not followed by a blank but some implementation does include it.
    // also this allows aGoto commands to be typed with blanks
    if (input[in] == ' ') return; 
    
    // acknowledge LX200 ACK signal (char(6)) for software that tries to autodetect protocol (i.e. Stellarium Plus)
    if (input[in] == char(6)) { SerialBT.print("P"); return; } // P = Polar

    if (input[in] == '#' || input[in] == '\n') { // after a # or a \n it is time to check what is in the buffer
      if (input[0] == ':') { // it's lx200 protocol
        //printLog("---Got LX200 Command---");
        // printLog(input);
        //printLog("-----------------------");
        lx200(input);
      } else {
        // unknown command, print message only
        // if buffer contains more than one char
        // since stellarium seems to send extra #'s
        if (in > 0) {
          String s = input;
          printLog("--Failed LX200 Command--");
          printLog(input);
          printLog("-----------------------");
        }
      }
      in = 0; // reset buffer // WARNING: the whole input buffer is passed anyway
    } else {
      if (in++>20) in = 0; // prepare for next char or reset buffer if max lenght reached
    } 
  }
}

void loop() {  
    doSerial();
    portENTER_CRITICAL(&timerMux);
    //get or sset shared stuff with core
    portEXIT_CRITICAL(&timerMux);
  //  Serial.println(String((currentInterruptTime-lastInterruptTime)*2));
  
}   

// Helpers to write on serial when DEBUG is active
void printLog(  String s)         { if (DEBUG) { Serial.print("");Serial.println(s); } }
void printLogL( long l)           { if (DEBUG) { Serial.print("");Serial.println(l); } }
void printLogUL(unsigned long ul) { if (DEBUG) { Serial.print("");Serial.println(ul);} }
