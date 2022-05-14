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
const bool debugEnabled=false;

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

   //Do some counting if debug is enabled
   if(debugEnabled)
      timerCount();
   
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

void testRA(){
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

 testRA();
  
  //Setup and start Timer
  setupTimer1();
 }   



void loop() {  
    portENTER_CRITICAL(&timerMux);
    //get or sset shared stuff with core
    portEXIT_CRITICAL(&timerMux);
  //  Serial.println(String((currentInterruptTime-lastInterruptTime)*2));
  
}   
