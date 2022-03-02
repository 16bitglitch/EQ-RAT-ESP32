/*  
# EQRAT-UNO
## Simple Equatorial Mount Ra Sidereal Tracker for ESP32 Based Boards

By Charles Gershom 
@charlesgershom 

See Readme.md

*/
 
#include <WiFi.h>
#include <BluetoothSerial.h>

//Debug Stuff
const bool debugEnabled=false;
const unsigned long _ver = 100000;
const String _appName = "EQ-RAT";

//Configuration
const int Mount_Worm_Gear_Ratio=130;
const int Motor_Gear_Ratio=3;
const int Steps_Per_Rev=400;
const int Microstep_Setting=32;


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
const int raDirPin = 16;   
const int raStepPin = 26;    

//Dec Stepper Config
const int decDirPin = 27;   
const int decStepPin = 17;    

//Ra Stepper State
uint8_t raStepState=LOW;

volatile uint8_t istracking=HIGH;

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


const long NORTH_DEC   = 324000; // 90°

// Current coords in Secs (default to true north)
long currRA  = 0;     
long currDEC = NORTH_DEC;

const int DEC_HALT  = 0;   // dec is not moving
const int DEC_NORTH = 1;   // 1st time dec button is pressed // FIXME: IT MAY BE THE VICEVERSA NEED TO TEST!
const int DEC_SOUTH = 2;   // 2nd time dec button is pressed (at 3rd press, re-set to DEC_HALT)
const int DEC_ST4   = 3;   // dec is moving due to ST4 pulse
int decState = DEC_HALT;   // initial DEC state (don't move)

const int RA_TRACKING = 0; // tracking at 1x
const int RA_EAST = 1;     // 1st time button is pressed
const int RA_WEST = 2;     // 2nd time button is pressed (at 3rd press, re-set to tracking)
const int RA_ST4  = 3;     // moving due to ST4 pulse
int raState = RA_TRACKING; // initial RA speed (tracking)


// Serial Input
char input[20];     // stores serial input
int  in = 0;        // current char in serial input
// Serial Input (New) coords
long inRA    = 0;
long inDEC   = 0;

// Current position in Meade lx200 format, see updateLx200Coords()
String lx200RA  = "00:00:00#";
String lx200DEC = "+90*00:00#";

unsigned long STEP_DELAY_SLEW = 1200;   // Slewing Pulse timing in micros (the higher the pulse, the slower the speed)

// Vars to implement accelleration
unsigned long MAX_DELAY      = 16383; // limit of delayMicroseconds() for Arduino Uno
unsigned long decStepDelay   = MAX_DELAY; // initial pulse lenght (slow, to start accelleration)
unsigned long decTargetDelay = STEP_DELAY/SLOW_SPEED; // pulse length to reach when Dec button is pressed
unsigned int  decPlayIdx     = 0; // pulse index, Dec will accellerate for first 100 pulses

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
     digitalWrite(raStepPin, raStepState); 

   //Do some counting if debug is enabled
   if(debugEnabled)
      timerCount();
   
   portEXIT_CRITICAL_ISR(&timerMux);
}



// Generated with http://www.arduinoslovakia.eu/application/timer-calculator
void setupTimer1() {#

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
   digitalWrite(raDirPin, LOW);   // invert this (HIGH) if wrong direction    
  for(int s=0;s<50000;s++){
       digitalWrite(raStepPin, HIGH); 
       delayMicroseconds(10);
       digitalWrite(raStepPin, LOW); 
       delayMicroseconds(10);
        
   
  } 
  
   digitalWrite(raDirPin, HIGH);   // invert this (HIGH) if wrong direction    
   for(int s=0;s<50000;s++){
       digitalWrite(raStepPin, HIGH); 
       delayMicroseconds(10);
       digitalWrite(raStepPin, LOW); 
       delayMicroseconds(10);
        
   
  } 
     
  
  
}


void setup() {  
  
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  Serial.println("Starting EQRAT");
  Serial.println("--------------");
  Serial.println("Timer Calc : " + String(Step_Delay_Microseconds));
  pinMode(raStepPin, OUTPUT);   
  pinMode(raDirPin, OUTPUT);    
  digitalWrite(raDirPin, HIGH);   // invert this (HIGH) if wrong direction    
  
  
  SerialBT.begin("EQ-RAT"); //Bluetooth device name
  Serial.println("Bluetooth started, now you can pair!");

 // testRA();
  
  //Setup and start Timer
  setupTimer1();
 }   



void loop() {  
    portENTER_CRITICAL(&timerMux);
    //get or sset shared stuff with core
    portEXIT_CRITICAL(&timerMux);
  //  Serial.println(String((currentInterruptTime-lastInterruptTime)*2));
   // Check if message on serial input
  if (SerialBT.available() > 0) {
    input[in] = SerialBT.read(); 

    // discard blanks. Meade LX200 specs states :Sd and :Sr are
    // not followed by a blank but some implementation does include it.
    // also this allows aGoto commands to be typed with blanks
    if (input[in] == ' ') 
        return; 
    
    // acknowledge LX200 ACK signal (char(6)) for software that tries to autodetect protocol (i.e. Stellarium Plus)
    if (input[in] == char(6)) 
    { 
        SerialBT.print("P"); return; 
    } // P = Polar

    if (input[in] == '#' || input[in] == '\n') { // after a # or a \n it is time to check what is in the buffer
       if (input[0] == ':') { // it's lx200 protocol
        printLog(input);
        lx200(input);
      } else {
        // unknown command, print message only
        // if buffer contains more than one char
        // since stellarium seems to send extra #'s
        if (in > 0)
        {
          String s = input;
          Serial.print(s.substring(0,in));
          Serial.println(" unknown. Expected lx200 commands");
        }
      }
      in = 0; // reset buffer // WARNING: the whole input buffer is passed anyway
    } 
    else 
    {
      if (in++>20) in = 0; // prepare for next char or reset buffer if max lenght reached
    } 
  }
}   

#
/*
 *  Slew RA and Dec by seconds/arceconds (ra/dec)
 *   motors direction is set according to sign
 *   RA 1x direction is re-set at the end
 *   microstepping is disabled for fast movements and (re-)enabled for finer ones
 */
int slewRaDecBySecs(long raSecs, long decSecs) {

  // If more than 12h, turn from the opposite side
  if (abs(raSecs) > Seconds_Earth_Rotate/2) { // reverse
    printLog("RA reversed, RA secs:");
    raSecs = raSecs+(raSecs>0?-1:1)*Seconds_Earth_Rotate;
    printLogUL(raSecs);
  }

  // check if within max range
  if ( (abs(decSecs) > (MAX_RANGE*60)) || ( abs(raSecs) > (MAX_RANGE*4)) ) {
    return 0; // failure
  }
  
  // set directions
  digitalWrite(raDirPin,  (raSecs  > 0 ? RA_DIR :(RA_DIR ==HIGH?LOW:HIGH)));
  digitalWrite(decDirPin, (decSecs > 0 ? DEC_DIR:(DEC_DIR==HIGH?LOW:HIGH)));

  // calculate how many micro-steps are needed
  unsigned long raSteps  = (abs(raSecs) * MICROSTEPS_PER_HOUR) / 3600;
  unsigned long decSteps = (abs(decSecs) * MICROSTEPS_PER_DEGREE_DEC) / 3600;

  printLog(" RA uSteps:");
  printLogUL(raMicroSteps);
  printLog(" DEC uSteps:");
  printLogUL(decMicroSteps);
   
   //move this to here since we only doing microsteps and still need to make sure its not taken too long
  unsigned long slewTime = micros(); // record when slew code starts, RA 1x movement will be on hold hence we need to add the gap later on
  slewRaDecBySteps(raSteps, decSteps);
  printLog("uSteps Slew Done");

  // If slewing took more than 5" (secs), adjust RA
  slewTime = micros() - slewTime; // time elapsed for slewing
  if ( slewTime > (5 * 1000000) ) {
    printLog("* adjusting Ra by secs: ");
    printLogUL(slewTime / 1000000);
    slewRaDecBySecs(slewTime / 1000000, 0); // it's the real number of seconds!
    // printLog("*** adjusting Ra done");
  }

  // reset RA to right sidereal direction
  digitalWrite(raDirPin,  RA_DIR);
  
  // Success
  return 1;
}


/*
 *  Slew RA and Dec by steps
 *   . assume direction and microstepping is set
 *   . turn system led on 
 *   . set SLEWING to true to hold RA interrupt tracking
 *   . while slewing, listen on serial port and reply to lx200 GR&GD
 *      commands with current (initial) position to avoid
 *      INDI timeouts during long slewings actions
 */
void slewRaDecBySteps(unsigned long raSteps, unsigned long decSteps) {
  digitalWrite(LED_BUILTIN, HIGH);
  SLEWING = true;

  unsigned long delaySlew = 0; 
  unsigned long delayLX200Micros = 0; // mesure delay introduced by LX200 polling reply
  in = 0; // reset the  input buffer read index
  
  for (unsigned long i = 0; (i < raSteps || i < decSteps) ; i++) {
    if ((i<100)) { // Accellerate during inital 100 steps from MAX_DELAY to STEP_DELAY_SLEW
      delaySlew = MAX_DELAY-( (MAX_DELAY-STEP_DELAY_SLEW)/100*i);
    } else if ( (i>raSteps-100 && i<raSteps)|| (i>decSteps-100 && i<decSteps)) {
      delaySlew = STEP_DELAY_SLEW*2;// twice as slow in last 100 steps before a motor is about to stop 
    } else { 
      delaySlew = STEP_DELAY_SLEW; // full speed
    } 
    
    if (i < raSteps)  { digitalWrite(raStepPin,  HIGH); }
    if (i < decSteps) { digitalWrite(decStepPin, HIGH); }
    delayMicroseconds(delaySlew);
    
    if (i < raSteps)  { digitalWrite(raStepPin,  LOW);  }
    if (i < decSteps) { digitalWrite(decStepPin, LOW);  }
    
    // support LX200 polling while slewing
    delayLX200Micros = 0;
    if (SerialBT.available() > 0) {
      delayLX200Micros = micros();
      input[in] = SerialBT.read();
      if (input[in] == '#' && in > 1 ) {
        if (input[in-1] == 'R') { // :GR#
          SerialBT.print(lx200RA);
        } else if (input[in-1] == 'D') { // :GD#
          SerialBT.print(lx200DEC);
        } else if (input[in-1] == 'Q') { // :Q# stop FIXME: motors stops but current coordinates are set to new target...
          printLog("Slew Stop");
          break;
        }
        in = 0;
      } else {
        if (in++ >5) in = 0;
      }
      delayLX200Micros = micros()-delayLX200Micros;
      if (delayLX200Micros>delaySlew) {
        Serial.println("LX200 polling slows too much!"); // this should never happen. But it happens if we'd use SoftwareSerial
        /* reset position to north and exit
        inRA  = 0;     
        inDEC = NORTH_DEC;
        break; */
      }

    } 
    delayMicroseconds(delaySlew-delayLX200Micros);
  }
  // set Dec to sleep
  if (decSteps != 0) {
    decSleep(true);
  }
  digitalWrite(LED_BUILTIN, LOW);
  SLEWING = false;
}

/* 
 *  Basic Meade LX200 protocol
 */
void lx200(String s) { // all :.*# commands are passed here 
  if (s.substring(1,3).equals("GR")) { // :GR# 
    printLog("GR");
    // send current RA to computer
    SerialBT.print(lx200RA);
  } else if (s.substring(1,3).equals("GD")) { // :GD# 
    printLog("GD");
    // send current DEC to computer
    SerialBT.print(lx200DEC);
  } else if (s.substring(1,3).equals("GV")) { // :GV*# Get Version *
    char c = s.charAt(3); 
    if ( c == 'P') {// GVP - Product name
       SerialBT.print(_appName);  
    } else if (c == 'N') { // GVN - firmware version
       SerialBT.print(_ver);  
    }
    SerialBT.print('#');
  } else if (s.substring(1,3).equals("Sr")) { // :SrHH:MM:SS# or :SrHH:MM.T# // no blanks after :Sr as per Meade specs
    printLog("Sr");
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
    printLog("Sd");
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
      printLog("MS");
      // assumes Sr and Sd have been processed hence
      // inRA and inDEC have been set, now it's time to move
      long deltaRaSecs  = currRA-inRA;
      long deltaDecSecs = currDEC-inDEC;
      // FIXME: need to implement checks, but can't wait for slewRaDecBySecs
      //        reply since it may takes several seconds:
      SerialBT.print(0); // slew is possible 
      // slewRaDecBySecs replies to lx200 polling with current position until slew ends:
      if (slewRaDecBySecs(deltaRaSecs, deltaDecSecs) == 1) { // success         
        currRA  = inRA;
        currDEC = inDEC;
        updateLx200Coords(currRA, currDEC); // recompute strings
      } else { // failure
        SerialBT.print("1Range_too_big#");
      }
    } 
  } else if (s.charAt(1) == 'Q') { // :Q# or :Qx# stop Dec Motor and set RA to Tracking
    moveDecHalt();
    moveRaTracking();
  } else if (s.substring(1,3).equals("CM")) { // :CM# sync
    // assumes Sr and Sd have been processed
    // sync current position with input
    printLog("CM");
    currRA  = inRA;
    currDEC = inDEC;
    SerialBT.print("Synced#");
    updateLx200Coords(currRA, currDEC); // recompute strings
  }
}

void moveRaWest() {
     
}

void moveRaEast() {
      
}

void moveRaTracking() {
    istracking=true;
}

void moveDecNorth() {
   
}

void moveDecSouth() {
    
}

void moveDecHalt() {
      
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

// Helpers to write on serial when DEBUG is active
void printLog(  String s)         { if (debugEnabled) { Serial.print(":");Serial.println(s); } }
void printLogL( long l)           { if (debugEnabled) { Serial.print(":");Serial.println(l); } }
void printLogUL(unsigned long ul) { if (debugEnabled) { Serial.print(":");Serial.println(ul);} }
