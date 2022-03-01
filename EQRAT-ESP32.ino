/*  
# EQRAT-UNO
##Simple Equatorial Mount Ra Sidereal Tracker for Arduino UNO/328

By Charles Gershom 
@charlesgershom 

See Readme.md

*/
 
//Debug Stuff
const bool debugEnabled=false;

//Configuration
const int Mount_Worm_Gear_Ratio=100;
const int Motor_Gear_Ratio=3;
const int Steps_Per_Rev=400;
const int Microstep_Setting=32;

//Stuff for timer calc (doing everything as floats until its time to convert to timer)

const float Seconds_Earth_Rotate=86164.09053;
const float processorSpeed=16000000;


float Earth_Seconds_Per_Degree =Seconds_Earth_Rotate / 360.0;
float MicroSteps_Per_Degree =((float)Mount_Worm_Gear_Ratio * (float)Motor_Gear_Ratio *  (float)Steps_Per_Rev * (float)Microstep_Setting) / 360.0;
float Step_Delay_Microseconds =(Earth_Seconds_Per_Degree / MicroSteps_Per_Degree) * 1000000.0;
float Step_Delay_Timer_Half_Phase=Step_Delay_Microseconds / 2.0;

float OCR1A_Calc_Value = ( 16000000.0 / (((1.0/(Step_Delay_Timer_Half_Phase)) * 1000000.0) * 8.0)) -1.0;
unsigned int OCR1A_Value = round(OCR1A_Calc_Value);

long lastTime=0;
long currentTime=0;

//Ra Stepper Config
const int RAdirPin = 5;   
const int RAstepPin = 2;    

//Dec Stepper Config
const int DECdirPin = 6;   
const int DECstepPin = 3;    

//Ra Stepper State
uint8_t raStepState=LOW;

// Generated with http://www.arduinoslovakia.eu/application/timer-calculator
void setupTimer1() {
  noInterrupts();
  
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  //EDIT THIS TO SET TIMER CORRECTLY  - See above
  OCR1A=OCR1A_Value;
  //OCR1A = 34519;
  //EDIT THIS TO SET TIMER CORRECTLY - See above

  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 8
  TCCR1B |= (1 << CS11);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void setup() {  
  
  Serial.begin(9600);
  Serial.println("Starting EQRAT");
  Serial.println("--------------");
  Serial.println("Timer Calc : " + String(OCR1A_Value));
  //Setup Pins  
  pinMode(RAstepPin, OUTPUT);   
  pinMode(RAdirPin, OUTPUT);    
  digitalWrite(RAdirPin, HIGH);   // invert this (HIGH) if wrong direction    

  //Setup and start Timer
  setupTimer1();
    
}   

ISR(TIMER1_COMPA_vect) {
  
  //Switch the step pin state
  if(raStepState==LOW){
    raStepState=HIGH;
  }
  else{
    raStepState=LOW;
  }

   //Write to the step pin
   digitalWrite(RAstepPin, raStepState); 

   //Do some counting if debug is enabled
   if(debugEnabled)
      timerCount();
}

void timerCount(){
  lastTime=currentTime;
  currentTime=micros();
}

void loop() {  
  //Spit out some debug stuff
  if(debugEnabled)
    Serial.println(String((currentTime-lastTime)*2));
}   
