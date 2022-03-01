# EQRAT-UNO
##Simple Equatorial Mount Ra Sidereal Tracker for Arduino UNO/328

By Charles Gershom 
@charlesgershom 

## About

Originally Built to run a Skywatcher EQ 3-2 for polar aligned RA only sidereal tracking.
This was built in response to not being able to figure out why onstep wasnt tracking right
and because I had no real idea what the drivers were doing, what the gear stuff was, or how 
uno timers worked. 
   
The intention is to have a super simple "just turn it on" sidereal motor driver for the mount since I enjoy
finding the stars manually by eye instead of goto. This is a result of a couple nights of head scratching, info reading, and learning that delayMicroseconds on arduino overflows and goes wonky above 16383... always read the documents... 
   
Some of thie might be wrong or innacurate.. its working "good enough" for me, but thought it might be useful to share


## References
Thanks to this thread that inspired me to make a super simple version. This is very similar, but I decided to use timers instead:
https://www.cloudynights.com/topic/731261-yet-another-diy-ra-drive-for-eq5-exos2-eq3-etc-etc/

## Parts Used 
Skywatcher EQ3-2
1.8degree/200 Step Nema 17 Motors 
DRV8825 Drivers set to 32 Micro Steps
Arduino UNO
CNC V3 Arduino Shield

## Configuration
   
To modify this for your setup, input the details for your scope with the variables:

Mount_Worm_Gear_Ratio

Motor_Gear_Ratio

Steps_Per_Rev

Microstep_Setting

## Timer Calculations

The sketch will auto calculate the timer value based on the information below.
  
Seconds_Earth_Rotate = 86164.09053
Earth_Seconds_Per_Degree = Seconds_Earth_Rotate / 360
MicroSteps_Per_Degree = (Mount_Worm_Gear_Ratio * Motor_Gear_Ratio *  Steps_Per_Rev * Microstep_Setting) / 360
Step_Delay_Microseconds = (Earth_Seconds_Per_Degree / MicroSteps_Per_Degree) * 1000000
Step_Delay_Timer_Half_Phase = Step_Delay_Microseconds / 2
   
I use Step_Delay_Timer_Half_Phase in the timer1 OCR1A calculations This alternates the 
step pin Low and High to complete one full step/microstep in two phases. Timer is then calculated using:  
   
OCR1A= ( 16000000 / (1/(Step_Delay_Timer_Half_Phase)) * 1000000) * 8) ) -1
   
For reasons, see below calcs    
https://www.unitjuggler.com/convert-frequency-from-%C2%B5s(p)-to-Hz.html?val=17256
and 
http://www.arduinoslovakia.eu/application/timer-calculator
   
Example
Earth_Seconds_Per_Degree = 86164.09053 / 360 = 239.34
MicroSteps_Per_Degree = (130  * 3 *  200 *  32) /360 = 6933.333333
Step_Delay_Microseconds = (239.34/6933.333333)  1000000 = 34520.8
Step_Delay_Timer_Half_Phase = 34520.8 / 2 = 17260.4
   
OCR1A= ( 16000000 / (1/(17260.4))*1000000)*8) ) -1 = 34520 (rounded to an int)
