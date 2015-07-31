/*
Main Tank-tread style drive control loop.

Two H-bridge motor drives, on left and right side

Motor drives take speed commands from -255..255,
with negative numbers for reverse.
If commands are not updated reguarly, the
motors will be commanded to stop.

See setup() funtion for pin assignments.

===========================================================

Aaron Birenboim, http://boim.com    30jul2015
provided under LGPL license

----------------- summary of LGPL :
You are free to use and/or alter this code as you need,
but no guarantee of correctness/appropriateness is implied.
If you do use this code please retain credit to above as originator.
*/

const char TAB = '\t';  // forces #include<Arduino.h> here, needed for following #include's

#define L298  // use L298 motor driver
#define DBH1  // use DBH1 modifications of 298 driver

#ifdef L298
  #include "MotorDrive298.h"
// params are decelRate, deadmanTimeout, startupPulseDuration, stopTimeout, maxPWM
  MotorDrive MotL(1.0f);
  MotorDrive MotR(1.0f);
#else
  //#include "MotorDriveDBH1.h"  // depricated DBH1 specific driver
  #include "MotorDriveC.h"  // vicky tan driver module

  // MotorDrive parameters :
  //        deadman timeout,
  //        startup pulse duration, stop pause after emergency,
  //        min PWM to move, max PWM
  MotorDrive MotL(250,50,3000,15);
  MotorDrive MotR(250,50,3000,15);
#endif

#include "Command.h"  // can re-use Command from DalekDrive
CommandReader Command;

void setup()
{
  // AVR 168, 328 based Arduinos have PWM on 3, 5, 6, 9, 10, and 11
  // Timer2 for pins 3,11 : Timer 0 for pins 6,5 : Timer 1 for 9,10
  // Mega has PWM on on pins 2 through 13.

#ifdef L298
 #ifdef DBH1
  // DBH1 needs PWM on all three inputs, and a current-sense output pin
// 9,10 are 16-bit timer, Timer1.  Changing freq on these messes up Servo
// 3,11 are Timer2, changing freq changes tone() function
// 5,6  are Timer0, changing freq messes up millis()... DO NOT CHANGE FREQ ON THESE!
  // en, in1, in2, cs
  MotR.begin( 9,5, 3,1);
  MotL.begin(10,6,11,2);
 #else
  // params: EN, IN1, IN2
  MotR.begin(3,2,4);
  MotL.begin(11,7,8);
 #endif
#else
    // depricated DBH1 specific driver
    //MotR.begin( 9,5, 3,1,1.0);
    //MotL.begin(10,6,11,2,1.0);

  MotR.begin(4,3,2);   // vicky tan motor drive module
  MotL.begin(8,11,2);
#endif

  //Serial.begin(9600);
  Serial.begin(57600);  // nano
  //Serial.begin(115200);  # uno

  // When doing diagnostics, we may want to increase deadman time
  //MotL.setCommandTimeout(16000);
  //MotR.setCommandTimeout(16000);

//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------  
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
//TCCR2A |= B00000011;  // Fast PWM (default for Arduino)
//TCCR2A = (TCCR2A & B11111100) | B00000001;  // timer 2, phase corrected PWM, which is 1/2 rate of Fast

//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
 
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
}

unsigned long prevCommandTime = 0;

#define FLASH_DT 800
unsigned long tFlash = 0;

// for diagnostics, just print a few messages, then be quiet to improve
// performance when in actual use.
int nMsg = 9;

void loop()
{
  unsigned long t = millis();

  char code;
  int val;
  if (Command.get(code,val))
    {
      prevCommandTime = t;
if (nMsg>0){nMsg--;Serial.print('>');Serial.print(code);Serial.println(val);}
      switch(code)
        {
        case 'L': MotL.setSpeed(val,t); break;
        case 'R': MotR.setSpeed(val,t); break;
        /* app may send bad commands.  just ignore them or they can clog the serial port
        default : 
          MotL.setSpeed(0,t);  // odd command.  just stop
          MotR.setSpeed(0,t);
          Serial.println("<stop");
        */
        }
    }
  else
    { // no command, do housekeeping (misc state update stuff)
      MotL.update(t);
      MotR.update(t);

      if ((prevCommandTime > 0xfffff000) && (t < 999))
        {  // time counter must have wrapped around
          prevCommandTime = tFlash = 0;
        }
        
      if (t - tFlash > FLASH_DT)
        { // Flash standard LED to show things are running
          tFlash = t;
          digitalWrite(13,digitalRead(13)?LOW:HIGH);  // toggle heartbeat
        }
    }
}

