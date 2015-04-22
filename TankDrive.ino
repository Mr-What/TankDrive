/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/TankDrive/TankDrive.ino $
$Id: TankDrive.ino 136 2013-06-09 14:04:33Z aaron $

Main Tank-tread style drive control loop.

Two H-bridge motor drives, on left and right side

Motor drives take speed commands from -255..255,
with negative numbers for reverse.
If commands are not updated reguarly, the
motors will be commanded to stop.

See setup() funtion for pin assignments.
*/

#define COMMAND_TIMEOUT 250   // allowable ms without a command before emergency shutdown

const char TAB = '\t';  // forces #include<Arduino.h> here, needed for following #include's

#ifdef L289
  #include "MotorDrive298.h"
  MotorDrive298 MotL, MotR;
#else
  #include "MotorDriveC.h"
  // MotorDrive parameters :
  //        deadman timeout,
  //        startup pulse duration, stop pause after emergency,
  //        min PWM to move, max PWM
  MotorDrive MotL(250,50,3000,15);
  MotorDrive MotR(250,50,3000,15);
#endif

#include "Command.h"  // can re-use Command from DalekDrive


void setup()
{
  // AVR 168, 328 based Arduinos have PWM on 3, 5, 6, 9, 10, and 11
  // Timer2 for pins 3,11 : Timer 0 for pins 6,5 : Timer 1 for 9,10
  // Mega has PWM on on pins 2 through 13.

#ifdef L298
#error code changed.  re-check L298 driver
  MotR.begin(4,5,3);
  MotL.begin(9,10,11);
#else
  MotR.begin(4,3,2);   // vicky tan motor drive module
  MotL.begin(8,11,2);
#endif

  //Serial.begin(9600);
  Serial.begin(57600);  // nano
  //Serial.begin(115200);  # uno
  
//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------  
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
//TCCR2A |= B00000011;  // Fast PWM (default for Arduino)
//TCCR2A = (TCCR2A & B11111100) | B00000001;  // timer 2, phase corrected PWM, which is 1/2 rate of Fast
}

unsigned long prevCommandTime = 0;

#define FLASH_DT 800
unsigned long tFlash = 0;

// for diagnostics, just print a few messages, then be quiet to improve
// performance when in actual use.
int nMsg = 99;

void loop()
{
  unsigned long t = millis();

#ifdef L298
  if (t - prevCommandTime > COMMAND_TIMEOUT)
    {
      deadmanReleased();  // timeout, do same as release of a deadman enable
      prevCommandTime = t;
    }
#endif

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
        default : 
          MotL.setSpeed(0,t);  // odd command.  just stop
          MotR.setSpeed(0,t);
          Serial.println("<stop");
        }
    }
  else
    { // no command, do housekeeping (misc state update stuff)
#ifndef L298
      MotL.update(t);
      MotR.update(t);
#endif

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

// make sure deadman release stops immediately
void deadmanReleased()
{
  MotR.emergencyStop();
  MotL.emergencyStop();
  Serial.println("STOP");
}
