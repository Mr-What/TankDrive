/*
$URL: svn+ssh://aaron@birenboim.com/home/svn/arduino/sketchbook/DalekDrive/MotorDriveC.h $
$Id: MotorDriveC.h 143 2015-03-23 00:24:53Z aaron $

Motor driver for DBH-1A/B/C series Motor controler (aliexpress, WINGXINE).
This controller has two inputs for each H-Bridge, and an enable pin,
and a current-sense.
It also provides a 5v-ish output appropriate for running the MCU

It was found that motors seemed to run a bit faster and more efficient
with PWM on the Enable.  However, since spec claims only 0-99% PWM,
We still wish to PWM the direction enable.  There might be a charge
pump someplace that needs some down time.

It seems to have a hard brake when both inputs set to 0, and enabled.
However, it does seem to coast when enable is 0, inputs don't matter in this state.

The goal of this driver is to PWM (mostly) EN when driving, but
hard-brake on speed 0 and speed 0 transitions.

*/

// Don't allow immediate change of direction.
// let it "brake" for a little bit before starting in other direction
#define MOTOR_STOP  0
#define MOTOR_2STOP 1     // 1's is transition state indicator bit
#define MOTOR_FWD   4
#define MOTOR_REV   8
#define MOTOR_2FWD  5
#define MOTOR_2REV  9

#ifndef ABS
#define ABS(x)  (((x)<0)?(-(x)):(x))
#endif

// ----------- Global state classes:
#include "MotorDriveBase.h"
class MotorDrive : public MotorDriveBase
{
protected:
  inline int getPWM(int pwm)
  {
    pwm = ABS(pwm);
    if (pwm > 255) return(255);
    if (pwm < _minPWM) return(0);
    return(pwm);
  }

  void setReverse(bool rev) // true==set reverse direction, false==forward
  {
    if (rev)
      {
        digitalWrite(Pin.IN2,    0   );
	analogWrite( Pin.IN1, _maxPWM);
      }
    else
      {
        digitalWrite(Pin.IN1,    0   );
	analogWrite( Pin.IN2, _maxPWM);
      }
  }
  
public:
  struct {
    // it does not seem to hurt to have IN1=IN2=1, but it doesn't seem
    // to do anything different/useful, so try to avoid this state.
    
    int IN1, IN2;  // forward/reverse selectors.
    int EN;        // enable pin
    int CS;        // current sense analog input pin
  } Pin;
  int _speed;  // current speed
  float _decel; // time to allow to stop, in ms / PWM count
  byte _mode;
  unsigned long _modeDoneTime;
  int _deadTime, _minPWM, _maxPWM, _startupTime, _stopTime;
  int _msgCount;
  
  MotorDrive(const int deadTime=250,
             const int startupTime=5,
             const int stopTime=3000,
             const int minPWM=9,
             const int maxPWM=252)
  {
    _deadTime = deadTime; // emergency stop if no command update in this time interval
    _minPWM = minPWM;     // Motors won't move below this level
    _maxPWM = maxPWM;     // don't go over this PWM level.  driver can't do it
    _startupTime = startupTime; // when starting from still, issue full power pulse this long to get motors started
    _stopTime = stopTime;  // Pause at least this long after emergency stop before restarting
    _msgCount = 11;  // issue this many diagnostic messages before going quiet
  }

  virtual void stop()
  {
    analogWrite(Pin.IN1, 0);
    analogWrite(Pin.IN2, 0);
    analogWrite(Pin.EN , 0);
    int stoppingTime = (int)ABS(_speed * _decel);
if(_msgCount>0){_msgCount--;Serial.print(stoppingTime);Serial.println(" ms to stop.");}
    _modeDoneTime = millis() + stoppingTime;
    //speed=0;  don't clobber command in case of direction change
    _mode = MOTOR_2STOP;
  }

  virtual void emergencyStop()
  {
    Serial.print("Emergency ");
    _msgCount = 11;  // turn on diagnostics for a few commands
    stop();
    _speed=0;
    _modeDoneTime += _stopTime;
  }

  void begin(const int en, const int in1, const int in2, const int cs,
	     const float de=2.0)
  {
    pinMode(en,OUTPUT);   digitalWrite(en,0);
    Pin.EN  =en;
    Pin.IN1 =in1;
    Pin.IN2 =in2;
    Pin.CS  =cs;
    _decel=de;

    pinMode(Pin.IN1,OUTPUT);  digitalWrite(Pin.IN1,0);
    pinMode(Pin.IN2,OUTPUT);  digitalWrite(Pin.IN2,0);
    
    emergencyStop();
  }

  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  virtual void setSpeed(const int spdReq, long t)
  {
    byte prevMode = _mode;
    bool rev;
    switch(prevMode)
      {
      case MOTOR_2STOP :
        if (t < _modeDoneTime)
          {  // make sure things are stopped
	    digitalWrite(Pin.IN1,0);
	    digitalWrite(Pin.IN2,0);
	    digitalWrite(Pin.EN ,0);
            return;
          }
        // done stoping, continue to STOP mode
        _mode = MOTOR_STOP;
if(_msgCount>0){_msgCount--;Serial.println(F("stopped."));}
      case MOTOR_STOP :
	if (ABS(spdReq) < _minPWM) return;
	_mode = (spdReq < 0) ? MOTOR_2REV : MOTOR_2FWD;
	rev = (_mode == MOTOR_2REV);
	digitalWrite(rev?Pin.IN1:Pin.IN2,1); // don't worry about PWM
	digitalWrite(rev?Pin.IN2:Pin.IN1,0); // this is transistional state
	digitalWrite(Pin.EN,1);   // hard kick to get started
	_modeDoneTime = t + _startupTime;
	_speed = spdReq;
if(_msgCount>0){_msgCount--;
Serial.print(F("Start "));
Serial.println(rev ? F("REV") : F("FWD"));}
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
        //digitalWrite(Pin.REV,(prevMode == MOTOR_REV) ? HIGH : LOW);
	setReverse(_mode == MOTOR_2REV);
	if (t > _modeDoneTime) { emergencyStop(); return; } // deadman expired
	if ( (ABS(spdReq) < _minPWM)  ||  // stop or change direction
	     ((spdReq < 0) && (prevMode == MOTOR_FWD)) ||
	     ((spdReq > 0) && (prevMode == MOTOR_REV)) )
	  {
	    stop();
            // set speed so that it goes to this speed after coast-down
            _speed = (ABS(spdReq) < _minPWM) ? 0 : spdReq;
	    return;
	  }
	_speed = spdReq;
	analogWrite(Pin.EN,getPWM(_speed));
	_modeDoneTime = t + _deadTime;
//if(_msgCount>0){_msgCount--;Serial.println(_speed);}
	return;
      case MOTOR_2REV :
      case MOTOR_2FWD :
	if (ABS(spdReq) < _minPWM)
          {
            _speed = 100;  // give it some time to decel, although just starting
            stop();
            _speed = 0;
            return;
          }
	if ( ((spdReq < 0) && (_mode == MOTOR_2FWD)) ||
	     ((spdReq > 0) && (_mode == MOTOR_2REV)) )
	  { // direction change
            _speed = 100;  // give it some time to decel, although just starting
	    stop();
            _speed = spdReq;  // go to this speed after coast-down period
	    return;
	  }
	// same direction, but speed request change
	_speed = spdReq;
	if (t >= _modeDoneTime)
	  {
	    _mode = (_speed > 0) ? MOTOR_FWD : MOTOR_REV;
	    _modeDoneTime = t + _deadTime;
	    setReverse(_mode == MOTOR_REV);  // make sure direction is correct
            analogWrite(Pin.EN,getPWM(_speed));
if(_msgCount>0){_msgCount--;Serial.println("Started");}
	  }
	return;
      }
  }

  // update state, but no new command (no deadman reset)
  // Checks if previous command is complete, and an automatic state transition
  // is needed
  virtual void update(long t)  // current time, from millis()
  {
//Serial.print(F("Update "));  Serial.println(t);
    if ((_modeDoneTime > 0xfffff000) && (t < 999))
      {  // time counter must have wrapped around
        _modeDoneTime = 0;
        Serial.println("Clock wrap-around");
      }

    byte prevMode = _mode;
    switch(prevMode)
      {
      case MOTOR_2STOP : 
      case MOTOR_STOP :
        if ((t > _modeDoneTime) && _speed)
          { // this was a temp stop in a direction change.  Command desired speed.
if(_msgCount>0){_msgCount--;Serial.print(F("Restart "));Serial.println(_speed);}
            setSpeed(_speed,t);
          }
//else Serial.println("stopped.");
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
	if (t > _modeDoneTime) emergencyStop(); // deadman expired
	return;
      case MOTOR_2REV :
      case MOTOR_2FWD :
	if (t > _modeDoneTime)
          {
            //mode = (prevMode == MOTOR_2REV) ? MOTOR_REV : MOTOR_FWD;
if(_msgCount>0){_msgCount--;Serial.println(F("moving"));}
            setSpeed(_speed,t);
          }
	return;
      }
  }

  int getCurrentCounts()
  {
    return(analogRead(Pin.CS));
  }
};
