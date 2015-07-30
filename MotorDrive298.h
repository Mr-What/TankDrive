/*
Motor driver for Motor controllers using the L298 Dual-full-H-Bridge chip,
or work alikes.
This chip generally has two direction inputs and an enable pin.
Typical use is:

    IN1=1, IN2=0, EN=PWM   -- reverse at PWM speed
    IN1=0, IN2=1, EN=PWM   -- forward at PWM speed
    IN1=0, IN2=0, EN=1     -- electrical brake
    IN1=X, IN2=X, EN=0     -- coast

This controller has two inputs for each H-Bridge, and an enable pin.
It also provides a 5v-ish OUTPUT appropriate for running the MCU

It was found that motors seemed to run a bit faster and more efficient
with PWM on the Enable.
Future versions of this driver may allow user to put the PWM
on the direction pin which seems to provide a more constant-velocity
response, where putting PWM on the enable is more like the accelerator
input on a car.  (It has a peak velocity, but until it gets there
is is closer related to acceleration)

Driver will apply electrical brake when stopping, but
try to use freewheeling PWM mode when driving.

Aaron Birenboim, http://boim.com    28jul2015
provided under LGPL license

----------------- summary of LGPL :
You are free to use and/or alter this code as you need,
but no guarantee of correctness/appropriateness is implied.
If you do use this code please retain credit to above as originator.

*/

// motor states (I'm afraid of enum which might be 16 bit on 8-bit MCU)
#define MOTOR_STOPPED   0     // 1's -- running bit
#define MOTOR_FWD       1     // 2's -- direction (1==rev)
#define MOTOR_REV       3
#define MOTOR_START_FWD 5     // 4's -- start-up pulse
#define MOTOR_START_REV 7
#define MOTOR_STOPPING  8     // 8 -- electrical braking

#ifndef ABS
#define ABS(x)  (((x)<0)?(-(x)):(x))
#endif

typedef byte  BYTE;  // signed char, 8-bit
typedef short SHORT; // signed int, 16-bit

// ----------- Global state classes:
#include "MotorDriveBase.h"
class MotorDrive : public MotorDriveBase
{
protected:
  inline int clipPWM(int pwm)
  {
    if (ABS(pwm) > 255)
      pwm = (pwm < 0) ? -255 : 255;
    return(pwm);
  }
  inline int getPWM(int pwm)
  {
    return(ABS(clipPWM(pwm)));
  }

  void setReverse(bool rev) // true==set reverse direction, false==forward
  {
    if (rev)
      {
        digitalWrite(Pin.IN2,0);
        digitalWrite(Pin.IN1,1);
      }
    else
      {
        digitalWrite(Pin.IN1,0);
        digitalWrite(Pin.IN2,1);
      }
  }
  
public:
  struct {
    // it does not seem to hurt to have IN1=IN2=1, but it doesn't seem
    // to do anything different/useful, so try to avoid this state.

    BYTE IN1, IN2;  // forward/reverse selectors.
    BYTE EN;        // enable pin
  } Pin;
  SHORT _speed;     // current speed
  SHORT _speedCmd;  // commanded speed

  float _decel; // time to allow to stop, in ms / PWM count
  BYTE _mode;
  unsigned long _doneTime;  // time when mode automatically transitions
  SHORT _deadTime;    // ms until deadman transition to emergency stop
  SHORT _maxPWM;      // clip PWM commands to this magnitude
  SHORT _startupTime; // ms of full-power pulse to start from dead stop
  SHORT _stopTime;    // ms to lock-out commands after emergency stop

  BYTE _msgCount;   // turn off diagnostics after this many messages
  
  MotorDrive(const float decel=2.0,
             const int deadTime=500,
             const int startupTime=50,
             const int stopTime=3000,
             const int maxPWM=255)
  {
    _deadTime = deadTime; // emergency stop if no command update in this time interval
    _maxPWM = maxPWM;     // don't go over this PWM level.  driver can't do it
    _startupTime = startupTime; // when starting from still, issue full power pulse this long to get motors started
    _stopTime = stopTime;  // Pause at least this long after emergency stop before restarting
    _decel = decel; // allow decel ms/speed_count to come to a full stop

    _speed = _speedCmd = 0;
    
    _msgCount = 11;  // issue this many diagnostic messages before going quiet
  }

  // in Arduino, users may set up motor drivers global,
  // but typically initialize pins in start() routine.
  // so I have a seperate begin() to set up pins
  void begin(const int en, const int in1, const int in2)
  {
    pinMode(en,OUTPUT);
    digitalWrite(en,0);   // make sure we are disabled ASAP
    Pin.EN  =en;
    Pin.IN1 =in1;
    Pin.IN2 =in2;
    pinMode(Pin.IN1,OUTPUT);
    pinMode(Pin.IN2,OUTPUT);
    
    emergencyStop();
  }

  virtual void setCommandTimeout(const int ms) { _deadTime = ms; }
  void setDecelRate(const float msPerCount) { _decel=msPerCount; }
  void setStartPulseDuration(const int ms) { _startupTime=ms; }
  void setStopTimeout(const int ms) { _stopTime=ms; }
  void showDiagnostics(const int n) { _msgCount=n; }
  void showState()
  {
    //BYTE IN1, IN2;  // forward/reverse selectors.
    //BYTE EN;        // enable pin
    //SHORT _speed;     // current speed
    //SHORT _speedCmd;  // commanded speed
    Serial.print(F("Decel "));Serial.print(_decel);Serial.println(F("ms/count"));
    //BYTE _mode;
    //unsigned long _doneTime;  // time when mode automatically transitions
    Serial.print(F("Deadman Timeout "));Serial.print(_deadTime);Serial.println(F("ms"));
  //SHORT _maxPWM;      // clip PWM commands to this magnitude
  //SHORT _startupTime; // ms of full-power pulse to start from dead stop
  //SHORT _stopTime;    // ms to lock-out commands after emergency stop

  }
  
  virtual void stop()
  {
    _speedCmd=0;
    digitalWrite(Pin.EN , 0);
    digitalWrite(Pin.IN1, 0);
    digitalWrite(Pin.IN2, 0);
    digitalWrite(Pin.EN , 1);
    int stoppingTime = (int)ABS(_speed * _decel);
if(_msgCount>0){_msgCount--;Serial.print(stoppingTime);Serial.println(" ms to stop.");}
    _doneTime = millis() + stoppingTime;
    //speed=0;  don't clobber command in case of direction change
    _mode = MOTOR_STOPPING;
  }

  virtual void emergencyStop()
  {
    Serial.print("Emergency ");
    _msgCount = 11;  // turn on diagnostics for a few commands
    stop();
    _speedCmd=0;
    _doneTime += _stopTime;
  }


  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  virtual void setSpeed(const int spdReq, long t)
  {
    BYTE prevMode = _mode;
    bool rev;
    switch(prevMode)
      {
      case MOTOR_STOPPING :
        _speedCmd = spdReq;
        if (t < _doneTime)
          {  // make sure things are stopped
            digitalWrite(Pin.IN1,0);
            digitalWrite(Pin.IN2,0);
            digitalWrite(Pin.EN ,1);
            return;
          }
        // done stoping, continue to STOP mode
        _speed = 0;
        _mode = MOTOR_STOPPED;
if(_msgCount>0){_msgCount--;Serial.println(F("stopped."));}
      case MOTOR_STOPPED :
        if (spdReq == 0) return;  // leave in full brake stop
        _mode = (spdReq < 0) ? MOTOR_START_REV : MOTOR_START_FWD;
        rev = (_mode == MOTOR_START_REV);
        digitalWrite(rev?Pin.IN1:Pin.IN2,1); // don't worry about PWM
        digitalWrite(rev?Pin.IN2:Pin.IN1,0); // this is transistional state
        digitalWrite(Pin.EN,1);   // hard kick to get started
        _doneTime = t + _startupTime;
        _speedCmd = spdReq;
if(_msgCount>0){_msgCount--;
Serial.print(F("Start "));
Serial.println(rev ? F("REV") : F("FWD"));}
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
        if (t > _doneTime) { emergencyStop(); return; } // deadman expired
        if ( (spdReq == 0)  ||  // stop or change direction
             ((spdReq < 0) && (prevMode == MOTOR_FWD)) ||
             ((spdReq > 0) && (prevMode == MOTOR_REV)) )
          {
            stop();
            _speedCmd = getPWM(spdReq);
            if (spdReq < 0) _speedCmd = -_speedCmd;
            return;
          }
        setReverse(spdReq < 0);
        _speed = _speedCmd = spdReq;
        analogWrite(Pin.EN,getPWM(_speed));
        _doneTime = t + _deadTime;
//if(_msgCount>0){_msgCount--;Serial.println(_speed);}
        return;
      case MOTOR_START_REV :
      case MOTOR_START_FWD :
        if (spdReq == 0)
          {
            _speed = 100;  // give it some time to decel, although just starting
            stop();
            return;
          }
        if ( ((spdReq < 0) && (_mode == MOTOR_START_FWD)) ||
             ((spdReq > 0) && (_mode == MOTOR_START_REV)) )
          { // direction change
            _speed = 100;  // give it some time to decel, although just starting
            stop();
            _speedCmd = spdReq;  // go to this speed after coast-down period
            return;
          }
        // same direction, but speed request change
        _speed = _speedCmd = spdReq;
        if (t >= _doneTime)
          {
            _mode = (_speedCmd > 0) ? MOTOR_FWD : MOTOR_REV;
            _doneTime = t + _deadTime;
            setReverse(_mode == MOTOR_REV);  // make sure direction is correct
            analogWrite(Pin.EN,getPWM(_speedCmd));
            if(_msgCount>0){_msgCount--;Serial.print(F("Started"));Serial.println(_speedCmd);}
          }
        return;
      }
  }

  // update state, but no new command was received
  // Check if previous command is complete,
  //   and an automatic state transition is needed
  virtual void update(long t)  // current time, from millis()
  {
//Serial.print(F("Update "));  Serial.println(t);
    if ((_doneTime > 0xfffff000) && (t < 999))
      {  // time counter must have wrapped around
        _doneTime = 0;
        Serial.println(F("Clock wrap-around"));
      }

    byte prevMode = _mode;
    switch(prevMode)
      {
      case MOTOR_STOPPING : 
      case MOTOR_STOPPED :
        if ((t > _doneTime) && _speedCmd)
          { // this was a temp stop in a direction change.  Command desired speed.
if(_msgCount>0){_msgCount--;Serial.print(F("Restart "));Serial.println(_speedCmd);}
            setSpeed(_speedCmd,t);
          }
//else Serial.println("stopped.");
        return;
      case MOTOR_FWD :
      case MOTOR_REV :
        if (t > _doneTime) emergencyStop(); // deadman expired
        return;
      case MOTOR_START_REV :
      case MOTOR_START_FWD :
        if (t > _doneTime)
          {
if(_msgCount>0){_msgCount--;Serial.println(F("moving"));}
            setSpeed(_speedCmd,t);
          }
        return;
      }
  }
};
