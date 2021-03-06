// Generic interface for "standard" H-Bridge motor driver

class MotorDriveBase
{
  virtual void stop() = 0;
  virtual void emergencyStop() = 0;
  virtual void setCommandTimeout(int ms) = 0;

  
  // Set speed -MAX_PWM for max reverse, MAX_PWM for max forward
  inline void setSpeed(const int spdReq)
  {
    setSpeed(spdReq,millis());
  }

  // many (all?) drivers may have some mode transition times and pauses.
  // use these to indicate current time, to avoid repeated calls to millis()
  virtual void setSpeed(const int spd, long t) = 0;  // pass in current time
  virtual void update(long t) = 0;  // check if some sort of state change needs to be processed
};
