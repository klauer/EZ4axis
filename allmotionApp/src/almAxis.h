// vim: tabstop=2 shiftwidth=2

#include "allmotion.h"

///// Axis
class almAxis : public asynMotorAxis
{
public:
  almAxis(almController *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);

  asynStatus setClosedLoop(bool closed);

  asynStatus setPosition(double position);
  asynStatus setUIntDigitalParam(int index, epicsUInt32 value);

  asynStatus setHoldCurrent(double value);
  asynStatus setMoveCurrent(double value);

  asynStatus setMicrosteps(unsigned int microsteps);
  asynStatus setMicrostepTweak(unsigned int size);

  asynStatus setLowLimitThreshold(epicsFloat64 value);
  asynStatus setHighLimitThreshold(epicsFloat64 value);

  asynStatus setLimitPolarity(bool inverted);

  asynStatus getMicrosteps();

  asynStatus queryParameter(const char *operand, almResponsePacket &response);

  /* And these are specific to this class: */
  asynStatus setServo(bool enabled);
  asynStatus queryStatus();
  asynStatus queryLimits();
  asynStatus terminateCommand();
  bool checkMoving();

  int position_to_counts(double position);
  double counts_to_position(int counts);

  inline bool isFlagSet(unsigned int flag) { return (flags_ & flag) == flag; }
  inline void setFlag(unsigned int flag)   { flags_ |= flag; }
  inline void clearFlag(unsigned int flag) { flags_ &= ~flag; }
  inline void setFlag(unsigned int flag, bool set) {
    if (set) {
      setFlag(flag);
    } else {
      clearFlag(flag);
    }
  }

protected:
  virtual asynStatus getIntegerParam(int param, epicsInt32 *value);
  virtual asynStatus getDoubleParam(int param, epicsFloat64 *value);

private:
  void motionFinished();

  friend class almController;
  almController *pc_;    /**< Pointer to the asynMotorController to which this axis belongs.
                          *   Abbreviated because it is used very frequently */
  double encoder_pos_;   /** < Cached copy of the encoder position */
  unsigned int flags_;   /** < Cached copy of the current flags */

  bool home_counts_;     // Maximum number of counts to use in a homing move

  bool has_encoder_;
  bool moving_;
  bool errored_;

  double limit_adc_[2];
  double limit_threshold_[2];
  bool limits_[2];

  int axis_num_;         // ez4axis axis number (1-based)
  int status_;
};


