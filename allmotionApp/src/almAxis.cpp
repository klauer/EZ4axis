/*
 * AllMotion EZ-series axis
 *
 * */
// vim: tabstop=2 shiftwidth=2
#include "allmotion.h"

/** Creates a new almAxis object.
  * \param[in] controller         The allmotion controller
  * \param[in] axis_num           The axis number
  */
almAxis::almAxis(almController *controller, int axis_num)
  :  asynMotorAxis((asynMotorController*)controller, axis_num)
{
  pc_ = controller;
  encoder_pos_ = 0.0;
  //param_num_ = -1;
  moving_ = false;
  has_encoder_ = false;

  getMicrosteps();
  motionFinished();
}

asynStatus almAxis::poll(bool *moving) {
  // & only responds to base device ID request
  almResponsePacket response;

  pc_->lock();

  // Controllers poll the positions at the same time and store them
  // all together.
  double pos, vel, enc;
  enc = pc_->encoder_positions_[axisNo_];
  pos = pc_->positions_[axisNo_];
  vel = pc_->velocities_[axisNo_];

  setDoubleParam(pc_->motorEncoderPosition_, enc);
  setDoubleParam(pc_->motorPosition_, pos);
  setDoubleParam(pc_->motorVelocity_, vel);

  queryStatus();

  queryLimits();

  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
            "Axis %d Position: %f (encoder %f) velocity: %f moving: %s\n",
            axisNo_, pos, enc, vel, (moving_ ? "yes" : "no"));

  pc_->unlock();

  *moving = moving_;
  callParamCallbacks();
  return asynSuccess;
}

asynStatus almAxis::getIntegerParam(int param, epicsInt32 *value) {
  return pc_->getIntegerParam(axisNo_, param, value);
}

asynStatus almAxis::getDoubleParam(int param, epicsFloat64 *value) {
  return pc_->getDoubleParam(axisNo_, param, value);
}

asynStatus almAxis::setPosition(double pos_) {
  epicsInt32 pos = (int)pos_;

  almCommandPacket command;
  pc_->initCommandPacket(command);

  command.select_axis(axisNo_);
  command.set_position(pos);
  asynStatus ret = pc_->runWrite(command);
  // command.dump();
  return ret;

}

asynStatus almAxis::queryLimits() {
  int lim[2];
  epicsInt32 invert;

  almResponsePacket response;
  almCommandPacket command;
  pc_->initCommandPacket(command);

  command.query_limits(axisNo_);

  if (pc_->writeRead(response, command) == asynSuccess) {
    sscanf((const char*)response.get_buffer(), "%d,%d",
           &lim[0], &lim[1]);

    getIntegerParam(pc_->param_limit_invert_, &invert);

    for (int i=0; i < 2; i++) {
      getDoubleParam(pc_->param_limit_thresh_[i], &limit_threshold_[i]);

      limit_adc_[i] = adc_to_volts(lim[i]);
      limits_[i] = (limit_adc_[i] < limit_threshold_[i]);

      if (invert) {
        limits_[i] = !limits_[i];
      }

      if (i == ALM_LIM_HIGH) {
        setIntegerParam(pc_->motorStatusHighLimit_, limits_[i]);
      } else {
        setIntegerParam(pc_->motorStatusLowLimit_, limits_[i]);
      }
    }
    // printf("axis %d limit adc %f %f\n", axisNo_, limit_adc_[0], limit_adc_[1]);
  }

  return asynSuccess;
}

asynStatus almAxis::queryStatus() {
  almResponsePacket response;
  almCommandPacket command;
  pc_->initCommandPacket(command);

  command.select_axis(axisNo_);
  command.append(ALM_QUERY_STATUS);

  if (pc_->writeRead(response, command) == asynSuccess) {
    if (moving_ && response.is_ready()) {
      motionFinished();
    }
  } else {
    // Overflow error means a "blocking" move command was executed,
    // (A0,0,0,0 would be "non-blocking" versus aM1A0 would be blocking)
    // and so some commands will just not work
    if (response.get_status() != ALM_OVERFLOW_ERROR) {
      asynPrint(pc_->pasynUser_, ASYN_TRACE_ERROR,
        "%s:%s: axis %d status check failed (%d) %s\n",
        driverName, __func__, axisNo_, response.get_status(),
        get_allmotion_error_string(response.get_status()));
    }
  }

  return asynSuccess;
}

void almAxis::motionFinished() {
  setIntegerParam(pc_->motorStatusMoving_, 0);
  setIntegerParam(pc_->motorStatusDone_, 1);
  moving_ = false;
}

asynStatus almAxis::terminateCommand() {
  almCommandPacket command;
  pc_->initCommandPacket(command);

  command.select_axis(axisNo_);
  command.terminate();
  asynStatus ret = pc_->runWrite(command);

  if (ret == asynError) {
    asynPrint(pc_->pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: axis %d terminate failed\n",
      driverName, __func__, axisNo_);
  }
  return ret;
}

asynStatus almAxis::stop(double acceleration)
{
  return terminateCommand();
}

asynStatus almAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards) {
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW | ASYN_TRACE_ERROR,
    "%s:%s: axis %d: home (forwards=%d)\n",
    driverName, __func__, axisNo_, forwards);

  almCommandPacket command;
  pc_->initCommandPacket(command);
  command.select_axis(axisNo_);
  command.home((forwards) ? home_counts_ : -home_counts_);
  command.run();

  asynStatus ret = pc_->write(command);
  if (ret == asynSuccess)
    moving_ = true;

  return ret;
}

int almAxis::position_to_counts(double position) {
  double res;
  pc_->getDoubleParam(axisNo_, pc_->motorResolution_, &res);

  if (res < 1e-10)
    return 0.0;
  else
    return position / res;
}

double almAxis::counts_to_position(int counts) {
  double res;
  pc_->getDoubleParam(axisNo_, pc_->motorResolution_, &res);
  return res * counts;
}

asynStatus almAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW | ASYN_TRACE_ERROR,
    "%s:%s: axis %d: move to %g (relative=%d)\n",
    driverName, __func__, axisNo_,
    position, relative);

  // Set slew speed
  //command.append("V%d", (int)max_velocity);
  //
  // Acceleration is in microsteps/sec^2
  //  = L * (400,000,000 / 65536)
  //
  // if L=1, at t=16.384, V=100,000
  // t = V / a
  //
  //command.set_accel((int)acceleration);

  almCommandPacket command;
  pc_->initCommandPacket(command);

  command.set_velocity(axisNo_, max_velocity);
  asynStatus ret = pc_->runWrite(command);

  pc_->initCommandPacket(command);
  command.move(axisNo_, position, (relative != 0));
  ret = pc_->runWrite(command);

  command.dump();
  if (ret == asynSuccess)
    moving_ = true;

  return ret;

}

asynStatus almAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration) {
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: jog with max velocity %g accel %g\n",
    driverName, __func__, axisNo_,
    max_velocity, acceleration);

  almCommandPacket command;
  pc_->initCommandPacket(command);
  command.select_axis(axisNo_);

  // Set slew speed
  //command.append("V%d", abs(max_velocity));
  if (max_velocity > 0)
    command.append("P0");
  else
    command.append("D0");

  asynStatus ret = pc_->runWrite(command);

  if (ret == asynSuccess)
    moving_ = true;

  return ret;
}


const char* get_allmotion_error_string(almStatus error) {
  switch (error) {
  case ALM_NO_ERROR:           return "NO_ERROR";
  case ALM_INIT_ERROR:         return "INIT_ERROR";
  case ALM_BAD_COMMAND:        return "BAD_COMMAND";
  case ALM_BAD_OPERAND:        return "BAD_OPERAND";
  case ALM_ERROR_4:            return "ERROR_4";
  case ALM_COMMS_ERROR:        return "COMMS_ERROR";
  case ALM_ERROR_6:            return "ERROR_6";
  case ALM_NOT_INIT_ERROR:     return "NOT_INIT_ERROR";
  case ALM_ERROR_8:            return "ERROR_8";
  case ALM_OVERLOAD_ERROR:     return "OVERLOAD_ERROR";
  case ALM_ERROR_10:           return "ERROR_10";
  case ALM_NOT_ALLOWED_ERROR:  return "NOT_ALLOWED_ERROR";
  case ALM_ERROR_12:           return "ERROR_12";
  case ALM_ERROR_13:           return "ERROR_13";
  case ALM_ERROR_14:           return "ERROR_14";
  case ALM_OVERFLOW_ERROR:     return "OVERFLOW_ERROR";
  default:
    return "Unknown error";
  }
}

asynStatus almAxis::setUIntDigitalParam(int index, epicsUInt32 value) {
  return pc_->setUIntDigitalParam(axisNo_, index, value, 0xffffffff);
}

asynStatus almAxis::setHoldCurrent(double amps) {
  almCommandPacket command;
  pc_->initCommandPacket(command);
  command.set_hold_current(axisNo_, pc_->ampsToPercent(amps));
  return pc_->runWrite(command);
}

asynStatus almAxis::setMoveCurrent(double amps) {
  almCommandPacket command;
  pc_->initCommandPacket(command);
  command.set_move_current(axisNo_, pc_->ampsToPercent(amps));
  return pc_->runWrite(command);
}

asynStatus almAxis::setClosedLoop(bool closed) {
  epicsUInt32 mode;
  pc_->getUIntDigitalParam(pc_->param_mode_, &mode, 0xffffff);

  // Get the current mode and wipe out both open/closed
  // loop settings first
  mode &= ~ALM_MODE_OPEN_LOOP;
  mode &= ~ALM_MODE_CLOSED_LOOP;

  if (closed)
    mode |= ALM_MODE_CLOSED_LOOP;
  else
    mode |= ALM_MODE_OPEN_LOOP;

  return pc_->setMode(mode);
}

asynStatus almAxis::setMicrosteps(unsigned int microsteps) {
  almCommandPacket command;
  pc_->initCommandPacket(command);
  command.set_microsteps(axisNo_, microsteps);

  asynStatus ret = pc_->runWrite(command);

  getMicrosteps();
  return ret;
}

asynStatus almAxis::setMicrostepTweak(unsigned int size) {
  almCommandPacket command;
  pc_->initCommandPacket(command);
  command.select_axis(axisNo_);
  command.set_microstep_tweak(size);
  return pc_->runWrite(command);
}

asynStatus almAxis::getMicrosteps() {
  almResponsePacket response;
  if (queryParameter(ALM_QUERY_MICROSTEPS, response) == asynSuccess) {
    setIntegerParam(pc_->param_microsteps_, response.as_int());
    return asynSuccess;
  }
  return asynError;
}

asynStatus almAxis::queryParameter(const char *operand, almResponsePacket &response) {
  return pc_->queryParameter(axisNo_, operand, response);
}

asynStatus almAxis::setLowLimitThreshold(epicsFloat64 value) {
  almCommandPacket command;
  int int_thresh = volts_to_adc(value);

  pc_->initCommandPacket(command);
  command.low_limit_threshold(axisNo_, int_thresh);
  return pc_->runWrite(command);
}

asynStatus almAxis::setHighLimitThreshold(epicsFloat64 value) {
  almCommandPacket command;
  int int_thresh = volts_to_adc(value);

  pc_->initCommandPacket(command);
  command.high_limit_threshold(axisNo_, int_thresh);
  return pc_->runWrite(command);
}

asynStatus almAxis::setLimitPolarity(bool inverted) {
  almCommandPacket command;
  pc_->initCommandPacket(command);

  command.select_axis(axisNo_);
  command.set_limit_polarity(inverted);
  return pc_->runWrite(command);
}
