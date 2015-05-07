#include "allmotion.h"

// vim: tabstop=2 shiftwidth=2
///// Controller
class almController : public asynMotorController {
public:
  almController(const char *portName, const char *asynPortName, int address,
                      int numAxes, double movingPollPeriod, double idlePollPeriod);
  almAxis* getAxis(int axisNo) {
    return (almAxis*)asynMotorController::getAxis(axisNo);
  }

  /** Returns a pointer to an allmotionMotorAxis object.
    * Returns NULL if the axis number encoded in pasynUser is invalid.
    * \param[in] pasynUser asynUser structure that encodes the axis index number. */
  almAxis* getAxis(asynUser *pasynUser)
  {
    return static_cast<almAxis*>(asynMotorController::getAxis(pasynUser));
  }

  virtual ~almController() {}

  asynStatus write(almCommandPacket &output, int retries=1);
  asynStatus write(almCommandPacket *output, int retries=1);

  asynStatus writeRead(almResponsePacket &input, almCommandPacket &output, int retries=1);
  asynStatus writeRead(almResponsePacket &input, almCommandPacket *output, int retries=1);

  asynStatus writeReadInt(int &ret, almCommandPacket &output);

  asynStatus write(int axis, char *command, unsigned int operand, bool run);
  asynStatus writeRead(int axis, char *command, unsigned int operand, bool run);

  asynStatus queryParameter(int axis, const char *operand, almResponsePacket &response);
  asynStatus queryControllerParam(const char *operand, almResponsePacket &response) {
    return queryParameter(0, operand, response);
  }

  asynStatus readADC();
  asynStatus writeProgram(int number, const char *program);
  asynStatus runProgram(int number);

  int getAxisCount() { return numAxes_; }
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,
                                size_t *nActual);
  almStatus getResponseCode() { return response_code_; }

  asynStatus terminateCommand();
  asynStatus setHomeFlagPolarity(bool polarity);
  asynStatus invertInputs(unsigned int mask);
  asynStatus invertInputs(unsigned int input, bool invert);
  asynStatus resetDevice();

  asynStatus setSwitchDebounce(int periods);
  asynStatus driverPower(int drivernum, bool enabled);
  asynStatus daughterCurrent(unsigned int current);
  asynStatus daughterCurrentFlow(bool direction);

  asynStatus getInputThresholds();
  asynStatus getLimitThresholds();
  asynStatus setInputThreshold(unsigned int chan, double thresh);

  asynStatus setPotOffset(unsigned int offset);
  asynStatus setPotMul(unsigned int mul);
  asynStatus setPotDeadband(unsigned int deadband);

  asynStatus queryInputs();
  asynStatus queryVelocity();
  asynStatus queryFirmware();
  asynStatus eraseEEPROM();

  asynStatus setEncoderOuterDeadband(unsigned int counts);
  asynStatus setEncoderInnerDeadband(unsigned int counts);
  asynStatus setEncoderRatio(unsigned int ticks_per_ustep);
  asynStatus setOverloadTimeout(unsigned int moves);
  asynStatus setIntegrationPeriod(unsigned int pd);
  asynStatus setRecoveryScriptRuns(unsigned int runs);

  asynStatus setMode(unsigned int mode);
  asynStatus setSpecialMode(unsigned int mode);
  int ampsToPercent(double amps);

  virtual asynStatus queryPositions();
  virtual asynStatus queryVelocities();
  virtual asynStatus queryEncoders();

protected:
  virtual almCommandPacket *getCommandPacket();
  virtual void initCommandPacket(almCommandPacket &packet);
  virtual asynStatus poll();

  int param_kill_all_;
#define FIRST_ALM_PARAM param_kill_all_
  int param_error_;

  int param_adc_[ALM_INPUT_COUNT];

  int param_prog_[ALM_PROG_COUNT];
  int param_prog_idx_;
  int param_prog_write_;

  int param_prog_run_;

  int param_read_adc_;
  int param_read_inp_;
  int param_read_thresh_;
  int param_read_limit_thresh_;
  int param_mode_;
  int param_sp_mode_;

  int param_inp_sw1_;
  int param_inp_sw2_;
  int param_inp_opto1_;
  int param_inp_opto2_;
  int param_inp_cha_;
  int param_inp_chb_;
  int param_inp_idx_;

  int param_firmware_;
  int param_microsteps_;
  int param_microstep_tweak_;
  int param_max_amps_;

  int param_driver1_power_;
  int param_driver2_power_;
  int param_switch_debounce_;
  int param_daughter_current_;
  int param_daughter_cur_flow_;
  int param_input_threshold_[ALM_INPUT_COUNT];

  int param_pot_offset_;
  int param_pot_mul_;
  int param_pot_deadband_;

  int param_enc_[ALM_ENC_COUNT];

  int param_enc_outer_db_;
  int param_enc_inner_db_;
  int param_enc_ratio_;
  int param_overload_timeout_;
  int param_int_period_;
  int param_recovery_runs_;

  int param_home_polarity_;
  int param_reset_;
  int param_invert_input_[ALM_INPUT_COUNT];
  int param_limit_invert_;
  int param_limit_thresh_[2];

  int param_hold_i_;
  int param_move_i_;
#define LAST_ALM_PARAM param_move_i_
#define NUM_ALM_PARAMS (&LAST_ALM_PARAM - &FIRST_ALM_PARAM + 1)
  double timeout_;

  double adc_[ALM_INPUT_COUNT];
  double thresholds_[ALM_INPUT_COUNT];

  int positions_[ALM_AXES];
  int velocities_[ALM_AXES];
  int encoder_positions_[ALM_AXES];
  int address_;

  almStatus response_code_;
  bool ready_;
  asynUser *pasynUser_;

private:
  friend class almAxis;

  asynStatus runWrite(almCommandPacket &packet);

};
