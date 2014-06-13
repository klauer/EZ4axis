/*
 * AllMotion controller
 * */

// vim: tabstop=2 shiftwidth=2
#include "allmotion.h"

const char *ALM_PSTR_PROG[] = {
  ALM_PSTR_PROG_0,
  ALM_PSTR_PROG_1,
  ALM_PSTR_PROG_2,
  ALM_PSTR_PROG_3,
  ALM_PSTR_PROG_4,
  ALM_PSTR_PROG_5,
  ALM_PSTR_PROG_6,
  ALM_PSTR_PROG_7,
  ALM_PSTR_PROG_8,
  ALM_PSTR_PROG_9,
  ALM_PSTR_PROG_10,
  ALM_PSTR_PROG_11,
  ALM_PSTR_PROG_12,
  ALM_PSTR_PROG_13,
  ALM_PSTR_PROG_14,
  ALM_PSTR_PROG_15
};

const char *ALM_PSTR_THRESH[] = {
  ALM_PSTR_INPUT_THRESHOLD0,
  ALM_PSTR_INPUT_THRESHOLD1,
  ALM_PSTR_INPUT_THRESHOLD2,
  ALM_PSTR_INPUT_THRESHOLD3
};

/** Creates a new almController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPort          The name of the drvAsynIPPPort that was created previously to connect to the controller
  * \param[in] address           The address of the device on the RS485 bus
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
almController::almController(const char *portName, const char *asynPortName, int address, 
                             int numAxes, double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_ALM_PARAMS,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int i;
  address_ = address;
  idlePollPeriod_ = idlePollPeriod;
  movingPollPeriod_ = movingPollPeriod;

  for (i=0; i < ALM_AXES; i++) {
    positions_[i] = 0;
    velocities_[i] = 0;
    encoder_positions_[i] = 0;
  }
  
  for (i=0; i < ALM_ADC_COUNT; i++) {
    thresholds_[i] = 0;
    adc_[i] = 0.0;
  }

  queryRate_ = 1;
  ready_ = true;

  // TODO: timeout being too slow could eventually be an issue
  // in writeRead() need to recv() while checking for ALM_OEM_START/ALM_OEM_STOP
  // with a lower timeout
  timeout_ = 0.1;
  response_code_ = ALM_NO_ERROR;

  if (!addToList(portName, this)) {
    printf("%s:%s: Init failed", driverName, portName);
    return;
  }

  // Write-only
  createParam(ALM_PSTR_KILL_ALL          ,    asynParamInt32,  &param_kill_all_);
  createParam(ALM_PSTR_READ_ADC          ,    asynParamInt32,  &param_read_adc_);
  createParam(ALM_PSTR_READ_INP          ,    asynParamInt32,  &param_read_inp_);
  createParam(ALM_PSTR_READ_THRESH       ,    asynParamInt32,  &param_read_thresh_);
  createParam(ALM_PSTR_PROG_RUN          ,    asynParamInt32,  &param_prog_run_);
  
  for (i=0; i < ALM_PROG_COUNT; i++) {
    createParam(ALM_PSTR_PROG[i], asynParamOctet, &param_prog_[i]);
  }
  
  createParam(ALM_PSTR_PROG_IDX          ,    asynParamInt32, &param_prog_idx_);
  createParam(ALM_PSTR_PROG_WRITE        ,    asynParamOctet, &param_prog_write_);

  createParam(ALM_PSTR_MODE              , asynParamUInt32Digital,  &param_mode_);
  createParam(ALM_PSTR_SP_MODE           , asynParamUInt32Digital,  &param_sp_mode_);

  createParam(ALM_PSTR_DRIVER1_POWER     ,    asynParamInt32, &param_driver1_power_);
  createParam(ALM_PSTR_DRIVER2_POWER     ,    asynParamInt32, &param_driver2_power_);
  createParam(ALM_PSTR_SWITCH_DEBOUNCE   ,    asynParamInt32, &param_switch_debounce_);
  createParam(ALM_PSTR_DAUGHTER_CURRENT  ,    asynParamInt32, &param_daughter_current_);
  createParam(ALM_PSTR_DAUGHTER_CUR_FLOW ,    asynParamInt32, &param_daughter_cur_flow_);

  createParam(ALM_PSTR_POT_OFFSET        ,    asynParamInt32, &param_pot_offset_);
  createParam(ALM_PSTR_POT_MUL           ,    asynParamInt32, &param_pot_mul_);
  createParam(ALM_PSTR_POT_DEADBAND      ,    asynParamInt32, &param_pot_deadband_);

  createParam(ALM_PSTR_ENC_OUTER_DB      ,    asynParamInt32, &param_enc_outer_db_);
  createParam(ALM_PSTR_ENC_INNER_DB      ,    asynParamInt32, &param_enc_inner_db_);
  createParam(ALM_PSTR_ENC_RATIO         ,    asynParamInt32, &param_enc_ratio_);
  createParam(ALM_PSTR_OVERLOAD_TIMEOUT  ,    asynParamInt32, &param_overload_timeout_);
  createParam(ALM_PSTR_INT_PERIOD        ,    asynParamInt32, &param_int_period_);
  createParam(ALM_PSTR_RECOVERY_RUNS     ,    asynParamInt32, &param_recovery_runs_);

  createParam(ALM_PSTR_HOME_POLARITY     ,    asynParamInt32, &param_home_polarity_);
  createParam(ALM_PSTR_RESET             ,    asynParamInt32, &param_reset_);
  createParam(ALM_PSTR_INVERT_INPUT0     ,    asynParamInt32, &param_invert_input_[0]);
  createParam(ALM_PSTR_INVERT_INPUT1     ,    asynParamInt32, &param_invert_input_[1]);
  createParam(ALM_PSTR_INVERT_INPUT2     ,    asynParamInt32, &param_invert_input_[2]);
  createParam(ALM_PSTR_INVERT_INPUT3     ,    asynParamInt32, &param_invert_input_[3]);

  createParam(ALM_PSTR_MICROSTEP_TWEAK   ,    asynParamInt32, &param_microstep_tweak_);

  // Read-only
  createParam(ALM_PSTR_ADC_1             ,  asynParamFloat64,  &param_adc_[0]);
  createParam(ALM_PSTR_ADC_2             ,  asynParamFloat64,  &param_adc_[1]);
  createParam(ALM_PSTR_ADC_3             ,  asynParamFloat64,  &param_adc_[2]);
  createParam(ALM_PSTR_ADC_4             ,  asynParamFloat64,  &param_adc_[3]);

  createParam(ALM_PSTR_ENC_1             ,    asynParamInt32,  &param_enc_[0]);
  createParam(ALM_PSTR_ENC_2             ,    asynParamInt32,  &param_enc_[1]);

  createParam(ALM_PSTR_INP_SW1           ,    asynParamInt32, &param_inp_sw1_);
  createParam(ALM_PSTR_INP_SW2           ,    asynParamInt32, &param_inp_sw2_);
  createParam(ALM_PSTR_INP_OPTO1         ,    asynParamInt32, &param_inp_opto1_);
  createParam(ALM_PSTR_INP_OPTO2         ,    asynParamInt32, &param_inp_opto2_);
  createParam(ALM_PSTR_INP_CHA           ,    asynParamInt32, &param_inp_cha_);
  createParam(ALM_PSTR_INP_CHB           ,    asynParamInt32, &param_inp_chb_);
  createParam(ALM_PSTR_INP_IDX           ,    asynParamInt32, &param_inp_idx_);
                                                                                    
  createParam(ALM_PSTR_FIRMWARE          ,    asynParamOctet, &param_firmware_);
  createParam(ALM_PSTR_ERROR             ,    asynParamOctet, &param_error_);

  // Read-write
  createParam(ALM_PSTR_MICROSTEPS        ,    asynParamInt32, &param_microsteps_);

  for (i=0; i < ALM_INPUT_COUNT; i++) {
    createParam(ALM_PSTR_THRESH[i],  asynParamFloat64, &param_input_threshold_[i]);
  }

  // Read-write
  createParam(ALM_PSTR_HOLD_I    ,    asynParamInt32,  &param_hold_i_);
  createParam(ALM_PSTR_MOVE_I    ,    asynParamInt32,  &param_move_i_);

#if 0
  const char *pname;
  for (i=0; i < 100; i++) {
    getParamName(i, &pname);
    printf("%d %s\n", i, pname);
  }
#endif

  setStringParam(param_error_, "");
  setIntegerParam(motorStatusHasEncoder_, 1);

  /* Connect to the AllMotion controller */
  asynStatus status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUser_, NULL);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: cannot connect to AllMotion controller\n",
      driverName, __func__);
  }

  status = pasynOctetSyncIO->setInputEos(pasynUser_, ALM_INPUT_EOS, 1);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set input EOS on %s: %s\n",
      __func__, asynPortName, pasynUser_->errorMessage);
  }

  status = pasynOctetSyncIO->setOutputEos(pasynUser_, ALM_OUTPUT_EOS, 1);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set output EOS on %s: %s\n",
      __func__, asynPortName, pasynUser_->errorMessage);
  }

  // Create the axis objects
  for (int axis=0; axis<numAxes; axis++) {
    new almAxis(this, axis);
  }

  startPoller(movingPollPeriod/1000., idlePollPeriod/1000., 2);

  queryFirmware();
  queryInputs();

  getInputThresholds();
}

asynStatus almController::queryPositions() {
  almResponsePacket response;
  asynStatus status = asynSuccess;
  for (int axis=0; axis < numAxes_; axis++) {
    if (queryParameter(axis, ALM_QUERY_POS, response) == asynSuccess)
      positions_[axis] = response.as_int();
    else
      status = asynError;
  }
  return status;
}

asynStatus almController::queryVelocities() {
  almResponsePacket response;
  asynStatus status = asynSuccess;
  for (int axis=0; axis<numAxes_; axis++) {
    if (queryParameter(axis, ALM_QUERY_VELOCITY, response) == asynSuccess)
      velocities_[axis] = response.as_int();
    else
      status = asynError;
  }
  return status;
}


asynStatus almController::queryEncoders() {
  almResponsePacket response;
  asynStatus status = asynSuccess;

  if (queryControllerParam(ALM_QUERY_ENC_1, response) == asynSuccess) {
    encoder_positions_[0] = response.as_int();
    encoder_positions_[2] = response.as_int();
  } else {
    status = asynError;
  }

  if (queryControllerParam(ALM_QUERY_ENC_2, response) == asynSuccess) {
    encoder_positions_[1] = response.as_int();
    encoder_positions_[3] = response.as_int();
  } else {
    status = asynError;
  }

  return status;
}

asynStatus almController::poll()
{
  asynStatus status = asynSuccess;
  asynStatus st;

  if ((st = queryPositions()) != asynSuccess)
    status = st;
  if ((st = queryVelocities()) != asynSuccess)
    status = st;
  if ((st = queryEncoders()) != asynSuccess)
    status = st;

  return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  *
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus almController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  almAxis *pAxis = getAxis(pasynUser);
  const char *paramName = "(unset)";

  if (!pAxis) 
    return asynError;

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setDoubleParam(pAxis->axisNo_, function, value);

#if DEBUG
  printf("Param %s value %g\n", paramName, value);
#endif

  if (function == param_input_threshold_[0]) {
    status = setInputThreshold(0, value);
  } else if (function == param_input_threshold_[1]) {
    status = setInputThreshold(1, value);
  } else if (function == param_input_threshold_[2]) {
    status = setInputThreshold(2, value);
  } else if (function == param_input_threshold_[3]) {
    status = setInputThreshold(3, value);
  } else {
    /* Call base class method */
    status = asynMotorController::writeFloat64(pasynUser, value);
  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%s (%d), value=%f\n", 
        driverName, __func__, status, paramName, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%s (%d), value=%f\n", 
        driverName, __func__, paramName, function, value);
  return status;
}

/** Called when asyn clients call pasynUInt32Digital->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  *
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus almController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  almAxis *pAxis = getAxis(pasynUser);
  const char *paramName = "(unset)";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setUIntDigitalParam(pAxis->axisNo_, function, value, mask);

#if DEBUG
  printf("%s:%s: mask=%x function=%s (%d), value=%d\n", 
        driverName, __func__, mask, paramName, function, value);
#endif

  if (param_mode_) {
    status = setMode(value);
  } else if (param_sp_mode_) {
    status = setSpecialMode(value);
  } else {
    /* Call base class method */
    status = asynMotorController::writeUInt32Digital(pasynUser, value, mask);
  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%s (%d), value=%d\n", 
        driverName, __func__, status, paramName, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%s (%d), value=%d\n", 
        driverName, __func__, paramName, function, value);
  return status;
}

/** Called when asyn clients call pasynOctet->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  *
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus almController::writeOctet(asynUser *pasynUser, const char *value, 
                                     size_t nChars, size_t *nActual)
{
  int addr=0;
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *paramName = "(unset)";
  bool handled=false;
  int i;

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

#if DEBUG
  printf("%s:%s: function=%s (%d), value=%s\n", 
        driverName, __func__, paramName, function, value);
#endif

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) 
    return status;

  /* Set the parameter in the parameter library. */
  status = (asynStatus)setStringParam(addr, function, (char *)value);
  *nActual = nChars;

  for (i=0; i < ALM_PROG_COUNT; i++) {
    if (function == param_prog_[i]) {
      status = writeProgram(i, value);
      handled = true;
    }
  }
  
  if (!handled) {
    if (param_prog_write_) {
      getIntegerParam(param_prog_idx_, &i);

      asynPrint(pasynUser, ASYN_TRACE_FLOW, 
            "%s:%s: write program %d value=%s\n", 
            driverName, __func__, i, value);
      
      if (0 <= i && i < ALM_PROG_COUNT) {
        setStringParam(addr, param_prog_[i], (char *)value);
        status = writeProgram(i, value);
      }
    } else {
      return asynPortDriver::writeOctet(pasynUser, value, nChars, nActual);
    }
  }

  if (status) 
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                "%s:%s: status=%d, function=%d, value=%s", 
                driverName, __func__, status, function, value);
  else        
      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
            "%s:%s: function=%d, value=%s\n", 
            driverName, __func__, function, value);

  return status;
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  *
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus almController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  almAxis *pAxis = getAxis(pasynUser);
  const char *paramName = "(unset)";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

#if DEBUG
  printf("%s:%s: function=%s (%d), value=%d\n", 
        driverName, __func__, paramName, function, value);
#endif

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);
  if (function == param_kill_all_) {
    terminateCommand();
  } else if (function == param_hold_i_) {
    //printf("Set hold current %d = %d\n", pAxis->axisNo_, value);
    status = pAxis->setHoldCurrent(value);
  } else if (function == param_move_i_) {
    //printf("Set move current %d = %d\n", pAxis->axisNo_, value);
    status = pAxis->setMoveCurrent(value);
  } else if (function == param_read_adc_) {
    status = readADC();
  } else if (function == param_read_inp_) {
    status = queryInputs();
  } else if (function == param_read_thresh_) {
    status = getInputThresholds();
  } else if (function == param_prog_run_) {
    status = runProgram(value);
  } else if (function == param_reset_) {
    status = resetDevice();
  } else if (function == param_driver1_power_) {
    status = driverPower(1, value);
  } else if (function == param_driver2_power_) {
    status = driverPower(2, value);
  } else if (function == param_switch_debounce_) {
    status = setSwitchDebounce(value);
  } else if (function == param_daughter_current_) {
    status = daughterCurrent(value);
  } else if (function == param_daughter_cur_flow_) {
    status = daughterCurrentFlow((bool)value);
  } else if (function == param_pot_offset_) {
    status = setPotOffset(value);
  } else if (function == param_pot_mul_) {
    status = setPotMul(value);
  } else if (function == param_pot_deadband_) {
    status = setPotDeadband(value);
  } else if (function == param_enc_outer_db_) {
    status = setEncoderOuterDeadband(value);
  } else if (function == param_enc_inner_db_) {
    status = setEncoderInnerDeadband(value);
  } else if (function == param_enc_ratio_) {
    status = setEncoderRatio(value);
  } else if (function == param_overload_timeout_) {
    status = setOverloadTimeout(value);
  } else if (function == param_int_period_) {
    status = setIntegrationPeriod(value);
  } else if (function == param_recovery_runs_) {
    status = setRecoveryScriptRuns(value);
  } else if (function == param_microsteps_) {
    status = pAxis->setMicrosteps(value);
  } else if (function == param_microstep_tweak_) {
    status = pAxis->setMicrostepTweak(value);
  } else if (function == param_invert_input_[0]) {
    status = invertInputs(0, value);
  } else if (function == param_invert_input_[1]) {
    status = invertInputs(1, value);
  } else if (function == param_invert_input_[2]) {
    status = invertInputs(2, value);
  } else if (function == param_invert_input_[3]) {
    status = invertInputs(3, value);
  } else {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%s (%d), value=%d\n", 
        driverName, __func__, status, paramName, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%s (%d), value=%d\n", 
        driverName, __func__, paramName, function, value);
  return status;
}

asynStatus almController::queryParameter(int axis, const char *operand, almResponsePacket &response) {
  almCommandPacket command;
  initCommandPacket(command);
  command.select_axis(axis);

  if (operand[0] == '&') {
    command.append(operand[0]);
  } else {
    command.append("?%s", operand);
  }
  
  //command.dump();

  asynStatus ret = writeRead(response, command);
  return ret;
}

asynStatus almController::terminateCommand() {
  almCommandPacket command;
  initCommandPacket(command);
  
  command.terminate();
  asynStatus ret = runWrite(command);

  if (ret == asynError) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: controller terminate failed\n",
      driverName, __func__);
  }
  return ret;
}


asynStatus almController::writeReadInt(int &ret, almCommandPacket &command) {
  almResponsePacket response;

  asynStatus status = writeRead(response, command);
  if (status != asynSuccess) {
    return status;
  }
  
  ret = response.as_int();
  return asynSuccess;
}

asynStatus almController::writeRead(almResponsePacket &input, almCommandPacket *command, int retries) {
  if (!command)
    return asynError;
  return writeRead(input, *command, retries);
}

asynStatus almController::writeRead(almResponsePacket &input, almCommandPacket &command, int retries) {
  size_t nwrite;
  size_t nread;
  asynStatus status;
  int eomReason;
  char buf[ALM_STRING_LEN];
  
  if (!command.finish()) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
        "%s failed: finish packet failed\n",
        __func__);
    return asynError;
  }

  input.invalidate();

  lock();

  command.dump(pasynUser_, ASYN_TRACEIO_DRIVER);

  status = pasynOctetSyncIO->writeRead(pasynUser_,
                                       (const char*)command.get_buffer(), command.length(),
                                       buf, ALM_STRING_LEN,
                                       timeout_, &nwrite, &nread, &eomReason);

  unlock();

  if (nread > 0) {
    input.received((const byte*)buf, nread);
    if (nread > 0) {
      input.dump(pasynUser_, ASYN_TRACEIO_DEVICE);
    }
  }

  if (!input.is_valid()) {
    if (retries > 0) {
      asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
        "%s:%s: retransmit\n",
        driverName, __func__);

      command.set_repeat();
      return writeRead(input, command, retries - 1);
    }

    asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER|ASYN_TRACE_ERROR,
        "%s failed: no confirmation from device (eomReason=%d) (nread=%d)\n",
        __func__, eomReason, (int)nread);

    status = asynError;
  } else {
    if (input.get_status() != ALM_NO_ERROR) {
      almStatus code = input.get_status();
      asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER|ASYN_TRACE_ERROR,
          "%s failed (AllMotion code=%d [%s] eomReason=%d nread=%d)\n",
          __func__, code, get_allmotion_error_string(code),
          eomReason, (int)nread);

      setStringParam(param_error_, get_allmotion_error_string(code));
      status = asynError;
    } else {
      status = asynSuccess;
      asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
        "%s:%s: %s OK\n",
        driverName, __func__, command.get_buffer());
    }
  }
 
  if (input.is_valid()) {
    input.dump(pasynUser_, ASYN_TRACEIO_DEVICE);

    response_code_ = input.get_status();
    ready_ = input.is_ready();
  } else {
    asynPrint(pasynUser_, ASYN_TRACEIO_DEVICE|ASYN_TRACE_ERROR,
              "Invalid response");
    setStringParam(param_error_, "Invalid response");
    input.dump(pasynUser_, ASYN_TRACEIO_DEVICE|ASYN_TRACE_ERROR);
  }
  return status;
}

asynStatus almController::write(almCommandPacket &command, int retries) {
  almResponsePacket response;
  return writeRead(response, command, retries);
}

asynStatus almController::write(almCommandPacket *command, int retries) {
  almResponsePacket response;
  if (!command)
    return asynError;

  return writeRead(response, command, retries);
}

almCommandPacket *almController::getCommandPacket() {
  return new almCommandPacket(address_);
}

void almController::initCommandPacket(almCommandPacket &packet) {
  packet.clear();
  packet.start(address_);
}

double adc_to_volts(int value) {
  // 0 to 16368 -> 0.0 to 3.3V
  return 3.3 * ((double)value / 16368.0);
}

unsigned int volts_to_adc(double volts) {
  // 0 to 16368 <- 0.0 to 3.3V
  return (unsigned int)((volts / 3.3) * 16368.0);
}

asynStatus almController::readADC() {
  almResponsePacket response;
  almCommandPacket command;
  initCommandPacket(command);
  
  command.read_adc();

  asynStatus status = writeRead(response, command);
  if (status == asynSuccess) {
    int adc_int[ALM_ADC_COUNT];

    sscanf((const char*)response.get_buffer(), "%d,%d,%d,%d", 
           &adc_int[3], &adc_int[2], &adc_int[1], &adc_int[0]);

    for (int i=0; i < ALM_ADC_COUNT; i++) {
      adc_[i] = adc_to_volts(adc_int[i]);
      setDoubleParam(param_adc_[i], adc_[i]);
    }

    asynPrint(pasynUser_, ASYN_TRACE_FLOW,
              "Read ADC: %gV %gV %gV %gV\n", 
              adc_[0], adc_[1], adc_[2], adc_[3]);

  }
  
  return status;
}

asynStatus almController::runWrite(almCommandPacket &command) {
  command.run();

  command.dump(pasynUser_, ASYN_TRACEIO_DRIVER);
  //command.dump();

  asynStatus ret = write(command);
  return ret;
}


asynStatus almController::writeProgram(int number, const char *program) {
  if (strnchr(program, strlen(program), '/')) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s:%s: program contains slash: %s\n",
      driverName, __func__, program);
    return asynError;
  }

  almCommandPacket command;
  initCommandPacket(command);

  command.append("s%d", number);
  command.append(program);
  
  command.dump();
  if (command.get_last_ch() == 'R')
    return write(command);
  else
    return runWrite(command);
}

asynStatus almController::runProgram(int number) {
  almCommandPacket command;
  initCommandPacket(command);
  command.append("e%d", number);
  return runWrite(command);
}

asynStatus almController::setHomeFlagPolarity(bool polarity) {
  almCommandPacket command;
  initCommandPacket(command);
  command.home_flag_polarity(polarity);
  return runWrite(command);
}

asynStatus almController::invertInputs(unsigned int input, bool invert) {
  int in[4];
  
  for (int i=0; i < ALM_INPUT_COUNT; i++) {
    getIntegerParam(param_invert_input_[i], &in[i]);
  }
  
  if (input >= ALM_INPUT_COUNT)
    return asynError;
  else
    in[input] = invert;

  almCommandPacket command;
  initCommandPacket(command);
  command.invert_inputs(in[0], in[1], in[2], in[3]);

  command.dump();
  return runWrite(command);
}

asynStatus almController::invertInputs(unsigned int mask) {
  almCommandPacket command;
  initCommandPacket(command);
  command.invert_inputs(mask);
  return runWrite(command);
}

asynStatus almController::resetDevice() {
  almCommandPacket command;
  initCommandPacket(command);
  command.reset_processor();
  return runWrite(command);
}

asynStatus almController::setSwitchDebounce(int periods) {
  almCommandPacket command;
  initCommandPacket(command);
  command.switch_debounce(periods);
  return runWrite(command);
}

asynStatus almController::driverPower(int driver, bool enabled) {
  int enabled1, enabled2;
  if (driver < 1 || driver > 2)
    return asynError;

  getIntegerParam(param_driver1_power_, &enabled1);
  getIntegerParam(param_driver2_power_, &enabled2);

  if (driver == 1)
    enabled1 = enabled;
  else
    enabled2 = enabled;

  almCommandPacket command;
  initCommandPacket(command);
  command.driver_power(enabled1, enabled2);

  command.dump();
  return runWrite(command);
}

asynStatus almController::daughterCurrent(unsigned int current) {
  almCommandPacket command;
  initCommandPacket(command);
  command.daughter_current_flow(current);
  return runWrite(command);
}

asynStatus almController::daughterCurrentFlow(bool direction) {
  almCommandPacket command;
  initCommandPacket(command);
  command.daughter_current_flow(direction);
  return runWrite(command);
}

asynStatus almController::setInputThreshold(unsigned int chan, double thresh) {
  if (thresh < 0.0 || thresh > 3.3)
    return asynError;

  unsigned int raw_thresh = volts_to_adc(thresh);

  almCommandPacket command;
  initCommandPacket(command);
  command.input_threshold(chan, raw_thresh);

  command.dump();
  asynStatus ret = runWrite(command);

  getInputThresholds();
  return ret;
}

asynStatus almController::getInputThresholds() {
  almResponsePacket response;
  
  int thresh[ALM_ADC_COUNT];

  if (queryControllerParam(ALM_QUERY_ADC_THRESHOLDS, response) == asynSuccess) {
    //printf("Input thresholds %s\n", response.get_buffer());
    sscanf((const char*)response.get_buffer(), "%d,%d,%d,%d", 
           &thresh[3], &thresh[2], &thresh[1], &thresh[0]);
    
    // 0 to 16368 -> 0.0 to 3.3V
    for (int i=0; i < ALM_ADC_COUNT; i++) {
      thresholds_[i] = adc_to_volts(thresh[i]);
      setDoubleParam(param_input_threshold_[i], thresholds_[i]);
    }

    return asynSuccess;
  }
  return asynError;
}

asynStatus almController::setPotOffset(unsigned int offset) {
  almCommandPacket command;
  initCommandPacket(command);
  command.pot_offset(offset);
  return runWrite(command);
}

asynStatus almController::setPotMul(unsigned int mul) {
  almCommandPacket command;
  initCommandPacket(command);
  command.pot_mul(mul);
  return runWrite(command);
}

asynStatus almController::setPotDeadband(unsigned int deadband) {
  almCommandPacket command;
  initCommandPacket(command);
  command.pot_deadband(deadband);
  return runWrite(command);
}

asynStatus almController::queryInputs() {
  almResponsePacket response;
  
  // TODO: not sure why, but /1?4 and /1?a4 return different results
  // (with the same input) for sw1/2 and opto1/2
  if (queryControllerParam(ALM_QUERY_INPUTS, response) == asynSuccess) {
    int ret = response.as_int();
    setIntegerParam(param_inp_sw1_, (ret & ALM_INP_SW1) != 0);
    setIntegerParam(param_inp_sw2_, (ret & ALM_INP_SW2) != 0);
    setIntegerParam(param_inp_opto1_, (ret & ALM_INP_OPTO1) != 0);
    setIntegerParam(param_inp_opto2_, (ret & ALM_INP_OPTO2) != 0);
  }

  if (queryControllerParam(ALM_QUERY_ALL_INPUTS, response) == asynSuccess) {
    int ret = response.as_int();
    //setIntegerParam(param_inp_sw1_, (ret & ALM_INP_SW1) != 0);
    //setIntegerParam(param_inp_sw2_, (ret & ALM_INP_SW2) != 0);
    //setIntegerParam(param_inp_opto1_, (ret & ALM_INP_OPTO1) != 0);
    //setIntegerParam(param_inp_opto2_, (ret & ALM_INP_OPTO2) != 0);
    setIntegerParam(param_inp_cha_, (ret & ALM_INP_ENC_CHA) != 0);
    setIntegerParam(param_inp_chb_, (ret & ALM_INP_ENC_CHB) != 0);
    setIntegerParam(param_inp_idx_, (ret & ALM_INP_ENC_IDX) != 0);
    return asynSuccess;
  }
  return asynError;
}

asynStatus almController::queryVelocity() {
  almResponsePacket response;

  if (queryControllerParam(ALM_QUERY_VELOCITY, response) == asynSuccess) {
    int ret = response.as_int();
    velocities_[0] = ret;
    for (int i=1; i < ALM_AXES; i++)
      velocities_[i] = 0;

    return asynSuccess;
  }
  return asynError;
}

asynStatus almController::queryFirmware() {
  almResponsePacket response;
  if (queryControllerParam(ALM_QUERY_FIRMWARE, response) == asynSuccess) {
    printf("Firmware version: %s\n", response.get_buffer());
    setStringParam(param_firmware_, (const char*)response.get_buffer());
    return asynSuccess;
  }
  return asynError;
}

asynStatus almController::eraseEEPROM() {
  almCommandPacket command;
  initCommandPacket(command);
  command.erase_eeprom();
  return runWrite(command);
}

asynStatus almController::setEncoderOuterDeadband(unsigned int counts) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_encoder_outer_deadband(counts);
  return runWrite(command);
}

asynStatus almController::setEncoderInnerDeadband(unsigned int counts) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_encoder_inner_deadband(counts);
  return runWrite(command);
}

asynStatus almController::setEncoderRatio(unsigned int ticks_per_ustep) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_encoder_ratio(ticks_per_ustep);
  return runWrite(command);
}

asynStatus almController::setOverloadTimeout(unsigned int moves) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_overload_timeout(moves);
  return runWrite(command);
}

asynStatus almController::setIntegrationPeriod(unsigned int pd) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_integration_period(pd);
  return runWrite(command);
}

asynStatus almController::setRecoveryScriptRuns(unsigned int runs) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_recovery_script_runs(runs);
  return runWrite(command);
}

asynStatus almController::setMode(unsigned int mode) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_mode(mode);
  return runWrite(command);
}

asynStatus almController::setSpecialMode(unsigned int mode) {
  almCommandPacket command;
  initCommandPacket(command);
  command.set_special_mode(mode);
  return runWrite(command);
}

