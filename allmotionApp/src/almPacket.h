#include "allmotion.h"

#ifndef byte
typedef unsigned char byte;
#endif

// vim: tabstop=2 shiftwidth=2
class almResponsePacket {
private:
  byte buf_[ALM_STRING_LEN];
  bool valid_;
  bool ready_;
  bool overflow_;
  almStatus code_;

public:
  almResponsePacket() : valid_(false) {
    buf_[0] = 0; 
    invalidate();
  }
  
  void invalidate();

  bool received(const byte *input, int len);
  const byte *find_response_start(const byte *str, int buflen);
  
  void dump();
  void dump(asynUser* user, int asyn_trace_mask);

  bool verifyChecksum(const byte *buf, int buflen);
  bool is_valid() { return valid_; }
  bool is_ready() { return ready_; }
  almStatus get_status() { return code_; }

  // return info on the data buffer
  const byte *get_buffer() { return buf_; }
  int as_float();
  int as_int();
};


class almCommandPacket {
protected:
  byte buf[ALM_STRING_LEN];
  int buf_pos_;
  byte axis_;
  bool finished_;

public:
  almCommandPacket();
  almCommandPacket(int address);
  almCommandPacket(const almCommandPacket &other);
  virtual ~almCommandPacket() {}

  int length() {
    return buf_pos_ + 1;
  }

  const byte *get_buffer() { return buf; }
  byte get_last_ch() { return buf[buf_pos_]; }

  bool set_repeat();

  bool append(byte c);
  bool append(const char *fmt, va_list argptr);
  bool append(const char *fmt, ...);
  bool append_four(byte c, int v1, int v2, int v3, int v4);
  bool append_four(byte c, byte axis, int value);

  virtual void start(int address);
  virtual void clear();

  virtual bool run();

  virtual bool finish();
  virtual void dump();
  void dump(asynUser* user, int asyn_trace_mask);

  virtual bool select_axis(byte axis);
  virtual bool move(byte axis, int position, bool relative);
  virtual bool set_axis_param(byte param, byte axis, int value);
  virtual bool home(int counts);
  virtual bool set_position(int counts);

  virtual bool home_flag_polarity(bool polarity);
  virtual bool invert_inputs(unsigned int mask);
  virtual bool invert_inputs(bool in0, bool in1, bool in2, bool in3);
  virtual bool set_accel(unsigned int factor);

  virtual bool set_bump_jog_dist(unsigned int counts);

  virtual bool read_adc();
  virtual bool query_limits(byte axis);

  bool halt_condition(unsigned int input, bool value);
  bool skip_next(unsigned int input, bool value);
  bool store_program(int program) { return append("s%d", (program & 0xF)); }
  bool wait(unsigned int ms) { return append("M%d", ms); }
  bool run_program(int program) { return append("s%d", (program & 0xF)); }

  bool n1_jog_distance(unsigned int dist) { return append("B%d", dist); }
  bool reset_processor() { return append("ar5073"); }
  bool response_delay(unsigned int ms) { return append("aP%d", ms); }
  bool switch_debounce(unsigned int periods) { return append("d%d", periods); }
  bool backlash(unsigned int counts) { return append("K%d", counts); }

  bool scan_amplitude(unsigned int value) { return append("aA%d", value); }
  bool scan_frequency(unsigned int value) { return append("aW%d", value); }

  bool terminate() { return append("T"); }
  bool query_target() { return append("?0"); }
  bool query_slew_speed() { return append("?2"); }
  bool query_inputs() { return append("?4"); }
  bool query_inputs_encoders() { return append("?a4"); }
  bool query_speed() { return append("?5"); }
  bool query_microsteps() { return append("?6"); }
  bool query_microstep_tweak() { return append("?7"); }
  bool query_encoder() { return append("?8"); }
  bool erase_eeprom() { return append("?9"); }
  bool query_encoder2() { return append("?10"); }
  bool query_firmware() { return append("?&"); }

  bool start_loop() { return append('g'); }
  bool end_loop(int reps) { return append('G'); }
 
  // Daughter card
  bool daughter_current(unsigned int current) { 
    return append("l%d", current);
  }

  bool daughter_current_flow(bool direction) { 
    if (direction)
      return append("O1");
    else
      return append("l1");
  }
  
  // Potentiometer
  bool pot_offset(unsigned int offset) { return append("ao%d", offset); }
  bool pot_mul(unsigned int mul) { return append("am%d", mul); }
  bool pot_deadband(unsigned int usteps) { return append("ad%d", usteps); }
  
  // Position correction mode commands (pg 42)
  bool set_encoder_outer_deadband(unsigned int counts) { 
    return append("aC%d", counts); 
  }

  bool set_encoder_ratio(unsigned int ticks_per_ustep) { 
    // ticks/rev to microsteps/rev
    return append("aE%d", ticks_per_ustep); 
  }

  bool set_overload_timeout(unsigned int moves) { 
    return append("au%d", moves); 
  }

  bool set_integration_period(unsigned int pd) { 
    return append("x%d", pd); 
  }

  bool set_encoder_inner_deadband(unsigned int counts) { 
    return append("zsac%d", counts); 
  }

  bool set_recovery_script_runs(unsigned int runs) { 
    return append("u%d", runs); 
  }

  // Miscellaneous
  bool input_threshold(unsigned int chan, unsigned int thresh) { 
    return append("at%d%.5d", chan + 1, thresh); 
  }

  bool driver_power(bool power1, bool power2) {
    return append("J%d", ((power2 << 1) | power1)); 
  }

  bool set_velocity(byte axis, unsigned int velocity) {
    return set_axis_param('V', axis, min(velocity, ALM_MAX_VELOCITY)); 
  }

  bool set_hold_current(byte axis, unsigned int current) { 
    return set_axis_param('h', axis, min(current, 100)); 
  }

  bool set_move_current(byte axis, unsigned int current) {
    return set_axis_param('m', axis, min(current, 100)); 
  }

  bool set_microsteps(byte axis, unsigned int microsteps) {
    return set_axis_param('j', axis, min(microsteps, ALM_MAX_MICROSTEPS)); 
  }

  bool use_switch_limits() {
    return append("an16384");
  }

  bool set_baud(unsigned int rate) {
    return append("b%d", rate);
  }

  bool set_mode(unsigned int mode) {
    return append("n%d", mode);
  }

  bool set_special_mode(unsigned int mode) {
    return append("N%d", mode);
  }

  bool set_microstep_tweak(unsigned int size) {
    return append("o%d", min(size, ALM_MAX_MICROSTEP_SIZE));
  }
};
