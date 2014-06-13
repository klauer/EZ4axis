#include "allmotion.h"

// vim: tabstop=2 shiftwidth=2
static byte sequence_num = 0;

#ifndef strnchr
char* strnchr(const char* str, size_t len, char c) {
  if (!str)
    return NULL;

  while (len > 0 && *str != '\0') {
    if (*str == c) {
      return (char*)str;
    }
    str++;
    len--;
  }

  if (len > 0 && *str == '\0' && c == '\0')
    return (char*)str;

  return NULL;
}
#endif

static byte next_sequence_num(void) {
  sequence_num = (sequence_num + 1) % 7 + 0x31;
  return sequence_num;
}


bool calcChecksum(const byte *buf, int buflen, byte &checksum) {
  int i = 0;
  checksum = 0;

  if (buf[0] == ALM_OEM_START_CHAR) {
    while (i < buflen && buf[i] != 0) {
      checksum ^= buf[i];
      if (buf[i] == ALM_OEM_END_CHAR)
        break;
      i++;
    }

    if (buf[i] == ALM_OEM_END_CHAR) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

/////////////////////
/* Response packet */
void almResponsePacket::invalidate() {
  valid_ = false;
  memset(buf_, '\0', (size_t)ALM_STRING_LEN);
}

bool almResponsePacket::received(const byte *input, int len) {
  const byte *response = find_response_start(input, len);
#if 0
  printf("received length=%d\n", len);
  for (int i=0; i <= len; i++) {
    printf("0x%x ", input[i]);
  }
  printf("\n");
#endif

  if (response) {
    // Parse out the codes in the first byte
    valid_ = ((response[0] & ALM_ALWAYS_SET_MASK) == ALM_ALWAYS_SET_MASK);
    ready_ = ((response[0] & ALM_READY_MASK) == ALM_READY_MASK);
    code_ = (almStatus)(response[0] & ALM_STATUS_MASK);
    overflow_ = (len - (response - input)) >= ALM_STRING_LEN;
  
#if DEBUG
      fprintf(stderr, "[debug] Valid=%d status=%d [%s] ready=%d\n", 
              valid_, code_, get_allmotion_error_string(code_), ready_);
#endif

    if (overflow_)
      valid_ = false;

    if (valid_) {
      // Copy over the buffer
      memcpy(buf_, response + 1, (len - (response - input)) - 1);
      return true;
    }
  }
  return false;
}

bool almResponsePacket::verifyChecksum(const byte *buf, int buflen) {
  byte csum;
  if (!calcChecksum(buf, buflen, csum))
    return false;

  byte *buf_end = (byte *)strnchr((char *)buf, buflen, ALM_OEM_END_CHAR);
  if (!buf_end)
    return false;

  // also replaces ALM_OEM_END_CHAR for easier parsing later
  *buf_end = 0;
  buf_end++;
  if ((buf_end - buf) < buflen) {
    //printf("checksum %x calculated %x\n", csum, *buf_end);
    return (csum == *buf_end);
  }
  return false;
}

const byte *almResponsePacket::find_response_start(const byte *str, int buflen) {
  // Response: /0bXXYY where b is the status byte and XXYY is optionally
  // any data contained in the response (in OEM protocol, / is ALM_OEM_START_CHAR)
  for (int i=0; i < buflen - 1; i++) {
    if (str[i] == 0) {
      return NULL;
    }

    if ((str[i] == '/' || str[i] == ALM_OEM_START_CHAR) && str[i + 1] == '0') {
      if ((i + 2) < buflen) {
        if (str[i] == ALM_OEM_START_CHAR) {
          if (verifyChecksum(&str[i], buflen - i))
            return &str[i + 2];
          else 
            return NULL;
        } else {
          return &str[i + 2];
        }
      } 
    }
  }

  return NULL;
}

void almResponsePacket::dump(asynUser* user, int asyn_trace_mask) {
  if ((pasynTrace->getTraceMask(user) & asyn_trace_mask) != 0) {
    printf("trace mask %x req trace mask %x\n", 
          pasynTrace->getTraceMask(user), asyn_trace_mask);
    dump();
  }

}

void almResponsePacket::dump() {
  for (int i=0; i <= ALM_STRING_LEN; i++) {
    printf("0x%x ", buf_[i]);

    if (buf_[i] == 0)
      break;
  }

  printf("\n");
}

int almResponsePacket::as_float() {
  if (valid_) {
    return atof((const char*)buf_);
  } else {
    return 0.0f;
  }
}

int almResponsePacket::as_int() {
  if (valid_) {
    return atoi((const char*)buf_);
  } else {
    return 0;
  }
}

////////////////////
/* Command packet */
almCommandPacket::almCommandPacket() {
  clear();
}

almCommandPacket::almCommandPacket(int address) {
  clear();
  start(address);
}

void almCommandPacket::clear() {
  axis_ = 0;
  
  finished_ = false;
  buf_pos_ = 0;
  memset(buf, '\0', (size_t)ALM_STRING_LEN);
}

bool almCommandPacket::set_repeat() {
  // bit 3 of the sequence bit should be set to indicate retransmission
  buf[2] |= (1 << 3);
  return finish();
}

void almCommandPacket::start(int address) {
#if ALM_USE_OEM_PROTOCOL
  byte seq = next_sequence_num();
  append("%c%d%c", ALM_OEM_START_CHAR, address, seq);
#else
  append("/%d", address);
#endif
  finished_ = false;
}

bool almCommandPacket::select_axis(byte axis) {
  axis_ = axis;
  return true;
}

bool almCommandPacket::run() {
  return append('R');
}

bool almCommandPacket::finish() {
  if (finished_)
    return true;

  finished_ = true;

#if ALM_USE_OEM_PROTOCOL
  byte csum;
  if (!append(ALM_OEM_END_CHAR))
    return false;

  if (!calcChecksum(buf, buf_pos_, csum))
    return false;

  if (!append(csum))
    return false;

  return append((byte)'\0');
#else
  return append("\r\n");
#endif

}

void almCommandPacket::dump(asynUser* user, int asyn_trace_mask) {
  //printf("current trace mask %x print trace mask %x\n", pasynTrace->getTraceMask(user), asyn_trace_mask);
  // can't use asynPrint directly as it spits out a timestamp for each call
  if ((pasynTrace->getTraceMask(user) & asyn_trace_mask) != 0)
    dump();
}

void almCommandPacket::dump() {
  for (int i=0; i <= buf_pos_; i++) {
    if (buf[i] >= 32 && buf[i] < 127)
      printf("%c", buf[i]);
    else
      printf(".");
  }

  printf(" [");
  for (int i=0; i <= buf_pos_; i++) {
    printf("0x%x ", buf[i]);
  }
  printf("]\n");
  
}

bool almCommandPacket::append(byte c) {
  if ((buf_pos_ + 1) < ALM_STRING_LEN) {
    buf[buf_pos_++] = c;
    return true;
  }
  return false;
}

bool almCommandPacket::append(const char *fmt, va_list argptr) {
  char temp[ALM_STRING_LEN];
  vsnprintf(temp, ALM_STRING_LEN, fmt, argptr);

  int len = strlen(temp);
  // Ensure there's still enough room with the null terminator
  if ((buf_pos_ + len + 1) < ALM_STRING_LEN) {
    memcpy(&buf[buf_pos_], temp, len);
    buf_pos_ += len;
    return true;
  } else {
    return false;
  }
}

bool almCommandPacket::append(const char *fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  bool ret = append(fmt, argptr);
  va_end(argptr);
  return ret;
}


bool almCommandPacket::append_four(byte c, int v1, int v2, int v3, int v4) {
  return append("%c%d,%d,%d,%d", v1, v2, v3, v4);
}

bool almCommandPacket::append_four(byte c, byte axis, int value) {
  switch (axis) {
  case 0: return append("%c%d,,,", c, value);
  case 1: return append("%c,%d,,", c, value);
  case 2: return append("%c,,%d,", c, value);
  case 3: return append("%c,,,%d", c, value);
  default: return false;
  }
}

bool almCommandPacket::read_adc() {
  return append("?aa");
}

bool almCommandPacket::move(byte axis, int position, bool relative) {
  select_axis(axis);

  if (!relative) {
    append("A%d", position);
  } else {
    if (position > 0)
      append("P%d", position);
    else
      append("D%d", -position);
  }
  return true;
}


bool almCommandPacket::set_accel(unsigned int factor) {
  factor = min(factor, 65000);
  return append("L%d", factor);
}

bool almCommandPacket::set_bump_jog_dist(unsigned int counts) {
  counts = min(counts, 65000);
  return append("B%d", counts);
}

bool almCommandPacket::halt_condition(unsigned int input, bool value) {
  if (input > 4)
    return false;

  return append("H%d%d", value, input);
}

bool almCommandPacket::skip_next(unsigned int input, bool value) {
  if (input > 4)
    return false;

  return append("S%d%d", value, input);
}

bool almCommandPacket::invert_inputs(bool in0, bool in1, bool in2, bool in3) {
  unsigned int mask = ((in3 & 1) << 3) | ((in2 & 1) << 2) | ((in1 & 1) << 1) | in0;
  return invert_inputs(mask);
}

bool almCommandPacket::invert_inputs(unsigned int mask) {
  return append("ap%d", (mask & 0xF));
}

bool almCommandPacket::home_flag_polarity(bool polarity) {
  int pol = (polarity ? 1 : 0);
  return append("f%d", pol);
}

bool almCommandPacket::set_position(int counts) {
  return append("z%d", counts);
}

bool almCommandPacket::home(int counts) {
  return append("Z%d", counts);
}


bool almCommandPacket::set_axis_param(byte param, byte axis, int value) {
  select_axis(axis);
  return append("%c%d", param, value);
}


almCommandPacket::almCommandPacket(const almCommandPacket &other) {
  buf_pos_ = other.buf_pos_;  
  axis_ = other.axis_;  
  finished_ = other.finished_;

  memcpy(buf, other.buf, ALM_STRING_LEN);
}

//////////////////////
/* EZ4Axis-specific */
almEZ4CommandPacket::almEZ4CommandPacket() : almCommandPacket() 
{
}

almEZ4CommandPacket::almEZ4CommandPacket(int address)
  : almCommandPacket()
{
  start(address);
}

bool almEZ4CommandPacket::select_axis(byte axis) {
  // Axes start counting at 1
  if (axis == 0 || axis == axis_)
    return true;
  
  axis_ = axis;
  return append("aM%d", axis);
}

bool almEZ4CommandPacket::move(byte axis, int position, bool relative) {
  // don't select a specific axis, but write in the format "A,,,"
  if (!relative) {
    append_four('A', axis, position);
  } else {
    if (position > 0)
      append_four('P', axis, position);
    else
      append_four('D', axis, -position);
  }
  return true;

}

bool almEZ4CommandPacket::set_axis_param(byte param, byte axis, int value) {
  return append_four(param, axis, value);
}
