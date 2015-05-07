/*
 * AllMotion EZ-series controller driver
 * */
// vim: tabstop=2 shiftwidth=2
#ifndef _ALLMOTION_H
#define _ALLMOTION_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <memory>

#include <iocsh.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#ifndef DEBUG
// Set this to 0 to disable debug messages
#define DEBUG 0
#endif

// Parameter string identifiers (matched in INP/OUT in records)
#define ALM_PSTR_KILL_ALL          "ALM_KILLALL"
#define ALM_PSTR_ERROR             "ALM_ERROR"
#define ALM_PSTR_READ_ADC          "ALM_READ_ADC"
#define ALM_PSTR_READ_INP          "ALM_READ_INP"
#define ALM_PSTR_READ_THRESH       "ALM_READ_THRESH"
#define ALM_PSTR_READ_LIM_THR      "ALM_READ_LIM_THR"
#define ALM_PSTR_ADC_1             "ALM_ADC_1"
#define ALM_PSTR_ADC_2             "ALM_ADC_2"
#define ALM_PSTR_ADC_3             "ALM_ADC_3"
#define ALM_PSTR_ADC_4             "ALM_ADC_4"

#define ALM_PSTR_PROG_0            "ALM_PROG_0"
#define ALM_PSTR_PROG_1            "ALM_PROG_1"
#define ALM_PSTR_PROG_2            "ALM_PROG_2"
#define ALM_PSTR_PROG_3            "ALM_PROG_3"
#define ALM_PSTR_PROG_4            "ALM_PROG_4"
#define ALM_PSTR_PROG_5            "ALM_PROG_5"
#define ALM_PSTR_PROG_6            "ALM_PROG_6"
#define ALM_PSTR_PROG_7            "ALM_PROG_7"
#define ALM_PSTR_PROG_8            "ALM_PROG_8"
#define ALM_PSTR_PROG_9            "ALM_PROG_9"
#define ALM_PSTR_PROG_10           "ALM_PROG_10"
#define ALM_PSTR_PROG_11           "ALM_PROG_11"
#define ALM_PSTR_PROG_12           "ALM_PROG_12"
#define ALM_PSTR_PROG_13           "ALM_PROG_13"
#define ALM_PSTR_PROG_14           "ALM_PROG_14"
#define ALM_PSTR_PROG_15           "ALM_PROG_15"

#define ALM_PSTR_PROG_IDX          "ALM_PROG_IDX"
#define ALM_PSTR_PROG_WRITE        "ALM_PROG_WRITE"
#define ALM_PSTR_PROG_RUN          "ALM_RUN"

#define ALM_PSTR_MODE              "ALM_MODE"
#define ALM_PSTR_SP_MODE           "ALM_SP_MODE"

#define ALM_PSTR_INP_SW1           "ALM_INP_SW1"
#define ALM_PSTR_INP_SW2           "ALM_INP_SW2"
#define ALM_PSTR_INP_OPTO1         "ALM_INP_OPTO1"
#define ALM_PSTR_INP_OPTO2         "ALM_INP_OPTO2"
#define ALM_PSTR_INP_CHA           "ALM_INP_CHA"
#define ALM_PSTR_INP_CHB           "ALM_INP_CHB"
#define ALM_PSTR_INP_IDX           "ALM_INP_IDX"

#define ALM_PSTR_FIRMWARE          "ALM_FIRMWARE"
#define ALM_PSTR_MICROSTEPS        "ALM_MICROSTEPS"
#define ALM_PSTR_MICROSTEP_TWEAK   "ALM_MICROSTEP_TWEAK"
#define ALM_PSTR_MAX_AMPS          "ALM_MAX_AMPS"

#define ALM_PSTR_DRIVER1_POWER     "ALM_DRIVER1_POWER"
#define ALM_PSTR_DRIVER2_POWER     "ALM_DRIVER2_POWER"
#define ALM_PSTR_SWITCH_DEBOUNCE   "ALM_SWITCH_DEBOUNCE"
#define ALM_PSTR_DAUGHTER_CURRENT  "ALM_DAUGHTER_CURRENT"
#define ALM_PSTR_DAUGHTER_CUR_FLOW "ALM_DAUGHTER_CUR_FLOW"

#define ALM_PSTR_INPUT_THRESHOLD0  "ALM_INPUT_THRESHOLD0"
#define ALM_PSTR_INPUT_THRESHOLD1  "ALM_INPUT_THRESHOLD1"
#define ALM_PSTR_INPUT_THRESHOLD2  "ALM_INPUT_THRESHOLD2"
#define ALM_PSTR_INPUT_THRESHOLD3  "ALM_INPUT_THRESHOLD3"

#define ALM_PSTR_POT_OFFSET        "ALM_POT_OFFSET"
#define ALM_PSTR_POT_MUL           "ALM_POT_MUL"
#define ALM_PSTR_POT_DEADBAND      "ALM_POT_DEADBAND"

#define ALM_PSTR_ENC_1             "ALM_ENC_1"
#define ALM_PSTR_ENC_2             "ALM_ENC_2"

#define ALM_PSTR_ENC_OUTER_DB      "ALM_ENC_OUTER_DB"
#define ALM_PSTR_ENC_INNER_DB      "ALM_ENC_INNER_DB"
#define ALM_PSTR_ENC_RATIO         "ALM_ENC_RATIO"
#define ALM_PSTR_OVERLOAD_TIMEOUT  "ALM_OVERLOAD_TIMEOUT"
#define ALM_PSTR_INT_PERIOD        "ALM_INT_PERIOD"
#define ALM_PSTR_RECOVERY_RUNS     "ALM_RECOVERY_RUNS"

#define ALM_PSTR_HOME_POLARITY     "ALM_HOME_POLARITY"
#define ALM_PSTR_RESET             "ALM_RESET"
#define ALM_PSTR_INVERT_INPUT0     "ALM_INVERT_INPUT0"
#define ALM_PSTR_INVERT_INPUT1     "ALM_INVERT_INPUT1"
#define ALM_PSTR_INVERT_INPUT2     "ALM_INVERT_INPUT2"
#define ALM_PSTR_INVERT_INPUT3     "ALM_INVERT_INPUT3"
#define ALM_PSTR_LIMIT_INVERT      "ALM_LIMIT_INVERT"
#define ALM_PSTR_LIMIT_THRESH_LOW  "ALM_LIMIT_THRESH_LOW"
#define ALM_PSTR_LIMIT_THRESH_HIGH "ALM_LIMIT_THRESH_HIGH"

#define ALM_PSTR_HOLD_I            "ALM_HOLD_I"
#define ALM_PSTR_MOVE_I            "ALM_MOVE_I"

// Status byte mask
#define ALM_ALWAYS_SET_MASK             (1 << 6)
#define ALM_READY_MASK                  (1 << 5)
#define ALM_STATUS_MASK                 0xF

// OEM protocol byte definitions
#define ALM_USE_OEM_PROTOCOL            1
#if ALM_USE_OEM_PROTOCOL
#    define ALM_INPUT_EOS               "\n"
#    define ALM_OUTPUT_EOS              "\0"
#else
#    define ALM_INPUT_EOS               "\03"
#    define ALM_OUTPUT_EOS              "\n"
#endif
#define ALM_OEM_START_CHAR              0x02
#define ALM_OEM_END_CHAR                0x03
#define ALM_OEM_SEQ_START               '1' // 0x31

// Query parameter strings
#define ALM_QUERY_ALL_POS               "aA"
#define ALM_QUERY_ALL_VEL               "aV"
#define ALM_QUERY_POS                   "0"
#define ALM_QUERY_INPUTS                "4"
#define ALM_QUERY_ALL_INPUTS            "a4"
#define ALM_QUERY_VELOCITY              "5"
#define ALM_QUERY_MICROSTEPS            "6"
#define ALM_QUERY_ENC_1                 "8"
#define ALM_QUERY_ENC_2                 "10"

#define ALM_QUERY_ADC                   "aa"
#define ALM_QUERY_ADC_THRESHOLDS        "at"
#define ALM_QUERY_LIMIT_THRESHOLDS      "aat"
#define ALM_QUERY_FIRMWARE              "&"
#define ALM_QUERY_STATUS                "Q"
#define ALM_ERASE_EEPROM                "?9"

// Input status byte
#define ALM_INP_SW1                     0x01
#define ALM_INP_SW2                     0x02
#define ALM_INP_OPTO1                   0x04
#define ALM_INP_OPTO2                   0x08
#define ALM_INP_ENC_CHA                 0x10
#define ALM_INP_ENC_CHB                 0x20
#define ALM_INP_ENC_IDX                 0x80

// Max values (and counts of features)
#define ALM_MAX_VELOCITY                305064
#define ALM_MAX_MICROSTEPS              256
#define ALM_MAX_MICROSTEP_SIZE          3000
#define ALM_STRING_LEN                  75
#define ALM_AXES                        4
#define ALM_INPUT_COUNT                 4
#define ALM_ENC_COUNT                   2
#define ALM_PROG_COUNT                  16

// Open/closed loop settings
#define ALM_MODE_OPEN_LOOP              0
#define ALM_MODE_CLOSED_LOOP            8

// Single mode allows only one axis to move at a time
#define ALM_CONTROL_SINGLE              0

// Lower and upper limits
#define ALM_LIM_HIGH                    0
#define ALM_LIM_LOW                     1

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

enum almStatus {
  ALM_NO_ERROR=0,
  ALM_INIT_ERROR,
  ALM_BAD_COMMAND,
  ALM_BAD_OPERAND,
  ALM_ERROR_4,
  ALM_COMMS_ERROR,
  ALM_ERROR_6,
  ALM_NOT_INIT_ERROR,
  ALM_ERROR_8,
  ALM_OVERLOAD_ERROR,
  ALM_ERROR_10,
  ALM_NOT_ALLOWED_ERROR,
  ALM_ERROR_12,
  ALM_ERROR_13,
  ALM_ERROR_14,
  ALM_OVERFLOW_ERROR,

  ALM_UNKNOWN_ERROR
};

static const char *driverName = "AllMotion";
class almController;
class almEZ4Controller;
class almAxis;
class almCommandPacket;
class almResponsePacket;
char *strnchr(const char* str, size_t len, char c);
const char *get_allmotion_error_string(almStatus error);

#include "almPacket.h"
#include "almAxis.h"
#include "almController.h"
#include "almEZ4Controller.h"

/* Use the following structure and functions to manage multiple instances
 * of the driver */
typedef struct allmotionNode {
    ELLNODE node;
    const char *portName;
    almController *pController;
} allmotionNode;

bool addToList(const char *portName, almController *drv);
almController* findByPortName(const char *portName);

double adc_to_volts(int value);
unsigned int volts_to_adc(double volts);

#endif
