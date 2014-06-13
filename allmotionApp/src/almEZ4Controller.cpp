/*
 * AllMotion EZ4Axis controller
 *
 * */
// vim: tabstop=2 shiftwidth=2
#include "allmotion.h"

/** Creates a new almAxis object.
  * \param[in] controller         The allmotion controller
  * \param[in] axis_num           The axis number (1-based)
  */
almEZ4Controller::almEZ4Controller(const char *portName, const char *asynPortName, int address, 
                       int numAxes, double movingPollPeriod, double idlePollPeriod)
  : almController(portName, asynPortName, address, numAxes, movingPollPeriod, idlePollPeriod)
{
}

asynStatus almEZ4Controller::queryPositions() {
  almResponsePacket response;

  if (queryParameter(0, ALM_QUERY_ALL_POS, response) == asynSuccess) {
    sscanf((const char*)response.get_buffer(), "%d,%d,%d,%d", 
           &positions_[0], &positions_[1], &positions_[2], &positions_[3]);
    return asynSuccess;
  }
  return asynError;
}

asynStatus almEZ4Controller::queryVelocities() {
  almResponsePacket response;

  if (queryParameter(0, ALM_QUERY_ALL_VEL, response) == asynSuccess) {
    sscanf((const char*)response.get_buffer(), "%d,%d,%d,%d", 
           &velocities_[0], &velocities_[1], &velocities_[2], &velocities_[3]);
  }

  return asynSuccess;
}
