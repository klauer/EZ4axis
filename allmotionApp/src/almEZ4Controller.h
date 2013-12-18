#include "allmotion.h"

///// Controller
class almEZ4Controller : public almController {
public:
  almEZ4Controller(const char *portName, const char *asynPortName, int address, 
                   int numAxes, double movingPollPeriod, double idlePollPeriod);

  asynStatus terminateCommand();

  virtual asynStatus queryPositions();
  virtual asynStatus queryVelocities();

protected:
  virtual almCommandPacket *getCommandPacket();
  //virtual asynStatus poll();

private:
  friend class allmotionAxis;

};
