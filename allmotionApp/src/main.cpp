#include "allmotion.h"
// vim: tabstop=2 shiftwidth=2

static ELLLIST allmotionList;
static int allmotionListInitialized = 0;

bool addToList(const char *portName, almController *drv) {
    if (!allmotionListInitialized) {
        allmotionListInitialized = 1;
        ellInit(&allmotionList);
    } else if (findByPortName(portName) != NULL) {
        fprintf(stderr, "ERROR: Re-using portName=%s\n", portName);
        return false;
    }

    allmotionNode *pNode = (allmotionNode*)calloc(1, sizeof(allmotionNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = drv;
    ellAdd(&allmotionList, (ELLNODE*)pNode);
    return true;
}

almController* findByPortName(const char *portName) {
    allmotionNode *pNode;
    static const char *functionName = "findByPortName";

    // Find this
    if (!allmotionListInitialized) {
        printf("%s:%s: ERROR, allmotion list not initialized\n",
               driverName, functionName);
        return NULL;
    }

    pNode = (allmotionNode*)ellFirst(&allmotionList);
    while(pNode) {
        if (!strcmp(pNode->portName, portName)) {
            return pNode->pController;
        }
        pNode = (allmotionNode*)ellNext((ELLNODE*)pNode);
    }

    printf("%s: allmotion on port %s not found\n",
        driverName, portName);
    return NULL;
}


///// almCreateController
//
/** Creates a new almController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the allmotion controller
  * \param[in] address           The address of the device on the RS485 bus
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int almCreateController(const char *portName, const char *asynPortName, int address, int numAxes,
                                   int movingPollPeriod, int idlePollPeriod)
{
  new almController(portName, asynPortName, address, numAxes, movingPollPeriod, idlePollPeriod);
  return(asynSuccess);
}

/*
 Code for iocsh registration */
static const iocshArg almCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg almCreateControllerArg1 = {"AllMotion port name", iocshArgString};
static const iocshArg almCreateControllerArg2 = {"RS485 address", iocshArgInt};
static const iocshArg almCreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg almCreateControllerArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg almCreateControllerArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const almCreateControllerArgs[] = {&almCreateControllerArg0,
                                                           &almCreateControllerArg1,
                                                           &almCreateControllerArg2,
                                                           &almCreateControllerArg3,
                                                           &almCreateControllerArg4,
                                                           &almCreateControllerArg5
                                                           };

static const iocshFuncDef almCreateControllerDef = {"almCreateController", 6, almCreateControllerArgs};
static void almCreateControllerCallFunc(const iocshArgBuf *args)
{
  if (!args[0].sval)
    return;
  almCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

///// almCreateEZ4Controller
//
/** Creates a new almController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the allmotion controller
  * \param[in] address           The address of the device on the RS485 bus
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int almCreateEZ4Controller(const char *portName, const char *asynPortName, int address, int numAxes,
                                   int movingPollPeriod, int idlePollPeriod)
{
  new almEZ4Controller(portName, asynPortName, address, numAxes, movingPollPeriod, idlePollPeriod);
  return(asynSuccess);
}

/*
 Code for iocsh registration */
static const iocshArg almCreateEZ4ControllerArg0 = {"Port name", iocshArgString};
static const iocshArg almCreateEZ4ControllerArg1 = {"AllMotion port name", iocshArgString};
static const iocshArg almCreateEZ4ControllerArg2 = {"RS485 address", iocshArgInt};
static const iocshArg almCreateEZ4ControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg almCreateEZ4ControllerArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg almCreateEZ4ControllerArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const almCreateEZ4ControllerArgs[] = {&almCreateEZ4ControllerArg0,
                                                           &almCreateEZ4ControllerArg1,
                                                           &almCreateEZ4ControllerArg2,
                                                           &almCreateEZ4ControllerArg3,
                                                           &almCreateEZ4ControllerArg4,
                                                           &almCreateEZ4ControllerArg5
                                                           };

static const iocshFuncDef almCreateEZ4ControllerDef = {"almCreateEZ4Controller", 6, almCreateEZ4ControllerArgs};
static void almCreateEZ4ControllerCallFunc(const iocshArgBuf *args)
{
  if (!args[0].sval)
    return;
  almCreateEZ4Controller(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

#if 0
///// allmotionConfigureAxis
/** Configures an allmotionAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the controller's asyn port
  * \param[in] axis              The axis
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int allmotionConfigureAxis(const char *portName, int axisNum,
                                      int movingPollPeriod, int idlePollPeriod)
{

  almController *controller;
  if ((controller = findByPortName(portName)) == NULL) {
    return 1;
  }

  allmotionAxis *axis = controller->getAxis(axisNum);
  if (!axis) {
    printf("Bad axis number #%d (axis count = %d)", axisNum, controller->getAxisCount());
    return 1;
  }

  axis->configure();
}

/** Code for iocsh registration */
static const iocshArg allmotionConfigureAxisArg0 = {"Port name", iocshArgString};
static const iocshArg allmotionConfigureAxisArg1 = {"AllMotion port name", iocshArgString};
static const iocshArg allmotionConfigureAxisArg2 = {"RS485 address", iocshArgInt};
static const iocshArg allmotionConfigureAxisArg3 = {"Number of axes", iocshArgInt};
static const iocshArg allmotionConfigureAxisArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg allmotionConfigureAxisArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const allmotionConfigureAxisArgs[] = {&allmotionConfigureAxisArg0,
                                                                 &allmotionConfigureAxisArg1,
                                                                 &allmotionConfigureAxisArg2,
                                                                 &allmotionConfigureAxisArg3,
                                                                 &allmotionConfigureAxisArg4,
                                                                 &allmotionConfigureAxisArg5
                                                                };

static const iocshFuncDef allmotionConfigureAxisDef = {"allmotionConfigureAxis", 5, allmotionConfigureAxisArgs};
static void allmotionConfigureAxisCallFunc(const iocshArgBuf *args)
{
  allmotionConfigureAxis(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}
#endif


/***********************************************************************/
static void allmotionMotorRegister(void)
{
  iocshRegister(&almCreateControllerDef, almCreateControllerCallFunc);
  iocshRegister(&almCreateEZ4ControllerDef, almCreateEZ4ControllerCallFunc);
  //iocshRegister(&allmotionConfigureAxisDef, allmotionConfigureAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(allmotionMotorRegister);
}
