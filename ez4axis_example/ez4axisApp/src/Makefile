TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ez4axis
# ez4axis.dbd will be created and installed
DBD += ez4axis.dbd

# ez4axis.dbd will be made up from these files:
ez4axis_DBD += base.dbd

# Include dbd files from all support applications:
ez4axis_DBD += allmotion.dbd motorSupport.dbd asyn.dbd drvAsynIPPort.dbd
ez4axis_DBD += stringinRecord.dbd stringoutRecord.dbd
ez4axis_DBD += asSupport.dbd calcSupport.dbd system.dbd

# Add all the support libraries needed by this IOC
ez4axis_LIBS += allmotion motor asyn
ez4axis_LIBS += autosave calc

# ez4axis_registerRecordDeviceDriver.cpp derives from ez4axis.dbd
ez4axis_SRCS += ez4axis_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ez4axis_SRCS_DEFAULT += ez4axisMain.cpp
ez4axis_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#ez4axis_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
ez4axis_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

