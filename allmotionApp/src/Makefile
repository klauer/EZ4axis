TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += allmotion

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install allmotion.dbd into <top>/dbd
DBD += allmotion.dbd

# specify all source files to be compiled and added to the library
allmotion_SRCS += main.cpp
allmotion_SRCS += almPacket.cpp almController.cpp almAxis.cpp almEZ4Controller.cpp

allmotion_LIBS += $(EPICS_BASE_IOC_LIBS)

allmotion_LIBS += motor

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

