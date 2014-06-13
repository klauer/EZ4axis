#!../../bin/linux-x86/ez4axis
## #!../../bin/linux-x86_64/ez4axis

## You may have to change ez4axis to something else
## everywhere it appears in this file

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/ez4axis.dbd",0,0)
ez4axis_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet(P      , "IOC:ALM:")
epicsEnvSet(EZ_IP  , "10.0.0.10")
epicsEnvSet(EZ_PORT, 4013)
epicsEnvSet(ASYN_P , "IP1")
epicsEnvSet(ALM_P  , "ALM1")
epicsEnvSet(AS     , "AS:")

## Load record instances
dbLoadRecords("$(ALLMOTION)/db/ez4axis.db","P=$(P),PORT=$(ALM_P),ADDR=0")
dbLoadRecords("$(TOP)/db/example.db","P=$(P),PORT=$(ALM_P),ADDR=0")

# drvAsynIPPortConfigure 'port name' 'host:port [protocol]' priority 'disable auto-connect' noProcessEos
drvAsynIPPortConfigure("$(ASYN_P)", "$(EZ_IP):$(EZ_PORT)", 0, 0, 0)

# almCreateController(AllMotion port name, asyn port name, RS485 address, Number of axes, Moving poll period (ms), Idle poll period (ms))
almCreateEZ4Controller("$(ALM_P)", "$(ASYN_P)", 2, 4, 100, 250)
## NOTE: RS485 address must match up with the address selected on the device with its address switch

dbLoadTemplate("motors.sub")

asynSetTraceMask("$(ALM_P)", -1, 0x01)
asynSetTraceMask("$(ASYN_P)", -1, 0x01)
#asynSetTraceMask("$(ALM_P)", -1, 0x010)
#asynSetTraceMask("$(ASYN_P)", -1, 0x010)
< save_restore.cmd
#iocInit()

## Start any sequence programs
#seq sncez4axis,"user=nanopos"

dbpf IOC:m1:MODE 0

