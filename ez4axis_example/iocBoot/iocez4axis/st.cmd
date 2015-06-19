#!../../bin/linux-x86_64/ez4axis

epicsEnvSet("ENGINEER",  "klauer")
epicsEnvSet("LOCATION",  "740 3IDC RG-C?")

# epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST", "NO")
# epicsEnvSet("EPICS_CA_ADDR_LIST", "10.3.0.255")

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/ez4axis.dbd",0,0)
ez4axis_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet(EZ_IP   , "10.3.2.61")
epicsEnvSet(EZ_PORT , 4001)

epicsEnvSet(CtlSys  , "XF:03IDC-CT")
epicsEnvSet(Sys     , "XF:03IDC-ES")
epicsEnvSet(CntlDev , "Ez4:1")

epicsEnvSet("IOCNAME", "ez4axis-1") # set by softioc init.d script
epicsEnvSet("IOC_P", "$(CtlSys){IOC:$(IOCNAME)}")

## NOTE: RS485 address must match up with the address selected on the device with its address switch:
epicsEnvSet(ALM_ADDR, 1)

## Load record instances
dbLoadRecords("$(ALLMOTION)/db/ez4axis.db","Sys=$(Sys),Dev={$(CntlDev)},PORT=ALM1,ADDR=0")
dbLoadTemplate("motors.sub")

# drvAsynIPPortConfigure 'port name' 'host:port [protocol]' priority 'disable auto-connect' noProcessEos
drvAsynIPPortConfigure("IP1", "$(EZ_IP):$(EZ_PORT)", 0, 0, 0)

# almCreateController(AllMotion port name, asyn port name, RS485 address,
#                     Number of axes, Moving poll period (ms), Idle poll period (ms))
almCreateEZ4Controller("ALM1", "IP1", "$(ALM_ADDR)", 4, 100, 250)

asynSetTraceMask("ALM1", -1, 0x01)
asynSetTraceMask("IP1", -1, 0x01)
# asynSetTraceMask("ALM1", -1, 0x0)
# asynSetTraceMask("IP1", -1, 0x0)

cd ${TOP}/

dbLoadRecords("$(EPICS_BASE)/db/save_restoreStatus.db", "P=$(IOC_P)")
save_restoreSet_status_prefix("$(IOC_P)")

set_savefile_path("${TOP}/as/save","")
set_requestfile_path("$(EPICS_BASE)/as/req")
set_requestfile_path("${TOP}/as/req")

system("install -m 777 -d ${TOP}/as/save")
system("install -m 777 -d ${TOP}/as/req")

set_pass0_restoreFile("allmotion_pass0.sav")
#set_pass0_restoreFile("ioc_settings.sav")

set_pass1_restoreFile("allmotion_pass1.sav")
#set_pass1_restoreFile("ioc_pass1_settings.sav")

iocInit()

makeAutosaveFileFromDbInfo("$(TOP)/as/req/allmotion_pass0.req", "autosaveFields_pass0")
makeAutosaveFileFromDbInfo("$(TOP)/as/req/allmotion_pass1.req", "autosaveFields_pass1")

create_monitor_set("allmotion_pass0.req", 10, "")
create_monitor_set("allmotion_pass1.req", 10, "")

# caPutLogInit("ioclog.cs.nsls2.local:7004", 1)

# cd ${TOP}
# dbl > ./records.dbl
# system "cp ./records.dbl /cf-update/$HOSTNAME.$IOCNAME.dbl"
