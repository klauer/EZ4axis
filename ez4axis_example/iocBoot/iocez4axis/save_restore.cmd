cd ${TOP}/

dbLoadRecords("$(EPICS_BASE)/db/save_restoreStatus.db", "P=$(P)$(AS)")
save_restoreSet_status_prefix("$(P)$(AS)")

set_savefile_path("${TOP}/as/save","")
set_requestfile_path("$(EPICS_BASE)/as/req")
set_requestfile_path("${TOP}/as/req")

system("install -m 777 -d ${TOP}/as/save")
system("install -m 777 -d ${TOP}/as/req")

set_pass0_restoreFile("all_motor_settings.sav")
set_pass0_restoreFile("allmotion_pass0.sav")
#set_pass0_restoreFile("ioc_settings.sav")

set_pass1_restoreFile("all_motor_positions.sav")
set_pass1_restoreFile("allmotion_pass1.sav")
#set_pass1_restoreFile("ioc_pass1_settings.sav")

iocInit()

makeAutosaveFileFromDbInfo("$(TOP)/as/req/allmotion_pass0.req", "autosaveFields_pass0")
makeAutosaveFileFromDbInfo("$(TOP)/as/req/allmotion_pass1.req", "autosaveFields_pass1")

create_monitor_set("all_motor_settings.req", 30, "P=$(P)")
create_monitor_set("all_motor_positions.req", 10, "P=$(P)")

create_monitor_set("allmotion_pass0.req", 10, "P=$(P)")
create_monitor_set("allmotion_pass1.req", 10, "P=$(P)")

cd iocBoot/$(IOC)
dbl > /epics/pv_lists/$(IOC).txt

