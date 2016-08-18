# Information such as position and accuracy of an RTK base station
float32 lat             #Latitude (WGS84) of RTK base station, in degrees
float32 lon             #Longitude (WGS84) of RTK base station, in degrees
float32 alt             #Altitude (AMSL, NOT WGS84), in meters (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
uint64 surveyin_dur   #Duration of survey-in process, in seconds
uint64 surveyin_acc   #Accuracy of survey-in process, in mm
uint8 flags           #Survey-in flags, bit 0: valid, bit 1: active
uint8 cnt             #Nr of satellites used by base station
