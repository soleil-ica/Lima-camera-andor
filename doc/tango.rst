Andor Tango device
=====================

This is the reference documentation of the Andor Tango device.

you can also find some useful information about prerequisite/installation/configuration/compilation in the :ref:`Andor camera plugin <camera-andor>` section.

Properties
----------

================= =============== =============== =========================================================================
Property name	  Mandatory	  Default value	  Description
================= =============== =============== =========================================================================
adc_speed  	  No		  max.		  The adc/Horiz. speed pair	
baseline_clamp 	  No		  Off		  Clamping for baseline threshold, ON or OFF	
camera_number	  No		  N/A		  The camera number,  default is  0	
cooler		  No		  Off		  Start/stop the cooling system of the camera mode	
config_path	  No		  N/A		  The configuration path, for linux default is /usr/local/etc/andor	
fast_ext_trigger  No		  Off		  Fast external trigger mode, see Andor documentation for usage	
fan_mode	  No		  N/A		  FAN mode, FAN_ON_FULL/FAN_ON_LOW/FAN_OFF	
high_capacity	  No		  High_capacity	  Camera can run in two modes, HIGH_CAPACITY or HIGH_SENSITIVITY	
p_gain  	  No		  max.	 	  The preamplifier gain [X1-Xn] (see detector spec.)
shutter_level	  No		  High		  The shutter output level mode
temperature_sp	  No		  N/A		  The temperature setpoint in Celsius
vs_speed  	  No		  fasten	  The vertical shift speed (see detector spec.)
================= =============== =============== =========================================================================



Attributes
----------
======================= ======= ======================= ======================================================================
Attribute name		RW	Type			Description
======================= ======= ======================= ======================================================================
adc_speed  		rw	DevString		The  ADC and Horizontal shift speed, in ADCchannel/Freq.Mhz, check 
							the documentatio for more help **(\*)**  
baseline_clamp		rw	DevString		The baseline clamping for threshold: **(\*\*)**
							 - **ON**
							 - **OFF**
cooler			rw	DevString		Start/stop the cooling system of the camera mode:
							 - **ON**, the cooler is started
							 - **OFF**, the cooler is stopped 	
cooling_status		ro	DevString		The status of the cooling system, tell if the setpoint 
							temperature is reached
fan_mode		rw	DevString		The FAN mode for extra-cooling: **(\*\*)**
							 - **FAN_OFF**  
							 - **FAN_ON_FULL**
							 - **FAN_ON_LOW**
fast_ext_trigger	rw	DevString		Fast external trigger mode, see Andor documentation for usage Mode are:
							 - **ON**, fast mode, the camera will not wait until the a keep clean cycle has been completed before accepting the next 
							   trigger
							 - **OFF**, slow mode	
high_capacity		rw	DevString		Off/On the High Capacity mode: **(\*\*)**
							 - **HIGH_CAPACITY**
							 - **HIGH_SENSITIVITY**
p_gain  		rw	DevString	 	The preamplifier gain from X1 to Xn (see detector spec.) **(\*)**
shutter_level		rw	DevString		The shutter output level mode:
							 - **LOW"**, output TTL low signal to open shutter
							 - **HIGH**, output TTL high signal to open shutter
temperature		ro	DevShort	 	The current sensor temperature in Celsius	
temperature_sp		rw	DevShort		The temperature setpoint in Celsius
timing			ro	Spectrum		The exposure and latency times	
vs_speed  		rw	DevString	 	The vertical shift speed, in us/pixel **(\*)**
======================= ======= ======================= ======================================================================

**(\*)** Use the command getAttrStringValueList to get the list of the supported value for these attributes. 

**(\*\*)** These attributes can not be supported by some camera models and the return value will be set to  **UNSUPPORTED**.


Commands
--------

=======================	=============== =======================	===========================================
Command name		Arg. in		Arg. out		Description
=======================	=============== =======================	===========================================
Init			DevVoid 	DevVoid			Do not use
State			DevVoid		DevLong			Return the device state
Status			DevVoid		DevString		Return the device state as a string
getAttrStringValueList	DevString:	DevVarStringArray:	Return the authorized string value list for
			Attribute name	String value list	a given attribute name
=======================	=============== =======================	===========================================

