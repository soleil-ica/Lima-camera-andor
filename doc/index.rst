Andor
-------

.. image:: ikon-m_934_header.jpg 

Introduction
````````````
Andor Technology manufactuer offers a large catalogue of scientific cameras. Covered scientific applications are low ligth imaging, spectroscopy, microscopy, time-resolved and high energy detection. 
Andor is providing a unique Software Development Tool (SDK) for both Windows and Linux, supporting different interface buses such as USB, CameraLink and also some specific acquisition PCI board.

The Lima module as been tested only with this cameras models:
  - IKon-M (USB interface, Linux OS)

Prerequisite
````````````
Previously to this you have to install the Andor SDK the default path (/usr/local).
For our test we us the SDK for Linux version **V2.91.30001.0** and ran the install script "install_andor"
for which option 5 (All USB Cameras) was selected, the default installation is made under /usr/local/ with:

  - /usr/local/include, header files
  - /usr/local/lib, library files
  - /usr/local/etc/andor, configuration files

The Linux SDK 2.91 has shared libraries which has been compiled on recent linux kernel, check first you have the right kernel and
libc available by compiling one of the example program available under examples/console.
Andor python module needs at least the lima core module.

For the USB camera the SDK is using the libusb under linux, check first your system is equiped with the libusb package otherwise you
will not compile the Andor Lima plugin.

Installation & Module configuration
````````````````````````````````````

-  follow first the steps for the linux installation :ref:`linux_installation`

The minimum configuration file is *config.inc* :

.. code-block:: sh

  COMPILE_CORE=1
  COMPILE_SIMULATOR=0
  COMPILE_SPS_IMAGE=1
  COMPILE_ESPIA=0
  COMPILE_FRELON=0
  COMPILE_MAXIPIX=0
  COMPILE_PILATUS=0
  COMPILE_BASLER=0
  COMPILE_ANDOR=1
  COMPILE_CBF_SAVING=0
  export COMPILE_CORE COMPILE_SPS_IMAGE COMPILE_SIMULATOR \
         COMPILE_ESPIA COMPILE_FRELON COMPILE_MAXIPIX COMPILE_PILATUS \
         COMPILE_BASLER COMPILE_ANDOR COMPILE_CBF_SAVING

-  start the compilation :ref:`linux_compilation`

-  finally for the Tango server installation :ref:`tango_installation`

Initialisation and Capabilities
````````````````````````````````
In order to help people to understand how the camera plugin has been implemented in LImA this section
provide some important information about the developer's choices.

Camera initialisation
......................

The camera will be initialized   within the AndorCamera  object.  The AndorCamera contructor
sets the camera with default parameters for Preampifier-Gain, VerticalShiftSpeed and the ADC/HorizontalSpeed.

These parameters are optimized for the faster mode, which means the maximum gain, the "fasten recommended" VSSpeed (i.e as returned
by GetFastestRecommendedVSSpeed() SDK function call) and the ADC with the faster Horizontal speed.

All the parameters can be set and get using the corresponding methods, the default values (max speeds and gain)
can be applied with -1 as passed value:

 set/getPGain()
 
 set/getVsSpeed()

 set/getADCSpeed()


Std capabilites
................

This plugin has been implement in respect of the mandatory capabilites but with some limitations which
are due to the camera and SDK features.  We only provide here extra information for a better understanding
of the capabilities for Andor cameras.

* HwDetInfo
  
  getCurrImageType/getDefImageType(): the methods call the  SDK GetBitDepth() function to resolve the image
  data type. The bit-depth correspond to the AD channel dynamic range which depends on the selected ADC channel.
  By experience and with IKon detectors we only have Bpp16 of dynamic range, but the methods can return Bpp8 and Bpp32
  as well.

  setCurrImageType(): this method do not change the image type which is fixed to 16bpp.

* HwSync

  get/setTrigMode(): the only supported mode are IntTrig, ExtTrigSingle, ExtGate and IntTrigMult
  
  

Optional capabilites
........................
In addition to the standard capabilities, we make the choice to implement some optional capabilities which
are supported by the SDK and the I-Kon cameras. A Shutter control, a hardware ROI and a hardware Binning are available.

* HwShutter

  setMode(): only ShutterAuto and ShutterManual modes are supported 

* HwRoi

  There is no restriction for the ROI setting

* HwBin 

  There is no restriction for the Binning but the maximum binning is given by the SDK function GetMaximumBinning() which depends
  on the camera model

Configuration
`````````````

 - Plug your USB camera  on any USB port of the computer, that's all !!!


How to use
````````````
This is a python code example for a simple test:

.. code-block:: python

  from Lima import Andor
  from lima impor Core

  cam = Andor.Camera("/usr/local/etc/andor", 0)
  hwint = Andor.Interface(cam)
  control = Core.control(hwint)

  acq = control.acquisition()

  # configure some hw parameters
  hwint.setTemperatureSP(-30)
  hwint.setCooler(True)
  .... wait here for cooling

  hwint.setPGain(2)


  # setting new file parameters and autosaving mode
  saving=c.saving()

  pars=saving.getParameters()
  pars.directory='/buffer/lcb18012/opisg/test_lima'
  pars.prefix='test1_'
  pars.suffix='.edf'
  pars.fileFormat=Core.CtSaving.EDF
  pars.savingMode=Core.CtSaving.AutoFrame
  saving.setParameters(pars)

  # set accumulation mode

  acq_pars= acq.getPars()

  #0-normal,1-concatenation,2-accumu
  acq_pars.acqMode = 2
  acq_pars.accMaxExpoTime = 0.05
  acq_pars.acqExpoTime =1
  acq_pars.acqNbFrames = 1

  acq.setPars(acq_pars)
  # here we should have 21 accumalated images per frame
  print acq.getAccNbFrames()

  # now ask for 2 sec. exposure and 10 frames
  acq.setAcqExpoTime(2)
  acq.setNbImages(10) 
  
  acq.prepareAcq()
  acq.startAcq()

  

  
