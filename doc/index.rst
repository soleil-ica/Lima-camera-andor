.. _camera-andor:

Andor SDK2 camera plugin
------------------------

.. image:: ikon-m_934_header.jpg
.. image:: ikon-L.jpg

Introduction
````````````
Andor Technology manufactuer offers a large catalogue of scientific cameras. Covered scientific applications are low ligth imaging, spectroscopy, microscopy, time-resolved and high energy detection.
Andor is providing a unique Software Development Tool (SDK) for both Windows and Linux, supporting different interface buses such as USB, CameraLink and also some specific acquisition PCI board.

The Lima module as been tested only with these camera models:
  - IKon-M and IKon-L (USB interface, Linux OS debian 6)
  - IKon-L (USB interface, Windows XP - 32bits)

Prerequisites
`````````````

Linux
.....

First, you have to install the Andor Software developpement Kit (SDK) in the default path (/usr/local). For our tests, we used the SDK for Linux version **V2.91.30001.0** and ran the install script ``install_andor`` for which option 5 (All USB Cameras) was selected, the default installation is made under ``/usr/local/`` with:

  - ``/usr/local/include``, header files
  - ``/usr/local/lib``, library files
  - ``/usr/local/etc/andor``, configuration files

The Linux SDK 2.91 has shared libraries which has been compiled on recent linux kernel, check first you have the right kernel and
libc available by compiling one of the example program available under examples/console.
Andor python module needs at least the lima core module.

For the USB camera the SDK is using the libusb under linux, check first your system is equiped with the libusb package otherwise you
will not compile the Andor Lima plugin.

Windows XP - 32 bits
....................

First, you have to install the Andor Software developpement Kit (SDK) in default path (``C:\\Program Files (x86)\\Andor iKon\\Drivers``).

Add the location of the file ``\\Lima\\camera\\andor\\sdk\\msvc\\bin\\ATMCD32D.DLL`` to your ``PATH`` environment variable.

Installation & Module configuration
```````````````````````````````````

Follow the generic instructions in :ref:`build_installation`. If using CMake directly, add the following flag:

.. code-block:: sh

  -DLIMACAMERA_ANDOR=true

For the Tango server installation, refers to :ref:`tango_installation`.

Initialisation and Capabilities
```````````````````````````````

Implementing a new plugin for new detector is driven by the LIMA framework but the developer has some freedoms to choose which standard and specific features will be made available. This section is supposed to give you the correct information regarding how the camera is exported within the LIMA framework.

Camera initialisation
.....................

The camera will be initialized within the :cpp:class:`AndorCamera`  object. The :cpp:func:`AndorCamera()` contructor sets the camera with default parameters for Preampifier-Gain, VerticalShiftSpeed and the ADC/HorizontalSpeed.

These parameters are optimized for the faster mode, which means the maximum gain, the "fasten recommended" VSSpeed (i.e as returned
by GetFastestRecommendedVSSpeed() SDK function call) and the ADC with the faster Horizontal speed.

All the parameters can be set and get using the corresponding methods, the default values (max speeds and gain)
can be applied with -1 as passed value:

 set/getPGain()

 set/getVsSpeed()

 set/getADCSpeed()

Some other methods are available but they can not be supported depending on which camera model you are using:

 set/getHighCapacity()

 set/getFanMode()

 set/getBaselineClamp()

The above parameters, only support enumerate type for values.

Std capabilities
................

This plugin has been implemented in respect of the mandatory capabilites but with some limitations which
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

Optional capabilities
.....................

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

Plug your USB camera on any USB port of the computer, that's all !

How to use
````````````
This is a python code example for a simple test:

.. code-block:: python

  from Lima import Andor
  from lima import Core

  cam = Andor.Camera("/usr/local/etc/andor", 0)
  hwint = Andor.Interface(cam)
  ct = Core.CtControl(hwint)

  acq = ct.acquisition()

  # configure some hw parameters
  hwint.setTemperatureSP(-30)
  hwint.setCooler(True)
  .... wait here for cooling

  # set some low level configuration
  hwint.setPGain(2)
  hwint.setCooler(True)
  hwint.setFanMode(cam.FAN_ON_FULL)
  hwint.setHighCapacity(cam.HIGH_SENSITIVITY)
  hwint.setBaselineClamp(cam.BLCLAMP_ENABLED)
  hwint.setFastExtTrigger(False)
  hwint.setShutterLevel(1)


  # setting new file parameters and autosaving mode
  saving=ct.saving()

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

  ct.prepareAcq()
  ct.startAcq()

  # wait for last image (#9) ready
  lastimg = ct.getStatus().ImageCounters.LastImageReady
  while lastimg !=9:
    time.sleep(1)
    lastimg = ct.getStatus().ImageCounters.LastImageReady

  # read the first image
  im0 = ct.ReadImage(0)
