Andor
-------

.. image:: ikon-m_934_header.jpg 

Intoduction
```````````
Andor Technology manufactuer offers a large catalogue of scientific cameras. Covered scientific applications are low ligth imaging, spectroscopy, microscopy, time-resolved and high energy detection. 
Andor is providing a unique Software Development Tool (SDK) for both Windows and Linux, supporting different interface buses such as USB, CameraLink and also some specific acquisition PCI board.

The Lima module as been tested only with this cameras models:
  - IKon-M (USB interface, Linux OS)

Module configuration
````````````````````
Previously to this you have to install the Andor SDK the default path (/usr/local).
For our test we us the SDK for Linux version **V2.91.30001.0** and ran the install script "install_andor"
for which option 5 (All USB Cameras) was selected, the default installation is made under /usr/local/ with:

  - /usr/local/include, header files
  - /usr/local/lib, library files
  - /usr/local/etc/andor, configuration files

The Linux SSK 2.91 has shared libraries which has been compiled on recent linux kernel, check first you have the right kernel and
libc available by compiling one of the example program available under examples/console.
Andor python module needs at least the lima core module.

For the USB camera the SDK is using the libusb under linux, check first your system is equiped with the libusb package otherwise you
will not compile the Andor Lima plugin.

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


See :ref:`Compilation`

Installation
`````````````

- After installing andor modules :ref:`installation`

- And probably Tango server :ref:`tango_installation`

Configuration
`````````````

 - Plug your USB camera  on any USB port of the computer, that's all !!!
