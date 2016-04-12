///###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2016
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//############################################################################

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include "AndorCamera.h"

using namespace lima;
using namespace lima::Andor;
using namespace std;

//---------------------------
//- utility function
//---------------------------
//static methods predefinition
static const char* error_code(unsigned int error_code);

#define THROW_IF_NOT_SUCCESS(command,error_prefix)			\
{									\
  unsigned int ret_code = command;						\
  if ( DRV_SUCCESS != ret_code )						\
    THROW_HW_ERROR(Error) << error_prefix << DEB_VAR1(error_code(ret_code)); \
}
//---------------------------
//- utility thread
//---------------------------

class Camera::_AcqThread : public Thread
{
    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "_AcqThread");
public:
    _AcqThread(Camera &aCam);
    virtual ~_AcqThread();
    
protected:
    virtual void threadFunction();
    
private:
    Camera&    m_cam;
};


//---------------------------
// @brief  Ctor
//---------------------------
Camera::Camera(const std::string& config_path,int camera_number)
    : m_status(Ready),
      m_wait_flag(true),
      m_quit(false),
      m_thread_running(true),
      m_image_number(0),
      m_latency_time(0.),
      m_bin(1,1),
      m_shutter_state(false),
      m_fan_mode(FAN_UNSUPPORTED),
      m_high_capacity(HC_UNSUPPORTED),
      m_camera_handle(0),
      m_adc_speed_number(0),
      m_adc_speed_max(0),
      m_adc(-1),
      m_vss_best(0),
      m_vss(-1),
      m_gain_number(0),
      m_gain_max(0),
      m_gain(-1),
      m_fasttrigger(0),
      m_shutter_level(0),
      m_shutter_close_time(0),
      m_shutter_open_time(0),
      m_temperature_sp(0),
      m_exp_time(1.)
{
    DEB_CONSTRUCTOR();
    m_config_path = config_path;
    m_camera_number = camera_number;
  
    // --- Get available cameras and select the choosen one.
#if defined(WIN32)
    long numCameras;	
#else
    int numCameras;
#endif	
    DEB_TRACE() << "Get all attached cameras";
    THROW_IF_NOT_SUCCESS(GetAvailableCameras(&numCameras), "No camera present!");
    
    DEB_TRACE() << "Found "<< numCameras << " camera" << ((numCameras>1)? "s": "");
    DEB_TRACE() << "Try to set current camera to number " << m_camera_number;
    
    if (m_camera_number < numCameras && m_camera_number >=0)
    {        
        THROW_IF_NOT_SUCCESS(GetCameraHandle(m_camera_number, &m_camera_handle),"Cannot get camera handle");
	THROW_IF_NOT_SUCCESS(SetCurrentCamera(m_camera_handle), "Cannot set camera handle");
    }
    else
    {
	DEB_ERROR() << "Invalid camera number " << m_camera_number << ", there is "<< numCameras << " available";
	THROW_HW_ERROR(InvalidValue) << "Invalid Camera number ";
    }


    // --- Initialize  the library    
    THROW_IF_NOT_SUCCESS(Initialize((char *)m_config_path.c_str()), "Library initialization failed, check the config. path");
    
    // --- Get camera capabilities
    m_camera_capabilities.ulSize = sizeof(AndorCapabilities);
    THROW_IF_NOT_SUCCESS(GetCapabilities(&m_camera_capabilities), "Cannot get camera capabilities");
     // --- maps detector type
    m_andor_type_maps[0]="PDA"; 
    m_andor_type_maps[1]="IXON";
    m_andor_type_maps[2]="ICCD";
    m_andor_type_maps[3]="EMCCD";
    m_andor_type_maps[4]="CCD";
    m_andor_type_maps[5]="ISTAR";
    m_andor_type_maps[6]="VIDEO";
    m_andor_type_maps[7]="IDUS";
    m_andor_type_maps[8]="NEWTON";
    m_andor_type_maps[9]="SURCAM";
    m_andor_type_maps[10]="USBICCD";
    m_andor_type_maps[11]="LUCA";
    m_andor_type_maps[12]="RESERVED";
    m_andor_type_maps[13]="IKON";
    m_andor_type_maps[14]="INGAAS";
    m_andor_type_maps[15]="IVAC";
    m_andor_type_maps[16]="UNPROGRAMMED";
    m_andor_type_maps[17]="CLARA";
    m_andor_type_maps[18]="USBISTAR";
        
    // --- Get Camera model
    char	model[AT_CONTROLLER_CARD_MODEL_LEN];
    int         serial;
    THROW_IF_NOT_SUCCESS(GetHeadModel(model), "Cannot get camera model");
    THROW_IF_NOT_SUCCESS(GetCameraSerialNumber(&serial), "Cannot get camera serial number");

    m_detector_model = model;
    m_detector_serial = serial;
    m_detector_type = m_andor_type_maps[m_camera_capabilities.ulCameraType];
    
    DEB_TRACE() << "Andor Camera device found:\n" 
		<< "    * Type     : " << m_detector_type << "("
		<< m_camera_capabilities.ulCameraType <<")\n"
		<< "    * Model    : " << m_detector_model <<"\n"
		<< "    * Serial # : " << m_detector_serial;

    
    // --- Initialise deeper parameters of the controller                
    initialiseController();            

    // Fan off and HighCapacity mode as default if supported
    if (m_camera_capabilities.ulSetFunctions & AC_SETFUNCTION_HIGHCAPACITY)
      setHighCapacity(HIGH_CAPACITY);
    if (m_camera_capabilities.ulFeatures & AC_FEATURES_FANCONTROL)
      setFanMode(FAN_OFF);

    //--- Set detector for single image acquisition and get max binning
    m_read_mode = 4;
    THROW_IF_NOT_SUCCESS(SetReadMode(m_read_mode), "Cannot camera read mode");
 
    int xbin_max, ybin_max;   
    THROW_IF_NOT_SUCCESS(GetMaximumBinning(m_read_mode, 0, &xbin_max), "Cannot get the horizontal maximum binning");
    THROW_IF_NOT_SUCCESS(GetMaximumBinning(m_read_mode, 1, &ybin_max), "Cannot get the vertical maximum binning");

    m_bin_max = Bin(xbin_max, ybin_max);   
    DEB_TRACE() << "Maximum binning : " << xbin_max << " x " << ybin_max;                   

    // --- set default ROI because there is no way to read bck th image size
    // --- BIN already set to 1,1 above.
    // --- Andor sets the ROI by starting coordinates at 1 and not 0 !!!!
    Size sizeMax;
    getDetectorImageSize(sizeMax);
    Roi aRoi = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    
    
    // --- setRoi applies both bin and roi
    DEB_TRACE() << "Set the ROI to full frame: "<< aRoi;
    setRoi(aRoi);
    
    
    // --- Get the maximum exposure time allowed and set default
    THROW_IF_NOT_SUCCESS(GetMaximumExposure(&m_exp_time_max), "Cannot get the maximum exposure time");
    DEB_TRACE() << "Maximum exposure time : "<< m_exp_time_max << "sec.";
    
    setExpTime(m_exp_time);
    
    // --- Set detector for software single image mode    
    m_trig_mode_maps[IntTrig] = 0;
    m_trig_mode_maps[ExtTrigMult] = 1;
    m_trig_mode_maps[ExtTrigSingle] = 6;
    m_trig_mode_maps[ExtGate] = 7;
    m_trig_mode_maps[IntTrigMult] = 10;  
    setTrigMode(IntTrig);
    
    // --- Set the Andor specific acquistion mode.
    // --- We set acquisition mode to run-till-abort 
    m_acq_mode = 5; //Run Till Abort
    m_nb_frames = 1;
    THROW_IF_NOT_SUCCESS(SetAcquisitionMode(m_acq_mode), "Cannot set the acquisition mode");
     // --- set shutter mode to FRAME
    setShutterMode(FRAME);        
    
    // --- finally start the acq thread
    m_acq_thread = new _AcqThread(*this);
    m_acq_thread->start();
}

//---------------------------
// @brief  Dtor
//---------------------------
Camera::~Camera()
{
    DEB_DESTRUCTOR();
    // Stop Acq thread
    delete m_acq_thread;
    m_acq_thread = NULL;
                
    // Close camera
    if (m_cooler)
    {
	DEB_ERROR() <<"Please stop the cooling before shuting dowm the camera\n"                             
		    << "brutale heating could damage the sensor.\n"
		    << "And wait until temperature rises above 5 deg, before shuting down.";

	THROW_HW_ERROR(Error)<<"Please stop the cooling before shuting dowm the camera\n"                             
			     << "brutale heating could damage the sensor.\n"
			     << "And wait until temperature rises above 5 deg, before shuting down.";
    }
    
    DEB_TRACE() << "Shutdown camera";
    ShutDown();
    m_camera_handle = 0;
}

//---------------------------
// @brief  prepare the acquistion
//---------------------------
void Camera::prepareAcq()
{
    DEB_MEMBER_FUNCT();
    m_image_number=0;
    
    THROW_IF_NOT_SUCCESS(PrepareAcquisition(), "Cannot prepare acquisition");
}
//---------------------------
// @brief  start the acquistion
//---------------------------
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();
        
    // --- check first the acquisition is idle
    int status;
    THROW_IF_NOT_SUCCESS(GetStatus(&status), "Cannot get status");
    if (status != DRV_IDLE)
    {
        _setStatus(Camera::Fault,false);        
        THROW_HW_ERROR(Error) << "Cannot start acquisition, camera is not idle";            
    }   
    
    // --- Don't forget to request the maximum number of images the circular buffer can store
    // --- based on the current acquisition settings.
    THROW_IF_NOT_SUCCESS(GetSizeOfCircularBuffer(&m_ring_buffer_size), "Cannot get size of circular buffer");

    DEB_TRACE() << "Andor Circular buffer size = " << m_ring_buffer_size << " images";
            
    // Wait running stat of acquisition thread
    AutoMutex aLock(m_cond.mutex());
    m_wait_flag = false;
    m_cond.broadcast();
    while(!m_thread_running)
        m_cond.wait();
       
    if(m_image_number == 0)
    {
        StdBufferCbMgr& buffer_mgr = m_buffer_ctrl_obj.getBuffer();
	buffer_mgr.setStartTimestamp(Timestamp::now());
	THROW_IF_NOT_SUCCESS(StartAcquisition(), "Cannot start acquisition");
    }
    // in external mode even with FastExtTrigger enabled the camera can not grab the trigger
    // within a too short delay, 100ms is the minimum required, very slow camera !!!
    // and unfortunately the status is not reflecting this lack of synchro.
    //while(1)
    //{
    //    if (andorError(GetStatus(&status)))
    //    {
    //        DEB_ERROR() << "Cannot get status" << " : error code = " << m_camera_error_str;
    //        THROW_HW_ERROR(Error) << "Cannot get status";            
    //    }
    //    if (status== DRV_ACQUIRING) break;
    //    usleep(1e3); 
    //}
    if (m_trig_mode != IntTrig && m_trig_mode != IntTrigMult)
    {
#if defined(WIN32)
        Sleep(0.1);
#else
        usleep(1e5);
#endif		
    }
    if (m_trig_mode == IntTrigMult)
        THROW_IF_NOT_SUCCESS(SendSoftwareTrigger(), "Cannot start acquisition");

}

//---------------------------
// @brief stop the acquisition
//---------------------------
void Camera::stopAcq()
{
    _stopAcq(false);
}

//---------------------------
// @brief private method
//---------------------------
void Camera::_stopAcq(bool internalFlag)
{
    DEB_MEMBER_FUNCT();

    AutoMutex aLock(m_cond.mutex());
    if(m_status != Camera::Ready)
    {
        while(!internalFlag && m_thread_running)
        {
	    // signal the acq. thread to stop acquiring and to return the wait state
            m_wait_flag = true;

	    // Thread is maybe waiting for the Andor acq. event
	    THROW_IF_NOT_SUCCESS(CancelWait(), "CancelWait() failed");
            m_cond.wait();
        }
	aLock.unlock();

        //Let the acq thread stop the acquisition
        if(!internalFlag) return;
            
        // Stop acquisition
        DEB_TRACE() << "Stop acquisition";
        THROW_IF_NOT_SUCCESS(AbortAcquisition(), "Cannot abort acquisition");
        _setStatus(Camera::Ready,false);    
    }
}
//---------------------------
// @brief the thread function for acquisition
//---------------------------
void Camera::_AcqThread::threadFunction()
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cam.m_cond.mutex());
    StdBufferCbMgr& buffer_mgr = m_cam.m_buffer_ctrl_obj.getBuffer();

    while(!m_cam.m_quit)
    {
        while(m_cam.m_wait_flag && !m_cam.m_quit)
        {
            DEB_TRACE() << "Wait";
            m_cam.m_thread_running = false;
            m_cam.m_cond.broadcast();
            m_cam.m_cond.wait();
        }
        DEB_TRACE() << "Run";
        m_cam.m_thread_running = true;
        if(m_cam.m_quit) return;
    
        m_cam.m_status = Camera::Exposure;
        m_cam.m_cond.broadcast();
        aLock.unlock();

        bool continueAcq = true;
	
#if defined(WIN32)
	long first = 0, last = 0, prev_last = 0;		
	long validfirst, validlast;
#else
	int first = 0, last = 0, prev_last = 0;
	int validfirst, validlast;	
#endif	
	FrameDim frame_dim = buffer_mgr.getFrameDim();
	Size  frame_size = frame_dim.getSize();
	int size = frame_size.getWidth() * frame_size.getHeight();
	int ret;

        while(continueAcq && (!m_cam.m_nb_frames || m_cam.m_image_number < m_cam.m_nb_frames))
        {
	    // Check first if acq. has been stopped
	    if (m_cam.m_wait_flag) 
	    {
		continueAcq = false;
		continue;
	    }
	    // Wait for an "acquisition" event, and use less cpu resources, in kinetic mode (multiframe)
            // an event is generated for each new image
	    if ((ret = WaitForAcquisition()) != DRV_SUCCESS)
	    {
		// If CancelWait() or acq. not started yet
		if(ret == DRV_NO_NEW_DATA) continue;
		else 
		{
		  DEB_ERROR() << "WaitForAcquisition() failed" << " : error code = " << error_code(ret);
		    THROW_HW_ERROR(Error) << "WaitForAcquisition() failed";
		}
	    }

            // --- Get the available images in cicular buffer            
            prev_last = last;
            if ((ret = GetNumberNewImages(&first, &last)) != DRV_SUCCESS)
            {
                if (ret == DRV_NO_NEW_DATA) continue;
                else
                {
		  DEB_ERROR() << "Cannot get number of new images" << " : error code = " << error_code(ret);
                    THROW_HW_ERROR(Error) << "Cannot get number of new images";
                }
            }        
            DEB_TRACE() << "Available images: first = " << first << " last = " << last;
            // Check if we lose an image
	    if(first != prev_last +1 )
	    {
		m_cam._setStatus(Camera::Fault,false);
		continueAcq = false;
		DEB_ERROR() << "Lost image(s) from " << prev_last << "to "<< first-1;
		THROW_HW_ERROR(Error) << "Lost image(s) from " << prev_last << "to "<< first-1;	
	    }
            // --- Images are available, process images
            m_cam._setStatus(Camera::Readout,false);
	    
            for (long im=first; im <= last; im++)
            {
                DEB_TRACE()  << "image #" << m_cam.m_image_number <<" acquired !";
                // ---  must get image one by one to copy to the buffer manager
                void *ptr = buffer_mgr.getFrameBufferPtr(m_cam.m_image_number);
                
                if ((ret=GetImages16(im, im,(unsigned short*) ptr, (unsigned long)size,&validfirst, &validlast))!= DRV_SUCCESS)
                {
                    m_cam._setStatus(Camera::Fault,false);
                    continueAcq = false;
                    DEB_TRACE() << "size = " << size;
                    DEB_ERROR() << "Cannot get image #" << im << " : error code = " << error_code(ret);
                    THROW_HW_ERROR(Error) << "Cannot get last image";                
                }
                HwFrameInfoType frame_info;
                frame_info.acq_frame_nb = m_cam.m_image_number;
                continueAcq = buffer_mgr.newFrameReady(frame_info);
                DEB_TRACE() << DEB_VAR1(continueAcq);
                ++m_cam.m_image_number;
            }
        }

        m_cam._stopAcq(true);

        aLock.lock();
        m_cam.m_wait_flag = true;
    }
}

//-----------------------------------------------------
// @brief the acquisition thread Ctor
//-----------------------------------------------------
Camera::_AcqThread::_AcqThread(Camera &aCam) :
    m_cam(aCam)
{
    pthread_attr_setscope(&m_thread_attr,PTHREAD_SCOPE_PROCESS);
}
//-----------------------------------------------------
// @brief the acquisition thread Dtor
//-----------------------------------------------------
Camera::_AcqThread::~_AcqThread()
{
    AutoMutex aLock(m_cam.m_cond.mutex());
    m_cam.m_quit = true;
    m_cam.m_cond.broadcast();
    aLock.unlock();
    
    join();
}

//-----------------------------------------------------
// brief return the detector image size 
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    int xmax, ymax;
    
    // --- Get the max image size of the detector
    THROW_IF_NOT_SUCCESS(GetDetector(&xmax, &ymax), "Cannot get detector size");
    size= Size(xmax, ymax);
}


//-----------------------------------------------------
// @brief return the image type 
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
    DEB_MEMBER_FUNCT();
    int bits;
    // --- Get the AD channel dynamic range in bits per pixel
    // --- suppose here channel 0 is set, in fact we do not provide any
    // --- command to select a different ADC if the detector has several.

    
    THROW_IF_NOT_SUCCESS(GetBitDepth(m_adc, &bits), "Cannot get detector bit depth");
    // --- not clear from documentation with bit depth are possible
    // --- according to the AndorCapabilites structure cameras can support more image type
    // --- with color ones as well.
 
 
    if (bits <=8) type = Bpp8;
    else if (bits <=16) type = Bpp16;
    else type = Bpp32;
       
    // --- previous code do not return the data image type.
    // --- can either be bpp16 or bpp32, just depends on the function used
    // --- to read the image (GetImages()- 32bpp, GetImage16() - 16bpp
    // ---  will later on add support for 32bpp if requested.

}

//-----------------------------------------------------
// @brief set the image type, if supported
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
    DEB_MEMBER_FUNCT();
    // --- see above for future immprovement
    return;
}
//-----------------------------------------------------
// @brief return the detector type
//-----------------------------------------------------
void Camera::getDetectorType(string& type)
{
    DEB_MEMBER_FUNCT();
    
    type = m_detector_type;
}

//-----------------------------------------------------
// @brief return the detector model
//-----------------------------------------------------
void Camera::getDetectorModel(string& type)
{
    DEB_MEMBER_FUNCT();
    stringstream ss;
    ss << ", S/N. "<< m_detector_serial;
    type = m_detector_model + ss.str();
}

//-----------------------------------------------------
// @brief return the internal buffer manager
//-----------------------------------------------------
HwBufferCtrlObj* Camera::getBufferCtrlObj()
{
    DEB_MEMBER_FUNCT();
    return &m_buffer_ctrl_obj;
}


//-----------------------------------------------------
// @brief return true if passed trigger mode is supported
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode trig_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(trig_mode);
    bool valid_mode; 
    int ret;



    switch (trig_mode)
    {       
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
    case ExtTrigMult:
    case ExtGate:
	ret = IsTriggerModeAvailable(m_trig_mode_maps[trig_mode]);
	switch (ret)
	{
	case DRV_SUCCESS:
	    valid_mode = true;
	    break;
	case DRV_INVALID_MODE:
	    valid_mode = false;
	    break;
	case DRV_NOT_INITIALIZED:                
	    valid_mode = false;
	    DEB_ERROR() << "System not initializsed, cannot get trigger mode status" << " : error code = " << error_code(ret);
	    THROW_HW_ERROR(Error) << "System not initializsed, cannot get trigger mode status";
	    break;                                                     
	}                
        break;

    default:
	valid_mode = false;
        break;
    }
    return valid_mode;
}
//-----------------------------------------------------
// @brief set the new trigger mode
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(mode);

    THROW_IF_NOT_SUCCESS(SetTriggerMode(m_trig_mode_maps[mode]), "Cannot set trigger mode");
    m_trig_mode = mode;    
}

//-----------------------------------------------------
// @brief return the current trigger mode
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_trig_mode;
    
    DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
// @brief set the new exposure time
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(exp_time);
    
    THROW_IF_NOT_SUCCESS(SetExposureTime((float)exp_time), "Cannot set exposure time");
    m_exp_time = exp_time;
}

//-----------------------------------------------------
// @brief return the current exposure time
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
    DEB_MEMBER_FUNCT();
    float exp, acc, kin;
    
    // --- because Andor can adjust the exposure time
    // --- need to hw read the acquisition timings here.
    // --- kin time is the kinetic (multi-frame) time between two frames
    THROW_IF_NOT_SUCCESS(GetAcquisitionTimings(&exp, &acc, &kin),  "Cannot get acquisition timings");
    m_exp_time = exp;
    m_kin_time = kin;
    
    exp_time = (double) exp;
    
    DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
// @brief set the new latency time between images
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(lat_time);
    float exp, acc, kin;
        
    // --- Changing the latency time changes the kinetic cycle time
    // --- need to read back the timings which can differ from the set values.
    
    THROW_IF_NOT_SUCCESS(GetAcquisitionTimings(&exp, &acc, &kin), "Cannot get acquisition timings");
    m_exp_time = exp;
    m_kin_time = exp + lat_time;
    THROW_IF_NOT_SUCCESS(SetKineticCycleTime(m_kin_time), "Cannot set kinetic cycle time");
    m_latency_time = lat_time;
    
}

//-----------------------------------------------------
// @brief return the current latency time
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
    DEB_MEMBER_FUNCT();
    // --- we do calculate the latency by using the kinetic cycle time (time between 2 frames)
    // --- minus the exposure time
    float exp, acc, kin;
    
    // --- because Andor can adjust the exposure time
    // --- need to hw read the acquisition timings here.
    // --- kin time is the kinetic (multi-frame) time between two frames
    // --- we do not know with andor how much is the readout time !!!!
    THROW_IF_NOT_SUCCESS(GetAcquisitionTimings(&exp, &acc, &kin), "Cannot get acquisition timings");
    m_exp_time = exp;
    m_kin_time = kin;
    
    lat_time = kin - exp;
    
    DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
// @brief returnt the exposure time range
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
    DEB_MEMBER_FUNCT();
    
    min_expo = 0.;    
    max_expo = (double) m_exp_time_max;
 
    DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
// @brief return the latency time range
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{   
    DEB_MEMBER_FUNCT();

    // --- no info on min latency
    min_lat = 0.;       
    
    // --- do not know how to get the max_lat, fix it as the max exposure time
    max_lat = (double) m_exp_time_max;

    DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
// @brief set the number of frames to be taken
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(nb_frames);
    // --- Hoops continuous mode not yet supported    
    //    if (nb_frames == 0)
    //    {
    //        DEB_ERROR() << "Sorry continuous acquisition (setNbFrames(0)) not yet implemented";
    //        THROW_HW_ERROR(Error) << "Sorry continuous acquisition (setNbFrames(0)) not yet implemented";                
    //    }
    //    // --- We only work on kinetics mode which allow multi-frames to be taken
    //    // ---
    //    THROW_IF_NOT_SUCCESS(SetNumberKinetics(nb_frames), "Cannot set number of frames");
    m_nb_frames = nb_frames;
}

//-----------------------------------------------------
// @brief return the number of frames to be taken
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
    DEB_MEMBER_FUNCT();
    nb_frames = m_nb_frames;
    DEB_RETURN() << DEB_VAR1(nb_frames);
}

//-----------------------------------------------------
// @brief return the current acquired frames
//-----------------------------------------------------
void Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{ 
    DEB_MEMBER_FUNCT();    
    nb_acq_frames = m_image_number;
}
  
//-----------------------------------------------------
// @brief return the camera status
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    status = m_status;
    //Check if the camera is not waiting for soft. trigger
    if (status == Camera::Readout && 
	m_trig_mode == IntTrigMult)
      {
	status = Camera::Ready;
      }
    DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

//-----------------------------------------------------
// @brief set the new camera status
//-----------------------------------------------------
void Camera::_setStatus(Camera::Status status,bool force)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    if(force || m_status != Camera::Fault)
        m_status = status;
    m_cond.broadcast();
}
//-----------------------------------------------------
// @brief do nothing, hw_roi = set_roi.
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    hw_roi = set_roi;

    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
// @brief set the new roi
//-----------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);

    Point topleft, size;
    int binx, biny;
    int hstart, hend, vstart, vend;
    Roi hw_roi, roiMax;
    Size sizeMax;
    
    getDetectorImageSize(sizeMax);
    roiMax = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    

    // --- Warning, SetImage() needs coodinates in full image size not with binning
    // --- but Lima passes image size with binning applied on
    // --- so set a internal binning factor (binx/biny) for size correction.

    if(m_roi == set_roi) return;    
               
    if(set_roi.isActive() && set_roi != roiMax)
    {
	// --- a real roi available
	hw_roi = set_roi;
	binx = m_bin.getX(); biny = m_bin.getY();    	
    }
    else
    {
	// ---  either No roi or roi fit with max size!!!	
	// --- in that case binning for full size calculation is 1
	hw_roi = roiMax;
	binx=1; biny=1;
    }    
    // --- Andor sets the ROI by starting coordinates at 1 and not 0 !!!!
    // --- Warning, SetImage() needs coodinates in full image size not with binning
    // --- but Lima passes here image size with binning applied on

    topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
    hstart = topleft.x*binx +1;          vstart = topleft.y*biny +1;
    hend   = hstart + size.x*binx -1;    vend   = vstart + size.y*biny -1;
    
    DEB_TRACE() << "bin =  " << m_bin.getX() <<"x"<< m_bin.getY();
    DEB_TRACE() << "roi = " << hstart << "-" << hend << ", " << vstart << "-" << vend;
    //- then fix the new ROI
    THROW_IF_NOT_SUCCESS(SetImage(m_bin.getX(), m_bin.getY(), hstart, hend, vstart, vend), "Cannot set detector ROI");
    // cache the real ROI, used when setting BIN
    m_roi = hw_roi;
}

//-----------------------------------------------------
// @brief return the new roi 
//-----------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    // ---  no way to read the roi, Andor does not provide any function to do that!
    hw_roi = m_roi;
    
    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
// @brief range the binning to the maximum allowed
//-----------------------------------------------------
void Camera::checkBin(Bin &hw_bin)
{
    DEB_MEMBER_FUNCT();


    int x = hw_bin.getX();
    if(x > m_bin_max.getX())
        x = m_bin_max.getX();

    int y = hw_bin.getY();
    if(y > m_bin_max.getY())
        y = m_bin_max.getY();

    hw_bin = Bin(x,y);
    DEB_RETURN() << DEB_VAR1(hw_bin);
}
//-----------------------------------------------------
// @brief set the new binning mode
//-----------------------------------------------------
void Camera::setBin(const Bin &set_bin)
{
    DEB_MEMBER_FUNCT();
    Point topleft, size;
    
    int binx, biny;
    int hstart, hend, vstart, vend;
    Roi hw_roi, roiMax;
    Size sizeMax;

    getDetectorImageSize(sizeMax);
    roiMax = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());

    if(m_bin == set_bin) return;

    // --- Warning, SetImage() needs coodinates in full image size not with binning
    // --- but Lima passes image size with binning applied on
    // --- so set a internal binning factor (binx/biny) for size correction.
               
    if(m_roi.isActive() && m_roi != roiMax) 
    {
	// --- a real available
	binx = set_bin.getX();  biny = set_bin.getY();
	hw_roi = m_roi;
    }
    else
    {
	// ---  either No roi or roi fit with max size!!!	
	// --- in that case binning for full size calculation is 1
	hw_roi = roiMax;
	binx = 1; biny = 1;
    }
    topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
    hstart = topleft.x*binx +1;          vstart = topleft.y*biny +1;
    hend   = hstart + size.x*binx -1;    vend   = vstart + size.y*biny -1;

    DEB_TRACE() << "bin =  " << set_bin.getX() <<"x"<< set_bin.getY();
    DEB_TRACE() << "roi = " << hstart << "-" << hend << ", " << vstart << "-" << vend;
    THROW_IF_NOT_SUCCESS(SetImage(set_bin.getX(), set_bin.getY(), hstart, hend, vstart, vend), "Cannot set detector BIN");
    m_bin = set_bin;
    
    DEB_RETURN() << DEB_VAR1(set_bin);
}

//-----------------------------------------------------
// @brief return the current binning mode
//-----------------------------------------------------
void Camera::getBin(Bin &hw_bin)
{
    DEB_MEMBER_FUNCT();
    // ---  no way to read the bin Andor does not provide any function to do that!
    hw_bin = m_bin;
    
    DEB_RETURN() << DEB_VAR1(hw_bin);
}

//-----------------------------------------------------
// @brief return always true, hw binning mode is supported
//-----------------------------------------------------
bool Camera::isBinningAvailable()
{
    DEB_MEMBER_FUNCT();
    bool isAvailable = true;

    // --- ok not realy need this function but could be completed
    // for further camera model which do not support binning
    return isAvailable;
}


//-----------------------------------------------------
// @brief return the detector pixel size in meter
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
    DEB_MEMBER_FUNCT();
    float xsize, ysize;
    
    THROW_IF_NOT_SUCCESS(GetPixelSize(&xsize, &ysize), "Cannot get pixel size");
    sizex = xsize * 1e-6;
    sizey = ysize * 1e-6;
    DEB_RETURN() << DEB_VAR2(sizex, sizey); 
}


//-----------------------------------------------------
// @brief reset the camera, no hw reset available on Andor camera
//-----------------------------------------------------
void Camera::reset()
{
    DEB_MEMBER_FUNCT();
    return;
}


//-----------------------------------------------------
// @brief    initialise controller with speeds and preamp gain
//-----------------------------------------------------
void Camera::initialiseController()
{
    DEB_MEMBER_FUNCT();
    // --- Init adc / speed
    initAdcSpeed();
       
    DEB_TRACE() << "* Horizontal Shift Speed:";
    int is;
    for (is=0; is< m_adc_speed_number; is++)
    {
        DEB_ALWAYS() << "    (" << is << ") adc #" << m_adc_speeds[is].adc << ", speed = " 
                    << m_adc_speeds[is].speed  << ((is == m_adc_speed_max)? " [max]": "");                        
    }
        
    // --- Set adc / speed to max
    setAdcSpeed(m_adc_speed_max);
    DEB_ALWAYS() << "    => Set to " << m_adc_speeds[m_adc_speed_max].speed << "MHz";
       
        
    // --- Init VS Speeds 
    initVsSpeed();
      
    DEB_TRACE() << "* Vertical Shift Speed:";
    for (is=0; is<m_vss_number; is++)
    {
        DEB_ALWAYS() << "    (" << is << ") speed = " << m_vsspeeds[is] << " us"
                    << ((is == m_vss_best)? " [recommended]": "");
    }
        
    // --- Set VS Speed to fasten recommended
    setVsSpeed(m_vss_best);
    DEB_ALWAYS() << "    => Set " << m_vsspeeds[m_vss] << "us";
        
        
    // --- Init Preamp Gain to max
    initPGain();
       
    DEB_ALWAYS() << "* Preamp Gain:";
        
    for (is=0; is< m_gain_number; is++)
    {
        DEB_ALWAYS() << "    (" << is << ") gain = x" << m_preamp_gains[is]
                    << ((is == m_gain_max)? " [max]": "");
    }
    
    // --- Set Preamp Gain
    setPGain(m_gain_max);
    DEB_ALWAYS() << "    => Set to x" << m_preamp_gains[m_gain];                 
}
//-----------------------------------------------------
// @brief get possible adc/speed for controller
//
// Initialise the list of possible pairs adc/speed
// and find the maximum speed.
//
//-----------------------------------------------------
void Camera::initAdcSpeed()
{
    DEB_MEMBER_FUNCT();
    int		ih, ia, is, nadc, *nSpeed;
    float	speedMax;
    
    
    // --- number of ADC
    THROW_IF_NOT_SUCCESS(GetNumberADChannels(&nadc), "Cannot get number of ADC");
    // --- Get Horizontal Shift Speed per ADC
    nSpeed = new int[nadc];
    
    m_adc_speed_number= 0;
    for (ia=0; ia<nadc; ia++) 
    {
      THROW_IF_NOT_SUCCESS(GetNumberHSSpeeds(ia, 0, &nSpeed[ia]), "Cannot get nb of Horizontal Speed for an ADC");
	m_adc_speed_number += nSpeed[ia];
			
    }

    m_adc_speeds = new Adc[m_adc_speed_number];
    speedMax= 0.;
    is= 0;
    for (ia=0; ia<nadc; ia++) {
	for (ih=0; ih<nSpeed[ia]; ih++) {
	    THROW_IF_NOT_SUCCESS(GetHSSpeed(ia, 0, ih, &m_adc_speeds[is].speed), "Cannot get Horizontal Speed ");
	    m_adc_speeds[is].adc= ia;
	    m_adc_speeds[is].hss= ih;	    // --- iKon/iXon= speed in MHz ; others in us/pixel shift --> convert in MHz
	    if ((m_camera_capabilities.ulCameraType!=1)&&(m_camera_capabilities.ulCameraType!=13))
		m_adc_speeds[is].speed = (float)(1./ m_adc_speeds[is].speed);

	    if (m_adc_speeds[is].speed > speedMax) {
		speedMax= m_adc_speeds[is].speed;
		m_adc_speed_max= is;
	    }
	    is++;
	}
    }      
}

 //-----------------------------------------------------
// @brief	set ADC/Speed settings
// @param	adc pair adc/speed index (if =-1, set to max speed)
//
//-----------------------------------------------------
void Camera::setAdcSpeed(int adc)
{
    DEB_MEMBER_FUNCT();
    int is;
    
    // -- Initialise ad speed
    if ((adc == -1) || (adc > m_adc_speed_number))
    {
        is = m_adc_speed_max;
    }
    else
    {
        is  = adc;
    }
    THROW_IF_NOT_SUCCESS(SetADChannel(m_adc_speeds[is].adc), "Failed to set ADC channel");
    THROW_IF_NOT_SUCCESS(SetHSSpeed(0, m_adc_speeds[is].hss), "Failed to set HSS");
    m_adc = is;
    
    DEB_TRACE() << "ADC speed set to " << m_adc_speeds[is].speed << " MHz";
}


//-----------------------------------------------------
// @brief	get ADC/Speed settings
// @param	adc index
//
//-----------------------------------------------------
void Camera::getAdcSpeed(int& adc)
{
    adc = m_adc;
}

//-----------------------------------------------------
// @brief	get ADC/Speed settings
// @param	adc frequency
//
//-----------------------------------------------------
void Camera::getAdcSpeedInMhz(float& adc)
{
    adc = m_adc_speeds[m_adc].speed;
}

//-----------------------------------------------------
// @brief get possible VSS (vertical shift speed) for controller
//
// Initialise the list of possible vss index and their value
// Get also the recommended VSS.
//-----------------------------------------------------
void Camera::initVsSpeed()
{
    DEB_MEMBER_FUNCT();
    float speed;
    int ivss;
    int ret;


    // --- number of ADC
    THROW_IF_NOT_SUCCESS(GetNumberVSSpeeds(&m_vss_number), "Cannot get number of possible VSS");

    // --- get VSS value for each
    m_vsspeeds = new float[m_vss_number];
    for (ivss=0; ivss<m_vss_number; ivss++)
    {
        THROW_IF_NOT_SUCCESS(GetVSSpeed(ivss, &m_vsspeeds[ivss]), "Cannot get VSS value");
    }

    // --- get recommended VSS value
    if ((ret=GetFastestRecommendedVSSpeed(&m_vss_best, &speed)) != DRV_SUCCESS)
      {
	m_vss_best = 0;
        DEB_ERROR() << "Cannot get recommended VSS speed. Set it to 0" <<" : error code = " << error_code(ret);
        THROW_HW_ERROR(Error) << "Cannot get recommended VSS speed. Set it to 0";

    }
}


//-----------------------------------------------------
// @brief	set Vertical Shift Speed
// @param	vss index (if =-1, set to recommended)
//
//-----------------------------------------------------
void Camera::setVsSpeed(int vss) 
{
    DEB_MEMBER_FUNCT();
    int is;

    if ((vss == -1)||(vss > m_vss_number))
    {
	is = m_vss_best;
    } 
    else
    {
	is = vss;
    }
    THROW_IF_NOT_SUCCESS(SetVSSpeed(is), "Failed to set VS speed");
    m_vss = is;

    DEB_TRACE() << "VS speed Set to " <<m_vsspeeds[is] << "us";
}

//-----------------------------------------------------
// @brief	get Vertical Shift Speed
// @param	vss index
//
//-----------------------------------------------------
void Camera::getVsSpeed(int& vss) 
{
    vss = m_vss;
}

//-----------------------------------------------------
// @brief	get possible Preamp Gain values
//
// Initialise the list of possible gain index and their value
//
//-----------------------------------------------------
void Camera::initPGain()
{
    DEB_MEMBER_FUNCT();
    int ig;
    float gmax;

    // --- get number of possible gains
    THROW_IF_NOT_SUCCESS(GetNumberPreAmpGains(&m_gain_number), "Failed to get number of preamp gain");
    // --- get gain value for each
    gmax = 0.;
    m_preamp_gains = new float[m_gain_number];
    for (ig=0; ig<m_gain_number; ig++)
    {
        THROW_IF_NOT_SUCCESS(GetPreAmpGain(ig, &m_preamp_gains[ig]), "Failed to get gain");
	if (m_preamp_gains[ig] >= gmax)
        {
	    gmax = m_preamp_gains[ig];
	    m_gain_max = ig;
	}
    }
}

//-----------------------------------------------------
// @brief	set Preamp Gain
// @param	gain preamp gain index
//
//-----------------------------------------------------
void Camera::setPGain(int gain) 
{
    DEB_MEMBER_FUNCT();
    int ig;

    if (gain==-1) 
    {
	ig= m_gain_max;
    }
    else
    {
        if (gain<-1 || gain>=m_gain_number)
        {
	    DEB_ERROR() << "Invalid gain index range is" << "[0," << m_gain_number-1 << "] or -1 for max gain";
	    THROW_HW_ERROR(Error) << "Invalid gain index range is" << "[0," << m_gain_number-1 << "] or -1 for max gain"; 
	}
	ig= gain;
    }

    THROW_IF_NOT_SUCCESS(SetPreAmpGain(ig), "Failed to set Preamp Gain");
    m_gain= ig;

    DEB_TRACE() << "Preamp Gain set to x" << m_preamp_gains[ig];
}

//-----------------------------------------------------
// @brief	get Preamp Gain
// @param	gain preamp gain index
//
//-----------------------------------------------------
void Camera::getPGain(int& gain) 
{
    gain = m_gain;
}

//-----------------------------------------------------
// @brief	set external trigger for fast mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Camera::setFastExtTrigger(bool flag)
{
    DEB_MEMBER_FUNCT();
    THROW_IF_NOT_SUCCESS(SetFastExtTrigger((flag)?1:0), "Failed to set ext-trigger fast mode");
    m_fasttrigger = flag;

}

//-----------------------------------------------------
// @brief	get external fast trigger mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Camera::getFastExtTrigger(bool& flag)
{
    DEB_MEMBER_FUNCT();
    flag = m_fasttrigger;
}


//-----------------------------------------------------
// @brief	set the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::setShutterLevel(int level)
{
    DEB_MEMBER_FUNCT();

    DEB_TRACE() << "Camera::setShutterLevel - " << DEB_VAR1(level);		
    THROW_IF_NOT_SUCCESS(SetShutter(level, m_shutter_mode, m_shutter_close_time, m_shutter_open_time),  "Failed to set shutter level");
    m_shutter_level = level;
}

//-----------------------------------------------------
// @brief	get the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::getShutterLevel(int& level)
{
    DEB_MEMBER_FUNCT();
    level = m_shutter_level;    
}


//-----------------------------------------------------
// @brief	set the shutter mode 
// @param	mode FRAME or MANUAL
//
//-----------------------------------------------------
void Camera::setShutterMode(ShutterMode mode)
{
    DEB_MEMBER_FUNCT();
    // --- SetShutter() param mode is both used to set  auto or manual mode and to open and close
    // --- 0 - Auto, 1 - Open, 2 - Close	
    int aMode = (mode == FRAME)? 0:2;
	DEB_TRACE() << "Camera::setShutterMode - " << DEB_VAR1(aMode)
                <<" - Close Time  = "<<m_shutter_close_time
                <<" - Open Time  = "<<m_shutter_open_time;		
    if (mode == FRAME)
    {    
        THROW_IF_NOT_SUCCESS(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time), "Failed to set the shutter mode");
    }
    m_shutter_mode = mode;
}

//-----------------------------------------------------
// @brief	return the shutter mode
// @param	mode FRAME or MANUAL
//
//-----------------------------------------------------
void Camera::getShutterMode(ShutterMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_shutter_mode;
}


//-----------------------------------------------------
// @brief	set the shutter open or close
// @param	flag True-Open / False-Close
//
//-----------------------------------------------------
void Camera::setShutter(bool flag)
{
    DEB_MEMBER_FUNCT();
    // --- SetShutter() param mode is both used to set in auto or manual mode and to open and close
    // --- 0 - Auto, 1 - Open, 2 - Close
    int aMode = (flag)? 1:2; 
    DEB_TRACE() << "Camera::setShutter - " << DEB_VAR1(aMode)
                <<" - Close Time  = "<<m_shutter_close_time
                <<" - Open Time  = "<<m_shutter_open_time;			
    THROW_IF_NOT_SUCCESS(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time), "Failed close/open the shutter");
    m_shutter_state = flag;
}


//-----------------------------------------------------
// @brief	return the status of the shutter
// @param	flag True-Open / False-Close
//
//-----------------------------------------------------
void Camera::getShutter(bool& flag)
{
    DEB_MEMBER_FUNCT();
    flag = m_shutter_state;
}

//-----------------------------------------------------
// @brief	set the shutter opening time
// @param	tm time in seconds
//
//-----------------------------------------------------
void Camera::setShutterOpenTime(double tm)
{
    DEB_MEMBER_FUNCT();
    int aTime = tm *1000;
    
    DEB_TRACE() << "Camera::setShutterOpenTime - " << DEB_VAR1(aTime);			
    THROW_IF_NOT_SUCCESS(SetShutter(m_shutter_level, m_shutter_mode, m_shutter_close_time, aTime),  "Failed to set shutter opening time");
    m_shutter_open_time = aTime;
}

//-----------------------------------------------------
// @brief	get the shutter opening time
// @param   tm time in seconds
//
//-----------------------------------------------------
void Camera::getShutterOpenTime(double& tm)
{
    DEB_MEMBER_FUNCT();
    tm = m_shutter_open_time/1000;
}

//-----------------------------------------------------
// @brief	set the shutter closing time
// @param	tm time in seconds
//
//-----------------------------------------------------
void Camera::setShutterCloseTime(double tm)
{
    DEB_MEMBER_FUNCT();
    int aTime = tm *1000;
    DEB_TRACE() << "Camera::setShutterCloseTime - " << DEB_VAR1(aTime);			    
    THROW_IF_NOT_SUCCESS(SetShutter(m_shutter_level, m_shutter_mode, aTime, m_shutter_open_time), "Failed to set shutter closing time");
    m_shutter_close_time = aTime;
}


//-----------------------------------------------------
// @brief	get the shutter closing time
// @param	tm time in seconds
//
//-----------------------------------------------------
void Camera::getShutterCloseTime(double& tm)
{
    DEB_MEMBER_FUNCT();
    tm = m_shutter_close_time/1000;
}


//-----------------------------------------------------
// @brief	set the temperature set-point
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::setTemperatureSP(int temp)
{
    DEB_MEMBER_FUNCT();
    THROW_IF_NOT_SUCCESS(SetTemperature(temp), "Failed to set temperature set-point");
    m_temperature_sp = temp;
}

//-----------------------------------------------------
// @brief	return the temperature set-point
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperatureSP(int& temp)
{
    DEB_MEMBER_FUNCT();
    temp = m_temperature_sp;
}


//-----------------------------------------------------
// @brief	Gets the real temperature of the detector sensor 
// @param	temp temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperature(int& temp)
{
    DEB_MEMBER_FUNCT();
    int tm;
    unsigned int ret;
    if ((ret = GetTemperature(&tm)) == DRV_ERROR_ACK)
    {
        THROW_HW_ERROR(Error) << "Failed to read temperature";          
    }
    temp = tm;   
}
//-----------------------------------------------------
// @brief	start or Stop the cooler 
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void Camera::setCooler(bool flag)
{
    DEB_MEMBER_FUNCT();
    if (flag)
    {
        THROW_IF_NOT_SUCCESS(CoolerON(), "Failed to start the cooler");
    } 
    else
    {
	THROW_IF_NOT_SUCCESS(CoolerOFF(), "Failed to stop the cooler");
    }
    
    m_cooler = flag;        
}
//-----------------------------------------------------
// @brief	Get the Cooler status  
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void Camera::getCooler(bool& flag)
{
    DEB_MEMBER_FUNCT();
    flag = m_cooler;
}

//-----------------------------------------------------
// @brief	Gets cooling status
// @param	status status as a string
//
//-----------------------------------------------------
void Camera::getCoolingStatus(std::string& status)   
{
    DEB_MEMBER_FUNCT();
    
    int tm;
    unsigned int stat;
    if ((stat = GetTemperature(&tm)) == DRV_ERROR_ACK)
    {
        DEB_ERROR() << "Failed to read temperature" <<" : error code = " << DRV_ERROR_ACK;
        THROW_HW_ERROR(Error) << "Failed to read temperature";          
    }

    switch (stat)
    {
    case DRV_NOT_INITIALIZED:
	status = "System not initialized"; break;
    case DRV_ACQUIRING:
	status = "Acquisition in progress"; break;        
    case DRV_TEMP_OFF:
	status = "Temperature is OFF"; break;        
    case DRV_TEMP_STABILIZED:
	status = "Temperature has stabilized at set point"; break;        
    case DRV_TEMP_NOT_REACHED:
	status = "Temperature has not reached set point"; break;        
    case DRV_TEMP_DRIFT:
	status = "Temperature had stabilized but has since drifted"; break;        
    case DRV_TEMP_NOT_STABILIZED:
	status = "Temperature reached but not stabilized"; break;
    default:
	status = "???";                                                                   
    }    
}

 //-----------------------------------------------------
// @brief	Sets spooling specific parameters
// @param	flag            enable/disable spooling
// @param   method          used spooling method
// @param   path            spooling dir/file
// @param   framBufferSize  size of the internal circular buffer
//
//-----------------------------------------------------
void Camera::setSpooling(bool flag, SpoolingMethod method, string path, int frameBufferSize)
{
    DEB_MEMBER_FUNCT();

    int intFlag = flag;
    int intMethod = method;

    THROW_IF_NOT_SUCCESS(SetSpool(intFlag,intMethod,const_cast<char*>(path.c_str()),frameBufferSize), "Failed to configure spooling");

}


//-----------------------------------------------------
// @brief	get the High capacity mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Camera::getHighCapacity(HighCapacityMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_high_capacity ;
}

//-----------------------------------------------------
// @brief	set the High capacity mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Camera::setHighCapacity(HighCapacityMode mode)
{
    DEB_MEMBER_FUNCT();

    if (m_camera_capabilities.ulSetFunctions & AC_SETFUNCTION_HIGHCAPACITY)
      {
	THROW_IF_NOT_SUCCESS(SetHighCapacity((int) mode), "Failed to set high capacity mode");
      }
    else 
	THROW_HW_ERROR(Error) << "HighCapacity not supported for this camera model";

    m_high_capacity = mode;
}

//-----------------------------------------------------
// @brief	set the FAN mode ON_FULL/ON_LOW/OFF
// @param	mode    mode to set
//
//-----------------------------------------------------
void Camera::setFanMode(FanMode mode)
{
    DEB_MEMBER_FUNCT();
    if (m_camera_capabilities.ulFeatures & AC_FEATURES_FANCONTROL)
      {
	THROW_IF_NOT_SUCCESS(SetFanMode((int) mode), "Error while setting gate mode");
      }
    else
      THROW_HW_ERROR(Error) << "Fan control not supported for this camera model";
    m_fan_mode = mode;
}

//-----------------------------------------------------
// @brief	get the FAN mode ON_FULL/ON_LOW/OFF
// @param	mode    mode 
//
//-----------------------------------------------------
void Camera::getFanMode(FanMode& mode)
{
    DEB_MEMBER_FUNCT();

    mode  = m_fan_mode;
}


//-----------------------------------------------------
// @brief	Sets the gate mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Camera::setGateMode(GateMode mode)
{
    DEB_MEMBER_FUNCT();

    THROW_IF_NOT_SUCCESS(SetGateMode((int) mode), "Error while setting gate mode");
}




//-----------------------------------------------------
// @brief	Sets the read mode
// @param	mode    mode to set
//
//-----------------------------------------------------
// CANNOT BE USED WITHOUT A MaxImageSizeCallback because the read-mode
// changes the image size.
//void Camera::setReadMode(ReadMode mode)
//{
//    DEB_MEMBER_FUNCT();

//    if(andorError(SetReadMode((int) mode)))
//    {
//        DEB_ERROR() << "Error while setting read mode" <<" : error code = " << m_camera_error_str;
//        THROW_HW_ERROR(Error) << "Failed to set read mode";
//    }
//}


//-----------------------------------------------------
// @brief	set enable (true) or disable (false) the baseline clamping
// @param	enable true or false    
//
//-----------------------------------------------------
void Camera::setBaselineClamp(bool enable)
{
    DEB_MEMBER_FUNCT();
    if (m_camera_capabilities.ulSetFunctions & AC_SETFUNCTION_BASELINECLAMP)
      {
	THROW_IF_NOT_SUCCESS(SetBaselineClamp((enable)?1:0), "Error while setting gate mode");
      }
    else
      THROW_HW_ERROR(Error) << "Baseline Clamp control not supported for this camera model";
}

//-----------------------------------------------------
// @brief	get baseline clamp status
// @param	enable true or false
//
//-----------------------------------------------------
void Camera::getBaselineClamp(bool& enable)
{
    DEB_MEMBER_FUNCT();
    int state;
    if (m_camera_capabilities.ulFeatures & AC_FEATURES_FANCONTROL)
      {
	THROW_IF_NOT_SUCCESS(GetBaselineClamp(&state), "Error while setting gate mode");
      }
    else
      THROW_HW_ERROR(Error) << "Baseline Clamp  control not supported for this camera model";
    enable = (state==1)? true: false;
}


//-----------------------------------------------------
// @brief just build a map of error codes
//-----------------------------------------------------
static const char* error_code(unsigned int error_code)
{
  const char *error;
  switch (error_code)
  {
  case DRV_ERROR_CODES: error = "DRV_ERROR_CODES";
    break;
  case DRV_SUCCESS: error = "DRV_SUCCESS";
    break;
  case DRV_VXDNOTINSTALLED: error  = "DRV_VXDNOTINSTALLED";
    break;
  case DRV_ERROR_SCAN: error = "DRV_ERROR_SCAN";
    break;
  case DRV_ERROR_CHECK_SUM: error = "DRV_ERROR_CHECK_SUM";
    break;
  case DRV_ERROR_FILELOAD: error = "DRV_ERROR_FILELOAD";
    break;
  case DRV_UNKNOWN_FUNCTION: error = "DRV_UNKNOWN_FUNCTION";
    break;
  case DRV_ERROR_VXD_INIT: error = "DRV_UNKNOWN_FUNCTION";
    break;
  case DRV_ERROR_ADDRESS: error = "DRV_UNKNOWN_FUNCTION";
    break;
  case DRV_ERROR_PAGELOCK: error = "DRV_ERROR_PAGELOCK";
    break;
  case DRV_ERROR_PAGEUNLOCK: error = "DRV_ERROR_PAGEUNLOCK";
    break;
  case DRV_ERROR_BOARDTEST: error = "DRV_ERROR_BOARDTEST";
    break;
  case DRV_ERROR_ACK: error = "DRV_ERROR_ACK";
    break;
  case DRV_ERROR_UP_FIFO: error = "DRV_ERROR_UP_FIFO";
    break;
  case DRV_ERROR_PATTERN: error = "DRV_ERROR_PATTERN";
    break;
  case DRV_KINETIC_TIME_NOT_MET: error = "DRV_KINETIC_TIME_NOT_MET";
    break;
  case DRV_ACCUM_TIME_NOT_MET: error = "DRV_ACCUM_TIME_NOT_MET";
    break;
  case DRV_NO_NEW_DATA: error = "DRV_NO_NEW_DATA";
    break;
  case DRV_SPOOLERROR: error = "DRV_SPOOLERROR";
    break;
  case DRV_SPOOLSETUPERROR: error = "DRV_SPOOLSETUPERROR";
    break;
  case DRV_FILESIZELIMITERROR: error = "DRV_FILESIZELIMITERROR";
    break;
  case DRV_ERROR_FILESAVE: error = "DRV_ERROR_FILESAVE";
    break;
  case DRV_TEMPERATURE_CODES: error = "DRV_TEMPERATURE_CODES";
    break;
  case DRV_TEMPERATURE_OFF: error = "DRV_TEMPERATURE_OFF";
    break;
  case DRV_TEMPERATURE_NOT_STABILIZED: error = "DRV_TEMPERATURE_NOT_STABILIZED";
    break;
  case DRV_TEMPERATURE_STABILIZED: error = "DRV_TEMPERATURE_STABILIZED";
    break;
  case DRV_TEMPERATURE_NOT_REACHED: error = "DRV_TEMPERATURE_NOT_REACHED";
    break;
  case DRV_TEMPERATURE_OUT_RANGE: error = "DRV_TEMPERATURE_OUT_RANGE";
  break;
  case DRV_TEMPERATURE_NOT_SUPPORTED: error = "DRV_TEMPERATURE_NOT_SUPPORTED";
    break;
  case DRV_TEMPERATURE_DRIFT: error = "DRV_TEMPERATURE_DRIFT";
    break;
  case DRV_GENERAL_ERRORS: error = "DRV_GENERAL_ERRORS";
    break;
  case DRV_INVALID_AUX: error = "DRV_INVALID_AUX";
    break;
  case DRV_COF_NOTLOADED: error = "DRV_COF_NOTLOADED";
    break;
  case DRV_FPGAPROG: error = "DRV_FPGAPROG";
    break;
  case DRV_FLEXERROR: error = "DRV_FLEXERROR";
    break;
  case DRV_GPIBERROR: error = "DRV_GPIBERROR";
    break;
  case DRV_EEPROMVERSIONERROR: error = "DRV_EEPROMVERSIONERROR";
    break;
  case DRV_DATATYPE: error = "DRV_DATATYPE";
    break;
  case DRV_DRIVER_ERRORS: error = "DRV_DRIVER_ERRORS";
    break;
  case DRV_P1INVALID: error = "DRV_P1INVALID";
    break;
  case DRV_P2INVALID: error = "DRV_P2INVALID";
    break;
  case DRV_P3INVALID: error = "DRV_P3INVALID";
    break;
  case DRV_P4INVALID: error = "DRV_P4INVALID";
    break;
  case DRV_INIERROR: error = "DRV_INIERROR";
    break;
  case DRV_COFERROR: error = "DRV_COFERROR";
    break;
  case DRV_ACQUIRING: error = "DRV_ACQUIRING";
    break;
  case DRV_IDLE: error = "DRV_IDLE";
    break;
  case DRV_TEMPCYCLE: error = "DRV_TEMPCYCLE";
    break;
  case DRV_NOT_INITIALIZED: error = "DRV_NOT_INITIALIZED";
    break;
  case DRV_P5INVALID: error = "DRV_P5INVALID";
    break;
  case DRV_P6INVALID: error = "DRV_P6INVALID";
    break;
  case DRV_INVALID_MODE: error = "DRV_INVALID_MODE";
    break;
  case DRV_INVALID_FILTER: error = "DRV_INVALID_FILTER";
    break;
  case DRV_I2CERRORS: error = "DRV_I2CERRORS";
    break;
  case DRV_I2CDEVNOTFOUND: error = "DRV_I2CDEVNOTFOUND";
    break;
  case DRV_I2CTIMEOUT: error = "DRV_I2CTIMEOUT";
    break;
  case DRV_P7INVALID: error = "DRV_P7INVALID";
    break;
  case DRV_P11INVALID: error = "DRV_P11INVALID";
    break;
  case DRV_USBERROR: error = "DRV_USBERROR";
    break;
  case DRV_IOCERROR: error = "DRV_IOCERROR";
    break;
  case DRV_VRMVERSIONERROR: error = "DRV_VRMVERSIONERROR";
    break;
  case DRV_GATESTEPERROR: error = "DRV_GATESTEPERROR";
    break;
  case DRV_USB_INTERRUPT_ENDPOINT_ERROR: error = "DRV_USB_INTERRUPT_ENDPOINT_ERROR";
    break;
  case DRV_RANDOM_TRACK_ERROR: error = "DRV_RANDOM_TRACK_ERROR";
    break;
  case DRV_INVALID_TRIGGER_MODE: error = "DRV_INVALID_TRIGGER_MODE";
    break;
  case DRV_LOAD_FIRMWARE_ERROR: error = "DRV_LOAD_FIRMWARE_ERROR";
    break;
  case DRV_DIVIDE_BY_ZERO_ERROR: error = "DRV_DIVIDE_BY_ZERO_ERROR";
    break;
  case DRV_INVALID_RINGEXPOSURES: error = "DRV_INVALID_RINGEXPOSURES";
    break;
  case DRV_BINNING_ERROR: error = "DRV_BINNING_ERROR";
    break;
  case DRV_INVALID_COUNTCONVERT_MODE: error = "DRV_INVALID_COUNTCONVERT_MODE";
    break;
  case DRV_NOT_SUPPORTED: error = "DRV_NOT_SUPPORTED";
    break;
  case DRV_NOT_AVAILABLE: error = "DRV_NOT_AVAILABLE";
    break;
  case DRV_ERROR_MAP: error = "DRV_ERROR_MAP";
    break;
  case DRV_ERROR_UNMAP: error = "DRV_ERROR_UNMAP";
    break;
  case DRV_ERROR_MDL: error = "DRV_ERROR_MDL";
    break;
  case DRV_ERROR_UNMDL: error = "DRV_ERROR_UNMDL";
    break;
  case DRV_ERROR_BUFFSIZE: error = "DRV_ERROR_BUFFSIZE";
    break;
  case DRV_ERROR_NOHANDLE: error = "DRV_ERROR_NOHANDLE";
    break;
  case DRV_GATING_NOT_AVAILABLE: error = "DRV_GATING_NOT_AVAILABLE";
    break;
  case DRV_FPGA_VOLTAGE_ERROR: error = "DRV_FPGA_VOLTAGE_ERROR";
    break;
  case DRV_MSTIMINGS_ERROR: error = "DRV_MSTIMINGS_ERROR";
    break;
  case DRV_OA_NULL_ERROR: error = "DRV_OA_NULL_ERROR";
    break;
  case DRV_OA_PARSE_DTD_ERROR: error = "DRV_OA_PARSE_DTD_ERROR";
    break;
  case DRV_OA_DTD_VALIDATE_ERROR: error = "DRV_OA_DTD_VALIDATE_ERROR";
    break;
  case DRV_OA_FILE_ACCESS_ERROR: error = "DRV_OA_FILE_ACCESS_ERROR";
    break;
  case DRV_OA_FILE_DOES_NOT_EXIST: error = "DRV_OA_FILE_DOES_NOT_EXIST";
    break;
  case DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR: error = "DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR";
    break;
  case DRV_OA_PRESET_FILE_NOT_LOADED: error = "DRV_OA_PRESET_FILE_NOT_LOADED";
    break;
  case DRV_OA_USER_FILE_NOT_LOADED: error = "DRV_OA_USER_FILE_NOT_LOADED";
    break;
  case DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED: error = "DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED";
    break;
  case DRV_OA_INVALID_FILE: error = "DRV_OA_INVALID_FILE";
    break;
  case DRV_OA_FILE_HAS_BEEN_MODIFIED: error = "DRV_OA_FILE_HAS_BEEN_MODIFIED";
    break;
  case DRV_OA_BUFFER_FULL: error = "DRV_OA_BUFFER_FULL";
    break;
  case DRV_OA_INVALID_STRING_LENGTH: error = "DRV_OA_INVALID_STRING_LENGTH";
    break;
  case DRV_OA_INVALID_CHARS_IN_NAME: error = "DRV_OA_INVALID_CHARS_IN_NAME";
    break;
  case DRV_OA_INVALID_NAMING: error = "DRV_OA_INVALID_NAMING";
    break;
  case DRV_OA_GET_CAMERA_ERROR: error = "DRV_OA_GET_CAMERA_ERROR";
    break;
  case DRV_OA_MODE_ALREADY_EXISTS: error = "DRV_OA_MODE_ALREADY_EXISTS";
    break;
  case DRV_OA_STRINGS_NOT_EQUAL: error = "DRV_OA_STRINGS_NOT_EQUAL";
    break;
  case DRV_OA_NO_USER_DATA: error = "DRV_OA_NO_USER_DATA";
    break;
  case DRV_OA_VALUE_NOT_SUPPORTED: error = "DRV_OA_VALUE_NOT_SUPPORTED";
    break;
  case DRV_OA_MODE_DOES_NOT_EXIST: error = "DRV_OA_MODE_DOES_NOT_EXIST";
    break;
  case DRV_OA_CAMERA_NOT_SUPPORTED: error = "DRV_OA_CAMERA_NOT_SUPPORTED";
    break;
  case DRV_OA_FAILED_TO_GET_MODE: error = "DRV_OA_FAILED_TO_GET_MODE";
    break;
#if !defined(WIN32)    
  case KERN_MEM_ERROR: error = "KERN_MEM_ERROR";
    break;
  case DRV_P8INVALID: error = "DRV_P8INVALID";
    break;
  case DRV_P9INVALID: error = "DRV_P9INVALID";
    break;
  case DRV_P10INVALID: error = "DRV_P10INVALID";
    break;
  case DRV_INVALID_AMPLIFIER: error = "DRV_INVALID_AMPLIFIER";
    break;
  case DRV_OW_CMD_FAIL: error = "DRV_OW_CMD_FAIL";
    break;
  case DRV_OWMEMORY_BAD_ADDR: error = "DRV_OWMEMORY_BAD_ADDR";
    break;
  case DRV_OWCMD_NOT_AVAILABLE: error = "DRV_OWCMD_NOT_AVAILABLE";
    break;
  case DRV_OW_NO_SLAVES: error = "DRV_OW_NO_SLAVES";
    break;
  case DRV_OW_NOT_INITIALIZED: error = "DRV_OW_NOT_INITIALIZED";
    break;
  case DRV_OW_ERROR_SLAVE_NUM: error = "DRV_OW_ERROR_SLAVE_NUM";
    break;
#endif
  default:
    error = "Unknown";break;
  }
  return error; 
}
