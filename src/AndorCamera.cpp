//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2012
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
      m_exp_time(1.)
{
    DEB_CONSTRUCTOR();
    m_config_path = config_path;
    m_camera_number = camera_number;
  
    _mapAndorError();
  
    // --- Get available cameras and select the choosen one.
    int numCameras;
    DEB_TRACE() << "Get all attached cameras";
    if (GetAvailableCameras(&numCameras)!= DRV_SUCCESS)
    {
	DEB_ERROR() << "No camera present!";
	THROW_HW_ERROR(Error) << "No camera present!";
    }
    DEB_TRACE() << "Found "<< numCameras << " camera" << ((numCameras>1)? "s": "");
    DEB_TRACE() << "Try to set current camera to number " << m_camera_number;
    
    if (m_camera_number < numCameras && m_camera_number >=0)
    {        
	if(andorError(GetCameraHandle(m_camera_number, &m_camera_handle)))
	{
	    DEB_ERROR() << "Cannot get camera handle" << " : error code = " << m_camera_error_str;;
	    THROW_HW_ERROR(InvalidValue) << "Cannot get camera handle";
	}
	if (andorError(SetCurrentCamera(m_camera_handle)))
	{
	    DEB_ERROR() << "Cannot set camera handle" << " : error code = " << m_camera_error_str;;
	    THROW_HW_ERROR(InvalidValue) << "Cannot set camera handle";
	}
    }
    else
    {
	DEB_ERROR() << "Invalid camera number " << m_camera_number << ", there is "<< numCameras << " available";
	THROW_HW_ERROR(InvalidValue) << "Invalid Camera number ";
    }


    // --- Initialize  the library    
    if (andorError(Initialize((char *)m_config_path.c_str())))
    {
	DEB_ERROR() << "Library initialization failed, check the config. path" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Library initialization failed, check the config. path";       
    }
    
    // --- Get camera capabilities
    m_camera_capabilities.ulSize = sizeof(AndorCapabilities);
    if (andorError(GetCapabilities(&m_camera_capabilities)))
    {
	DEB_ERROR() << "Cannot get camera capabilities" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get camera capabilities";            
    }
    
    
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
    if (andorError(GetHeadModel(model)))
    {
	DEB_ERROR() << "Cannot get camera model: " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get camera model";            
    }
    m_detector_model = model;
    m_detector_type = m_andor_type_maps[m_camera_capabilities.ulCameraType];
    
    DEB_TRACE() << "Andor Camera device found:\n" 
		<< "    * Type  : " << m_detector_type << "("
		<< m_camera_capabilities.ulCameraType <<")\n"
		<< "    * Model : " << m_detector_model;


        
    // --- Initialise deeper parameters of the controller                
    initialiseController();            
    
    //--- Set detector for single image acquisition and get max binning
    m_read_mode = 4;
    if (andorError(SetReadMode(m_read_mode)))
    {
	DEB_ERROR() << "Cannot set camera read mode" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot camera read mode";                
    }
 
    int xbin_max, ybin_max;   
    if (andorError(GetMaximumBinning(m_read_mode, 0, &xbin_max)))
    {
	DEB_ERROR() << "Cannot get the horizontal maximum binning" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get the horizontal maximum binning";            
    }
    if (andorError(GetMaximumBinning(m_read_mode, 1, &ybin_max)))
    {
	DEB_ERROR() << "Cannot get the vertical maximum binning" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get the vertical maximum binning";            
    }
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
    if (this->andorError(GetMaximumExposure(&m_exp_time_max)))
    {
	DEB_ERROR() << "Cannot get the maximum exposure time: " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get the maximum exposure time";    
    }    
    DEB_TRACE() << "Maximum exposure time : "<< m_exp_time_max << "sec.";
    
    setExpTime(m_exp_time);
    
    // --- Set detector for software single image mode    
    m_trig_mode_maps[IntTrig] = 0;
    m_trig_mode_maps[ExtTrigSingle] = 1;
    m_trig_mode_maps[ExtGate] = 7;
    m_trig_mode_maps[IntTrigMult] = 10;  
    setTrigMode(IntTrig);
    
    // --- Set the Andor specific acquistion mode.
    // --- We set acquisition mode to kinetics which is the more useful for us
    // --- This mode allows to manage latency between images and multi-frame acquisition as well
    m_acq_mode = 3; // Andor Kinetics mode
    m_nb_frames = 1;
    if (andorError(SetAcquisitionMode(m_acq_mode)))
    {
	DEB_ERROR() << "Cannot get the vertical maximum binning" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get the vertical maximum binning";            
    }               

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
// @brief  start the acquistion
//---------------------------
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();
    m_image_number=0;
        
    // --- check first the acquisition is idle
    int status;
    if (andorError(GetStatus(&status)))
    {
        DEB_ERROR() << "Cannot get status" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get status";            
    }
    if (status != DRV_IDLE)
    {
        _setStatus(Camera::Fault,false);        
        DEB_ERROR() << "Cannot start acquisition, camera is not idle" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot start acquisition, camera is not idle";            
    }   
    
    // --- Don't forget to request the maximum number of images the circular buffer can store
    // --- based on the current acquisition settings.
    if (andorError(GetSizeOfCircularBuffer(&m_ring_buffer_size)))
    {
        DEB_ERROR() << "Cannot get size of circular buffer" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get size of circular buffer";            
    }        
    DEB_TRACE() << "Andor Circular buffer size = " << m_ring_buffer_size << " images";
            
    // Wait running stat of acquisition thread
    AutoMutex aLock(m_cond.mutex());
    m_wait_flag = false;
    m_cond.broadcast();
    while(!m_thread_running)
        m_cond.wait();
        
    StdBufferCbMgr& buffer_mgr = m_buffer_ctrl_mgr.getBuffer();
    buffer_mgr.setStartTimestamp(Timestamp::now());
    if (andorError(StartAcquisition()))
    {
        DEB_ERROR() << "Cannot start acquisition" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot start acquisition";            
    }
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
            m_wait_flag = true;
            m_cond.wait();
        }
	aLock.unlock();

        //Let the acq thread stop the acquisition
        if(!internalFlag) return;
            
        // Stop acquisition
        DEB_TRACE() << "Stop acquisition";
        if (andorError(AbortAcquisition()))
        {
            DEB_ERROR() << "Cannot abort acquisition" << " : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Cannot abort acquisition";
        }
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
    StdBufferCbMgr& buffer_mgr = m_cam.m_buffer_ctrl_mgr.getBuffer();

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
        while(continueAcq && (!m_cam.m_nb_frames || m_cam.m_image_number < m_cam.m_nb_frames))
        {
            // --- Get the available images in cicular buffer            
            int first, last;
            if (m_cam.andorError(GetNumberNewImages(&first, &last)))
            {
                if (m_cam.m_camera_error == DRV_NO_NEW_DATA) continue;
                else
                {
                    DEB_ERROR() << "Cannot get number of new images" << " : error code = " << m_cam.m_camera_error_str;
                    THROW_HW_ERROR(Error) << "Cannot get number of new images";
                }
            }        
            DEB_TRACE() << "Available images: first = " << first << " last = " << last;
             
            // --- Images are available, process images
            m_cam._setStatus(Camera::Readout,false);
 
            FrameDim frame_dim = buffer_mgr.getFrameDim();
            Size  frame_size = frame_dim.getSize();
            int size = frame_size.getWidth() * frame_size.getHeight();
            int validfirst, validlast;
            
            for (long im=first; im <= last; im++)
            {
                DEB_TRACE()  << "image #" << m_cam.m_image_number <<" acquired !";
                // ---  must get image one by one to copy to the buffer manager
                void *ptr = buffer_mgr.getFrameBufferPtr(m_cam.m_image_number);
                
                if (m_cam.andorError(GetImages16(im, im,(unsigned short*) ptr, (unsigned long)size,&validfirst, &validlast)))
                {
                    m_cam._setStatus(Camera::Fault,false);
                    continueAcq = false;
                    DEB_TRACE() << "size = " << size;
                    DEB_ERROR() << "Cannot get image #" << im << " : error code = " << m_cam.m_camera_error_str;
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
    if (andorError(GetDetector(&xmax, &ymax)))
    {
	DEB_ERROR() << "Cannot get detector size" << " : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Cannot get detector size";                    
    }     
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
    DEB_TRACE() << "m_adc = "<< m_adc;
    
    if (andorError(GetBitDepth(m_adc, &bits)))
    {
        DEB_ERROR() << "Cannot get detector bit depth" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get detector bit depth";                        
    }
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
    type = m_detector_model;
}

//-----------------------------------------------------
// @brief return the internal buffer manager
//-----------------------------------------------------
HwBufferCtrlObj* Camera::getBufferMgr()
{
    DEB_MEMBER_FUNCT();
    return &m_buffer_ctrl_mgr;
}


//-----------------------------------------------------
// @brief return true if passed trigger mode is supported
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode trig_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(trig_mode);
    bool valid_mode;    



    switch (trig_mode)
    {       
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
    case ExtGate:
	m_camera_error = IsTriggerModeAvailable(m_trig_mode_maps[trig_mode]);
	switch (m_camera_error)
	{
	case DRV_SUCCESS:
	    valid_mode = true;
	    break;
	case DRV_INVALID_MODE:
	    valid_mode = false;
	    break;
	case DRV_NOT_INITIALIZED:                
	    valid_mode = false;
	    DEB_ERROR() << "System not initializsed, cannot get trigger mode status" << " : error code = " << m_camera_error_str;
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

    if (andorError(SetTriggerMode(m_trig_mode_maps[mode])))
    {
        DEB_ERROR() << "Cannot get detector size" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get detector size";                    
    }
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
    
    if (andorError(SetExposureTime((float)exp_time)))
    {
        DEB_ERROR() << "Cannot set exposure time" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot set exposure time";                        
    }
                 
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
    if (andorError(GetAcquisitionTimings(&exp, &acc, &kin)))
    {
        DEB_ERROR() << "Cannot get acquisition timings" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get acquisition timings";                        
    }
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
    
    if (andorError(GetAcquisitionTimings(&exp, &acc, &kin)))
    {
        DEB_ERROR() << "Cannot get acquisition timings" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get acquisition timings";                        
    }
    m_exp_time = exp;
    m_kin_time = exp + lat_time;
    if (andorError(SetKineticCycleTime(m_kin_time)))
    {
        DEB_ERROR() << "Cannot set kinetic cycle time" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot set kinetic cycle time";                        
    }
    
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
    if (andorError(GetAcquisitionTimings(&exp, &acc, &kin)))
    {
        DEB_ERROR() << "Cannot get acquisition timings" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get acquisition timings";                        
    }
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
    if (nb_frames == 0)
    {
        DEB_ERROR() << "Sorry continuous acquisition (setNbFrames(0)) not yet implemented";
        THROW_HW_ERROR(Error) << "Sorry continuous acquisition (setNbFrames(0)) not yet implemented";                
    }
    // --- We only work on kinetics mode which allow multi-frames to be taken
    // ---
    if (andorError(SetNumberKinetics(nb_frames)))
    {
        DEB_ERROR() << "Cannot set number of frames" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot set number of frames";            
    }        
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
    int hstart, hend, vstart, vend;
    Roi hw_roi;

    if(m_roi == set_roi) return;    
               
    if(set_roi.isActive()) hw_roi = set_roi;
    else
    {
        // ROI has been reset (0x0)
        // --- read back full frame size
        Size sizeMax;
        getDetectorImageSize(sizeMax);
        hw_roi = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    
    }    
    // --- Andor sets the ROI by starting coordinates at 1 and not 0 !!!!
    topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
    hstart = topleft.x +1;          vstart = topleft.y +1;
    hend   = hstart + size.x -1;      vend   = vstart + size.y -1;
            
    //- then fix the new ROI
    if (andorError(SetImage(m_bin.getX(), m_bin.getY(), hstart, hend, vstart, vend)))
    {
        DEB_ERROR() << "Cannot set detector ROI" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot set detector ROI";                                        
    }
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
    
    int hstart, hend, vstart, vend;

    if(m_bin == set_bin) return;
               
    if(m_roi.isActive())
    {
        topleft = m_roi.getTopLeft(); size = m_roi.getSize();
        hstart = topleft.x+1;          vstart = topleft.y+1;
        hend   = hstart + size.x -1;      vend   = vstart + size.y -1;
        //- then fix the new BIN
    }
    else
    {   Size size;
        getDetectorImageSize(size);
        hstart = 1; vstart = 1;
        hend = size.getHeight(); vend = size.getWidth();
    }
    if (andorError(SetImage(set_bin.getX(), set_bin.getY(), hstart, hend, vstart, vend)))
    {
        DEB_ERROR() << "Cannot set detector BIN" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot set detector BIN";                                        
    }
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
void Camera::getPixelSize(double& size)
{
    DEB_MEMBER_FUNCT();
    float xsize, ysize;
    
    if (andorError(GetPixelSize(&xsize, &ysize)))
    {
        DEB_ERROR() << "Cannot pixel sizes" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get pixel size";                                        
    }
    //-- what to do with x and y size when lima on support only one size
    //Lima will be modified, with a new SizeUtils class for pixel size 
    // today just return x size supposing it the the same for y.
    // remember standard is metric units so size is in meter
    size = xsize * 1e-6;
    DEB_RETURN() << DEB_VAR1(size); 
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
        DEB_TRACE() << "    (" << is << ") adc #" << m_adc_speeds[is].adc << ", speed = " 
                    << m_adc_speeds[is].speed  << ((is == m_adc_speed_max)? " [max]": "");                        
    }
        
    // --- Set adc / speed
    setAdcSpeed(m_adc_speed_max);
    DEB_TRACE() << "    => Set to " << m_adc_speeds[m_adc_speed_max].speed << "MHz";
       
        
    // --- Init VS Speeds 
    initVSS();
      
    DEB_TRACE() << "* Vertical Shift Speed:";
    for (is=0; is<m_vss_number; is++)
    {
        DEB_TRACE() << "    (" << is << ") speed = " << m_vsspeeds[is] << " us"
                    << ((is == m_vss_best)? " [recommended]": "");
    }
        
    // --- Set VS Speed
    setVSS(m_vss);
    DEB_TRACE() << "    => Set " << m_vsspeeds[m_vss] << "us";
        
        
    // --- Init Preamp Gain
    initPGain();
       
    DEB_TRACE() << "* Preamp Gain:";
        
    for (is=0; is< m_gain_number; is++)
    {
        DEB_TRACE() << "    (" << is << ") gain = x" << m_preamp_gains[is]
                    << ((is == m_gain_max)? " [max]": "");
    }
    
    // --- Set Preamp Gain
    setPGain(m_gain);
    DEB_TRACE() << "    => Set to x" << m_preamp_gains[m_gain];                 
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
    if (andorError(GetNumberADChannels(&nadc))) 
    {
        DEB_ERROR() << "Cannot get number of ADC" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get number of ADC";            
    }
    // --- Get Horizontal Shift Speed per ADC
    nSpeed = new int[nadc];
    
    m_adc_speed_number= 0;
    for (ia=0; ia<nadc; ia++) 
    {
	if (andorError(GetNumberHSSpeeds(ia, 0, &nSpeed[ia]))) {
	    DEB_ERROR() << "Cannot get nb of Horizontal Speed for ADC " <<  ia <<" : error code = " << m_camera_error_str;
	    THROW_HW_ERROR(Error) << "Cannot get nb of Horizontal Speed for an ADC";            
	}
	m_adc_speed_number += nSpeed[ia];
			
    }

    m_adc_speeds = new Adc[m_adc_speed_number];
    speedMax= 0.;
    is= 0;
    for (ia=0; ia<nadc; ia++) {
	for (ih=0; ih<nSpeed[ia]; ih++) {
	    if (andorError(GetHSSpeed(ia, 0, ih, &m_adc_speeds[is].speed))) {
                DEB_ERROR() << "Cannot get Horizontal Speed " << ih << " for ADC " << ia <<" : error code = " << m_camera_error_str;
                THROW_HW_ERROR(Error) << "Cannot get Horizontal Speed ";            
	    }
	    m_adc_speeds[is].adc= ia;
	    m_adc_speeds[is].hss= ih;

	    // --- iKon/iXon= speed in MHz ; others in us/pixel shift --> convert in MHz
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
// @brief	get ADC/Speed settings
// @param	adc pais adc/speed index (if =-1, set to max speed)
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
    if (andorError(SetADChannel(m_adc_speeds[is].adc)))
    {
        DEB_ERROR() << "Failed to set ADC channel #" << m_adc_speeds[is].adc <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set ADC channel";            
    }
    if (andorError(SetHSSpeed(0, m_adc_speeds[is].hss)))
    {
        DEB_ERROR() << "Failed to set HSS #" << m_adc_speeds[is].hss <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set HSS";            
    }    
    m_adc = is;
    
    DEB_TRACE() << "ADC speed set to " << m_adc_speeds[is].speed << " MHz";
}


//-----------------------------------------------------
// @brief get possible VSS (vertical shift speed) for controller
//
// Initialise the list of possible vss index and their value
// Get also the recommended VSS.
//-----------------------------------------------------
void Camera::initVSS()
{
    DEB_MEMBER_FUNCT();
    float speed;
    int ivss;


    // --- number of ADC
    if (andorError(GetNumberVSSpeeds(&m_vss_number)))
    {
        DEB_ERROR() << "Cannot get number of possible VSS" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get number of possible VSS";            
    }

    // --- get VSS value for each
    m_vsspeeds = new float[m_vss_number];
    for (ivss=0; ivss<m_vss_number; ivss++)
    {
	if (andorError(GetVSSpeed(ivss, &m_vsspeeds[ivss])))
        {
	    DEB_ERROR() << "Cannot get VSS value for #" << ivss <<" : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Cannot get VSS value";
	}
    }

    // --- get recommended VSS value
    if (andorError(GetFastestRecommendedVSSpeed(&m_vss_best, &speed)))
    {
	m_vss_best = 0;
        DEB_ERROR() << "Cannot get recommended VSS speed. Set it to 0" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get recommended VSS speed. Set it to 0";

    }
}


//-----------------------------------------------------
// @brief	get Vertical Shift Speed
// @param	vss index (if =-1, set to recommended)
//
//-----------------------------------------------------
void Camera::setVSS(int vss) 
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
    if (andorError(SetVSSpeed(is)))
    {
	DEB_ERROR() << "Failed to set VSS #" << is <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set VSS";
    }
    m_vss = is;

    DEB_TRACE() << "VSSpeed Set to " <<m_vsspeeds[is] << "us";
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
    if (andorError(GetNumberPreAmpGains(&m_gain_number)))
    {
	DEB_ERROR() << "Failed to get number of preamp gain" <<" : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Failed to get number of preamp gain";
    }

    // --- get gain value for each
    gmax = 0.;
    m_preamp_gains = new float[m_gain_number];
    for (ig=0; ig<m_gain_number; ig++)
    {
	if (andorError(GetPreAmpGain(ig, &m_preamp_gains[ig])))
        {
	    DEB_ERROR() << "Failed to get gain #" << ig <<" : error code = " << m_camera_error_str;
	    THROW_HW_ERROR(Error) << "Failed to get gain";
	}
	if (m_preamp_gains[ig] >= gmax)
        {
	    gmax = m_preamp_gains[ig];
	    m_gain_max = ig;
	}
    }
}

//-----------------------------------------------------
// @brief	set Preamp Gain
// @param	gain premap gain index
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
	ig= gain;
    }

    if (andorError(SetPreAmpGain(ig)))
    {
	DEB_ERROR() << "Failed to set Preamp Gain #" << ig <<" : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Failed to set Preamp Gain";
    }
    m_gain= ig;

    DEB_TRACE() << "Preamp Gain set to x" << m_preamp_gains[ig];
}

//-----------------------------------------------------
// @brief	set external trigger for fast mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Camera::setFastExtTrigger(bool flag)
{
    DEB_MEMBER_FUNCT();
    if (andorError(SetFastExtTrigger((flag)?1:0)))
    {
        DEB_ERROR() << "Failed to set ext-trigger fast mode" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set ext-trigger fast mode";      
    }
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

    if (andorError(SetShutter(level, m_shutter_mode, m_shutter_close_time, m_shutter_open_time)))
    {
        DEB_ERROR() << "Failed to set shutter level" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set shutter level";          
    }
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
    if (mode == FRAME)
    {    
        if (andorError(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time)))
        {
            DEB_ERROR() << "Failed to set the shutter mode" <<" : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Failed to set the shutter mode";          
        }
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
    if (andorError(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time)))
    {
        DEB_ERROR() << "Failed close/open the shutter" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set close/open the shutter";          
    }
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
    
    if (andorError(SetShutter(m_shutter_level, m_shutter_mode, m_shutter_close_time, aTime)))
    {
        DEB_ERROR() << "Failed to set shutter openning time" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set shutter opening time";          
    }
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
    
    if (andorError(SetShutter(m_shutter_level, m_shutter_mode, aTime, m_shutter_open_time)))
    {
        DEB_ERROR() << "Failed to set shutter closing time" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set shutter closing time";          
    }
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
    if (andorError(SetTemperature(temp)))
    {
        DEB_ERROR() << "Failed to set temperature set-point" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set temperature set-point";          
    }
    m_temperature_sp = temp;
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
    unsigned int status;
    if ((status = GetTemperature(&tm)) == DRV_ERROR_ACK)
    {
        DEB_ERROR() << "Failed to read temperature" <<" : error code = " << DRV_ERROR_ACK;
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
        if (andorError(CoolerON()))
        {
            DEB_ERROR() << "Failed start the cooler" <<" : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Failed to start the cooler";          
        }
        
    }
    else
    {
        if (andorError(CoolerOFF()))
        {
            DEB_ERROR() << "Failed to stop the cooler" <<" : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Failed to stop the cooler";          
        }
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
// @brief handle the andor error codes
//-----------------------------------------------------
bool Camera::andorError(unsigned int code)
{
    if (code != DRV_SUCCESS)
    {
        m_camera_error = code;
        m_camera_error_str = m_andor_error_maps[code];
        return true;
    }
    return false;
}

//-----------------------------------------------------
// @brief just build a map of error codes
//-----------------------------------------------------
void Camera::_mapAndorError()
{
    m_andor_error_maps[DRV_ERROR_CODES] = "DRV_ERROR_CODES";
    m_andor_error_maps[DRV_SUCCESS] = "DRV_SUCCESS";
    m_andor_error_maps[DRV_VXDNOTINSTALLED] = "DRV_VXDNOTINSTALLED";
    m_andor_error_maps[DRV_ERROR_SCAN] = "DRV_ERROR_SCAN";
    m_andor_error_maps[DRV_ERROR_CHECK_SUM] = "DRV_ERROR_CHECK_SUM";
    m_andor_error_maps[DRV_ERROR_FILELOAD] = "DRV_ERROR_FILELOAD";
    m_andor_error_maps[DRV_UNKNOWN_FUNCTION] = "DRV_UNKNOWN_FUNCTION";
    m_andor_error_maps[DRV_ERROR_VXD_INIT] = "DRV_UNKNOWN_FUNCTION";
    m_andor_error_maps[DRV_ERROR_ADDRESS] = "DRV_UNKNOWN_FUNCTION";
    m_andor_error_maps[DRV_ERROR_PAGELOCK] = "DRV_ERROR_PAGELOCK";
    m_andor_error_maps[DRV_ERROR_PAGEUNLOCK] = "DRV_ERROR_PAGEUNLOCK";
    m_andor_error_maps[DRV_ERROR_BOARDTEST] = "DRV_ERROR_BOARDTEST";
    m_andor_error_maps[DRV_ERROR_ACK] = "DRV_ERROR_ACK";
    m_andor_error_maps[DRV_ERROR_UP_FIFO] = "DRV_ERROR_UP_FIFO";
    m_andor_error_maps[DRV_ERROR_PATTERN] = "DRV_ERROR_PATTERN";
    m_andor_error_maps[DRV_KINETIC_TIME_NOT_MET] = "DRV_KINETIC_TIME_NOT_MET";
    m_andor_error_maps[DRV_ACCUM_TIME_NOT_MET] = "DRV_ACCUM_TIME_NOT_MET";
    m_andor_error_maps[DRV_NO_NEW_DATA] = "DRV_NO_NEW_DATA";
    m_andor_error_maps[KERN_MEM_ERROR] = "KERN_MEM_ERROR";
    m_andor_error_maps[DRV_SPOOLERROR] = "DRV_SPOOLERROR";
    m_andor_error_maps[DRV_SPOOLSETUPERROR] = "DRV_SPOOLSETUPERROR";
    m_andor_error_maps[DRV_FILESIZELIMITERROR] = "DRV_FILESIZELIMITERROR";
    m_andor_error_maps[DRV_ERROR_FILESAVE] = "DRV_ERROR_FILESAVE";
    m_andor_error_maps[DRV_TEMPERATURE_CODES] = "DRV_TEMPERATURE_CODES";
    m_andor_error_maps[DRV_TEMPERATURE_OFF] = "DRV_TEMPERATURE_OFF";
    m_andor_error_maps[DRV_TEMPERATURE_NOT_STABILIZED] = "DRV_TEMPERATURE_NOT_STABILIZED";
    m_andor_error_maps[DRV_TEMPERATURE_STABILIZED] = "DRV_TEMPERATURE_STABILIZED";
    m_andor_error_maps[DRV_TEMPERATURE_NOT_REACHED] = "DRV_TEMPERATURE_NOT_REACHED";
    m_andor_error_maps[DRV_TEMPERATURE_OUT_RANGE] = "DRV_TEMPERATURE_OUT_RANGE";
    m_andor_error_maps[DRV_TEMPERATURE_NOT_SUPPORTED] = "DRV_TEMPERATURE_NOT_SUPPORTED";
    m_andor_error_maps[DRV_TEMPERATURE_DRIFT] = "DRV_TEMPERATURE_DRIFT";
    m_andor_error_maps[DRV_TEMP_CODES] = "DRV_TEMP_CODES";
    m_andor_error_maps[DRV_TEMP_OFF] = "DRV_TEMP_OFF";
    m_andor_error_maps[DRV_TEMP_NOT_STABILIZED] = "DRV_TEMP_NOT_STABILIZED";
    m_andor_error_maps[DRV_TEMP_STABILIZED] = "DRV_TEMP_STABILIZED";
    m_andor_error_maps[DRV_TEMP_NOT_REACHED] = "DRV_TEMP_NOT_REACHED";
    m_andor_error_maps[DRV_TEMP_OUT_RANGE] = "DRV_TEMP_OUT_RANGE";
    m_andor_error_maps[DRV_TEMP_NOT_SUPPORTED] = "DRV_TEMP_NOT_SUPPORTED";
    m_andor_error_maps[DRV_TEMP_DRIFT] = "DRV_TEMP_DRIFT";
    m_andor_error_maps[DRV_GENERAL_ERRORS] = "DRV_GENERAL_ERRORS";
    m_andor_error_maps[DRV_INVALID_AUX] = "DRV_INVALID_AUX";
    m_andor_error_maps[DRV_COF_NOTLOADED] = "DRV_COF_NOTLOADED";
    m_andor_error_maps[DRV_FPGAPROG] = "DRV_FPGAPROG";
    m_andor_error_maps[DRV_FLEXERROR] = "DRV_FLEXERROR";
    m_andor_error_maps[DRV_GPIBERROR] = "DRV_GPIBERROR";
    m_andor_error_maps[DRV_EEPROMVERSIONERROR] = "DRV_EEPROMVERSIONERROR";
    m_andor_error_maps[DRV_DATATYPE] = "DRV_DATATYPE";
    m_andor_error_maps[DRV_DRIVER_ERRORS] = "DRV_DRIVER_ERRORS";
    m_andor_error_maps[DRV_P1INVALID] = "DRV_P1INVALID";
    m_andor_error_maps[DRV_P2INVALID] = "DRV_P2INVALID";
    m_andor_error_maps[DRV_P3INVALID] = "DRV_P3INVALID";
    m_andor_error_maps[DRV_P4INVALID] = "DRV_P4INVALID";
    m_andor_error_maps[DRV_INIERROR] = "DRV_INIERROR";
    m_andor_error_maps[DRV_COFERROR] = "DRV_COFERROR";
    m_andor_error_maps[DRV_ACQUIRING] = "DRV_ACQUIRING";
    m_andor_error_maps[DRV_IDLE] = "DRV_IDLE";
    m_andor_error_maps[DRV_TEMPCYCLE] = "DRV_TEMPCYCLE";
    m_andor_error_maps[DRV_NOT_INITIALIZED] = "DRV_NOT_INITIALIZED";
    m_andor_error_maps[DRV_P5INVALID] = "DRV_P5INVALID";
    m_andor_error_maps[DRV_P6INVALID] = "DRV_P6INVALID";
    m_andor_error_maps[DRV_INVALID_MODE] = "DRV_INVALID_MODE";
    m_andor_error_maps[DRV_INVALID_FILTER] = "DRV_INVALID_FILTER";
    m_andor_error_maps[DRV_I2CERRORS] = "DRV_I2CERRORS";
    m_andor_error_maps[DRV_I2CDEVNOTFOUND] = "DRV_I2CDEVNOTFOUND";
    m_andor_error_maps[DRV_I2CTIMEOUT] = "DRV_I2CTIMEOUT";
    m_andor_error_maps[DRV_P7INVALID] = "DRV_P7INVALID";
    m_andor_error_maps[DRV_P8INVALID] = "DRV_P8INVALID";
    m_andor_error_maps[DRV_P9INVALID] = "DRV_P9INVALID";
    m_andor_error_maps[DRV_P10INVALID] = "DRV_P10INVALID";
    m_andor_error_maps[DRV_P11INVALID] = "DRV_P11INVALID";
    m_andor_error_maps[DRV_USBERROR] = "DRV_USBERROR";
    m_andor_error_maps[DRV_IOCERROR] = "DRV_IOCERROR";
    m_andor_error_maps[DRV_VRMVERSIONERROR] = "DRV_VRMVERSIONERROR";
    m_andor_error_maps[DRV_GATESTEPERROR] = "DRV_GATESTEPERROR";
    m_andor_error_maps[DRV_USB_INTERRUPT_ENDPOINT_ERROR] = "DRV_USB_INTERRUPT_ENDPOINT_ERROR";
    m_andor_error_maps[DRV_RANDOM_TRACK_ERROR] = "DRV_RANDOM_TRACK_ERROR";
    m_andor_error_maps[DRV_INVALID_TRIGGER_MODE] = "DRV_INVALID_TRIGGER_MODE";
    m_andor_error_maps[DRV_LOAD_FIRMWARE_ERROR] = "DRV_LOAD_FIRMWARE_ERROR";
    m_andor_error_maps[DRV_DIVIDE_BY_ZERO_ERROR] = "DRV_DIVIDE_BY_ZERO_ERROR";
    m_andor_error_maps[DRV_INVALID_RINGEXPOSURES] = "DRV_INVALID_RINGEXPOSURES";
    m_andor_error_maps[DRV_BINNING_ERROR] = "DRV_BINNING_ERROR";
    m_andor_error_maps[DRV_INVALID_AMPLIFIER] = "DRV_INVALID_AMPLIFIER";
    m_andor_error_maps[DRV_INVALID_COUNTCONVERT_MODE] = "DRV_INVALID_COUNTCONVERT_MODE";
    m_andor_error_maps[DRV_NOT_SUPPORTED] = "DRV_NOT_SUPPORTED";
    m_andor_error_maps[DRV_NOT_AVAILABLE] = "DRV_NOT_AVAILABLE";
    m_andor_error_maps[DRV_ERROR_MAP] = "DRV_ERROR_MAP";
    m_andor_error_maps[DRV_ERROR_UNMAP] = "DRV_ERROR_UNMAP";
    m_andor_error_maps[DRV_ERROR_MDL] = "DRV_ERROR_MDL";
    m_andor_error_maps[DRV_ERROR_UNMDL] = "DRV_ERROR_UNMDL";
    m_andor_error_maps[DRV_ERROR_BUFFSIZE] = "DRV_ERROR_BUFFSIZE";
    m_andor_error_maps[DRV_ERROR_NOHANDLE] = "DRV_ERROR_NOHANDLE";
    m_andor_error_maps[DRV_GATING_NOT_AVAILABLE] = "DRV_GATING_NOT_AVAILABLE";
    m_andor_error_maps[DRV_FPGA_VOLTAGE_ERROR] = "DRV_FPGA_VOLTAGE_ERROR";
    m_andor_error_maps[DRV_OW_CMD_FAIL] = "DRV_OW_CMD_FAIL";
    m_andor_error_maps[DRV_OWMEMORY_BAD_ADDR] = "DRV_OWMEMORY_BAD_ADDR";
    m_andor_error_maps[DRV_OWCMD_NOT_AVAILABLE] = "DRV_OWCMD_NOT_AVAILABLE";
    m_andor_error_maps[DRV_OW_NO_SLAVES] = "DRV_OW_NO_SLAVES";
    m_andor_error_maps[DRV_OW_NOT_INITIALIZED] = "DRV_OW_NOT_INITIALIZED";
    m_andor_error_maps[DRV_OW_ERROR_SLAVE_NUM] = "DRV_OW_ERROR_SLAVE_NUM";
    m_andor_error_maps[DRV_MSTIMINGS_ERROR] = "DRV_MSTIMINGS_ERROR";
    m_andor_error_maps[DRV_OA_NULL_ERROR] = "DRV_OA_NULL_ERROR";
    m_andor_error_maps[DRV_OA_PARSE_DTD_ERROR] = "DRV_OA_PARSE_DTD_ERROR";
    m_andor_error_maps[DRV_OA_DTD_VALIDATE_ERROR] = "DRV_OA_DTD_VALIDATE_ERROR";
    m_andor_error_maps[DRV_OA_FILE_ACCESS_ERROR] = "DRV_OA_FILE_ACCESS_ERROR";
    m_andor_error_maps[DRV_OA_FILE_DOES_NOT_EXIST] = "DRV_OA_FILE_DOES_NOT_EXIST";
    m_andor_error_maps[DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR] = "DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR";
    m_andor_error_maps[DRV_OA_PRESET_FILE_NOT_LOADED] = "DRV_OA_PRESET_FILE_NOT_LOADED";
    m_andor_error_maps[DRV_OA_USER_FILE_NOT_LOADED] = "DRV_OA_USER_FILE_NOT_LOADED";
    m_andor_error_maps[DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED] = "DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED";
    m_andor_error_maps[DRV_OA_INVALID_FILE] = "DRV_OA_INVALID_FILE";
    m_andor_error_maps[DRV_OA_FILE_HAS_BEEN_MODIFIED] = "DRV_OA_FILE_HAS_BEEN_MODIFIED";
    m_andor_error_maps[DRV_OA_BUFFER_FULL] = "DRV_OA_BUFFER_FULL";
    m_andor_error_maps[DRV_OA_INVALID_STRING_LENGTH] = "DRV_OA_INVALID_STRING_LENGTH";
    m_andor_error_maps[DRV_OA_INVALID_CHARS_IN_NAME] = "DRV_OA_INVALID_CHARS_IN_NAME";
    m_andor_error_maps[DRV_OA_INVALID_NAMING] = "DRV_OA_INVALID_NAMING";
    m_andor_error_maps[DRV_OA_GET_CAMERA_ERROR] = "DRV_OA_GET_CAMERA_ERROR";
    m_andor_error_maps[DRV_OA_MODE_ALREADY_EXISTS] = "DRV_OA_MODE_ALREADY_EXISTS";
    m_andor_error_maps[DRV_OA_STRINGS_NOT_EQUAL] = "DRV_OA_STRINGS_NOT_EQUAL";
    m_andor_error_maps[DRV_OA_NO_USER_DATA] = "DRV_OA_NO_USER_DATA";
    m_andor_error_maps[DRV_OA_VALUE_NOT_SUPPORTED] = "DRV_OA_VALUE_NOT_SUPPORTED";
    m_andor_error_maps[DRV_OA_MODE_DOES_NOT_EXIST] = "DRV_OA_MODE_DOES_NOT_EXIST";
    m_andor_error_maps[DRV_OA_CAMERA_NOT_SUPPORTED] = "DRV_OA_CAMERA_NOT_SUPPORTED";
    m_andor_error_maps[DRV_OA_FAILED_TO_GET_MODE] = "DRV_OA_FAILED_TO_GET_MODE";
}
