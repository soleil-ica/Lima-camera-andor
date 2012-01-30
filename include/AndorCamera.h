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
//###########################################################################
#ifndef ANDORCAMERA_H
#define ANDORCAMERA_H

#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif

#include <atmcdLXd.h>

#include <stdlib.h>
#include <limits>
#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"

#include <ostream>

using namespace std;


namespace lima
{
    namespace Andor
    {
/*******************************************************************
 * \class Camera
 * \brief object controlling the andor camera via Pylon driver
 *******************************************************************/
	class Camera
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Andor");
	    friend class Interface;
	public:

	    enum Status {
		Ready, Exposure, Readout, Latency, Fault
	    };

	    enum ShutterMode {
		FRAME,
		MANUAL
	    };
    
	    Camera(const std::string& config_path,int camera_number=0);
	    ~Camera();

	    void startAcq();
	    void stopAcq();
    
	    // -- detector info object
	    void getImageType(ImageType& type);
	    void setImageType(ImageType type);

	    void getDetectorType(std::string& type);
	    void getDetectorModel(std::string& model);
	    void getDetectorImageSize(Size& size);
    
	    // -- Buffer control object
	    HwBufferCtrlObj* getBufferMgr();
    
	    //-- Synch control object
	    bool checkTrigMode(TrigMode trig_mode);
	    void setTrigMode(TrigMode  mode);
	    void getTrigMode(TrigMode& mode);
    
	    void setExpTime(double  exp_time);
	    void getExpTime(double& exp_time);

	    void setLatTime(double  lat_time);
	    void getLatTime(double& lat_time);

	    void getExposureTimeRange(double& min_expo, double& max_expo) const;
	    void getLatTimeRange(double& min_lat, double& max_lat) const;    

	    void setNbFrames(int  nb_frames);
	    void getNbFrames(int& nb_frames);
	    void getNbHwAcquiredFrames(int &nb_acq_frames);

	    void checkRoi(const Roi& set_roi, Roi& hw_roi);
	    void setRoi(const Roi& set_roi);
	    void getRoi(Roi& hw_roi);    

	    void checkBin(Bin&);
	    void setBin(const Bin&);
	    void getBin(Bin&);
	    bool isBinningAvailable();
    
	    void setShutterOpenTime(double tm);
	    void getShutterOpenTime(double& tm);
	    void setShutterCloseTime(double tm);
	    void getShutterCloseTime(double& tm);
	    void setShutter(bool flag);
	    void getShutter(bool& flag);
	    void setShutterMode(ShutterMode mode);
	    void getShutterMode(ShutterMode& mode);
    

	    void getPixelSize(double& x_size,double &y_size);
    
	    void getStatus(Camera::Status& status);
    
	    void reset();

	    // -- andor specific, LIMA don't worry about it !        
	    void _mapAndorError();
	    bool andorError(unsigned int code);
	    void initialiseController();
	    void initAdcSpeed();
	    void setAdcSpeed(int adc);
	    void initVSS();
	    void setVSS(int vss);
	    void initPGain();
	    void setPGain(int gain);
	    void setFastExtTrigger(bool flag);
	    void getFastExtTrigger(bool& flag);
	    void setShutterLevel(int level);
	    void getShutterLevel(int& level);
	    void setTemperatureSP(int temp);
	    void getTemperature(int& temp);
	    void setCooler(bool flag);
	    void getCooler(bool& flag);
	    void getCoolingStatus(std::string& status);    
    
    
    
	private:
	    class _AcqThread;
	    friend class _AcqThread;
	    void _stopAcq(bool);
	    void _setStatus(Camera::Status status,bool force);

	    //- acquisition thread stuff    
	    _AcqThread*                 m_acq_thread;
	    Cond                        m_cond;

	    //- lima stuff
	    SoftBufferCtrlMgr		    m_buffer_ctrl_mgr;
	    int                         m_nb_frames;    
	    Camera::Status              m_status;
	    volatile bool               m_wait_flag;
	    volatile bool               m_quit;
	    volatile bool               m_thread_running;
	    int                         m_image_number;
	    int                         m_timeout;
	    double                      m_latency_time;
	    Roi                         m_roi;
	    Bin                         m_bin;
	    Bin                         m_bin_max;
	    TrigMode                    m_trig_mode;

	    ShutterMode                 m_shutter_mode;
	    bool                        m_shutter_state;
        
	    //- camera stuff 
	    string                      m_detector_model;
	    string                      m_detector_type;
    
	    //- andor SDK stuff
	    string                      m_config_path;
	    int                         m_camera_number;
	    at_32                       m_camera_handle;
	    AndorCapabilities           m_camera_capabilities;
	    string                      m_camera_error_str;
	    int                         m_camera_error;
    
	    struct Adc 
	    {
		int		adc;
		int		hss;
		float	        speed;
	    };
    
	    Adc*                        m_adc_speeds;
	    int                         m_adc_speed_number;
	    int                         m_adc_speed_max;
	    int                         m_adc;
	    int                         m_vss_number;
	    float*                      m_vsspeeds;
	    int                         m_vss_best;
	    int                         m_vss;
	    int                         m_gain_number;
	    int                         m_gain_max;
	    float*                      m_preamp_gains;
	    int                         m_gain;
	    bool                        m_fasttrigger;
	    int                         m_shutter_level;
	    int                         m_shutter_close_time;
	    int                         m_shutter_open_time;
	    int                         m_temperature_sp;   
	    bool                        m_cooler;   
	    int                         m_read_mode;
	    int                         m_acq_mode;    
	    map<TrigMode, int>          m_trig_mode_maps;
	    float                       m_exp_time;
	    float                       m_exp_time_max;
	    float                       m_kin_time;
	    int                         m_ring_buffer_size;                
	    map<int, string>            m_andor_type_maps;            
	    map<int, string>            m_andor_error_maps;
	};
    } // namespace Andor
} // namespace lima


#endif // ANDORCAMERA_H
