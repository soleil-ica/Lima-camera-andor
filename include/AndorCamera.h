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
    
    void getPixelSize(double&);
    
    void getStatus(Camera::Status& status);

    // -- andor specific, LIMA don't worry about it !        
    bool andorError(AndorErrorCode code);
    void initialiseController(void);
    void initAdcSpeed(void);
    void setAdcSpeed(int adc);
    void InitVSS(void);
    void setVSS(int vss);
    void initPGain(void);
    void setPGain(int gain);
    void setFastExtTrigger(bool flag);
    void getFastExtTrigger(bool& flag);
    void setShutterLevel(int level);
    void getShutterLevel(int& level);
    void setShutterOpenTime(int tm);
    void geShutterOpenTime(int& tm);
    void setShutterCloseTime(int tm);
    void geShutterCloseTime(int& tm);
    void setTemperatureSP(int temp);
    void getTemperature(int& temp);
    void setCooler(bool flag);
    void getCooler(bool& flag);
    void getCoolingStatus(string& status);    
    
    
    
 private:
    class _AcqThread;
    friend class _AcqThread;
    void _stopAcq(bool);
    void _setStatus(Camera::Status status,bool force);

    //- acquisition thread stuff    
    _AcqThread*                   m_acq_thread;
    Cond                          m_cond;

    //- lima stuff
    SoftBufferCtrlMgr		m_buffer_ctrl_mgr;
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
    
    //- camera stuff 
    string                      m_detector_model;
    string                      m_detector_type;
    
    //- andor SDK stuff
    string                      m_config_path;
    int                         m_camera_number;
    at_32                       m_camera_handle;
    AndorErrorCode              m_camera_error;
    AndorCapabilities           m_camera_capabilities;

    struct Adc 
    {
	    int		adc;
	    int		hss;
	    float	speed;

    };
    
    Adc*                        m_adc_speeds;
    int                         m_adc_speed_number;
    float                       m_adc_speed_max;
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
    int                         m_shutter_mode;
    int                         m_temperature_sp;   
    bool                        m_cooler;   
    int                         m_read_mode;
    int                         m_acq_mode;    
    map<TrigMode, int>          m_trig_mode_maps;
    float                       m_exp_time;
    float                       m_kin_time;
    long                        m_ring_buffer_size;
    

    
        
    char *AndorCcdTypes[]= {
        "PDA", 
	    "IXON",
    	"ICCD",
    	"EMCCD",
    	"CCD",
    	"ISTAR",
    	"VIDEO",
    	"IDUS",
    	"NEWTON",
    	"SURCAM",
    	"USBICCD",
    	"LUCA",
    	"RESERVED",
    	"IKON",
    	"INGAAS",
    	"IVAC",
    	"UNPROGRAMMED",
    	"CLARA",
        "USBISTAR"
    };
    
    enum AndorErrorCode {        
        _DRV_ERROR_CODES = 20001,
        _DRV_SUCCESS = 20002
        _DRV_VXDNOTINSTALLED = 20003
        _DRV_ERROR_SCAN = 20004
        _DRV_ERROR_CHECK_SUM = 20005
        _DRV_ERROR_FILELOAD = 20006
        _DRV_UNKNOWN_FUNCTION = 20007
        _DRV_ERROR_VXD_INIT = 20008
        _DRV_ERROR_ADDRESS = 20009
        _DRV_ERROR_PAGELOCK = 20010
        _DRV_ERROR_PAGEUNLOCK = 20011
        _DRV_ERROR_BOARDTEST = 20012
        _DRV_ERROR_ACK = 20013
        _DRV_ERROR_UP_FIFO = 20014
        _DRV_ERROR_PATTERN = 20015

        _DRV_ACQUISITION_ERRORS = 20017
        _DRV_ACQ_BUFFER = 20018
        _DRV_ACQ_DOWNFIFO_FULL = 20019
        _DRV_PROC_UNKONWN_INSTRUCTION = 20020
        _DRV_ILLEGAL_OP_CODE = 20021
        _DRV_KINETIC_TIME_NOT_MET = 20022
        _DRV_ACCUM_TIME_NOT_MET = 20023
        _DRV_NO_NEW_DATA = 20024
        _KERN_MEM_ERROR = 20025
        _DRV_SPOOLERROR = 20026
        _DRV_SPOOLSETUPERROR = 20027
        _DRV_FILESIZELIMITERROR = 20028
        _DRV_ERROR_FILESAVE = 20029

        _DRV_TEMPERATURE_CODES = 20033
        _DRV_TEMPERATURE_OFF = 20034
        _DRV_TEMPERATURE_NOT_STABILIZED = 20035
        _DRV_TEMPERATURE_STABILIZED = 20036
        _DRV_TEMPERATURE_NOT_REACHED = 20037
        _DRV_TEMPERATURE_OUT_RANGE = 20038
        _DRV_TEMPERATURE_NOT_SUPPORTED = 20039
        _DRV_TEMPERATURE_DRIFT = 20040

        _DRV_TEMP_CODES = 20033
        _DRV_TEMP_OFF = 20034
        _DRV_TEMP_NOT_STABILIZED = 20035
        _DRV_TEMP_STABILIZED = 20036
        _DRV_TEMP_NOT_REACHED = 20037
        _DRV_TEMP_OUT_RANGE = 20038
        _DRV_TEMP_NOT_SUPPORTED = 20039
        _DRV_TEMP_DRIFT = 20040

        _DRV_GENERAL_ERRORS = 20049
        _DRV_INVALID_AUX = 20050
        _DRV_COF_NOTLOADED = 20051
        _DRV_FPGAPROG = 20052
        _DRV_FLEXERROR = 20053
        _DRV_GPIBERROR = 20054
        _DRV_EEPROMVERSIONERROR = 20055

        _DRV_DATATYPE = 20064
        _DRV_DRIVER_ERRORS = 20065
        _DRV_P1INVALID = 20066
        _DRV_P2INVALID = 20067
        _DRV_P3INVALID = 20068
        _DRV_P4INVALID = 20069
        _DRV_INIERROR = 20070
        _DRV_COFERROR = 20071
        _DRV_ACQUIRING = 20072
        _DRV_IDLE = 20073
        _DRV_TEMPCYCLE = 20074
        _DRV_NOT_INITIALIZED = 20075
        _DRV_P5INVALID = 20076
        _DRV_P6INVALID = 20077
        _DRV_INVALID_MODE = 20078
        _DRV_INVALID_FILTER = 20079

        _DRV_I2CERRORS = 20080
        _DRV_I2CDEVNOTFOUND = 20081
        _DRV_I2CTIMEOUT = 20082
        _DRV_P7INVALID = 20083
        _DRV_P8INVALID = 20084
        _DRV_P9INVALID = 20085
        _DRV_P10INVALID = 20086
        _DRV_P11INVALID = 20087

        _DRV_USBERROR = 20089
        _DRV_IOCERROR = 20090
        _DRV_VRMVERSIONERROR = 20091
        _DRV_GATESTEPERROR = 20092
        _DRV_USB_INTERRUPT_ENDPOINT_ERROR = 20093
        _DRV_RANDOM_TRACK_ERROR = 20094
        _DRV_INVALID_TRIGGER_MODE = 20095
        _DRV_LOAD_FIRMWARE_ERROR = 20096
        _DRV_DIVIDE_BY_ZERO_ERROR = 20097
        _DRV_INVALID_RINGEXPOSURES = 20098
        _DRV_BINNING_ERROR = 20099
        _DRV_INVALID_AMPLIFIER = 20100
        _DRV_INVALID_COUNTCONVERT_MODE = 20101

        _DRV_ERROR_NOCAMERA = 20990
        _DRV_NOT_SUPPORTED = 20991
        _DRV_NOT_AVAILABLE = 20992

        _DRV_ERROR_MAP = 20115
        _DRV_ERROR_UNMAP = 20116
        _DRV_ERROR_MDL = 20117
        _DRV_ERROR_UNMDL = 20118
        _DRV_ERROR_BUFFSIZE = 20119
        _DRV_ERROR_NOHANDLE = 20121

        _DRV_GATING_NOT_AVAILABLE = 20130
        _DRV_FPGA_VOLTAGE_ERROR = 20131

        _DRV_OW_CMD_FAIL = 20150
        _DRV_OWMEMORY_BAD_ADDR = 20151
        _DRV_OWCMD_NOT_AVAILABLE = 20152
        _DRV_OW_NO_SLAVES = 20153
        _DRV_OW_NOT_INITIALIZED = 20154
        _DRV_OW_ERROR_SLAVE_NUM = 20155
        _DRV_MSTIMINGS_ERROR = 20156

        _DRV_OA_NULL_ERROR = 20173
        _DRV_OA_PARSE_DTD_ERROR = 20174
        _DRV_OA_DTD_VALIDATE_ERROR = 20175
        _DRV_OA_FILE_ACCESS_ERROR = 20176
        _DRV_OA_FILE_DOES_NOT_EXIST = 20177
        _DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR = 20178
        _DRV_OA_PRESET_FILE_NOT_LOADED = 20179
        _DRV_OA_USER_FILE_NOT_LOADED = 20180
        _DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED = 20181
        _DRV_OA_INVALID_FILE = 20182
        _DRV_OA_FILE_HAS_BEEN_MODIFIED = 20183
        _DRV_OA_BUFFER_FULL = 20184
        _DRV_OA_INVALID_STRING_LENGTH = 20185
        _DRV_OA_INVALID_CHARS_IN_NAME = 20186
        _DRV_OA_INVALID_NAMING = 20187
        _DRV_OA_GET_CAMERA_ERROR = 20188
        _DRV_OA_MODE_ALREADY_EXISTS = 20189
        _DRV_OA_STRINGS_NOT_EQUAL = 20190
        _DRV_OA_NO_USER_DATA = 20191
        _DRV_OA_VALUE_NOT_SUPPORTED = 20192
        _DRV_OA_MODE_DOES_NOT_EXIST = 20193
        _DRV_OA_CAMERA_NOT_SUPPORTED = 20194
        _DRV_OA_FAILED_TO_GET_MODE = 20195
    
    };
    

    
LIMACORE_API std::ostream& operator <<(std::ostream& os, AndorErrorCode andor_error_code);
    

};
} // namespace Andor
} // namespace lima


#endif // ANDORCAMERA_H
