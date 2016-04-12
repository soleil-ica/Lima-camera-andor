//###########################################################################
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
//###########################################################################
#include "AndorInterface.h"
#include <algorithm>

using namespace lima;
using namespace lima::Andor;
using namespace std;


//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
Interface::Interface(Camera& cam)
    : m_cam(cam),m_det_info(cam),
      m_sync(cam),m_bin(cam),m_roi(cam),
      m_shutter(cam)
{
    DEB_CONSTRUCTOR();
    
    HwDetInfoCtrlObj *det_info = &m_det_info;
    m_cap_list.push_back(HwCap(det_info));
    
    m_cap_list.push_back(HwCap(cam.getBufferCtrlObj()));
    
    HwSyncCtrlObj *sync = &m_sync;
    m_cap_list.push_back(HwCap(sync));
    
    HwRoiCtrlObj *roi = &m_roi;
    m_cap_list.push_back(HwCap(roi));
    
    if(m_cam.isBinningAvailable())
    {
	 HwBinCtrlObj *bin = &m_bin;
	 m_cap_list.push_back(HwCap(bin));
    }
    HwShutterCtrlObj *shutter = &m_shutter;
    m_cap_list.push_back(HwCap(shutter));
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
Interface::~Interface()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
// @brief return the capability list
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
    DEB_MEMBER_FUNCT();
    cap_list = m_cap_list;
}

//-----------------------------------------------------
// @brief reset the interface, stop the acqisition
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(reset_level);

    stopAcq();

    m_cam._setStatus(Camera::Ready,true);
}

//-----------------------------------------------------
// @brief do nothing
//-----------------------------------------------------
void Interface::prepareAcq()
{
    DEB_MEMBER_FUNCT();
    m_cam.prepareAcq();
}

//-----------------------------------------------------
// @brief start the camera acquisition
//-----------------------------------------------------
void Interface::startAcq()
{
    DEB_MEMBER_FUNCT();
    m_cam.startAcq();
}

//-----------------------------------------------------
// @brief stop the camera acquisition
//-----------------------------------------------------
void Interface::stopAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.stopAcq();
}

//-----------------------------------------------------
// @brief return the status of detector/acquisition
//-----------------------------------------------------
void Interface::getStatus(StatusType& status)
{
    DEB_MEMBER_FUNCT();
    
    Camera::Status andor_status = Camera::Ready;
    m_cam.getStatus(andor_status);
    switch (andor_status)
    {
    case Camera::Ready:
	status.acq = AcqReady;
	status.det = DetIdle;
	break;
    case Camera::Exposure:
	status.det = DetExposure;
	status.acq = AcqRunning;
	break;
    case Camera::Readout:
	status.det = DetReadout;
	status.acq = AcqRunning;
	break;
    case Camera::Latency:
	status.det = DetLatency;
	status.acq = AcqRunning;
	break;
    case Camera::Fault:
	status.det = DetFault;
	status.acq = AcqFault;
    }
    status.det_mask = DetExposure | DetReadout | DetLatency;
    
    DEB_RETURN() << DEB_VAR1(status);
}


//-----------------------------------------------------
// @brief return the hw number of acquired frames
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
     DEB_MEMBER_FUNCT();
     int acq_frames;
     m_cam.getNbHwAcquiredFrames(acq_frames);
     return acq_frames;
}

/////////////////////////////////////////////////////////////
// HERE we just map setter/getter methods of the AndorCamera 
// class for a public access, stupid but useful !!
// one could merge AndorCamera and AndorInterface, ok next 
// release.
/////////////////////////////////////////////////////////////

//-----------------------------------------------------
// @brief	set ADC/Speed settings
// @param	adc pais adc/speed index (if =-1, set to max speed)
//
//-----------------------------------------------------
void Interface::setAdcSpeed(int adc)
{
    m_cam.setAdcSpeed(adc);
}

//-----------------------------------------------------
// @brief	get ADC/Speed settings
// @param	adc index
//
//-----------------------------------------------------
void Interface::getAdcSpeed(int& adc)
{
    m_cam.getAdcSpeed(adc);
}

//-----------------------------------------------------
// @brief	set Vertical Shift Speed
// @param	vss index (if =-1, set to recommended)
//
//-----------------------------------------------------
void Interface::setVsSpeed(int vss)
{
    m_cam.setVsSpeed(vss);
}

//-----------------------------------------------------
// @brief	get Vertical Shift Speed
// @param	vss index
//
//-----------------------------------------------------
void Interface::getVsSpeed(int& vss)
{
    m_cam.getVsSpeed(vss);
}

//-----------------------------------------------------
// @brief	set Preamp Gain
// @param	gain premap gain index
//
//-----------------------------------------------------
void Interface::setPGain(int gain) 
{
    m_cam.setPGain(gain);
}

//-----------------------------------------------------
// @brief	get Preamp Gain
// @param	gain premap gain index
//
//-----------------------------------------------------
void Interface::getPGain(int& gain) 
{
    m_cam.getPGain(gain);
}

//-----------------------------------------------------
// @brief	set external trigger for fast mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Interface::setFastExtTrigger(bool flag)
{
    m_cam.setFastExtTrigger(flag);
}

//-----------------------------------------------------
// @brief	get external fast trigger mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Interface::getFastExtTrigger(bool& flag)
{
    m_cam.getFastExtTrigger(flag);
}

//-----------------------------------------------------
// @brief	set the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Interface::setShutterLevel(int level)
{
    m_cam.setShutterLevel(level);
}

//-----------------------------------------------------
// @brief	get the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Interface::getShutterLevel(int& level)
{
    m_cam.getShutterLevel(level);
}

//-----------------------------------------------------
// @brief	set the temperature set-point
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Interface::setTemperatureSP(int temp)
{
    m_cam.setTemperatureSP(temp);
}

//-----------------------------------------------------
// @brief	return the temperature set-point
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Interface::getTemperatureSP(int& temp)
{
    m_cam.getTemperatureSP(temp);
}


//-----------------------------------------------------
// @brief	Gets the real temperature of the detector sensor 
// @param	temp temperature in centigrade
//
//-----------------------------------------------------
void Interface::getTemperature(int& temp)
{
    m_cam.getTemperature(temp);
}

//-----------------------------------------------------
// @brief	start or Stop the cooler 
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void Interface::setCooler(bool flag)
{
    m_cam.setCooler(flag);
}
//-----------------------------------------------------
// @brief	Get the Cooler status  
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void Interface::getCooler(bool& flag)
{
    m_cam.getCooler(flag);
}

//-----------------------------------------------------
// @brief	Gets cooling status
// @param	status status as a string
//
//-----------------------------------------------------
void Interface::getCoolingStatus(std::string& status)   
{
    m_cam.getCoolingStatus(status);
}

 //-----------------------------------------------------
// @brief	Sets spooling specific parameters
// @param	flag            enable/disable spooling
// @param   method          used spooling method
// @param   path            spooling dir/file
// @param   framBufferSize  size of the internal circular buffer
//
//-----------------------------------------------------
void Interface::setSpooling(bool flag, SpoolingMethod method, string path, int frameBufferSize)
{
    m_cam.setSpooling(flag, method, path, frameBufferSize);
}


//-----------------------------------------------------
// @brief	Sets the High capacity mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Interface::setHighCapacity(HighCapacityMode mode)
{
    m_cam.setHighCapacity(mode);
}

//-----------------------------------------------------
// @brief	Gets the High capacity mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Interface::getHighCapacity(HighCapacityMode& mode)
{
    m_cam.getHighCapacity(mode);
}

//-----------------------------------------------------
// @brief	Sets the FAN mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Interface::setFanMode(FanMode mode)
{
    m_cam.setFanMode(mode);
}

//-----------------------------------------------------
// @brief	Gets the Fan mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Interface::getFanMode(FanMode& mode)
{
    m_cam.getFanMode(mode);
}


//-----------------------------------------------------
// @brief	Sets the gate mode
// @param	mode    mode to set
//
//-----------------------------------------------------
void Interface::setGateMode(GateMode mode)
{
    m_cam.setGateMode(mode);
}

//-----------------------------------------------------
// @brief	Gets the baseline clamp status
// @param	enable    enable (true) or disable (false)
//
//-----------------------------------------------------
void Interface::getBaselineClamp(bool& enable)
{
    m_cam.getBaselineClamp(enable);
}


//-----------------------------------------------------
// @brief	Sets the gate mode
// @param	enable    enable (true) or disable (false)
//
//-----------------------------------------------------
void Interface::setBaselineClamp(bool enable)
{
    m_cam.setBaselineClamp(enable);
}

//-----------------------------------------------------
//-----------------------------------------------------
//
//-----------------------------------------------------
