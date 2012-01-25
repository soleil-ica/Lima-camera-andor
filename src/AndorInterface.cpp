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
    
    m_cap_list.push_back(HwCap(cam.getBufferMgr()));
    
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
//
//-----------------------------------------------------
Interface::~Interface()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
    DEB_MEMBER_FUNCT();
    cap_list = m_cap_list;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(reset_level);

    stopAcq();

    m_cam._setStatus(Camera::Ready,true);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::prepareAcq()
{
    DEB_MEMBER_FUNCT();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::startAcq()
{
    DEB_MEMBER_FUNCT();
    m_cam.startAcq();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::stopAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.stopAcq();
}

//-----------------------------------------------------
//
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
//
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
     DEB_MEMBER_FUNCT();
     int acq_frames;
     m_cam.getNbHwAcquiredFrames(acq_frames);
     return acq_frames;
}



//-----------------------------------------------------
//-----------------------------------------------------
//
//-----------------------------------------------------
