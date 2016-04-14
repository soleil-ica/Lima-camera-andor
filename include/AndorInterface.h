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
#ifndef ANDORINTERFACE_H
#define ANDORINTERFACE_H

#include "AndorCompatibility.h"
#include "lima/HwInterface.h"
#include "AndorCamera.h"
#include "AndorDetInfoCtrlObj.h"
#include "AndorSyncCtrlObj.h"
#include "AndorShutterCtrlObj.h"
#include "AndorBinCtrlObj.h"
#include "AndorRoiCtrlObj.h"


namespace lima
{
    namespace Andor
    {
	class Interface;

/*******************************************************************
 * \class Interface
 * \brief Andor hardware interface
 *******************************************************************/

	class LIBANDOR_API Interface : public HwInterface
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "AndorInterface", "Andor");

	public:
	    Interface(Camera& cam);
	    virtual ~Interface();

	    //- From HwInterface
	    virtual void    getCapList(CapList&) const;
	    virtual void    reset(ResetLevel reset_level);
	    virtual void    prepareAcq();
	    virtual void    startAcq();
	    virtual void    stopAcq();
	    virtual void    getStatus(StatusType& status);
	    virtual int     getNbHwAcquiredFrames();

	    // - From AndorCamera
	    void setAdcSpeed(int index);
	    void getAdcSpeed(int& index);
	    void getAdcSpeedMaxIndex(int& max_index){m_cam.getAdcSpeedMaxIndex(max_index);};
	    void getAdcSpeedPaireString(int index, string& pgain){m_cam.getAdcSpeedPaireString(index, pgain);};
	    void setVsSpeed(int index);
	    void getVsSpeed(int& index);
	    void getVsSpeedMaxIndex(int& max_index){m_cam.getVsSpeedMaxIndex(max_index);};
	    void getVsSpeedString(int index, string& speed){m_cam.getVsSpeedString(index, speed);};
	    void setPGain(int index);
	    void getPGain(int& index);
	    void getPGainMaxIndex(int& max_index) {m_cam.getPGainMaxIndex(max_index);};
	    void getPGainString(int index, string& pgain){m_cam.getPGainString(index, pgain);};
	    void setFastExtTrigger(bool flag);
	    void getFastExtTrigger(bool& flag);
	    void setShutterLevel(int level);
	    void getShutterLevel(int& level);
	    void setTemperatureSP(int temp);
	    void getTemperatureSP(int& temp);
	    void getTemperature(int& temp);
	    void setCooler(bool flag);
	    void getCooler(bool& flag);
	    void getCoolingStatus(std::string& status);    
	    void setSpooling(bool flag, SpoolingMethod method, std::string path, int frameBufferSize);
	    void setHighCapacity(HighCapacityMode mode);
	    void getHighCapacity(HighCapacityMode& mode);
	    void setFanMode(FanMode mode);
	    void getFanMode(FanMode& mode);
	    void setGateMode(GateMode mode);
	    //! get the camera object to access it directly from client
	    Camera& getCamera() { return m_cam;}
	    void setBaselineClamp(bool enable);
	    void getBaselineClamp(bool& enable);

	private:
	    Camera&         m_cam;
	    CapList         m_cap_list;
	    DetInfoCtrlObj  m_det_info;
	    SyncCtrlObj     m_sync;
	    BinCtrlObj      m_bin;
	    RoiCtrlObj      m_roi;
	    ShutterCtrlObj  m_shutter;
	    mutable Cond    m_cond;
	};



    } // namespace Andor
} // namespace lima

#endif // ANDORINTERFACE_H
