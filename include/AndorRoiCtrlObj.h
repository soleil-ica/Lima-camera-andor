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
#ifndef ANDORROICTRLOBJ_H
#define ANDORROICTRLOBJ_H

#include <andor_export.h>

#include "lima/HwRoiCtrlObj.h"
#include "AndorCamera.h"

namespace lima
{
    namespace Andor
    {


/*******************************************************************
 * \class RoiCtrlObj
 * \brief Control object providing Andor Roi interface
 *******************************************************************/

	class ANDOR_EXPORT RoiCtrlObj : public HwRoiCtrlObj
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "RoiCtrlObj", "Andor");

	public:
	    RoiCtrlObj(Camera& cam);
	    virtual ~RoiCtrlObj();

	    virtual void setRoi(const Roi& set_roi);
	    virtual void getRoi(Roi& hw_roi);
	    virtual void checkRoi(const Roi& set_roi, Roi& hw_roi);

	private:
	    Camera& m_cam;
	};


    } // namespace Andor
} // namespace lima

#endif // ANDORROICTRLOBJ_H

