/****************************************************************************

 This file is part of the GLC-lib library.
 Copyright (C) 2005-2008 Laurent Ribon (laumaya@users.sourceforge.net)
 http://glc-lib.sourceforge.net

 GLC-lib is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 GLC-lib is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with GLC-lib; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 *****************************************************************************/

//! \file glc_panmover.h Interface for the GLC_HelicopterMover class.

#ifndef GLC_HELICOPTERMOVER_H_
#define GLC_HELICOPTERMOVER_H_

#include "glc_mover.h"

#include <glc_boundingbox.h>
#include "../glc_config.h"

//////////////////////////////////////////////////////////////////////
//! \class GLC_HelicopterMover
/*! \brief GLC_HelicopterMover : Panoramic interactive manipulation */
//////////////////////////////////////////////////////////////////////
class GLC_LIB_EXPORT GLC_HelicopterMover : public GLC_Mover
{
public:
	//! Default constructor
	GLC_HelicopterMover(GLC_Viewport*, const bool forwardDir, const QList<GLC_RepMover*>& repsList = QList<GLC_RepMover*>());

	//! Copy constructor
	GLC_HelicopterMover(const GLC_HelicopterMover&);

	//! Destructor
	virtual ~GLC_HelicopterMover();

//////////////////////////////////////////////////////////////////////
/*! \name Get Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Return a clone of the mover
	virtual GLC_Mover* clone() const;
//@}

//////////////////////////////////////////////////////////////////////
/*! \name Set Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Initialized the mover
	virtual void init(const GLC_UserInput& userInput);

	//! Move the camera
	virtual bool move(const GLC_UserInput& userInput);

	//! Ends this mover
	virtual void ends();

    void setBoundingBox(const GLC_BoundingBox &bbox) 
	{
		m_DistanceFactor = (bbox.xLength() + bbox.yLength() + bbox.zLength()) / 3.0 * -0.00001;
	};
//@}

protected:
	virtual void timerEvent(QTimerEvent*);

private:
    GLC_Vector2d m_startingCoord;
	//! The timer id
	int m_TimerId;

	//! the timer interval
    int m_TimerInterval;

	//! screenspace point difference
	GLC_Vector2d m_VecDiff;
	
	//!	Factor of which translate the screen space difference to world units
	double m_RotationFactor, m_DistanceFactor;

	//! Whether the helicopter direction is upwards(false) or forwards(true)
	bool m_MoveForward;
};

#endif /* GLC_HELICOPTERMOVER_H_ */
