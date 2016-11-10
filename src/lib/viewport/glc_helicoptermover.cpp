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

//! \file glc_helicoptermover.cpp Implementation of the GLC_HelicopterMover class.

#include "glc_helicoptermover.h"
#include "glc_viewport.h"
#include "QMouseEvent"

// Default constructor
GLC_HelicopterMover::GLC_HelicopterMover(GLC_Viewport* pViewport, const bool forwardDir, const QList<GLC_RepMover*>& repsList)
	: GLC_Mover(pViewport, repsList)
    ,m_TimerInterval(0)
	,m_TimerId(0)
	,m_RotationFactor(-0.000005)
	, m_DistanceFactor(-0.0001)
	, m_MoveForward(forwardDir)
{

}

// Copy constructor
GLC_HelicopterMover::GLC_HelicopterMover(const GLC_HelicopterMover& helicopterMover)
: GLC_Mover(helicopterMover)
, m_TimerInterval(helicopterMover.m_TimerInterval)
, m_TimerId(0)
, m_RotationFactor(helicopterMover.m_RotationFactor)
, m_DistanceFactor(helicopterMover.m_DistanceFactor)
, m_MoveForward(helicopterMover.m_MoveForward)
{

}


GLC_HelicopterMover::~GLC_HelicopterMover()
{
	if (0 != m_TimerId)
	{
		QObject::killTimer(m_TimerId);
	}
}

//////////////////////////////////////////////////////////////////////
// Get Functions
//////////////////////////////////////////////////////////////////////

// Return a clone of the mover
GLC_Mover* GLC_HelicopterMover::clone() const
{
	return new GLC_HelicopterMover(*this);
}


//////////////////////////////////////////////////////////////////////
// Set Functions
//////////////////////////////////////////////////////////////////////

// Initialized the mover
void GLC_HelicopterMover::init(const GLC_UserInput& userInput)
{
    m_startingCoord.setX(userInput.x());
    m_startingCoord.setY(userInput.y());
    m_VecDiff = GLC_Vector2d(0,0);

	m_TimerId = QObject::startTimer(m_TimerInterval);
}

// Move the camera
bool GLC_HelicopterMover::move(const GLC_UserInput& userInput)
{
    const GLC_Point2d currCord(userInput.x(), userInput.y());
	m_VecDiff = currCord - m_startingCoord;	// moving Vector 
	return true;
}

void GLC_HelicopterMover::timerEvent(QTimerEvent*)
{

     GLC_Camera* cam = m_pViewport->cameraHandle();


	 auto eye = cam->eye();
	 auto target = cam->target();
	 auto up = GLC_Vector3d(0, 1, 0);
	 auto forward = cam->forward();
	 GLC_Vector3d increment;
	 if (m_MoveForward)
	 {
		 //Rotation first
		 double angle = m_VecDiff.x() * m_RotationFactor;

		 const GLC_Matrix4x4 rotationMatrix(up, angle);

		 target = rotationMatrix * (target - eye);
		 target = target + eye;
		 increment = target - eye;

		 increment.setY(0);

		 increment = increment.normalize();
		 increment = increment * m_VecDiff.y() * m_DistanceFactor;
	 }
	 else
	 {
		 //diff x is actually a pan, not a rotation in this case
		 double panFactor = m_VecDiff.x() * m_DistanceFactor;

		 auto normal = up ^ forward;
		 normal = normal.normalize();
		 normal.setY(0);

		 increment = normal * panFactor;

		 increment = increment + up * m_VecDiff.y() * m_DistanceFactor;
		 
	 }
    
	auto newTar = target + increment;
	auto newEye = eye + increment;
    cam->setTargetCam(newTar);
    cam->setEyeCam(newEye);
	cam->setUpCam(GLC_Vector3d(0, 1, 0));
	emit updated();
}

void GLC_HelicopterMover::ends()
{
	QObject::killTimer(m_TimerId);
	m_TimerId = 0;
}

