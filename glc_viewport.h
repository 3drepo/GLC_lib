/****************************************************************************

 This file is part of the GLC-lib library.
 Copyright (C) 2005-2008 Laurent Ribon (laumaya@users.sourceforge.net)
 Version 1.0.0, packaged on August, 2008.

 http://glc-lib.sourceforge.net

 GLC-lib is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 GLC-lib is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with GLC-lib; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*****************************************************************************/

//! \file glc_viewport.h interface for the GLC_Viewport class.

#ifndef GLC_VIEWPORT_H_
#define GLC_VIEWPORT_H_
#include "glew/glew.h"
#include <QGLWidget>
#include "glc_camera.h"
#include "glc_orbitcircle.h"
#include "glc_imageplane.h"
#include "glc_boundingbox.h"

#define MAXZOOMFACTOR 3.0

// Define selection square
#define SELECT_WIDTH 5.0

#define SELECT_HEIGHT 5.0

// end selection quare


//////////////////////////////////////////////////////////////////////
//! \class GLC_Viewport
/*! \brief GLC_Viewport : OpenGL Viewport */

/*! An GLC_Viewport define Viewport with these specification
 * 		- Default GLC_Camera
 * 		- Max distance of view
 * 		- Min distance of view
 * 		- Angle of view
 * 		- Selection square
 * 		- Maximum zoom factor
 */
//////////////////////////////////////////////////////////////////////

class GLC_Viewport  
{

//////////////////////////////////////////////////////////////////////
/*! @name Constructor / Destructor */
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Default constructor
	/*! Construct Viewport with these specifications :
	 * 		- Default GLC_Camera
	 * 		- Max distance of view	: <b>500</b>
	 * 		- Min distance of view	: <b>0.01</b>
	 * 		- Angle of view			: <b>35</b>
	 * 		- Selection square		: <b>5.0</b>
	 * 		- Maximum zoom factor	: <b>3.0</b>
	 * */
	GLC_Viewport(QGLWidget *GLWidget);
	
	//! Delete Camera, Image Plane and orbit circle
	virtual ~GLC_Viewport();
	
//@}

//////////////////////////////////////////////////////////////////////
/*! \name Get Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Get The viewport Horizontal size
	int getWinHSize() const
	{
		return m_nWinHSize;
	}
	
	//! Get The viewport Vertical size
	int getWinVSize() const
	{
		return m_nWinVSize;
	}

	//! Map Screen position to OpenGL position (On image Plane)
	GLC_Vector4d mapPosMouse( GLdouble Posx, GLdouble Posy) const;
	
	//! Get camera's angle of view
	double getFov() const
	{
		return m_dFov;
	}

	//! Get near clipping distance
	double getDistMin(void) const
	{
		return m_dCamDistMin;
	}

	//! Get far clipping distance
	double getDistMax(void) const
	{
		return m_dCamDistMax;
	}
	
	//! Get background Color
	QColor getBackgroundColor(void) const
	{
		return m_BackgroundColor;
	}

//@}

//////////////////////////////////////////////////////////////////////
/*! \name OpenGL Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:	
	//! Initialize OpenGL with default values
	/*! Glew initialisation is made here */
	void initGl();
	
	//! Load camera's transformation Matrix and if image plane exist, display it
	void glExecuteCam(void)
	{
		m_pViewCam->glExecute();
		glExecuteImagePlane();
	}
	//! If necessary, display orbit circle and camera's target
	void glExecute()
	{
		glExecuteOrbitCircle();
		glExecuteTargetCam();
	}
	
	//! Define camera's target position
	void glPointing(GLint x, GLint y);
	
	//! Update OpenGL Projection Matrix
	void updateProjectionMat(void) const;

	//! Force the aspect ratio of the window
	void forceAspectRatio(double) const;
//@}
	
//////////////////////////////////////////////////////////////////////
/*! \name OpenGL Functions*/
//@{
//////////////////////////////////////////////////////////////////////
private:
	//! Display orbit circle
	void glExecuteOrbitCircle();

	//! Display camera's target
	void glExecuteTargetCam();

	//! display image plane
	void glExecuteImagePlane();

//@}

//////////////////////////////////////////////////////////////////////
/*! \name Set Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:

	//! Inform the viewport that the OpenGL Viewport has been modified
	void setWinGLSize(int HSize, int VSize);

	//! select an object and return is UID
	/*! Return UID of the nearest picked object */
	GLC_uint select(QGLWidget *pGLWidget, int x, int y);
	
	//! load background image
	void loadBackGroundImage(const QString Image);

	//! delete background image
	void deleteBackGroundImage();

	//! Set Camera's angle of view
	void setFov(double TargetFov)
	{
		m_dFov= TargetFov;

		updateProjectionMat();	// Mise � jour de la matrice de projection opengl
		
		updateOrbitCircle();	// Mise � jour du diam�tre du cercle de la sphere de rotation
	}

	//! Set near clipping distance
	bool setDistMin(double DistMin);

	//! Set far clipping distance
	bool setDistMax(double DistMax);
	
	//! Set Near and Far clipping distance
	/*! box shouldn't be empty*/
	void setDistMinAndMax(const GLC_BoundingBox& bBox);
	
	//! Set the Background color
	void setBackgroundColor(QColor setColor);

//@}

/////////////////////////////////////////////////////////////////////	
//! @name  Panning Functions
//@{
	//! Prepare panning operation
	/*!	- Save the initial mouse position
	 * 	- Show Camera's target if required
	 */
	void  preparePanning(double Cx, double Cy, bool targetVisibility= true)
	{
		m_VectPrevPan.setVect(mapPosMouse(Cx,Cy));
		m_CameraTargetIsVisible= targetVisibility;
	}
	//! pan to the position (Cx, Cy)
	void pan(double Cx, double Cy);

	//! Hide Camera'target
	void endPanning()
	{
		m_CameraTargetIsVisible= false;
	}
//@} End of panning functions
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
//! @name orbit Functions
//@{
	//! Prepare orbiting operation
	/*!	- Save initial mouse position
	 * 	- Show orbit Circle
	 * 	- Show Camera's Target
	 */
	void prepareOrbiting(double Cx, double Cy, bool circleVisibility= true, bool targetVisibility= true);
	
	//! orbit in accordance of mouse position
	void orbit(double Cx, double Cy);

	//! Hide camera's target and orbit circle
	void endOrbit()
	{
		m_OrbitCircleIsVisible= false;
		m_CameraTargetIsVisible= false;
	}
//@} End of orbit functions
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
//! @name zoom Functions
//{@
	//! Prepare Zooming operation
	/*!	- Save Y mouse position
	 * 	- Show Camera's target
	 */
	void prepareZooming(int Cy, bool tragetVisibility= true);
	
	//! zoom in accordance of Y mouse position
	void zoom(int Cy);
	
	//! Hide Camera's target
	void EndZooming()
	{
		m_CameraTargetIsVisible= false;
	}
	
	//! reframe the current scene
	/*! box shouldn't be empty*/
	void reframe(const GLC_BoundingBox& box);
	
//@} End Zooming functions
/////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Private services Functions
//////////////////////////////////////////////////////////////////////
private:
	//! Convert mouse View coordinate to orbit coordinate (Centred and betwen (-1,-1) and (1,1))
	GLC_Vector4d mapForOrbit( double Posx, double Posy) const;
	
	//! Update orbit circle dimensions
	void updateOrbitCircle();

	//! Change to selection mode, save Visualisation state
	void beginSelection(GLdouble x, GLdouble y);

	//! End of selection mode, restore Visualisation state
	void endSelection(void);

//////////////////////////////////////////////////////////////////////
// Publics Member
//////////////////////////////////////////////////////////////////////
public:

	//! Viewport's Camera
	GLC_Camera *m_pViewCam;

//////////////////////////////////////////////////////////////////////
// Private Members
//////////////////////////////////////////////////////////////////////
private:

	GLdouble m_dCamDistMax;		//!< Camera Maximum distance (far clipping plane)
	GLdouble m_dCamDistMin;		//!< Camera Minimum distance (near clipping plane)
	GLdouble m_dFov;			//!< Camera angle of view
	
	//! Image plane (Background image)
	GLC_ImagePlane* m_pImagePlane;

	//! Show state of camera's target
	bool m_CameraTargetIsVisible;


	// OpenGL View Definition
	int m_nWinHSize;			//!< Horizontal OpenGL viewport size
	int m_nWinVSize;			//!< Vertical OpenGL viewport size

	//! Previous zoom Factor
	double m_dPrevZoom;	
	
	//! orbit Circle
	GLC_OrbitCircle* m_pOrbitCircle;

	//! Circle ratio size / window size
	double m_dRatWinSph;
	
	//! Show state of orbit Circle
	bool m_OrbitCircleIsVisible;

	//! Panning Vector
	GLC_Vector4d m_VectPrevPan;

	//! Orbiting Vector
	GLC_Vector4d m_VectPrevOrbit;	

	//! The QGLWidget attached to the viewport (rendering context)
	QGLWidget* m_pQGLWidget;
	
	//! Viewport Background color
	QColor m_BackgroundColor;
	
	//! OpenGL list of the Image Plane
	GLuint m_ImagePlaneListID;
	

};

#endif //GLC_VIEWPORT_H_
