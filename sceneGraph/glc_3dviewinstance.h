/****************************************************************************

 This file is part of the GLC-lib library.
 Copyright (C) 2005-2008 Laurent Ribon (laumaya@users.sourceforge.net)
 Version 1.2.0, packaged on September 2009.

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

//! \file glc_instance.h interface for the GLC_3DViewInstance class.

#ifndef GLC_3DVIEWINSTANCE_H_
#define GLC_3DVIEWINSTANCE_H_

#include "../glc_global.h"
#include "../glc_boundingbox.h"
#include "../glc_object.h"
#include "../maths/glc_matrix4x4.h"
#include "../glc_state.h"
#include "../geometry/glc_3drep.h"
#include "../shading/glc_renderproperties.h"

#include <QMutex>

#include "glc_config.h"

class GLC_Viewport;

//////////////////////////////////////////////////////////////////////
//! \class GLC_3DViewInstance
/*! \brief GLC_3DViewInstance : GLC_3DRep + bounding box*/

/*! An GLC_3DViewInstance contain  :
 * 		- GLC_3DRep
 * 		- Geometry Bounding box in position
 * 		- Positionning 4 x 4 matrix
 */
//////////////////////////////////////////////////////////////////////

class GLC_LIB_EXPORT GLC_3DViewInstance : public GLC_Object
{
//////////////////////////////////////////////////////////////////////
/*! @name Constructor / Destructor */
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Default constructor
	GLC_3DViewInstance();

	//! Contruct instance with a geometry
	GLC_3DViewInstance(GLC_Geometry* pGeom);

	//! Contruct instance with a 3DRep
	GLC_3DViewInstance(const GLC_3DRep&);

	//! Copy constructor
	GLC_3DViewInstance(const GLC_3DViewInstance& );

	//! Assignement operator
	GLC_3DViewInstance &operator=(const GLC_3DViewInstance&);

	//! Destructor
	~GLC_3DViewInstance();

//@}

//////////////////////////////////////////////////////////////////////
/*! \name Get Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Return true if the all instance's mesh are transparent
	inline bool isTransparent() const;

	//! Return true if the instance contains mesh which contains transparent material
	inline bool hasTransparentMaterials() const;

	//! Return true if the instance as no geometry
	inline const bool isEmpty() const
	{return m_3DRep.isEmpty();}

	//! Return true if the instance is selected
	inline const bool isSelected(void) const
	{return m_RenderProperties.isSelected();}

	//! Return the number of geometry
	inline int numberOfGeometry() const
	{return m_3DRep.numberOfBody();}


	//! Return the geometry at the specified position
	inline GLC_Geometry* geomAt(int index) const
	{
		if (!m_3DRep.isEmpty()) return m_3DRep.geomAt(index);
		else return NULL;
	}

	//! Get the bounding box
	GLC_BoundingBox boundingBox();

	//! Get the validity of the Bounding Box
	inline bool boundingBoxValidity() const
	{return (m_pBoundingBox != NULL) && m_IsBoundingBoxValid && m_3DRep.boundingBoxIsValid();}

	//! Return transfomation 4x4Matrix
	inline const GLC_Matrix4x4 matrix() const
	{return m_MatPos;}

	//! Make a deep copy of the instance
	GLC_3DViewInstance deepCopy() const;

	//! Instanciate the instance
	GLC_3DViewInstance instanciate();

	//! Get the Polygon mode off the instance
	/*! Polygon Mode can Be : GL_POINT, GL_LINE, or GL_FILL*/
	inline GLenum polygonMode() const
	{return m_RenderProperties.polygonMode();}

	//! Return an handle to the renderProperties
	inline GLC_RenderProperties* renderPropertiesHandle()
	{return &m_RenderProperties;}

	//! Get the visibility state of instance
	inline bool isVisible() const {return m_IsVisible;}

	//! Get number of faces
	inline unsigned int numberOfFaces() const
	{return m_3DRep.numberOfFaces();}

	//! Get number of vertex
	inline unsigned int numberOfVertex() const
	{return m_3DRep.numberOfVertex();}

	//! Get number of materials
	inline unsigned int numberOfMaterials() const
	{return m_3DRep.numberOfMaterials();}

	//! Get materials List
	inline QSet<GLC_Material*> materialSet() const
	{return m_3DRep.materialSet();}

	//! Return the default LOD Value
	inline int defaultLodValue() const
	{return m_DefaultLOD;}

	//! Return the instance representation
	inline GLC_3DRep representation() const
	{return m_3DRep;}

	//! Return the number of body contains in the 3DRep
	inline int numberOfBody() const
	{return m_3DRep.numberOfBody();}

	//! Return the global default LOD value
	inline static int globalDefaultLod()
	{
		return m_GlobalDefaultLOD;
	}

//@}

//////////////////////////////////////////////////////////////////////
/*! \name Set Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:

	//! Set the instance Geometry
	/*!
	 *  instance must be null
	 */
	bool setGeometry(GLC_Geometry* pGeom);

	//! Remove empty geometries
	inline void removeEmptyGeometry()
	{m_3DRep.clean();}

	//! Reverse geometry normals
	inline void reverseGeometriesNormals()
	{m_3DRep.reverseNormals();}

	//! Translate Instance
	GLC_3DViewInstance& translate(double Tx, double Ty, double Tz);

	//! Translate Instance
	inline GLC_3DViewInstance& translate(const GLC_Vector4d& v)
	{
		return translate(v.X(), v.Y(), v.Z());
	}

	//! Move instance with a 4x4Matrix
	GLC_3DViewInstance& multMatrix(const GLC_Matrix4x4 &MultMat);

	//! Replace the instance Matrix
	GLC_3DViewInstance& setMatrix(const GLC_Matrix4x4 &SetMat);

	//! Reset the instance Matrix
	GLC_3DViewInstance& resetMatrix(void);

	//! Polygon's display style
	/*! Face Polygon Mode can be : GL_FRONT_AND_BACK, GL_FRONT, or GL_BACK
	 *  mode can be : GL_POINT, GL_LINE, or GL_FILL */
	inline void setPolygonMode(GLenum Face, GLenum Mode)
	{m_RenderProperties.setPolygonMode(Face, Mode);}

	//! Select the instance
	inline void select(bool primitive)
	{m_RenderProperties.select(primitive);}

	//! Unselect the instance
	inline void unselect(void)
	{m_RenderProperties.unselect();}

	//! Set instance visibility
	inline void setVisibility(const bool visibility)
	{m_IsVisible= visibility;}

	//! Set Instance Id
	inline void setId(const GLC_uint id)
	{
		GLC_Object::setId(id);
		glc::encodeRgbId(m_Uid, m_colorId);
	}

	//! Set the default LOD value
	inline void setDefaultLodValue(int lod)
	{
		m_DefaultLOD= lod;
	}

	//! Set the global default LOD value
	static void setGlobalDefaultLod(int);

//@}

//////////////////////////////////////////////////////////////////////
/*! \name OpenGL Functions*/
//@{
//////////////////////////////////////////////////////////////////////
public:
	//! Display the instance
	void glExecute(glc::RenderFlag renderFlag= glc::ShadingFlag, bool useLoad= false, GLC_Viewport* pView= NULL);

	//! Display the instance in Body selection mode
	void glExecuteForBodySelection();

	//! Display the instance in Primitive selection mode of the specified body id and return the body index
	int glExecuteForPrimitiveSelection(GLC_uint);


private:
	//! Set instance visualisation properties
	inline void OpenglVisProperties()
	{
		// Polygons display mode
		glPolygonMode(m_RenderProperties.polyFaceMode(), m_RenderProperties.polygonMode());
		// Change the current matrix
		glMultMatrixd(m_MatPos.data());
	}


//@}

//////////////////////////////////////////////////////////////////////
// private services functions
//////////////////////////////////////////////////////////////////////
private:
	//! compute the instance bounding box
	void computeBoundingBox(void);

	//! Clear current instance
	void clear();

	//! Compute LOD
	int choseLod(const GLC_BoundingBox&, GLC_Viewport*);

//////////////////////////////////////////////////////////////////////
// Private members
//////////////////////////////////////////////////////////////////////
private:

	//! The 3D rep of the instance
	GLC_3DRep m_3DRep;

	//! BoundingBox of the instance
	GLC_BoundingBox* m_pBoundingBox;

	//! Geometry matrix
	GLC_Matrix4x4 m_MatPos;

	//! Bounding box validity
	bool m_IsBoundingBoxValid;

	//! The 3DViewInstance rendering properties
	GLC_RenderProperties m_RenderProperties;

	//! Visibility
	bool m_IsVisible;

	//! The instance color ID
	GLubyte m_colorId[4];

	//! The Default LOD
	int m_DefaultLOD;

	//! A Mutex
	static QMutex m_Mutex;

	//! The global default LOD
	static int m_GlobalDefaultLOD;

};

// Return true if the all instance's mesh are transparent
bool GLC_3DViewInstance::isTransparent() const
{
	if (m_3DRep.isEmpty()) return false;
	if (m_RenderProperties.renderingMode() == glc::OverwriteTransparency) return true;
	if (m_RenderProperties.renderingMode() == glc::OverwriteMaterial)
	{
		return m_RenderProperties.overwriteMaterial()->isTransparent();
	}
	const int size= m_3DRep.numberOfBody();
	bool result= true;
	int i= 0;
	while((i < size) && result)
	{
		result= result && m_3DRep.geomAt(i)->isTransparent();
		++i;
	}
	return result && m_RenderProperties.needToRenderWithTransparency();
}

// Return true if the instance contains mesh which contains transparent material
bool GLC_3DViewInstance::hasTransparentMaterials() const
{
	if (m_3DRep.isEmpty()) return false;
	if (m_RenderProperties.needToRenderWithTransparency()) return true;
	const int size= m_3DRep.numberOfBody();
	bool result= false;
	int i= 0;
	while ((i < size) && !result)
	{
		result= result || m_3DRep.geomAt(i)->hasTransparentMaterials();
		++i;
	}
	return result;
}


#endif /*GLC_3DVIEWINSTANCE_H_*/
