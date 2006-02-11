/****************************************************************************

 This file is part of the GLC-lib library.
 Copyright (C) 2005-2006 Laurent Ribon (laumaya@users.sourceforge.net)
 Version 0.9.4, packaged on January, 2006.

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

//! \file glc_collection.cpp implementation of the GLC_Collection class.

#include <QtDebug>

#include "glc_collection.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GLC_Collection::GLC_Collection()
: m_ListID(0)
, m_ListIsValid(false)
{
}

GLC_Collection::~GLC_Collection()
{
	Erase();
}
//////////////////////////////////////////////////////////////////////
// Fonctions Set
//////////////////////////////////////////////////////////////////////
// Ajoute une g�om�trie � la collection
bool GLC_Collection::AddGLC_Geom(GLC_Geometry* pGeom)
{
	CGeomMap::iterator iGeom= m_TheMap.find(pGeom->GetID());
	
	if (iGeom == m_TheMap.end())
	{	// Ok, la cl� n'est pas prise
		// Ajoute la g�om�trie
		m_TheMap[pGeom->GetID()]= pGeom;
		// Ajoute la sous liste
		m_ListMap[pGeom->GetID()] = 0;
		qDebug("GLC_Collection::AddGLC_Geom : Element Ajout� avec succ�s");
		
		// Validit� de la liste
		m_ListIsValid= false;
		return true;
		
	}
	else
	{	// KO, la cl� est prise
		qDebug("GLC_Collection::AddGLC_Geom : Element already in collection");
		return false;
	}
	
}

// Supprime une g�om�trie de la collection et la g�om�trie
bool GLC_Collection::DelGLC_Geom(GLC_uint Key)
{

	CGeomMap::iterator iGeom= m_TheMap.find(Key);
		
	if (iGeom != m_TheMap.end())
	{	// Ok, la cl� existe
		delete iGeom.value();		// Supprime la g�om�trie
		m_TheMap.remove(Key);		// Supprime le conteneur
		// Recherche la liste
		CListeMap::iterator iListe= m_ListMap.find(Key);
		// \todo V�rifier que la liste d'affichage est trouv�e
		if (!!iListe.value())
		{
			glDeleteLists(iListe.value(),1);
			qDebug("GLC_Collection::DelGLC_Geom : Sous liste Supprim�e");
		}
		m_ListMap.remove(Key);		// Supprime le conteneur
			
		// Validit� de la liste
		m_ListIsValid= false;
		
		qDebug("GLC_Collection::DelGLC_Geom : Element succesfuly deleted");
		return true;
		
	}
	else
	{	// KO, la cl� n'existe pas
		qDebug("GLC_Collection::DelGLC_Geom : El�ment non Supprim�");
		return false;
	}
	
}

// Supprime une g�om�trie de la collection
bool GLC_Collection::RemGLC_Geom(GLC_uint Key)
{
	CGeomMap::iterator iGeom= m_TheMap.find(Key);
		
	if (iGeom != m_TheMap.end())
	{	// Ok, la cl� existe
		// On ne Supprime pas la g�om�trie
		m_TheMap.remove(Key);		// Supprime le conteneur
		// Recherche la liste
		CListeMap::iterator iListe= m_ListMap.find(Key);
		if (iListe != m_ListMap.end())
		{
			qDebug("GLC_Collection::RemGLC_Geom : List not found!!");
		}
		// TODO V�rifier que la liste d'affichage est trouv�e
		if (!!iListe.value())
		{
			glDeleteLists(iListe.value(),1);
			qDebug("GLC_Collection::RemGLC_Geom : Sous liste Supprim�e");
		}
		m_ListMap.remove(Key);		// Supprime le conteneur
			
		// Validit� de la liste
		m_ListIsValid= false;
		
		//qDebug("GLC_Collection::RemGLC_Geom : Element Supprim� avec succ�s");
		return true;
		
	}
	else
	{	// KO, la cl� n'existe pas
		qDebug("GLC_Collection::RemGLC_Geom : El�ment non Supprim�");
		return false;
	}
	
}


// Vide la collection
void GLC_Collection::Erase(void)
{
	// Suppression des g�om�tries
	CGeomMap::iterator iEntry= m_TheMap.begin();
	
    while (iEntry != m_TheMap.constEnd())
    {
        // Supprime l'objet        
        delete iEntry.value();
        ++iEntry;
    }
    // Vide la table de hachage principale
    m_TheMap.clear();
	
	// Fin de la Suppression des g�om�tries

	// Suppression des sous listes d'affichages
	CListeMap::iterator iListEntry= m_ListMap.begin();
	
    while (iListEntry != m_ListMap.constEnd())
    {
        qDebug() << "GLC_Collection::Erase : Try to delete display list ";
        // Supprime l'objet
        if (!!iListEntry.value())
        {
        	qDebug() << "GLC_Collection::Erase : delete display list";
        	glDeleteLists(iListEntry.value(), 1);
        }
        ++iListEntry;
    }
    // Vide la table de hachage de liste
    m_ListMap.clear();

	// Supprime la liste d'affichage
	DeleteList();
	// Fin des Suppressions des sous listes d'affichages

}

// Retourne le pointeur d'un �l�ment de la collection
GLC_Geometry* GLC_Collection::GetElement(GLC_uint Key)
{
	CGeomMap::iterator iGeom= m_TheMap.find(Key);
	
	if (iGeom != m_TheMap.end())
	{	// Ok, la cl� est trouv�
		return iGeom.value();
	}
	else
	{	// KO, la cl� n'est pas trouv�
		return NULL;
	}
}

// Retourne le pointeur d'un �l�ment de la collection
GLC_Geometry* GLC_Collection::GetElement(int Index)
{
	// Warning, performance will be poor
	int CurrentPos= 0;
	CGeomMap::iterator iEntry= m_TheMap.begin();
	GLC_Geometry* pGeom= NULL;
	
    while ((iEntry != m_TheMap.constEnd()) && (CurrentPos <= Index ))
    {
        // retrieve the object        
        if(CurrentPos == Index) pGeom= iEntry.value();
        ++iEntry;
        ++CurrentPos;
    }
 	
 	return pGeom;
}


//////////////////////////////////////////////////////////////////////
// Fonctions OpenGL
//////////////////////////////////////////////////////////////////////
void GLC_Collection::GlExecute(void)
{
	//qDebug() << "GLC_Collection::GlExecute";
	if (GetNumber() > 0)
	{
		CreateMemberLists();		// Si n�cessaire

		CreateSubLists();		// Si n�cessaire

		if (m_ListIsValid)
		{	// La liste de la collection OK
			glCallList(m_ListID);
		}
		else
		{
			if(MemberIsUpToDate())
			{
				CreateList();
			}
			else
			{
				m_ListIsValid= false;
				qDebug("GLC_Collection::GlExecute : CreatMemberList KO -> display list not use");
			}
		}

		// Gestion erreur OpenGL
		GLenum errCode;
		if ((errCode= glGetError()) != GL_NO_ERROR)
		{
			const GLubyte* errString;
			errString = gluErrorString(errCode);
			qDebug("GLC_Collection::GlExecute OPENGL ERROR %s", errString);
		}
	}
}

// Affiche les �l�ments de la collection
void GLC_Collection::GlDraw(void)
{
	CListeMap::iterator iEntry= m_ListMap.begin();
	
    while (iEntry != m_ListMap.constEnd())
    {
        glCallList(iEntry.value());
        ++iEntry;
    }
	

	// Gestion erreur OpenGL
	GLenum errCode;
	if ((errCode= glGetError()) != GL_NO_ERROR)
	{
		const GLubyte* errString;
		errString = gluErrorString(errCode);
		qDebug("GLC_Collection::GlDraw OPENGL ERROR %s", errString);
	}

}

// Cr�ation des listes d'affichages des membres
void GLC_Collection::CreateMemberLists(void)
{
	CGeomMap::iterator iEntry= m_TheMap.begin();
	//qDebug("GLC_Collection::CreateMemberList ENTER");
	
    while (iEntry != m_TheMap.constEnd())
    {
    	if(!iEntry.value()->GetListIsValid())
    	{
     		iEntry.value()->CreateList(GL_COMPILE);
    	}
    	// Passe au Suivant
    	iEntry++;
    }

	// Gestion erreur OpenGL
	if (glGetError() != GL_NO_ERROR)
	{
		qDebug("GLC_Collection::CreateMemberList OPENGL ERROR");
	}


}
// Cr�ation des sous listes d'affichages
void GLC_Collection::CreateSubLists(void)
{
	//qDebug() << "GLC_Collection::CreateSubLists";
	CGeomMap::iterator iEntry= m_TheMap.begin();
	CListeMap::iterator iListEntry;
	
	GLuint ListeID= 0;
    while (iEntry != m_TheMap.constEnd())
    {
    	if(!iEntry.value()->GetValidity())
    	{
    		iListEntry= m_ListMap.find(iEntry.key());
    		assert(iListEntry != m_ListMap.constEnd());
    		if (iListEntry.value() == 0)
    		{// Num�ro non g�n�r�
   				qDebug() << "GLC_Collection::CreateSubLists: List not found";
    			ListeID= glGenLists(1);
    			m_ListMap[iListEntry.key()]= ListeID;
    			m_ListIsValid= false;    			
    		}
    		else
    		{
    			ListeID= iListEntry.value();
    		}
    		// Cr�ation de la liste
    		glNewList(ListeID, GL_COMPILE);
    			iEntry.value()->GlExecute(GL_COMPILE);
    		glEndList();
    		qDebug("GLC_Collection::CreateSubLists : Display list %u created", ListeID);
     	}
    	// Passe au Suivant
    	iEntry++;
    }
	

	// Gestion erreur OpenGL
	if (glGetError() != GL_NO_ERROR)
	{
		qDebug("GLC_Collection::CreateSubLists ERREUR OPENGL");
	}

}

//////////////////////////////////////////////////////////////////////
// Fonctions de services priv�es
//////////////////////////////////////////////////////////////////////
// Verifie si les listes d'affichage des membres sont � jour
bool GLC_Collection::MemberListIsUpToDate(void)
{
	CGeomMap::iterator iEntry= m_TheMap.begin();
	
    while (iEntry != m_TheMap.constEnd())
    {
    	if(iEntry.value()->GetListIsValid() || !iEntry.value()->GetIsVisible())
    	{	// G�om�trie valide ou non visible.
    		iEntry++;   		
    	}
    	else
    	{
 			qDebug("GLC_Collection::MemberListIsUpToDate : Child display list not updated");
			return false;
    	}
     }
		
	return true;	// Toutes les listes sont � jour

}

// Verifie si les membres sont � jour
bool GLC_Collection::MemberIsUpToDate(void)
{
		
	CGeomMap::iterator iEntry= m_TheMap.begin();
	
    while (iEntry != m_TheMap.constEnd())
    {
    	if(iEntry.value()->GetValidity() || !iEntry.value()->GetIsVisible())
    	{	// Membre valide ou non visible.
    		iEntry++;   		
    	}
    	else
    	{
 			//qDebug("GLC_Collection::MemberIsUpToDate : Prop Geom d'un enfant non � jour");
			return false;
    	}
     }
	
	return true;	// Toutes les Membres sont � jour

}

// Cr�ation de la liste d'affichage de la collection
bool GLC_Collection::CreateList(void)
{
	qDebug("GLC_Collection::CreateList");
	
	if(!m_ListID)		// La liste n'a jamais �t� cr��
	{
		m_ListID= glGenLists(1);

		if (!m_ListID)	// ID de liste non obtenu
		{
			GlDraw();
			qDebug("GLC_Collection::CreateList : Display list nor created");
			return false;	// G�om�trie affich� mais pas de liste de cr��
		}
	}

	// Cr�ation de la liste
	glNewList(m_ListID, GL_COMPILE_AND_EXECUTE);				
		// Affichage des �l�ments de la collection
		GlDraw();
	glEndList();
	
	// Validit� de la liste
	m_ListIsValid= true;

	//qDebug("GLC_Collection::CreateList : Liste d'affichage %u cr��", m_ListID);	

	// Gestion erreur OpenGL
	GLenum errCode;
	if ((errCode= glGetError()) != GL_NO_ERROR)
	{
		const GLubyte* errString;
		errString = gluErrorString(errCode);
		qDebug("GLC_Collection::CreateList OPENGL ERROR %s\n", errString);
	}

	return true;	// G�om�trie affich� et liste cr��

}
