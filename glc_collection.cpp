/****************************************************************************

 This file is part of the GLC-lib library.
 Copyright (C) 2005 Laurent Ribon (laumaya@users.sourceforge.net)
 Version 0.9, packaged on Novemeber, 2005.

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
: m_ListeID(0)
, m_bListeIsValid(false)
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
		m_ListeMap[pGeom->GetID()] = 0;
		//TRACE("GLC_Collection::AddGLC_Geom : Element Ajout� avec succ�s\n");
		
		// Validit� de la liste
		m_bListeIsValid= false;
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
		CListeMap::iterator iListe= m_ListeMap.find(Key);
		// \todo V�rifier que la liste d'affichage est trouv�e
		if (!!iListe.value())
		{
			glDeleteLists(iListe.value(),1);
			//qDebug("GLC_Collection::DelGLC_Geom : Sous liste %u Supprim�e\n", RemListe);
		}
		m_ListeMap.remove(Key);		// Supprime le conteneur
			
		// Validit� de la liste
		m_bListeIsValid= false;
		
		//qDebug("GLC_Collection::DelGLC_Geom : Element succesfuly deleted);
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
		CListeMap::iterator iListe= m_ListeMap.find(Key);
		// TODO V�rifier que la liste d'affichage est trouv�e
		if (!!iListe.value())
		{
			glDeleteLists(iListe.value(),1);
			//qDebug("GLC_Collection::RemGLC_Geom : Sous liste %u Supprim�e", RemListe);
		}
		m_ListeMap.remove(Key);		// Supprime le conteneur
			
		// Validit� de la liste
		m_bListeIsValid= false;
		
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
	CListeMap::iterator iListEntry= m_ListeMap.begin();
	
    while (iListEntry != m_ListeMap.constEnd())
    {
        // Supprime l'objet
        if (!!iListEntry.value()) glDeleteLists(iListEntry.value(), 1);
        ++iListEntry;
    }
    // Vide la table de hachage de liste
    m_ListeMap.clear();

	// Supprime la liste d'affichage
	DeleteListe();
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

//////////////////////////////////////////////////////////////////////
// Fonctions OpenGL
//////////////////////////////////////////////////////////////////////
void GLC_Collection::GlExecute(void)
{
	if (GetNumber() > 0)
	{
		CreateMemberList();		// Si n�cessaire

		CreateSousList();		// Si n�cessaire

		if (m_bListeIsValid)
		{	// La liste de la collection OK
			glCallList(m_ListeID);
		}
		else
		{
			if(MemberIsUpToDate())
			{
				CreationListe();
			}
			else
			{
				m_bListeIsValid= false;
				qDebug("GLC_Collection::DrawGl : CreatMemberList KO -> Affichage �l�ments\n");
			}
		}

		// Gestion erreur OpenGL
		GLenum errCode;
		if ((errCode= glGetError()) != GL_NO_ERROR)
		{
			const GLubyte* errString;
			errString = gluErrorString(errCode);
			qDebug("GLC_Collection::DrawGl ERREUR OPENGL %s\n", errString);
		}
	}
}

// Affiche les �l�ments de la collection
void GLC_Collection::GlDraw(void)
{
	CListeMap::iterator iEntry= m_ListeMap.begin();
	
    while (iEntry != m_ListeMap.constEnd())
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
		qDebug("GLC_Collection::GlDraw ERREUR OPENGL %s\n", errString);
	}

}

// Cr�ation des listes d'affichages des membres
void GLC_Collection::CreateMemberList(void)
{
	CGeomMap::iterator iEntry= m_TheMap.begin();
	
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
		qDebug("GLC_Collection::CreateMemberList ERREUR OPENGL");
	}

}
// Cr�ation des sous listes d'affichages
void GLC_Collection::CreateSousList(void)
{
	CGeomMap::iterator iEntry= m_TheMap.begin();
	CListeMap::iterator iListEntry;
	
	GLuint ListeID= 0;
    while (iEntry != m_TheMap.constEnd())
    {
    	if(!iEntry.value()->GetValidity())
    	{
    		iListEntry= m_ListeMap.find(iEntry.key());
    		if(iListEntry != m_ListeMap.constEnd())
    		{	// Num�ro non g�n�r�
    			ListeID= glGenLists(1);
    			m_ListeMap[iEntry.key()]= ListeID;
    			m_bListeIsValid= false;
    		}
    		// Cr�ation de la liste
    		glNewList(ListeID, GL_COMPILE);
    			iEntry.value()->GlExecute(GL_COMPILE);
    		glEndList();
    		//qDebug("GLC_Collection::CreateSousList : Liste d'affichage %u cr��", ListeID);
     	}
    	// Passe au Suivant
    	iEntry++;
    }
	

	// Gestion erreur OpenGL
	if (glGetError() != GL_NO_ERROR)
	{
		qDebug("GLC_Collection::CreateSousList ERREUR OPENGL\n");
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
 			//qDebug("GLC_Collection::MemberListIsUpToDate : Liste d'affichage d'un enfant non � jour");
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
bool GLC_Collection::CreationListe(void)
{
	
	if(!m_ListeID)		// La liste n'a jamais �t� cr��
	{
		m_ListeID= glGenLists(1);

		if (!m_ListeID)	// ID de liste non obtenu
		{
			GlDraw();
			//qDebug("GLC_Collection::CreateList : ERREUR Liste d'affichage NON cr��");
			return false;	// G�om�trie affich� mais pas de liste de cr��
		}
	}

	// Cr�ation de la liste
	glNewList(m_ListeID, GL_COMPILE_AND_EXECUTE);				
		// Affichage des �l�ments de la collection
		GlDraw();
	glEndList();
	
	// Validit� de la liste
	m_bListeIsValid= true;

	//qDebug("GLC_Collection::CreateList : Liste d'affichage %u cr��", m_ListID);	

	// Gestion erreur OpenGL
	GLenum errCode;
	if ((errCode= glGetError()) != GL_NO_ERROR)
	{
		const GLubyte* errString;
		errString = gluErrorString(errCode);
		qDebug("GLC_Collection::CreationListe ERREUR OPENGL %s\n", errString);
	}

	return true;	// G�om�trie affich� et liste cr��

}
