//  Ac3d - 3D objects
//
//  Copyright (C) 2003 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _AC3D_H_
#define _AC3D_H_

#include "../geometry/three-vector.h"

#ifdef WIN32
# define WINDOWS_LEAN_AND_MEAN 1
# define NOMINMAX 1               // Do not define MS' min()/max() macros.
# include <windows.h>
#endif

#include <GL/gl.h>

#include <string>
#include <vector>

namespace Vamos_Media
{
  class Ac3d_Exception
  {
	std::string m_message;
  public:
	Ac3d_Exception (std::string message) : m_message (message) {};
	std::string message () const { return m_message; }
  };

  class No_File : public Ac3d_Exception 
  {
  public:
	No_File (std::string message) : Ac3d_Exception (message) {};
  };

  class Not_An_Ac3d_File : public Ac3d_Exception
  {
  public:
	Not_An_Ac3d_File (std::string message) : Ac3d_Exception (message) {};
  };

  class Malformed_Ac3d_File : public Ac3d_Exception
  {
  public:
	Malformed_Ac3d_File (std::string message) : Ac3d_Exception (message) {};
  };

  class Ac3d_Material;
  class Ac3d_Surface;
  class Ac3d_Object;

  class Ac3d
  {
	std::string m_file;
	int m_version;

	std::vector <const Ac3d_Material*> m_materials;
	std::vector <const Ac3d_Object*> m_objects;

	double m_scale;
	Vamos_Geometry::Three_Vector m_translation;
	Vamos_Geometry::Three_Vector m_rotation; // angle-axis representation

	void read_header (std::ifstream& is);
	const Ac3d_Material* read_material (std::ifstream& is);
	const Ac3d_Object* read_object (std::ifstream& is, 
                                    double scale,
									const Vamos_Geometry::Three_Vector& 
                                    translation,
									const Vamos_Geometry::Three_Vector& 
                                    rotation);
	Ac3d_Surface* read_surface (std::ifstream& is, Ac3d_Object* obj);

  public:
	Ac3d (std::string file, double scale, 
		  const Vamos_Geometry::Three_Vector& rotation,
		  const Vamos_Geometry::Three_Vector& translation);
	~Ac3d ();

	GLuint build ();
  };
}

#endif // not _AC3D_H_