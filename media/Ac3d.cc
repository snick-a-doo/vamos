//  Ac3d - 3D objects
//
//	Vamos Automotive Simulator
//  Copyright (C) 2003 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include "Ac3d.h"
#include "Texture_Image.h"
#include "../geometry/Constants.h"
#include "../geometry/Three_Matrix.h"

#include <fstream>
#include <sstream>
#include <cassert>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <functional>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

namespace Vamos_Media
{
  // The material properties for a surface.  The names of their
  // members and accessors mean the same things as the OpenGL
  // material parameters.
  class Ac3d_Material
  {
	std::string m_name;
	GLfloat m_color [3];
	GLfloat m_ambient [3];
	GLfloat m_emission [3];
	GLfloat m_specular [3];
	GLfloat m_shininess;
	GLfloat m_transparency;
  
  public:
	Ac3d_Material (std::string name_in,
				   GLfloat color_in [3],
				   GLfloat ambient_in [3],
				   GLfloat emission_in [3],
				   GLfloat specular_in [3],
				   GLfloat shininess_in,
				   GLfloat transparency_in);

	void set_gl_properties (GLenum face) const;
  };

  Ac3d_Material::Ac3d_Material (std::string name_in,
								GLfloat color_in [3],
								GLfloat ambient_in [3],
								GLfloat emission_in [3],
								GLfloat specular_in [3],
								GLfloat shininess_in,
								GLfloat transparency_in) :
	m_name (name_in),
	m_shininess (shininess_in),
	m_transparency (transparency_in)
  {
	for (size_t i = 0; i < 3; i++)
	  {
		m_color [i] = color_in [i];
		m_ambient [i] = ambient_in [i];
		m_emission [i] = emission_in [i];
		m_specular [i] = specular_in [i];
	  }
  }

  void
  Ac3d_Material::set_gl_properties (GLenum face) const
  {
	glColor4f (m_color [0], m_color [1], m_color [2], 1.0 - m_transparency);
	glMaterialfv (face, GL_AMBIENT, m_ambient);
	glMaterialfv (face, GL_EMISSION, m_emission);
	glMaterialfv (face, GL_SPECULAR, m_specular);
	glMaterialfv (face, GL_SHININESS, &m_shininess);
  }


  // A surface is a rendered figure such as a triangle, or a strip of
  // quadrilaterals.  Vertices, normals, and material properties
  // are stored in a surface object.
  class Ac3d_Surface
  {
  public:
	struct Vertex
	{
	  Vertex (const Three_Vector* coordinates, 
			  const Three_Vector* normal,
			  double texture_x, double texture_y);
	  const Three_Vector* coordinates;
	  const Three_Vector* normal;
	  double texture_x;
	  double texture_y;
	};

	// The first three are defined by the AC3D file format.  The
	// POLYGON type may be changed to one of the triangle or
	// quadrilateral types.
	enum Figure_Type { POLYGON, LINE, CLOSED_LINE, 
					   TRIANGLE, TRIANGLE_STRIP, TRIANGLE_FAN,
					   QUADRILATERAL, QUADRILATERAL_STRIP };

  private:
	// Materials and vertices are handled by pointer.  Vertices on
	// different surfaces may point to the same vertex object.
	const Ac3d_Material* mp_material;
	std::vector <const Vertex*> m_vertices;

	// The normal vector for a flat-shaded surface.  Smooth shading
	// uses the `normal' member of the Vertex structure.
	Three_Vector m_normal;

	// See enum Figure_Type above.
	Figure_Type m_figure_type;
	bool m_shaded;
	bool m_two_sided;

	// Transformations
	double m_scale;
	Three_Vector m_offset;
	Three_Matrix m_rotation;

	// Set the appropriate OpenGL attributes.
	void set_attributes () const;

	// Return the appropriate argument for glBegin().
	GLenum get_gl_figure_type () const;

	void set_material_properties () const;

	void draw_figure () const;

  public:
	Ac3d_Surface (std::string figure_type_code, 
				  double scale,
				  const Three_Vector& offset,
				  const Three_Matrix& rotation);

	virtual ~Ac3d_Surface ();

	void set_material (const Ac3d_Material* mat) { mp_material = mat; }
	const Ac3d_Material* get_material () const { return mp_material; }

	void set_figure_type (Figure_Type type) { m_figure_type = type; }
	Figure_Type get_figure_type () const { return m_figure_type; }

	void add_vertex (const Vertex* vert) { m_vertices.push_back (vert); }
	void set_vertices (const std::vector <const Vertex*>& verts)
	{ m_vertices = verts; }
	const std::vector <const Vertex*> get_vertices () const
	{ return m_vertices; }

	// Change the order of the vertices when converting a polygon to a
	// strip.
	void rearrange_vertices (size_t i0, size_t i1, size_t i2, size_t i3);
	void rearrange_vertices (size_t i0, size_t i1, size_t i2);

	void set_normal (const Three_Vector& norm) { m_normal = norm; }

	bool is_shaded () const { return m_shaded; }
	bool is_two_sided () const { return m_two_sided; }

	// Add the OpenGL calls for rendering this surface to the display list.
	void build () const;
  };


  Ac3d_Surface::Ac3d_Surface (std::string figure_type_code, 
							  double scale,
							  const Three_Vector& offset,
							  const Three_Matrix& rotation) :
	m_scale (scale),
	m_offset (offset),
	m_rotation (rotation)
  {
	m_normal.z = 1.0;

	std::istringstream is (figure_type_code);
	is.setf (std::ios_base::hex, std::ios_base::basefield);
	int code;
	is >> code;

	int figure_type = Figure_Type (code & 0x0f);
	if ((figure_type != POLYGON) 
		&& (figure_type != CLOSED_LINE) 
		&& (figure_type != LINE))
	  {
		throw Malformed_Ac3d_File ("Unrecognized figure type");
	  }
	m_figure_type = Figure_Type (figure_type);
	m_shaded = bool (code & 0x10);
	m_two_sided = bool (code & 0x20);
  }


  Ac3d_Surface::~Ac3d_Surface ()
  {
	for (std::vector <const Vertex*>::iterator it = m_vertices.begin ();
		 it != m_vertices.end ();
		 it++)
	  {
		delete *it;
	  }
  }


  void
  Ac3d_Surface::build () const
  {
	if (m_vertices.size () == 0) return;

	glPushAttrib (GL_ENABLE_BIT);
	{
	  set_attributes ();

	  glBegin (get_gl_figure_type ());
	  {
		set_material_properties ();
		draw_figure ();
	  }
	  glEnd ();
	}
	glPopAttrib ();
  }


  void
  Ac3d_Surface::set_attributes () const
  {
	if (m_two_sided)
	  {
		glDisable (GL_CULL_FACE);
	  }
	else
	  {
		glEnable (GL_CULL_FACE);
	  }
  }


  void
  Ac3d_Surface::rearrange_vertices (size_t i0, size_t i1, size_t i2, size_t i3)
  {
	std::vector <const Vertex*> new_vertex_order;
	new_vertex_order.resize (4);
	new_vertex_order [0] = m_vertices [i0];
	new_vertex_order [1] = m_vertices [i1];
	new_vertex_order [2] = m_vertices [i2];
	new_vertex_order [3] = m_vertices [i3];
	m_vertices = new_vertex_order;
  }

  void
  Ac3d_Surface::rearrange_vertices (size_t i0, size_t i1, size_t i2)
  {
	std::vector <const Vertex*> new_vertex_order;
	new_vertex_order.resize (3);
	new_vertex_order [0] = m_vertices [i0];
	new_vertex_order [1] = m_vertices [i1];
	new_vertex_order [2] = m_vertices [i2];
	m_vertices = new_vertex_order;
  }


  GLenum 
  Ac3d_Surface::get_gl_figure_type () const
  {
	const size_t number_of_vertices = m_vertices.size ();

	switch (m_figure_type)
	  {
	  case LINE:
		return GL_LINE_STRIP;

	  case CLOSED_LINE:
		return GL_LINE_LOOP;

	  case TRIANGLE:
		assert (number_of_vertices == 3);
		return GL_TRIANGLES;

	  case TRIANGLE_STRIP:
		assert (number_of_vertices > 3);
		return GL_TRIANGLE_STRIP;

	  case TRIANGLE_FAN:
		assert (number_of_vertices > 3);
		return GL_TRIANGLE_FAN;

	  case QUADRILATERAL:
		assert (number_of_vertices == 4);
		return GL_QUADS;

	  case QUADRILATERAL_STRIP:
		assert (number_of_vertices >= 4);
		assert (number_of_vertices % 2 == 0);
		return GL_QUAD_STRIP;

	  case POLYGON:
		assert (number_of_vertices > 4);
		return GL_POLYGON;

	  default:
		throw Malformed_Ac3d_File ("Unrecognized figure type");
	  }
  }

  void
  Ac3d_Surface::set_material_properties () const
  {
	GLenum face = m_two_sided ? GL_FRONT_AND_BACK : GL_FRONT;

	glColorMaterial (face, GL_AMBIENT_AND_DIFFUSE);
	glEnable (GL_COLOR_MATERIAL);

	mp_material->set_gl_properties (face);
  }


  void
  Ac3d_Surface::draw_figure () const
  {
	Three_Vector norm = (m_rotation * m_normal);
	for (std::vector <const Vertex*>::const_iterator it = m_vertices.begin ();
		 it != m_vertices.end ();
		 it++)
	  {
		glTexCoord2f ((*it)->texture_x, (*it)->texture_y);

		if (m_shaded)
		  {
			norm = (m_rotation * *((*it)->normal)).unit ();
		  }
		glNormal3d (norm.x, norm.y, norm.z);

		// Rotate and translate the vertex.
		const Three_Vector& pos = m_offset 
		  + m_scale * (m_rotation * *((*it)->coordinates));
		  
		glVertex3f (pos.x, pos.y, pos.z);
	  }
  }


  Ac3d_Surface::Vertex::Vertex (const Three_Vector* coordinates_in, 
								const Three_Vector* normal_in,
								double texture_x_in, double texture_y_in) :
	coordinates (coordinates_in),
	normal (normal_in),
	texture_x (texture_x_in),
	texture_y (texture_y_in)
  {
  }


  // An array of surfaces for an object.  This class turns individual
  // polygons into strips if it can.
  class Surface_List : public std::vector <Ac3d_Surface*>
  {
	typedef std::vector <const Ac3d_Surface::Vertex*> Surface_Vertices;

	// Vertex indices used to see if a quadrilateral candidate for
	// joining would join at the same side as the others.
	size_t m_last_vertex1;
	size_t m_last_vertex2;

	bool join_surface (const Ac3d_Surface* surface);

	bool join_quadrilateral_to_edge (size_t index1, size_t index2, 
									 const Surface_Vertices& old_vertices, 
									 const Surface_Vertices& new_vertices);
	bool join_triangle_to_edge (size_t index1, size_t index2, 
								const Surface_Vertices& old_vertices, 
								const Surface_Vertices& new_vertices);
	bool join_quadrilateral (const Surface_Vertices& new_vertices,
							 size_t old_index1, size_t old_index2,
							 size_t new_index1, size_t new_index2);
	bool join_triangle (const Surface_Vertices&	new_vertices,
						size_t new_index1, size_t new_index2,
						Ac3d_Surface::Figure_Type new_figure_type);

  public:
	// Override vector's push_back.  We may join the surface instead
	// of adding it to the vector.
	void push_back (Ac3d_Surface* surface);
  };

  void
  Surface_List::push_back (Ac3d_Surface* surface)
  {
	if (!surface->is_shaded () || !join_surface (surface))
	  {
		std::vector <Ac3d_Surface*>::push_back (surface);
	  }
  }

  bool
  Surface_List::join_surface (const Ac3d_Surface* surface)
  {
	if (size () == 0) return false;
	if (surface->get_material () != back ()->get_material ()) return false;

	if ((surface->get_figure_type () != Ac3d_Surface::QUADRILATERAL)
		&& (surface->get_figure_type () != Ac3d_Surface::TRIANGLE))
	  {
		return false;
	  }

	const Surface_Vertices& new_vertices = surface->get_vertices ();
	const Surface_Vertices& old_vertices = back ()->get_vertices ();
	const size_t n_old_vertices = old_vertices.size ();

	const Ac3d_Surface::Figure_Type new_figure_type 
	  = surface->get_figure_type ();
	const Ac3d_Surface::Figure_Type old_figure_type 
	  = back ()->get_figure_type ();

	if (new_figure_type == Ac3d_Surface::QUADRILATERAL)
	  {
		if (old_figure_type == Ac3d_Surface::QUADRILATERAL)
		  {
			for (size_t i1 = 0; i1 < n_old_vertices; i1++)
			  {
				size_t i2 = (i1 + 1) % n_old_vertices;
				
				if (join_quadrilateral_to_edge (i1, i2, old_vertices, 
												new_vertices))
				  {
					return true;
				  }
			  }
		  }
		else if (old_figure_type == Ac3d_Surface::QUADRILATERAL_STRIP)
		  {
			return join_quadrilateral_to_edge (n_old_vertices - 1, 
											   n_old_vertices - 2, 
											   old_vertices, new_vertices);
		  }
	  }
	if (new_figure_type == Ac3d_Surface::TRIANGLE)
	  {
		if (old_figure_type == Ac3d_Surface::TRIANGLE)
		  {
			for (size_t i1 = 0; i1 < n_old_vertices; i1++)
			  {
				size_t i2 = (i1 + 1) % n_old_vertices;
				
				if (join_triangle_to_edge (i1, i2, old_vertices, new_vertices))
				  {
					return true;
				  }
			  }
		  }
		else if (old_figure_type == Ac3d_Surface::TRIANGLE_STRIP)
		  {
			return join_triangle_to_edge (n_old_vertices - 2, 
										  n_old_vertices - 1, 
										  old_vertices, new_vertices);
		  }
		else if (old_figure_type == Ac3d_Surface::TRIANGLE_FAN)
		  {
			return join_triangle_to_edge (0,
										  n_old_vertices - 1, 
										  old_vertices, new_vertices);
		  }
	  }

	return false;
  }

  bool
  Surface_List::
  join_quadrilateral_to_edge (size_t index1, size_t index2,
							  const Surface_Vertices& old_vertices,
							  const Surface_Vertices& new_vertices)
  {
	const size_t n_new_vertices = new_vertices.size ();

	const Ac3d_Surface::Vertex* v1 = old_vertices [index1];
	const Ac3d_Surface::Vertex* v2 = old_vertices [index2];

	for (size_t j1 = 0; j1 < n_new_vertices; j1++)
	  {
		size_t j2 = (j1 + 1) % n_new_vertices;
		
		// The common vertices are on opposite sides, so j1
		// connects with i2 and j2 with i1.
		if ((new_vertices [j1]->coordinates == v2->coordinates) 
			&& (new_vertices [j2]->coordinates == v1->coordinates))
		  {
			return join_quadrilateral (new_vertices, index1, index2, j1, j2);
		  }
	  }

	return false;
  }

  bool
  Surface_List::
  join_triangle_to_edge (size_t index1, size_t index2,
						 const Surface_Vertices& old_vertices,
						 const Surface_Vertices& new_vertices)
  {
	const size_t n_new_vertices = new_vertices.size ();

	const Ac3d_Surface::Vertex* v1 = old_vertices [index1];
	const Ac3d_Surface::Vertex* v2 = old_vertices [index2];

	const Ac3d_Surface::Figure_Type old_figure_type 
	  = back ()->get_figure_type ();

	for (size_t j1 = 0; j1 < n_new_vertices; j1++)
	  {
		size_t j2 = (j1 + 1) % n_new_vertices;
		bool even = (old_vertices.size () % 2 == 0);
		bool match = false;
		if ((even && (old_figure_type == Ac3d_Surface::TRIANGLE_STRIP))
			|| (old_figure_type == Ac3d_Surface::TRIANGLE_FAN))
		  {
			match = ((new_vertices [j1]->coordinates == v1->coordinates) 
					 && (new_vertices [j2]->coordinates == v2->coordinates));
		  }
		else
		  {
			// The common vertices are on opposite sides, so j1
			// connects with i2 and j2 with i1.
			match = ((new_vertices [j1]->coordinates == v2->coordinates) 
					 && (new_vertices [j2]->coordinates == v1->coordinates));
		  }
		if (match)
		  {
			Ac3d_Surface::Figure_Type new_figure_type = 
			  Ac3d_Surface::TRIANGLE_STRIP;
			if (((old_figure_type == Ac3d_Surface::TRIANGLE) && (index2 == 0))
				|| ((old_figure_type == Ac3d_Surface::TRIANGLE_FAN) 
					&& (index1 == 0)))
			  {
				new_figure_type = Ac3d_Surface::TRIANGLE_FAN;
			  }
			return join_triangle (new_vertices, j1, j2, new_figure_type);
		  }
	  }

	return false;
  }


  bool
  Surface_List::
  join_quadrilateral (const Surface_Vertices& new_vertices,
					  size_t old_index1, size_t old_index2,
					  size_t new_index1, size_t new_index2)
  {
	size_t n_new_vertices = new_vertices.size ();
	size_t new_index3 = (new_index1 + 2) % n_new_vertices;
	size_t new_index4 = (new_index1 + 3) % n_new_vertices;
	
	if (back ()->get_figure_type () == Ac3d_Surface::QUADRILATERAL)
	  {
		size_t old_index3 = (old_index1 + 2) % n_new_vertices;
		size_t old_index4 = (old_index1 + 3) % n_new_vertices;
		back ()->rearrange_vertices (old_index3, old_index4, 
									 old_index2, old_index1);
		back ()->set_figure_type (Ac3d_Surface::QUADRILATERAL_STRIP);
		
		m_last_vertex1 = new_index3;
		m_last_vertex2 = new_index4;
	  }
	else if ((m_last_vertex1 != new_index3) || (m_last_vertex2 != new_index4))
	  {
		return false;
	  }
	
	back ()->add_vertex (new_vertices [new_index4]);
	back ()->add_vertex (new_vertices [new_index3]);
	return true;
  }


  bool
  Surface_List::join_triangle (const Surface_Vertices& new_vertices,
							   size_t new_index1, size_t new_index2,
							   Ac3d_Surface::Figure_Type new_figure_type)
  {
	size_t n_new_vertices = new_vertices.size ();
	size_t new_index3 = (new_index1 + 2) % n_new_vertices;
	Ac3d_Surface::Figure_Type old_figure_type = back ()->get_figure_type ();

	if (old_figure_type == Ac3d_Surface::TRIANGLE)
	  {
		back ()->set_figure_type (new_figure_type);
		back ()->add_vertex (new_vertices [new_index3]);
		return true;
	  }
	else if (new_figure_type != old_figure_type)
	  {
		return false;
	  }

	back ()->add_vertex (new_vertices [new_index3]);
	return true;
  }


  // An object is a collection of surfaces.  Objects store the
  // vertices that the surfaces' vertices point to.
  class Ac3d_Object
  {
  public:
	struct Vertex
	{
	  Vertex (const Three_Vector* coords_in) : coordinates (coords_in) {};
	  ~Vertex () { delete coordinates; }
	  const Three_Vector* coordinates;
	  Three_Vector normal;
	};

  private:
	std::string m_type;
	std::string m_name;
	std::string m_data;
	std::string m_url;

	double m_scale;
	Three_Matrix m_rotation;
	Three_Vector m_location;

	Texture_Image* mp_texture;
	double m_texture_repeat [2];

	std::vector <const Ac3d_Object*> m_kids;
	std::vector <Vertex*> m_vertices;
	Surface_List m_surfaces;

  public:
	Ac3d_Object (std::string type, double scale, 
				 const Three_Vector& translation,
				 const Three_Vector& rotation);
	~Ac3d_Object ();

	void read_data (std::ifstream& is);

	void set_name (std::string name) { m_name = name; }
	void set_texture_image (std::string file);
	void set_texture_repeat (double x, double y);

	double get_scale () const { return m_scale; }
	void set_rotation (const Three_Matrix& rot);
	const Three_Matrix& get_rotation () const { return m_rotation; }
	void set_location (const Three_Vector& loc);
	const Three_Vector& get_location () const { return m_location; }
	
	void set_url (std::string url) { m_url = url; }
	void add_kid (const Ac3d_Object* obj) { m_kids.push_back (obj); }
	void add_vertex (double x, double y, double z);
	const Three_Vector* get_vertex (size_t index) const;
	void add_normal (size_t index, const Three_Vector& norm);
	const Three_Vector* get_normal (size_t index) const;
	void add_surface (Ac3d_Surface* surf);

	void build () const;
  };


  // Convert the AC3D version in the file's header to a n integer.
  int get_version_number (char ver)
  {
	int version = -1;
	if ((ver >= '0') && (ver <= '9'))
	  {
		version = ver - '0';
	  }
	else if ((ver >= 'a') && (ver <= 'f'))
	  {
		version = ver - 'a' + 10;
	  }
	else if ((ver >= 'A') && (ver <= 'F'))
	  {
		version = ver - 'A' + 10;
	  }
	else
	  {
		std::ostringstream message;
		message << "The version number " << ver 
				<< "is not a hexadecimal character.";
		throw Malformed_Ac3d_File (message.str ());
	  }
	assert (version != -1);
	return version;
  }

  // Read as element and return it as a string.  Quotes are stripped if
  // present. 
  std::string get_quoted (std::ifstream& is)
  {
	// Read the material's name.
	std::string word;
	is >> word;
	if (word [0] != '\"')
	  {
		return word;
	  }
	// Get multi-word names.
	while (word [word.size () - 1] != '\"')
	  {
		std::string more;
		is >> more;
		word = word + ' ' + more;
	  }
	// Strip the quotes.
	return word.substr (1, word.size () - 2);
  }

Ac3d_Object::Ac3d_Object (std::string type, double scale,
							const Three_Vector& translation,
							const Three_Vector& rotation) : 
	m_type (type),
	m_scale (scale),
	m_location (translation),
	mp_texture (0)
  {
	m_texture_repeat [0] = m_texture_repeat [1] = 1.0;
	m_rotation.identity ();

	// Conversion from Blender to AC3D puts the z-axis in the
	// y-direction.  Compensate by...  
	// ...adding 90 to the x-rotation...
	m_rotation.rotate (Three_Vector (rotation.x + pi/2.0, 0.0, 0.0));
	// ...using the z rotation for y...
	m_rotation.rotate (Three_Vector (0.0, rotation.z, 0.0));
	// ...using the -y rotation for z.
	m_rotation.rotate (Three_Vector (0.0, 0.0, -rotation.y));
  }


  Ac3d_Object::~Ac3d_Object ()
  {
	for (std::vector <const Ac3d_Object*>::iterator it = m_kids.begin ();
		 it != m_kids.end ();
		 it++)
	  {
		delete *it;
	  }
	for (std::vector <Vertex*>::iterator it = m_vertices.begin ();
		 it != m_vertices.end ();
		 it++)
	  {
		delete *it;
	  }
	for (std::vector <Ac3d_Surface*>::iterator it = m_surfaces.begin ();
		 it != m_surfaces.end ();
		 it++)
	  {
		delete *it;
	  }

	delete mp_texture;
  }


  void
  Ac3d_Object::read_data (std::ifstream& is)
  {
	size_t length;
	is >> length;
	char* buffer = new char [length + 1];

	// Throw out the new line after the data length.
	is.get (buffer [0]);

	size_t i = 0;
	while (i < length)
	  {
		is.get (buffer [i++]);
	  }
	buffer [length] = '\0';
	m_data = std::string (buffer);
	delete buffer;
  }

  void 
  Ac3d_Object::build () const
  {
	if (mp_texture != 0)
	  {
		glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		mp_texture->activate ();
	  }
	else
	  {
		glDisable (GL_TEXTURE_2D);
	  }

	std::for_each (m_surfaces.begin (), m_surfaces.end (), 
				   std::mem_fun (&Ac3d_Surface::build));

	glDisable (GL_TEXTURE_2D);
	std::for_each (m_kids.begin (), m_kids.end (), 
				   std::mem_fun (&Ac3d_Object::build));
	glEnable (GL_TEXTURE_2D);
  }


  void
  Ac3d_Object::set_texture_repeat (double x, double y)
  {
	m_texture_repeat [0] = x;
	m_texture_repeat [1] = y;
  }


  void
  Ac3d_Object::set_texture_image (std::string file)
  {
	mp_texture = new Texture_Image (file);
  }


  void
  Ac3d_Object::set_rotation (const Three_Matrix& rot)
  {
	m_rotation *= rot;
  }


  void
  Ac3d_Object::set_location (const Three_Vector& loc)
  {
	m_location += loc;
  }


  void
  Ac3d_Object::add_vertex (double x, double y, double z)
  {
	m_vertices.push_back (new Vertex (new Three_Vector (x, y, z)));
  }


  const Three_Vector*
  Ac3d_Object::get_vertex (size_t index) const
  {
	assert (index < m_vertices.size ());
	return m_vertices [index]->coordinates;
  }


  void
  Ac3d_Object::add_normal (size_t index, const Three_Vector& norm)
  {
	m_vertices [index]->normal += norm;
  }


  const Three_Vector*
  Ac3d_Object::get_normal (size_t index) const
  {
	assert (index < m_vertices.size ());
	return &(m_vertices [index]->normal);
  }


  void
  Ac3d_Object::add_surface (Ac3d_Surface* surf)
  { 
	m_surfaces.push_back (surf); 
  }


  Ac3d::Ac3d (std::string file,  double scale, 
			  const Three_Vector& translation,
			  const Three_Vector& rotation) :
	m_file (file),
	m_scale (scale),
	m_translation (translation),
	m_rotation (rotation)
  {
	std::ifstream is (m_file.c_str ());
	if (is == 0)
	  {
		throw No_File (m_file);
	  }

	read_header (is);

	std::string label;
	while (is >> label)
	  {
		if (label == "MATERIAL")
		  {
			m_materials.push_back (read_material (is));
		  }
		else if (label == "OBJECT")
		  {
			m_objects.
			  push_back (read_object (is, scale, translation, rotation));
		  }
		else if (label [0] == '#')
		  {
			continue;
		  }
		else
		  {
			throw Malformed_Ac3d_File ("Not part of an object definition");
		  }
	  }
	is.close ();
  }

  Ac3d::~Ac3d ()
  {
	for (std::vector <const Ac3d_Material*>::iterator it = m_materials.begin ();
		 it != m_materials.end ();
		 it++)
	  {
		delete *it;
	  }
	for (std::vector <const Ac3d_Object*>::iterator it = m_objects.begin ();
		 it != m_objects.end ();
		 it++)
	  {
		delete *it;
	  }
  }

  
  Three_Matrix
  read_matrix (std::ifstream& is)
  {
	Three_Matrix mat;
	for (size_t i = 0; i < 3; i++)
	  {
		for (size_t j = 0; j < 3; j++)
		  {
			is >> mat [i][j];
		  }
	  }
	return mat;
  }


  Three_Vector
  read_vector (std::ifstream& is)
  {
	Three_Vector vec;
    is >> vec.x;
    is >> vec.y;
    is >> vec.z;
	return vec;
  }
	

  const Ac3d_Object*
  Ac3d::read_object (std::ifstream& is, double scale, 
					 const Three_Vector& translation, 
					 const Three_Vector& rotation)
  {
	std::string type;
	is >> type;
	Ac3d_Object* obj = new Ac3d_Object (type, scale, translation, rotation);

	std::string label;
	while (is >> label)
	  {
		if (label == "name")
		  {
			obj->set_name (get_quoted (is));
		  }
		else if (label == "data")
		  {
			obj->read_data (is);
		  }
		else if (label == "texture")
		  {
			std::string dir = m_file.substr (0, m_file.find_last_of ("/") + 1);
			std::string file = get_quoted (is);
			obj->set_texture_image (dir + file);
		  }
		else if (label == "texrep")
		  {
			double x, y;
			is >> x >> y;
			obj->set_texture_repeat (x, y);
		  }
		else if (label == "rot")
		  {
			obj->set_rotation (read_matrix (is));
		  }
		else if (label == "loc")
		  {
			obj->set_location (read_vector (is));
		  }
		else if (label == "url")
		  {
			obj->set_url (get_quoted (is));
		  }
		else if (label == "numvert")
		  {
			size_t num;
			is >> num;
			for (size_t i = 0; i < num; i++)
			  {
				double x, y, z;
				// The label is the x-value.
				is >> x >> y >> z;
				obj->add_vertex (x, y, z);
			  }
		  }
		else if (label == "numsurf")
		  {
			size_t num;
			is >> num;
			for (size_t i = 0; i < num; i++)
			  {
				obj->add_surface (read_surface (is, obj));
			  }
		  }
		else if (label == "kids")
		  {
			size_t n_kids;
			is >> n_kids;
			for (size_t i = 0; i < n_kids; i++)
			  {
				is >> label;
				if (label != "OBJECT")
				  {
					std::string message = 
					  "An OBJECT line must follow a kids line."; 
					throw Malformed_Ac3d_File (message);
				  }
				obj->add_kid (read_object (is, scale, translation, rotation));
			  }
			break;
		  }
		else
		  {
            std::cerr << "Ac3d::read_object(): Unrecognized OBJECT data: "
                      << label << std::endl;
            continue;
		  }
	  }
	return obj;
  }


  Ac3d_Surface*
  Ac3d::read_surface (std::ifstream& is, Ac3d_Object* obj)
  {
	std::string label;
	is >> label;
	if (label != "SURF")
	  {
		throw Malformed_Ac3d_File ("Expected A SURF section.");
	  }
	std::string surf_type;
	is >> surf_type;
	Ac3d_Surface* surf = new Ac3d_Surface (surf_type, 
										   obj->get_scale (),
										   obj->get_location (),
										   obj->get_rotation ());

	int mat = -1;
	is >> label;
			
	// Read the surface.
	if (label == "mat")
	  {
		is >> mat;
		surf->set_material (m_materials [mat]);
				
		is >> label;
	  }
	if (label == "refs")
	  {
		size_t num;
		is >> num;

		if (num == 3)
		  {
			surf->set_figure_type (Ac3d_Surface::TRIANGLE);
		  }
		else if (num == 4)
		  {
			surf->set_figure_type (Ac3d_Surface::QUADRILATERAL);
		  }

		std::vector <const Ac3d_Surface::Vertex*> verts;
		verts.resize (num);
		std::vector <size_t> refs;
		refs.resize (num);

		for (size_t i = 0; i < num; i++)
		  {
			double texture_x, texture_y;
			is >> refs [i] >> texture_x >> texture_y;
			verts [i] = new Ac3d_Surface::Vertex (obj->get_vertex (refs [i]),
												  obj->get_normal (refs [i]),
												  texture_x, texture_y);
		  }
		surf->set_vertices (verts);

		Three_Vector normal;
		if (num >= 3)
		  {
			const Three_Vector& p0 = *(verts [0]->coordinates);
			const Three_Vector& p1 = *(verts [1]->coordinates);
			const Three_Vector& p2 = *(verts [num - 1]->coordinates);
		  
			Three_Vector r1 = p1 - p0;
			Three_Vector r2 = p2 - p0;
			for (size_t i = 0; i < num; i++)
			  {
				normal = (r1.cross (r2)).unit ();
				obj->add_normal (refs [i], normal);
			  }
		  }
		surf->set_normal (normal);
	  }
	else
	  {
		throw Malformed_Ac3d_File ("Expected a mat or refs section.");
	  }

	if (mat == -1)
	  {
		throw Malformed_Ac3d_File ("Expected a mat section.");
	  }
	return surf;
  }


  void
  read_material_parameters (std::ifstream& is,
							std::string label, 
							GLfloat* values,
							size_t n_values)
  {
	std::string actual_label;
	is >> actual_label;
	if (actual_label != label)
	  {
		throw Malformed_Ac3d_File ("Expected \"" + label +"\".");
	  }
	for (size_t i = 0; i < n_values; i++)
	  {
		is >> values [i];
	  }
  }


  const Ac3d_Material*
  Ac3d::read_material (std::ifstream& is)
  { 
	// Read the material's name.
	std::string name = get_quoted (is);

	GLfloat color [3];
	read_material_parameters (is, "rgb", color, 3);
	
	GLfloat ambient [3];
	read_material_parameters (is, "amb", ambient, 3);
	
	GLfloat emission [3];
	read_material_parameters (is, "emis", emission, 3);

	GLfloat specular [3];
	read_material_parameters (is, "spec", specular, 3);

	GLfloat shininess;
	read_material_parameters (is, "shi", &shininess, 1);

	GLfloat transparency;
	read_material_parameters (is, "trans", &transparency, 1);

	return new Ac3d_Material (name, color, ambient, emission, 
							  specular, shininess, transparency);
  }


  void 
  Ac3d::read_header (std::ifstream& is)
  {
	std::string header;
	is >> header;

	if ((header.size () < 5) || (header.substr (0, 4) != "AC3D"))
	  {
		throw Not_An_Ac3d_File (m_file + " does not have an AC3D header");
	  }

	m_version = get_version_number (header [4]);
  }


  GLuint
  Ac3d::build ()
  {
	GLuint id = glGenLists (1);
	glNewList (id, GL_COMPILE);

	std::for_each (m_objects.begin (), m_objects.end (), 
				   std::mem_fun (&Ac3d_Object::build));

	glEndList ();

	return id;
  }
}
