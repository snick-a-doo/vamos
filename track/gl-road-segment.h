//  Copyright (C) 2001--2007 Sam Varner
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

#ifndef _GL_ROAD_SEGMENT_H_
#define _GL_ROAD_SEGMENT_H_

#include "road-segment.h"
#include "../geometry/material.h"

#include <GL/glu.h>

#include <string>
#include <vector>

namespace Vamos_Media
{
  class Texture_Image;
  class Facade;
}

namespace Vamos_Track
{
  class Braking_Marker
  {
    Vamos_Media::Facade* mp_image;
    double m_distance;
    Side m_side;
    double m_from_edge;
    double m_off_ground;

  public:
    Braking_Marker (std::string image_file,
                    double distance,
                    Side side,
                    double from_edge,
                    double off_ground,
                    double width,
                    double height);
    ~Braking_Marker ();

    double distance () const { return m_distance; }
    Side side() const { return m_side; }
    double from_edge () const { return m_from_edge; }
    double off_ground () const { return m_off_ground; }
    double width () const;
    double height () const;
    void draw () const;
  }; 

  class Segment_Iterator;

  class Gl_Road_Segment : public Road_Segment
  {
    friend class Segment_Iterator;

  public:
	struct Model_Info
	{
	  std::string file;
	  double scale;
	  Vamos_Geometry::Three_Vector translation;
	  Vamos_Geometry::Three_Vector rotation;
	};

	Gl_Road_Segment (double resolution, double length, 
                     double radius, 
                     double skew, 
                     const std::vector <Vamos_Geometry::Two_Vector>& left_width, 
                     const std::vector <Vamos_Geometry::Two_Vector>& right_width, 
                     const std::vector <Vamos_Geometry::Two_Vector>& left_road_width, 
                     const std::vector <Vamos_Geometry::Two_Vector>& right_road_width, 
                     Kerb* left_kerb, 
                     Kerb* right_kerb,
                     double left_wall_height, 
                     double right_wall_height,
                     const std::vector <Vamos_Geometry::Two_Vector>& elevation_points,
                     double end_bank, 
                     double bank_pivot,
                     const std::vector <Vamos_Geometry::Material>& materials,
                     const std::vector <Braking_Marker*>& braking_markers);

    ~Gl_Road_Segment ();

	const std::vector <Vamos_Geometry::Material>& materials () const 
	{ return m_materials; }

    void set_materials (const std::vector <Vamos_Geometry::Material>& 
                        materials,
                        double resolution);


	void set_start (const Vamos_Geometry::Three_Vector& start_corrds, 
                    double start_distance,
                    double start_angle, 
                    double start_bank,
                    const std::vector <double>& texture_offsets);

    void set_braking_marker(Braking_Marker* marker);

	void build ();
	void draw () const;

	void add_model_info (const Model_Info& info);

	std::vector <double> texture_offsets () const { return m_texture_offsets; }

    virtual Vamos_Geometry::Rectangle bounds () const { return m_bounds; }

	const Vamos_Geometry::Material& left_material (double height) const;
	const Vamos_Geometry::Material& right_material (double height) const; 
    Vamos_Geometry::Material const& material_at(double along, double from_center) const;

  private:
	std::vector <Model_Info> m_models;
	
	enum Strip
	  { 
		LEFT_BARRIER, 
		LEFT_SHOULDER, 
		LEFT_KERB,
		TRACK, 
		RIGHT_KERB,
		RIGHT_SHOULDER,
		RIGHT_BARRIER,
        N_STRIPS
	  };

	GLuint* m_gl_texture_name;
	GLuint m_gl_list_id;

	std::vector <GLuint> m_scenery_lists;

	std::vector <double> m_texture_offsets;

	Segment_Iterator* mp_iterator;

	std::vector <Braking_Marker*> m_braking_markers;

	std::vector <Vamos_Geometry::Material> m_materials;

    std::vector <Vamos_Media::Texture_Image*> m_textures;

	// bounding dimensions
    Vamos_Geometry::Rectangle m_bounds;

    void add_textures ();
	void draw_strip (Strip strip, double texture_offset);
  };
}

#endif // not _GL_ROAD_SEGMENT_H_
