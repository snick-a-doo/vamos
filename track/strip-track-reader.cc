//  Strip_Track_Reader.cc - reader for track definition files.
//
//  Copyright (C) 2005 Sam Varner
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

#include "strip-track.h"
#include "../geometry/two-vector.h"
#include "../geometry/conversions.h"
#include "../geometry/numeric.h"

#include <sstream>

using namespace Vamos_Track;
using namespace Vamos_Geometry;

//* Class Track_Reader

//** Constructor

Strip_Track_Reader::Strip_Track_Reader (std::string data_dir, 
										std::string track_file, 
										Strip_Track* road) 
  : m_first_road (true),
	m_data_dir (data_dir),
	mp_road (road),
    m_close (false),
    m_adjusted_road_segments (0),
    m_join_pit_lane (false),
    m_length (0.0),
    m_adjusted_pit_segments (0)
{
  read (track_file);
}

void 
Strip_Track_Reader::on_start_tag (const Vamos_Media::XML_Tag& tag)
{
  const Vamos_Media::XML_Tag::Attribute_List& 
	attribs = tag.get_attributes ();

  if (label () == "road")
	{
	  if (m_first_road)
		{
		  // Clear the vector so that it gets filled with 0.0 when
		  // it's resized.
		  m_doubles.clear ();
		  m_doubles.resize (23);
		  m_bools.clear ();
		  m_bools.resize (8);
		  m_points.clear ();
		  m_points.resize (2);
		  m_left_profile.clear ();
		  m_right_profile.clear ();
          m_pit_in_active = false;
          m_pit_out_active = false;

		  m_first_road = false;
		}

	  m_doubles [3] = 0.0; // left kerb start
	  m_doubles [6] = 0.0; // left kerb end
	  m_doubles [9] = 0.0; // right kerb start
	  m_doubles [12] = 0.0; // right kerb end
      m_doubles [20] = 0.0; // skew
      m_doubles [21] = 1.0; // racing line curvature factor
	  m_strings.clear ();
	  m_strings.resize (2);
	  m_strings [0] = attribs [0].value;
	  m_bools [0] = false;
	  m_bools [1] = false; // left kerb tag
	  m_bools [2] = false; // right kerb tag
	  m_bools [3] = false; // left kerb start tag
	  m_bools [4] = false; // left kerb end tag
	  m_bools [5] = false; // right kerb start tag
	  m_bools [6] = false; // right kerb end tag
	  m_elev_points.clear ();

	  for (size_t i = 0; i < 4; i++)
		{
		  if (m_point_vectors [i].size () > 0)
			{
			  Two_Vector last_point = *(m_point_vectors [i].end () - 1);
			  last_point.x = 0.0;
			  m_point_vectors [i].clear ();
			  m_point_vectors [i].push_back (last_point);
			}
		}

      m_line_adjust.x = 0.0;
	  m_braking_markers.clear ();
	  m_model_info.clear ();
	}
  else if (label () == "left-kerb")
	{
	  m_bools [1] = true;
	}
  else if (label () == "right-kerb")
	{
      m_bools [2] = true;
	}
  else if (match ("*/road/left-kerb/start"))
	{
	  m_bools [3] = true;
	}
  else if (match ("*/road/left-kerb/end"))
	{
	  m_bools [4] = true;
	}
  else if (match ("*/road/right-kerb/start"))
	{
	  m_bools [5] = true;
	}
  else if (match ("*/road/right-kerb/end"))
	{
	  m_bools [6] = true;
	}
  else if (label () == "circuit")
	{
      m_close = true;
      if (attribs.size () == 1)
        {
          std::istringstream is (attribs [0].value.c_str ());
          is >> m_adjusted_road_segments;
        }
      else
        {
          m_adjusted_road_segments = 3;
        }
    }
  else if ((path () == "/track/racing-line") && (attribs.size () == 1))
    {
      std::istringstream is (attribs [0].value.c_str ());
      is >> mp_road->mp_track->m_draw_racing_line;
    }
  else if (path () == "/track/pit/join")
	{
	  m_join_pit_lane = true;
      if (attribs.size () == 1)
        {
          std::istringstream is (attribs [0].value.c_str ());
          is >> m_adjusted_pit_segments;
        }
      else
        {
          m_adjusted_pit_segments = 3;
        }
	}
  else if (label () == "timing-line")
	{
	  m_doubles.clear ();
	}
  else if (label () == "track-length")
	{
	  m_doubles.clear ();
	}

  else if (label () == "sky")
	{
	  m_strings.clear ();
	  m_strings.resize (3);
	  m_bools.clear ();
	  m_bools.resize (1);
	}
  else if (label () == "map-background")
    {
	  m_strings.clear ();
	  m_strings.resize (1);
	  m_doubles.clear ();
	  m_doubles.resize (4);
    }
  else if (label () == "material")
	{
	  m_name = attribs [0].value;
	  std::string type = attribs [1].value;
	  if (type == "rubber")
		m_material_type = Material::RUBBER;
	  else if (type == "metal")
		m_material_type = Material::METAL;
	  else if (type == "asphalt")
		m_material_type = Material::ASPHALT;
	  else if (type == "concrete")
		m_material_type = Material::CONCRETE;
	  else if (type == "kerb")
		m_material_type = Material::KERB;
	  else if (type == "grass")
		m_material_type = Material::GRASS;
	  else if (type == "gravel")
		m_material_type = Material::GRAVEL;
	  else if (type == "dirt")
		m_material_type = Material::DIRT;
	  else
		{
		  std::cerr << "Strip_Track_Reader: Warning: Unknown material \"" 
					<< attribs [1].value << '\"' << std::endl;
		  m_material_type = Material::UNKNOWN;
		}
			
	  m_bools.clear ();
	  m_bools.resize (2);
	  m_doubles.clear ();
	  m_doubles.resize (8);
	  m_strings.clear ();
	}
  else if (label () == "segment")
	{
	  m_name = attribs [0].value;
	  m_strings.resize (7);
	}
  else if (label () == "pit")
    {
	  for (size_t i = 0; i < 4; i++)
		{
          m_point_vectors [i].clear ();
        }
    }
}

void 
Strip_Track_Reader::on_end_tag (const Vamos_Media::XML_Tag& tag)
{
  if (label () == "road")
	{
      // Ignore zero-length segments.
      if (m_doubles [1] == 0.0) return;

	  if (!m_bools [0])
		{
		  m_doubles [2] = 0.0;
		}

      const double length = m_doubles [1];
      bool left_full_length = (m_bools [1] && (m_doubles [6] == 0.0));
      double end_left = left_full_length ? length : wrap (m_doubles [6], length);
      bool right_full_length = (m_bools [2] && (m_doubles [12] == 0.0));
      double end_right = right_full_length ? length : wrap (m_doubles [12], length);

	  Gl_Road_Segment* segment 
        = new Gl_Road_Segment (m_doubles [0], 
                               length,
                               m_doubles [2],
                               m_doubles [20],
                               m_point_vectors [0], 
                               m_point_vectors [1],
                               m_point_vectors [2], 
                               m_point_vectors [3],
                               new Kerb (m_left_profile, 
                                         wrap (m_doubles [3], length),
                                         m_bools [3] ? m_doubles [4] : 0.0,
                                         m_doubles [5],
                                         end_left,
                                         m_bools [4] ? m_doubles [7] : 0.0,
                                         m_doubles [8],
                                         left_full_length),
                               new Kerb (m_right_profile,
                                         wrap (m_doubles [9], length),
                                         m_bools [5] ? m_doubles [10] : 0.0,
                                         m_doubles [11],
                                         end_right,
                                         m_bools [6] ? m_doubles [13] : 0.0,
                                         m_doubles [14],
                                         right_full_length),
                               m_doubles [15], 
                               m_doubles [16],
                               m_elev_points, 
                               m_doubles [17], 
                               m_doubles [18],
                               m_segments [m_strings [0]],
                               m_braking_markers);
      segment->set_racing_line_adjustment (m_line_adjust.x, m_line_adjust.y);
      segment->set_racing_line_curvature_factor (m_doubles [21]);
      if (path () == "/track/pit/road")
        {
          mp_road->add_pit_segment (segment);
        }
      else
        {
          size_t size = mp_road->add_segment (segment);
          if (m_pit_in_active)
            {
              mp_road->set_pit_in (size - 1, m_angle);
              segment->set_pit_lane (IN, m_pit_side, m_split_or_join, m_merge, m_angle);
            }
          if (m_pit_out_active)
            {
              mp_road->set_pit_out (size - 1, m_angle);
              segment->set_pit_lane (OUT, m_pit_side, m_split_or_join, m_merge, m_angle);
            }
          m_pit_in_active = false;
          m_pit_out_active = false;
        }
      for (std::vector <Gl_Road_Segment::Model_Info>::iterator
             it = m_model_info.begin ();
           it !=  m_model_info.end ();
           it++)
        {
          segment->add_model_info (*it);
        }
      m_camera.segment_index++;
      m_camera.fixed = false;
	}
  else if (label () == "timing-line")
	{
	  mp_road->timing_line (m_doubles [0]);
	}
  else if (label () == "track")
    {
      mp_road->build (m_close, m_adjusted_road_segments, 
                      m_length, 
                      m_join_pit_lane, m_adjusted_pit_segments);
    }
  else if (label () == "camera")
    {
      mp_road->add_camera (m_camera);
    }
  else if (label () == "sky")
	{
	  mp_road->set_sky_box (m_data_dir + m_strings [0], 
                            m_data_dir + m_strings [1],
                            m_data_dir + m_strings [2],
                            m_bools [0]);
	}
  else if (path () == "/track/map-background")
    {
      mp_road->set_map_background (m_strings [0],
                                   m_doubles [0], m_doubles [1],
                                   m_doubles [2], m_doubles [3]);
    }
  else if (label () == "material")
	{
	  m_materials [m_name] = Material (m_material_type,
                                       m_doubles [0], m_doubles [1],
                                       m_doubles [2], m_doubles [3],
                                       m_bump_amplitude,
                                       m_doubles [5], 
                                       m_data_dir + m_strings [0],
                                       m_bools [0],
                                       m_bools [1],
                                       m_doubles [6],
                                       m_doubles [7]);
	}
  else if (label () == "smooth")
	{
	  m_bools [0] = true;
	}
  else if (label () == "mipmap")
	{
	  m_bools [1] = true;
	}
  else if (label () == "segment")
	{
	  std::vector <Material> mat;
	  mat.resize (7);
	  mat [0] = m_materials [m_strings [0]];
	  mat [1] = m_materials [m_strings [1]];
	  mat [2] = m_materials [m_strings [2]];
	  mat [3] = m_materials [m_strings [3]];
	  mat [4] = m_materials [m_strings [4]];
	  mat [5] = m_materials [m_strings [5]];
	  mat [6] = m_materials [m_strings [6]];
	  m_segments [m_name] = mat;
	}
  else if (label () == "braking-marker")
	{
      Direction side = m_bools [7] ? RIGHT : LEFT;
      m_braking_markers.
        push_back (new Braking_Marker (m_data_dir + m_strings [1],
                                       m_doubles [19],
 									   side,
                                       m_points [1].x,
                                       m_points [1].y,
                                       m_points [0].x,
                                       m_points [0].y));
	}
  else if (label () == "model")
    {
      m_model_info.push_back (m_current_model_info);
 	}
  else if (path () == "/track/road/pit-in")
    m_pit_in_active = true;
  else if (path () == "/track/road/pit-out")
    m_pit_out_active = true;
}

void 
Strip_Track_Reader::on_data (std::string data)
{
  if (data.size () == 0)
	{
	  return;
	}
  std::istringstream is (data.c_str ());

  char delim;

  if (match ("*/road/resolution"))
	{
	  is >> m_doubles [0];
	}
  else if (match ("*/road/length"))
	{
	  is >> m_doubles [1];
	}
  else if (label () == "radius")
	{
	  m_bools [0] = true;
	  is >> m_doubles [2];
	}
  else if (match ("*/road/skew"))
	{
	  is >> m_doubles [20];
	}
  else if (match ("*/road/left-width"))
	{
	  Two_Vector point;
	  is >> point;
	  m_point_vectors [0].push_back (point);
	}
  else if (match ("*/road/right-width"))
	{
	  Two_Vector point;
	  is >> point;
	  m_point_vectors [1].push_back (point);
	}
  else if (match ("*/road/left-road-width"))
	{
	  Two_Vector point;
	  is >> point;
	  m_point_vectors [2].push_back (point);
	}
  else if (match ("*/road/right-road-width"))
	{
	  Two_Vector point;
	  is >> point;
	  m_point_vectors [3].push_back (point);
	}
  else if (match ("*/road/racing-line-adjustment"))
	{
      if (data [0] == '[')
        is >> m_line_adjust;
      else
        {
          is >> m_line_adjust.x;
          m_line_adjust.y = -1.0;
        }
	}
  else if (match ("*/road/curvature-factor"))
	{
	  is >> m_doubles [21];
	}
  // Left Kerb
  else if (match ("*/road/left-kerb/start/distance"))
	{
	  is >> m_doubles [3];
	}
  else if (match ("*/road/left-kerb/start/transition/length"))
	{
	  is >> m_doubles [4];
	}
  else if (match ("*/road/left-kerb/start/transition/width"))
	{
	  is >> m_doubles [5];
	}
  else if (match ("*/road/left-kerb/end/distance"))
	{
	  is >> m_doubles [6];
	}
  else if (match ("*/road/left-kerb/end/transition/length"))
	{
	  is >> m_doubles [7];
	}
  else if (match ("*/road/left-kerb/end/transition/width"))
	{
	  is >> m_doubles [8];
	}
  else if (match ("*/road/left-kerb/profile"))
	{
	  Two_Vector point;
	  m_left_profile.clear ();
	  m_left_profile.push_back (point);
	  while (is >> point)
		{
		  m_left_profile.push_back (point);
		}
	}
  // Right Kerb
  else if (match ("*/road/right-kerb/start/distance"))
	{
	  is >> m_doubles [9];
	}
  else if (match ("*/road/right-kerb/start/transition/length"))
	{
	  is >> m_doubles [10];
	}
  else if (match ("*/road/right-kerb/start/transition/width"))
	{
	  is >> m_doubles [11];
	}
  else if (match ("*/road/right-kerb/end/distance"))
	{
	  is >> m_doubles [12];
	}
  else if (match ("*/road/right-kerb/end/transition/length"))
	{
	  is >> m_doubles [13];
	}
  else if (match ("*/road/right-kerb/end/transition/width"))
	{
	  is >> m_doubles [14];
	}
  else if (match ("*/road/right-kerb/profile"))
	{
	  Two_Vector point;
	  m_right_profile.clear ();
	  m_right_profile.push_back (point);
	  while (is >> point)
		{
		  m_right_profile.push_back (point);
		}
	}

  else if (match ("*/road/left-wall-height"))
	{
	  is >> m_doubles [15];
	}
  else if (match ("*/road/right-wall-height"))
	{
	  is >> m_doubles [16];
	}
  else if (match ("*/road/elevation"))
	{
	  char delim;
	  double dist;
	  double elev;
	  is >> delim >> dist >> delim >> elev;
	  m_elev_points.push_back (Two_Vector (dist, elev));
	}
  else if (match ("*/road/bank"))
	{
	  is >> m_doubles [17];
	}
  else if (match ("*/road/bank-pivot"))
	{
	  is >> m_doubles [18];
	}
  else if (match ("*/road/braking-marker/file"))
	{
	  m_strings [1] = data;
	}
  else if (match ("*/road/braking-marker/distance"))
	{
	  is >> m_doubles [19];
	}
  else if (match ("*/road/braking-marker/size"))
	{
	  is >> m_points [0];
	}
  else if (match ("*/road/braking-marker/offset"))
	{
	  is >> m_points [1];
	}
  else if (match ("*/road/braking-marker/side"))
	{
	  m_bools [7] = (data == "right");
	}
  else if (match ("*/road/model/file"))
	{
	  std::string file;
	  is >> file;
      m_current_model_info.file = m_data_dir + "tracks/" + file;
	}
  else if (match ("*/road/model/scale"))
	{
      is >> m_current_model_info.scale;
	}
  else if (match ("*/road/model/translate"))
	{
      is >> m_current_model_info.translation;
	}
  else if (match ("*/road/model/rotate"))
	{
      is >> m_current_model_info.rotation;
	}
  else if (path () == "/track/track-length")
    {
      is >> m_length;
    }
  else if (match ("*/road/camera/position"))
    {
      is >> m_camera.position;
    }
  else if (match ("*/road/camera/direction"))
    {
      m_camera.fixed = true;
      is >> m_camera.direction;
    }
  else if (match ("*/road/camera/field"))
    {
      is >> m_camera.vertical_field_angle;
    }
  else if (match ("*/road/camera/range"))
    {
      is >> m_camera.range;
    }
  else if (path () == "/track/sky/sides")
	{
	  m_strings [0] = data;
	}
  else if (path () == "/track/sky/top")
	{
	  m_strings [1] = data;
	}
  else if (path () == "/track/sky/bottom")
	{
	  m_strings [2] = data;
	}

  else if (path () == "/track/map-background/image")
    {
      m_strings [0] = data;
    }
  else if (path () == "/track/map-background/offset")
	{
	  char delim;
	  is >> delim >> m_doubles [0] >> delim >> m_doubles [1];
    }
  else if (path () == "/track/map-background/size")
	{
	  char delim;
	  is >> delim >> m_doubles [2] >> delim >> m_doubles [3];
    }

  else if (path () == "/track/material/friction")
	{
	  is >> m_doubles [0];
	}
  else if (path () == "/track/material/restitution")
	{
	  is >> m_doubles [1];
	}
  else if (path () == "/track/material/rolling")
	{
	  is >> m_doubles [2];
	}
  else if (path () == "/track/material/drag")
	{
	  is >> m_doubles [3];
	}
  else if (path () == "/track/material/bump-amplitude")
    {
      if (data [0] == '[')
        is >> m_bump_amplitude;
      else
        {
          double amp;
          is >> amp;
          m_bump_amplitude = Two_Vector (amp, 0.0);
        }
	}
  else if (path () == "/track/material/bump-wavelength")
	{
	  is >> m_doubles [5];
	}
  else if (path () == "/track/material/texture/width")
	{
	  is >> m_doubles [6];
	}
  else if (path () == "/track/material/texture/length")
	{
	  is >> m_doubles [7];
	}
  else if (path () == "/track/material/texture/file")
	{
	  m_strings.push_back (data);
	}
  else if (path () == "/track/segment")
	{
	  is >> delim >> m_strings [0] 
		 >> m_strings [1] 
		 >> m_strings [2] 
		 >> m_strings [3] 
		 >> m_strings [4]
		 >> m_strings [5]
		 >> m_strings [6];
	}
  else if (path () == "/track/start-direction")
    {
      double degrees;
      is >> degrees;
      mp_road->set_start_direction (degrees);
    }
  else if (path () == "/track/racing-line/iterations")
    is >> mp_road->mp_track->m_racing_line.m_iterations;
  else if (path () == "/track/racing-line/stiffness")
    is >> mp_road->mp_track->m_racing_line.m_stiffness;
  else if (path () == "/track/racing-line/damping")
    is >> mp_road->mp_track->m_racing_line.m_damping;
  else if (path () == "/track/racing-line/margin")
    is >> mp_road->mp_track->m_racing_line.m_margin;
  else if (path () == "/track/racing-line/resolution")
    is >> mp_road->mp_track->m_racing_line.m_resolution;
  else if (path () == "/track/road/pit-in/split")
    is >> m_split_or_join;
  else if (path () == "/track/road/pit-in/merge")
    is >> m_merge;
  else if (path () == "/track/road/pit-in/angle")
    is >> m_angle;
  else if (path () == "/track/road/pit-out/join")
    is >> m_split_or_join;
  else if (path () == "/track/road/pit-out/merge")
    is >> m_merge;
  else if (path () == "/track/road/pit-out/angle")
    is >> m_angle;
  else if ((path () == "/track/road/pit-in/side")
           || (path () == "/track/road/pit-out/side"))
    {
      std::string side;
      is >> side;
      if (side == "left")
        m_pit_side = LEFT;
      else
        m_pit_side = RIGHT;
    }
  else
	{
	  double arg;
	  is >> arg;
	  m_doubles.push_back (arg);
	}
}
