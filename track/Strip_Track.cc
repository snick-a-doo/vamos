//  Strip_Track.cc - a track.
//
//  Copyright (C) 2001--2004 Sam Varner
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

#include "Strip_Track.h"
#include "../geometry/Numeric.h"
#include "../geometry/Spline.h"
#include "../media/Texture_Image.h"
#include "../geometry/Parameter.h"

#include <GL/glu.h>

#include <cmath>
#include <cassert>

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;

Camera s_default_camera (0, Three_Vector (100.0, -20.0, 10.0), 0.0);

namespace Vamos_Track
{
  class Sky_Box
  {
    Texture_Image* mp_sides;
    Texture_Image* mp_top;
    Texture_Image* mp_bottom;
	GLuint m_list_id;

  public:
    Sky_Box (double side_length,
             std::string sides_image,
             std::string top_image,
             std::string bottom_image,
             bool smooth);
    ~Sky_Box ();

    void draw (const Three_Vector& view) const;
  };
}

Sky_Box::Sky_Box (double side_length,
                  std::string sides_image,
                  std::string top_image,
                  std::string bottom_image,
                  bool smooth)
  // Clamp the textures to aviod showing seams.
  : mp_sides (new Texture_Image (sides_image, smooth, true, 
                                 GL_CLAMP_TO_EDGE)),
    mp_top (new Texture_Image (top_image, smooth, true, 
                               GL_CLAMP_TO_EDGE)),
    mp_bottom (new Texture_Image (bottom_image, smooth, true, 
                                  GL_CLAMP_TO_EDGE)),
    m_list_id (glGenLists (1))
{
  double height = side_length;
  double length = side_length;
  double width = side_length;

  double x = -length / 2.0;
  double y = -width / 2.0;
  double z = -height / 2.0;

  glNewList (m_list_id, GL_COMPILE);
  glColor3f (1.0, 1.0, 1.0);
  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  mp_sides->activate ();

  // front
  glBegin (GL_QUAD_STRIP);
  glTexCoord2d (0.0, 0.0); 
  glVertex3d (x + length, y + width, z + height);
  glTexCoord2d (0.0, 1.0); 
  glVertex3d (x + length, y + width, z);
  glTexCoord2d (0.25, 0.0); 
  glVertex3d (x + length, y, z + height); 
  glTexCoord2d (0.25, 1.0); 
  glVertex3d (x + length, y, z);

  // right
  glTexCoord2d (0.25, 0.0); 
  glVertex3d (x + length, y, z + height);
  glTexCoord2d (0.25, 1.0); 
  glVertex3d (x + length, y, z);
  glTexCoord2d (0.5, 0.0); 
  glVertex3d (x, y, z + height); 
  glTexCoord2d (0.5, 1.0); 
  glVertex3d (x, y, z);

  // back
  glTexCoord2d (0.5, 0.0); 
  glVertex3d (x, y, z + height);
  glTexCoord2d (0.5, 1.0); 
  glVertex3d (x, y, z);
  glTexCoord2d (0.75, 0.0); 
  glVertex3d (x, y + width, z + height); 
  glTexCoord2d (0.75, 1.0); 
  glVertex3d (x, y + width, z);

  // left
  glTexCoord2d (0.75, 0.0); 
  glVertex3d (x, y + width, z + height);
  glTexCoord2d (0.75, 1.0); 
  glVertex3d (x, y + width, z);
  glTexCoord2d (1.0, 0.0); 
  glVertex3d (x + length, y + width, z + height); 
  glTexCoord2d (1.0, 1.0); 
  glVertex3d (x + length, y + width, z);
  glEnd();

  // top
  mp_top->activate ();

  glBegin (GL_QUADS);		
  glTexCoord2d (0.0, 0.0); 
  glVertex3d (x, y + width, z + height);
  glTexCoord2d (0.0, 1.0); 
  glVertex3d (x + length, y + width, z + height); 
  glTexCoord2d (1.0, 1.0); 
  glVertex3d (x + length, y, z + height);
  glTexCoord2d (1.0, 0.0); 
  glVertex3d (x, y, z + height);
  glEnd ();

  // bottom
  mp_bottom->activate ();

  glBegin (GL_QUADS);		
  glTexCoord2d (0.0, 0.0); 
  glVertex3d (x + length, y + width, z);
  glTexCoord2d (0.0, 1.0); 
  glVertex3d (x, y + width, z);
  glTexCoord2d (1.0, 1.0); 
  glVertex3d (x, y, z);
  glTexCoord2d (1.0, 0.0); 
  glVertex3d (x + length, y, z); 
  glEnd ();

  glFlush ();
  glEndList ();
}

Sky_Box::~Sky_Box ()
{
  delete mp_bottom;
  delete mp_top;
  delete mp_sides;
  glDeleteLists (m_list_id, 1);
}

void
Sky_Box::draw (const Three_Vector& view) const
{
  glLoadIdentity ();
  glTranslatef (view.x, view.y, view.z);
  glCallList (m_list_id);
  // Clear the depth buffer to keep the sky behind everything else.
  glClear (GL_DEPTH_BUFFER_BIT);
}

//* Class Map_Background

namespace Vamos_Track
{
  class Map_Background
  {
    Texture_Image* mp_image;
    double m_x_offset;
    double m_y_offset;
    double m_x_size;
    double m_y_size;

  public:
    Map_Background (std::string image_file_name,
                    double x_offset, double y_offset,
                    double x_size, double y_size);
    ~Map_Background ();

    void draw () const;
  };
}

Map_Background::Map_Background (std::string image_file_name,
                                double x_offset, double y_offset,
                                double x_size, double y_size)
  : mp_image (new Texture_Image (image_file_name, true)),
    m_x_offset (x_offset),
    m_y_offset (y_offset),
    m_x_size (x_size),
    m_y_size (y_size)
{
}

Map_Background::~Map_Background ()
{
  delete mp_image;
}

void
Map_Background::draw () const
{
  glColor3f (1.0, 1.0, 1.0);
  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  mp_image->activate ();

  glLoadIdentity ();
  glTranslatef (m_x_offset, m_y_offset, 0.0);
  
  glBegin (GL_QUADS);
  glTexCoord2d (0.0, 1.0);
  glVertex3d (m_x_offset, m_y_offset, 0.0);
  glTexCoord2d (0.0, 0.0); 
  glVertex3d (m_x_offset, m_y_offset + m_y_size, 0.0);
  glTexCoord2d (1.0, 0.0); 
  glVertex3d (m_x_offset + m_x_size, m_y_offset + m_y_size, 0.0); 
  glTexCoord2d (1.0, 1.0); 
  glVertex3d (m_x_offset + m_x_size, m_y_offset, 0.0);
  glEnd ();

  // Clear the depth buffer to keep the background behind the track.
  glClear (GL_DEPTH_BUFFER_BIT);
}

//* Struct Camera
Camera::Camera (size_t segment_index_in,
                const Three_Vector& position_in,
                double range_in)
  : segment_index (segment_index_in),
    position (position_in),
    fixed (false),
    vertical_field_angle (10.0),
    range (range_in)
{}

Camera::Camera () 
  : segment_index (0), 
    fixed (false),
    vertical_field_angle (10.0), 
    range (0.0) 
{}

//* Class Racing_Line
Racing_Line::Racing_Line ()
  : m_length (0.0),
    mp_line (0),
    m_list_id (0),
    m_iterations (1500),
    m_stiffness (1.0),
    m_damping (0.01),
    m_margin (1.6),
    m_resolution (0.0)
{
}

Racing_Line::~Racing_Line ()
{
  delete mp_line;
  glDeleteLists (m_list_id, 1);
}

Two_Vector
Racing_Line::position (double along) const
{
  assert (mp_line != 0);
  return mp_line->interpolate (wrap (along, m_length));
}

Three_Vector
Racing_Line::curvature (double along, double offline_fraction) const
{
  along = wrap (along, m_length);
  const Three_Vector c1 = m_curvature.interpolate (along);
  const Three_Vector c2 = (offline_fraction > 0.0)
    ? m_left_curvature.interpolate (along)
    : m_right_curvature.interpolate (along);
  //  linearly interpolate from line to edge.
  const double f = std::abs (offline_fraction);
  return Three_Vector (Vamos_Geometry::interpolate (f, 0.0, c1.x, 1.0, c2.x),
                       Vamos_Geometry::interpolate (f, 0.0, c1.y, 1.0, c2.y),
                       Vamos_Geometry::interpolate (f, 0.0, c1.z, 1.0, c2.z));
}

Three_Vector
Racing_Line::tangent (double along) const
{
  return m_tangent.interpolate (wrap (along, m_length));
}

Three_Vector Racing_Line::normal_curvature (const Three_Vector& p1,
                                            const Three_Vector& p2,
                                            const Three_Vector& p3) const
{
  Three_Vector r21 (p1 - p2);
  Three_Vector r23 (p3 - p2);
  Three_Vector up (r23.cross (r21));

  const double length_23 = r23.magnitude ();

  // Assume the angle is small so that sin x = x.
  return up / (r21.dot (r21) * length_23);
}

Three_Vector Racing_Line::planar_curvature (const Three_Vector& p1,
                                            const Three_Vector& p2,
                                            const Three_Vector& p3) const
{
  return normal_curvature (p1, p2, p3).magnitude () * (p2 - (p1 + p3)/2.0).unit ();
}

void
Racing_Line::force (const Three_Vector& p1,
                    const Three_Vector& p2,
                    const Three_Vector& p3,
                    Three_Vector& f1,
                    Three_Vector& f2,
                    Three_Vector& f3)
{
  Three_Vector r21 (p1 - p2);
  Three_Vector r23 (p3 - p2);
  Three_Vector curvature = normal_curvature (p1, p2, p3);
  Three_Vector df1 = m_stiffness * curvature.cross (r21);
  Three_Vector df3 = -m_stiffness * curvature.cross (r23);

  f1 += df1;
  f2 -= (df1 + df3);
  f3 += df3;
}

double
Racing_Line::right_width (const Road& road, double along) const
{
  return road.right_racing_line_width (along) - m_margin;
}

double
Racing_Line::left_width (const Road& road, double along) const
{
  return road.left_racing_line_width (along) - m_margin;
}

void
Racing_Line::propagate (const Road& road,
                        std::vector <Three_Vector>& positions,
                        std::vector <Three_Vector>& velocites,
                        double interval,
                        bool close)
{
  const size_t points = positions.size ();
  std::vector <Three_Vector> forces (points);

  force (positions [points-1], positions [0], positions [1],
         forces [points-1], forces [0], forces [1]);

  for (size_t i = 1; i < points - 1; i++)
    {
      force (positions [i-1], positions [i], positions [i+1],
             forces [i-1], forces [i], forces [i+1]);
    }

  force (positions [points-2], positions [points-1], positions [0],
         forces [points-2], forces [points-1], forces [0]);

  size_t index = 0;
  for (size_t i = 0; i < points; i++)
    {
      velocites [i] += (forces [i] - velocites [i] * m_damping);
      positions [i] += velocites [i];

      // Constrain the racing line to the track.
      const double along = i*interval;
      const double across = clip (road.track_coordinates (positions [i], index).y,
                                  -right_width (road, along),
                                  left_width (road, along));
      positions [i] = road.position (along, across);
    }
}

void
Racing_Line::build (const Road& road, bool close)
{
  m_length = road.length ();
  if (m_length <= 0.0)
    throw Bad_Racing_Line_Length (m_length);

  delete mp_line;
  mp_line = new Parametric_Spline ();

  // Divide the track into the smallest number of equal intervals not longer
  // than 'max_interval' 
  const double max_interval = m_resolution > 0.0 
    ? m_resolution
    : 0.5*(left_width (road, 0.0) + right_width (road, 0.0));
  double interval = max_interval;
  const int divisions = std::ceil (m_length / interval);
  if (divisions <= 0)
    throw No_Racing_Line_Segments (divisions);
  interval = m_length / divisions;

  // Use the center of the track as the initial guess.
  std::vector <Three_Vector> positions;
  for (int node = 0; node < divisions; node++)
    positions.push_back (road.position (node * interval, 0.0));

  std::vector <Three_Vector> velocities (positions.size ());
  for (size_t i = 0; i < m_iterations; i++)
    propagate (road, positions, velocities, interval, close);

  m_curvature.clear ();
  m_left_curvature.clear ();
  m_right_curvature.clear ();
  m_tangent.clear ();

  for (size_t i = 1; i < positions.size () - 1; ++i)
      load_curvature (i*interval, 
                      positions [i - 1], 
                      positions [i], 
                      positions [i + 1],
                      road);

  if (close)
    {
      mp_line->set_periodic (m_length);
      m_curvature.set_periodic (m_length);
      m_left_curvature.set_periodic (m_length);
      m_right_curvature.set_periodic (m_length);
      m_tangent.set_periodic (m_length);
    }

  build_list (road);
}

void
Racing_Line::load_curvature (double along,
                             const Three_Vector& p1,
                             const Three_Vector& p2,
                             const Three_Vector& p3,
                             const Road& road)
{
  const Gl_Road_Segment& segment = *road.segment_at (along);
  mp_line->load (along, p2.x, p2.y);

  m_tangent.load (along, (p3 - p1).unit ());

  const double factor = segment.racing_line_curvature_factor ();
  m_curvature.load (along, factor * planar_curvature (p1, p2, p3));

  if (segment.radius () == 0.0)
    {
      m_left_curvature.load (along, Three_Vector::ZERO);
      m_right_curvature.load (along, Three_Vector::ZERO);
    }
  else
    {
      double across = segment.left_racing_line_width (along);
      Three_Vector p1 = road.position (along - 10, across);
      Three_Vector p2 = road.position (along, across);
      Three_Vector p3 = road.position (along + 10, across);
      m_left_curvature.load (along, planar_curvature (p1, p2, p3));

      across = segment.right_racing_line_width (along);
      p1 = road.position (along - 10, across);
      p2 = road.position (along, across);
      p3 = road.position (along + 10, across);
      m_right_curvature.load (along, planar_curvature (p1, p2, p3));
    }
}

void
Racing_Line::build_list (const Road& road)
{
  if (m_list_id != 0)
    glDeleteLists (m_list_id, 1);

  m_list_id = glGenLists (1);
  glNewList (m_list_id, GL_COMPILE);

  glDisable (GL_TEXTURE_2D);
  glLineWidth (2.0);

  glBegin (GL_LINE_STRIP);
  Three_Vector last_world = position (0.0);
  for (double along = 0.0; along < m_length; along += 0.1)
    {
      Three_Vector world = position (along);
      Three_Vector forward = (world - last_world).unit ();
      Three_Vector curve = curvature (along, 0.0);
      double color = 100.0 * curve.magnitude ();
      if (curve.cross (forward).z < 0.0)
        color *= -1.0;
      glColor4f (1.0 - color, 1.0 + color, 1.0, 0.5);
      glVertex3d (world.x, 
                  world.y, 
                  road.segment_at (along)->world_elevation (world) + 0.05);
      last_world = world;
    }
  glEnd ();

  glPointSize (4.0);
  glColor4f (0.8, 0.0, 0.0, 0.5);
  glBegin (GL_POINTS);

  for (size_t i = 0; i < mp_line->size (); i++)
    {
      Three_Vector world = (*mp_line) [i];
      glVertex3d (world.x, 
                  world.y, 
                  road.segment_at (mp_line->parameter (i))->world_elevation (world) + 0.04);
    }
  glEnd ();

  glEnable (GL_TEXTURE_2D);
  glEndList ();
}

void
Racing_Line::draw () const
{
  glCallList (m_list_id);
}

//* Class Road
Road::Road ()
  : mp_elevation (new Spline ()),
    m_start_direction (0.0),
    m_racing_line (),
    m_draw_racing_line (false),
    m_is_closed (false)
{
  clear ();
}

Road::~Road ()
{
  clear ();
  delete mp_elevation;
}

void
Road::clear ()
{
  mp_elevation->clear ();
  mp_elevation->load (0.0, 0.0);
  m_length = 0.0;
  m_bounds = Rectangle ();

  for (Segment_List::iterator it = m_segments.begin ();
	   it != m_segments.end ();
	   it++)
	{
	  delete (*it);
	}
  m_segments.clear ();
}

size_t
Road::add_segment (Gl_Road_Segment* segment)
{
  if (!m_segments.empty ())
    {
      const Gl_Road_Segment* last = *(m_segments.end () - 1);
      segment->set_start (last->end_coords (),
                          last->end_distance (),
                          last->end_angle (),
                          0.0,
                          last->texture_offsets ());
    }
  m_segments.push_back (segment);
  return m_segments.size ();
}

double 
Road::build_elevation (bool periodic)
{
  double length = 0.0;
  for (Segment_List::iterator it = m_segments.begin ();
	   it != m_segments.end ();
	   it++)
	{
	  (*it)->build_elevation (mp_elevation, length);
	  length += (*it)->length ();
	}
  if (periodic)
    mp_elevation->set_periodic (length);
  return length;
}

void
Road::build_segments (Three_Vector start_coords, 
                      double start_angle, 
                      double start_bank)
{
  std::vector <double> 
    texture_offsets ((*(m_segments.begin ()))->materials ().size ());

  m_length = 0.0;
  for (Segment_List::iterator it = m_segments.begin ();
	   it != m_segments.end ();
	   it++)
	{
      (*it)->set_start (start_coords, m_length, start_angle, start_bank,
                        texture_offsets);
	  (*it)->build ();

	  // Update the bounding dimensions.
      m_bounds.enclose ((*it)->bounds ());
      m_length += (*it)->length ();

	  start_coords = (*it)->end_coords ();
	  start_angle = (*it)->end_angle ();
	  start_bank = (*it)->banking ().end_angle ();
	  texture_offsets = (*it)->texture_offsets ();
	}
}

Three_Vector
Road::position (double along, double from_center, const Gl_Road_Segment& segment) const
{
  return segment.position (along - segment.start_distance (), from_center);
}

Three_Vector
Road::position (double along, double from_center) const
{
  along = wrap (along, length ());
  const Gl_Road_Segment* segment = segment_at (along);
  return position (along, from_center, *segment);
}

// Return WORLD_POS transformed to the track's coordinate system.
// SEGMENT_INDEX will be modified if the position on another
// segment.
Three_Vector 
Road::track_coordinates (const Three_Vector& world_pos,
                         size_t& segment_index,
                         bool forward_only) const
{
  // Find the distance along the track, distance from center, and elevation
  // for the world coordinates `world_pos.x' and `world_pos.y'.
  Three_Vector track_pos;
  assert (segment_index < m_segments.size ());
  Gl_Road_Segment* segment = m_segments [segment_index];
  size_t i = 0;
  bool found = false;
  while (i < m_segments.size () + 1)
	{
	  double off = segment->coordinates (world_pos, track_pos); 
	  if (std::abs (off) < 1.0e-6)
		{
          found = true;
		  break;
		}

	  if (forward_only || (off > 0.0))
		{
          // We're off the end of the current segment.  Find a new
          // candidate segment.
          segment_index++;
          if (segment_index == m_segments.size ())
            {
              if (m_is_closed)
                segment_index = 0;
              else
                {
                  segment_index--;
                  found = true;
                  break;
                }
            }
        }
	  else
		{
          // Try the previous segment.
          if (segment_index == 0)
            {
              if (m_is_closed)
                segment_index = m_segments.size ();
              else
                {
                  found = true;
                  break;
                }
            }
          segment_index--;
		}
	  segment = m_segments [segment_index];
	  i++;
	}

  // Throw an exception if a segment could not be found.
  if (!found)
    throw Segment_Not_Found (world_pos, segment_index);

  assert (segment_index < m_segments.size ());
  track_pos.x += segment->start_distance ();
  return track_pos;
}

double
Road::distance (double along1, double along2) const
{
  const double limit = 0.5 * m_length;
  return wrap (along1 - along2, -limit, limit);
}

double
Road::left_road_width (double along) const
{
  return segment_at (along)->left_road_width (along);
}

double
Road::right_road_width (double along) const
{
  return segment_at (along)->right_road_width (along);
}

double
Road::right_racing_line_width (double along) const
{
  return segment_at (along)->right_racing_line_width (along);
}

double
Road::left_racing_line_width (double along) const
{
  return segment_at (along)->left_racing_line_width (along);
}

const Gl_Road_Segment*
Road::segment_at (double along) const
{
  double distance = 0.0;
  for (Segment_List::const_iterator it = segments ().begin ();
	   it != segments ().end ();
	   it++)
	{
      if (distance + (*it)->length () >= along)
        return *it;
      distance += (*it)->length ();
	}
  return segments ()[0];
}

void 
Road::build_racing_line ()
{   
  m_racing_line.build (*this, m_is_closed); 
}

void
Road::narrow_pit_segments ()
{
  Gl_Road_Segment* last_from_out = 0;
  Gl_Road_Segment* last_from_in = 0;

  for (Segment_List::iterator it = m_segments.begin ();
	   it != m_segments.end ();
	   it++)
	{
      const Pit_Lane_Transition& pit = (*it)->pit ();
      if (!pit.active ())
        continue;
      if (pit.direction () == OUT)
        {
          for (Segment_List::reverse_iterator rit (it);
               ((rit != m_segments.rend ())
                && (*rit != last_from_in)
                && !(*rit)->pit ().active ());
               rit++)
            {
              (*rit)->narrow (pit.side (), (*it)->pit_width ());
              last_from_out = *rit;
            }
        }
      else
        {
          for (Segment_List::iterator it2 (it + 1);
               ((it2 != m_segments.end ())
                && (*it2 != last_from_out)
                && !(*it2)->pit ().active ());
               it2++)
            {
              (*it2)->narrow (pit.side (), (*it)->pit_width ());
              last_from_in = *it2;
            }
       }
    }
}

void 
Road::build (bool close, int adjusted_segments, double length)
{
  narrow_pit_segments ();
  set_skews ();

  Gl_Road_Segment& first = **m_segments.begin ();
  Gl_Road_Segment& last = **(m_segments.end () - 1);

  if (close)
    {
      join (first.start_coords (), 
            first.start_angle (),
            first.start_coords (), 
            first.start_angle (),
            adjusted_segments);
      // Force the segment to end at 0, 0.
      last.last_segment (true);
    }
  if (length != 0.0)
      set_length (length);

  // Make sure the walls join.
  last.set_left_width (last.length (), first.left_width (0.0));
  last.set_right_width (last.length (), first.right_width (0.0));

  build_elevation (m_is_closed);
  build_segments (Three_Vector (),
                  start_direction (),
                  close ? last.banking ().end_angle () : 0.0);
}

double perpendicular_distance (const Three_Vector& p1, 
                               const Three_Vector& p2,
                               double angle)
{
  return (p2 - p1).magnitude () 
            * sin (atan2 (p1.y - p2.y, p1.x - p2.x) - angle);
}

// Force a road that starts at START_COORDS and START_ANGLE, to end at
// END_COORDS and END_ANGLE.
void 
Road::join (const Three_Vector& start_coords,
            double start_angle,
            const Three_Vector& end_coords,
            double end_angle,
            int adjusted_segments)
{
  m_is_closed = true;

  if ((adjusted_segments < 0) || (adjusted_segments > 3))
    {
      std::ostringstream message;
      message << "The number of segments to be adjusted (" << adjusted_segments
              << ") is not in the range [0, 3]";
      throw Can_Not_Close (message.str ());
    }

  if (m_segments.size () < size_t (adjusted_segments))
    {
      std::ostringstream message;
      message << "Track has fewer segments (" << m_segments.size() 
              << ") than the number of segments to be adjusted (" 
              << adjusted_segments << ")"; 
      throw Can_Not_Close (message.str ());
    }

  if (adjusted_segments == 0) 
    {
      // Call it closed without adjusting anything.
      return;
    }

  Gl_Road_Segment* last_segment = *(m_segments.end () - 1);
  Gl_Road_Segment* last_curve = (adjusted_segments > 1)
    ? *(m_segments.end () - 2)
    : last_segment->is_straight () ? 0 : last_segment;
  Gl_Road_Segment* other_straight = (adjusted_segments == 3)
    ? *(m_segments.end () - 3)
    : 0;

  if ((adjusted_segments > 1)
      && (last_curve->is_straight () || !last_segment->is_straight ()))
    {
      throw Can_Not_Close ("Track must end with a curve followed by "
                           "a straight when more than one segment "
                           "is to be adjusted.");
    }
  if ((adjusted_segments == 3) && !other_straight->is_straight ())
    {
      throw Can_Not_Close ("Track must end with a straight, a curve and a "
                           "straight when three segments are to be adjusted.");
    }

  // Make the last segment parallel to the first by changing the length of
  // the last curve.
  double last_arc = 0.0;
  if (last_curve != 0)
    {
      last_arc = last_curve->arc () 
        + branch (end_angle - last_curve->end_angle (), -pi);
      last_curve->set_arc (last_arc);

      // If we're only adjusting the last curve, we're done.
      if (last_segment == last_curve)
        return;
    }

  if (adjusted_segments > 1)
    {
      // Make the last segment collinear with the first.
      const double perp = perpendicular_distance (last_curve->end_coords (),
                                                  end_coords,
                                                  end_angle);
      switch (adjusted_segments)
        {
        case 2:
          {
            // Change the radius of the curve.
            //  last_curve->set_radius (last_curve->radius () + perp / cos (last_arc));
            last_curve->set_radius (last_curve->radius () 
                                    + perp / (1.0 - cos (last_arc)));
            break;
          }
        case 3:
          {
            // Change the length of segment[-3].
            other_straight->set_length (other_straight->length () 
                                        + perp / sin (last_arc));
            break;
          }
        default:
          // 2 and 3 should be the only possibilities in this branch. 
          assert (false);
        }
      // Propagate any adjustments to the end of the track.
      connect (m_segments.end () - 2);
    }

  // Extend the last segment to meet the first.  Assume they are collinear.
  // This is guaranteed for 'adjusted_segments' == 2 or 3 but not 1.
  last_segment->set_length ((last_segment->start_coords () - end_coords).magnitude ());
}

void 
Road::set_skews ()
{
  for (Segment_List::iterator it = m_segments.begin () + 1;
	   it != m_segments.end ();
	   it++)
	{
      double skew = (*it)->start_skew ();
      if ((skew != 0.0) && ((*it)->arc () != 0.0))
        {
          if ((*(it - 1))->arc () == 0.0)
            (*(it - 1))->set_end_skew (skew);
          if ((*(it + 1))->arc () == 0.0)
            (*(it + 1))->set_start_skew (-skew);
        }
    }
}

// Scale the track to a particular length.
void 
Road::set_length (double length)
{
  assert (m_segments.size () != 0);

  // Find the current length.
  double old_length = 0.0;
  for (Segment_List::iterator it = m_segments.begin ();
       it != m_segments.end ();
       it++)
    {
      old_length += (*it)->length ();
    }

  assert (old_length != 0.0);
  double factor = length / old_length;

  // Adjust the segments.
  for (Segment_List::iterator it = m_segments.begin ();
       it != m_segments.end ();
       it++)
    {
      (*it)->scale (factor);
    }
}

void 
Road::set_start_direction (double degrees)
{ 
  m_start_direction = branch (deg_to_rad (degrees), 0.0);

  if (m_segments.empty ())
    return;

  Gl_Road_Segment* first = *(m_segments.begin ());
  first->set_start_angle (m_start_direction);

  connect (m_segments.begin () + 1);
}

void
Road::connect (Segment_List::iterator it)
{
  // There's nothing to do to the first segment.
  if (it == m_segments.begin ())
    it++;

  const Gl_Road_Segment* last = *(it - 1);
  for (; it != m_segments.end (); it++)
    {
      (*it)->set_start_angle (last->end_angle ());
      (*it)->set_start_coords (last->end_coords ());
      last = (*it);
    }
}

void
Road::draw ()
{
  std::for_each (m_segments.begin (), m_segments.end (), 
                 std::mem_fun (&Gl_Road_Segment::draw));
  if (m_draw_racing_line)
    m_racing_line.draw ();
}

Three_Vector
Pit_Lane::pit_in_offset (Gl_Road_Segment& pit_in) const
{
  double pit_width;
  if (pit_in.pit ().side () == LEFT)
    pit_width = (*segments ().begin ())->left_width (0.0);
  else
    pit_width = (*segments ().begin ())->right_width (0.0);
  pit_width /= cos (start_direction ());

  double along = pit_in.pit ().split_or_join ();
  double offset;
  if (pit_in.pit ().side () == LEFT)
    offset = pit_in.left_width (along) - pit_width;
  else
    offset = -pit_in.right_width (along) + pit_width;

  if (pit_in.radius () == 0.0)
    return Three_Vector (along, offset, 0.0).
      rotate (pit_in.angle (along) * Three_Vector::Z);
  else
    return pit_in.center_of_curve () - pit_in.start_coords ()
      + Three_Vector (pit_in.radius () - offset, 
                      pit_in.angle (along) - pi/2);
}

Three_Vector
Pit_Lane::pit_out_offset (Gl_Road_Segment& pit_out) const
{
  double pit_width;
  const Gl_Road_Segment& last_segment = **(segments ().end () - 1); 
  if (pit_out.pit ().side () == LEFT)
    pit_width = last_segment.left_width (last_segment.length ());
  else
    pit_width = last_segment.right_width (last_segment.length ());
  pit_width /= cos (start_direction ());

  double along = pit_out.pit ().split_or_join ();
  double offset;
  if (pit_out.pit ().side () == LEFT)
    offset = pit_out.left_width (along) - pit_width;
  else
    offset = -pit_out.right_width (along) + pit_width;

  if (pit_out.radius () == 0.0)
    return Three_Vector (along, offset, 0.0).
      rotate (pit_out.angle (along) * Three_Vector::Z);
  else
    return pit_out.center_of_curve () - pit_out.start_coords ()
      + Three_Vector (pit_out.radius () - offset, 
                      pit_out.angle (along) - pi/2);
}

void
Pit_Lane::build (bool join_to_track, 
                 int adjusted_segments,
                 Gl_Road_Segment& pit_in,
                 Gl_Road_Segment& pit_out,
                 const Spline& track_elevation)
{
  if (m_segments.size () == 0) return;

  // Skew the ends of the pit lane to meet the track.
  set_skews ();
  (*m_segments.begin ())->set_start_skew (tan (start_direction ()));
  (*(m_segments.end () - 1))->set_end_skew (tan (end_direction ()));

  // The ends of the pit lane attach to points on the track.  Call
  // build_segments() to transform the pit lane's coordinates to the track's
  // orientation and pit-in position. 
  build_elevation (false);
  build_segments (pit_in.start_coords () + pit_in_offset (pit_in),
                  pit_in.pit_angle () + start_direction (),
                  pit_in.banking ().end_angle ());

  // Make the end of the pit lane meet the track's pit-out position.
  if (join_to_track)
    {
      join (pit_in.start_coords () + pit_in_offset (pit_in),
            pit_in.pit_angle () + start_direction (),
            pit_out.start_coords () + pit_out_offset (pit_out),
            pit_out.pit_angle () + end_direction (),
            adjusted_segments);
    }

  // Load the pit lane with elevations from the track.
  {
    m_length = build_elevation (false);
    mp_elevation->clear ();
    double in_distance = 
      pit_in.start_distance () + pit_in.pit ().split_or_join ();
    double out_distance = 
      pit_out.start_distance () + pit_out.pit ().split_or_join ();
    double track_length = track_elevation [track_elevation.size () - 1].x;
    double delta = wrap (out_distance - in_distance, track_length);

    static const int elevations = 10;
    for (int i = 0; i < elevations; i++)
      {
        double along_pit = i * m_length / elevations;
        double along_track = wrap (in_distance + i * delta / elevations, track_length);
        double z = track_elevation.interpolate (along_track);
        mp_elevation->load (along_pit, z);
      }
    mp_elevation->load (m_length, track_elevation.interpolate (out_distance));
  }

  // Finalize the elevations and segments.
  build_elevation (false);
  build_segments (pit_in.start_coords () + pit_in_offset (pit_in),
                  pit_in.pit_angle () + start_direction (),
                  pit_in.banking ().end_angle ());
}

//* Class Strip_Track
Strip_Track::Strip_Track () :
  mp_track (new Road),
  mp_pit_lane (new Pit_Lane),
  m_pit_in_index (-1),
  m_pit_out_index (-1),
  mp_sky_box (0),
  mp_map_background (0)
{
  m_timing_lines.clear ();
  m_cameras.clear ();
}


Strip_Track::~Strip_Track ()
{
  delete mp_pit_lane;
  delete mp_track;
  delete mp_sky_box;
  delete mp_map_background;
}

void
Strip_Track::show_racing_line (bool show)
{
  if (mp_track)
    mp_track->show_racing_line (show);
}

// Read the track definition file.
void 
Strip_Track::read (std::string data_dir, std::string track_file)
{
  // Remember the file name for re-reading.
  if ((data_dir != "") && (track_file != ""))
	{
	  m_data_dir = data_dir;
	  m_track_file = track_file;
	}

  mp_track->clear ();
  mp_pit_lane->clear ();
  m_timing_lines.clear ();

  Strip_Track_Reader reader (m_data_dir, m_track_file, this);
}

// Add a straight or curved segment of road to the track.
size_t
Strip_Track::add_segment (Gl_Road_Segment* segment)
{
  return mp_track->add_segment (segment);
}

size_t
Strip_Track::add_pit_segment (Gl_Road_Segment* segment)
{
  const bool start = (mp_pit_lane->segments ().size () == 0);
  const double distance = start ? 0.0 : segment->length ();
  const double width = 
    segment->left_width (distance) + segment->right_width (distance);
  const double left_shoulder = 
    segment->left_width (distance) - segment->left_road_width (distance);
  const double right_shoulder = 
    segment->right_width (distance) - segment->right_road_width (distance);

  if (start)
    {
      mp_track->segments ()[m_pit_in_index]->
        set_pit_width (width, left_shoulder, right_shoulder);
    }
  else
    {
      mp_track->segments ()[m_pit_out_index]->
        set_pit_width (width, left_shoulder, right_shoulder);
    }
  return mp_pit_lane->add_segment (segment);
}

void 
Strip_Track::set_pit_in (size_t index, double angle) 
{ 
  m_pit_in_index = index; 
  mp_pit_lane->set_start_direction (angle);
}

void 
Strip_Track::set_pit_out (size_t index, double angle) 
{ 
  m_pit_out_index = index; 
  mp_pit_lane->set_end_direction (angle);
}

// Make the track.
void 
Strip_Track::build (bool close,
                    int adjusted_road_segments, 
                    double track_length, 
                    bool join_pit_lane,
                    int adjusted_pit_segments)
{
  mp_track->build (close, adjusted_road_segments, track_length);

  if ((m_pit_in_index != -1) && (m_pit_out_index != -1))
    {
      Gl_Road_Segment& in = *mp_track->segments ()[m_pit_in_index];
      Gl_Road_Segment& out = *mp_track->segments ()[m_pit_out_index];
      mp_pit_lane->build (join_pit_lane, 
                          adjusted_pit_segments,
                          in,
                          out,
                          mp_track->elevation ());

      double along = in.pit ().split_or_join() + 1.0e-6;
      double from_center = in.pit ().side () == RIGHT
        ? -in.right_width (along)
        : in.left_width (along);
      m_objects.push_back (Track_Object (position (along + in.start_distance (), 
                                                   from_center), 
                                         in.pit ().side () == RIGHT
                                         ? in.right_material (0.0)
                                         : in.left_material (0.0)));

      along = out.pit ().split_or_join() - 1.0e-6;;
      from_center = out.pit ().side () == RIGHT
        ? -out.right_width (along)
        : out.left_width (along);
      m_objects.push_back (Track_Object (position (along + out.start_distance(), 
                                                   from_center), 
                                         in.pit ().side () == RIGHT
                                         ? out.right_material (0.0)
                                         : out.left_material (0.0)));
    }
}

void
Strip_Track::set_sky_box (std::string sides_image, 
                          std::string top_image,
                          std::string bottom_image,
                          bool smooth)
{
  delete mp_sky_box;
  mp_sky_box = new Sky_Box (100.0, 
                            sides_image, 
                            top_image, 
                            bottom_image,
                            smooth);
}

void
Strip_Track::set_map_background (std::string background_image,
                                 double x_offset, double y_offset,
                                 double x_size, double y_size)
{
  delete mp_map_background;
  mp_map_background = new Map_Background (background_image,
                                          x_offset, y_offset,
                                          x_size, y_size);
}

void
Strip_Track::draw_sky (const Three_Vector& view) const
{
  mp_sky_box->draw (view);
}

void
Strip_Track::draw_map_background () const
{
  if (mp_map_background != 0)
    mp_map_background->draw ();
}

void 
Strip_Track::draw () const
{
  glLoadIdentity ();
  mp_track->draw ();
  mp_pit_lane->draw ();
}

// Return the new position for a vehicle at POS when a reset is
// performed.
Three_Vector 
Strip_Track::reset_position (const Three_Vector& pos,
							 size_t& road_index,
							 size_t& segment_index)
{
  Three_Vector track_pos = track_coordinates (pos, road_index, segment_index);
  track_pos.y = 0.0;
  track_pos.z = 0.0;
  return track_pos;
}

// Return the new orientation for a vehicle at POS when a reset is
// performed.
Three_Matrix 
Strip_Track::reset_orientation (const Three_Vector& pos,
								size_t& road_index,
								size_t& segment_index)
{
  Three_Matrix orientation;
  orientation.identity ();

  // Align the car's up direction with the normal.
  const Three_Vector& track_pos = 
    track_coordinates (pos, road_index, segment_index);

  const Gl_Road_Segment* segment = 
    get_road (road_index).segments ()[segment_index];
  const double along = track_pos.x - segment->start_distance ();
  const double across = track_pos.y;
  Three_Vector normal = segment->normal (along, across); 
  orientation.rotate (Three_Vector (-asin (normal.y), 
									asin (normal.x),
									segment->angle (along)));
  return orientation;
}

// Return the elevation of the track at the x and y components of POS.
double 
Strip_Track::elevation (const Three_Vector& pos, 
						double x,
                        double y,
						size_t& road_index,
						size_t& segment_index)
{
  Three_Vector bump = m_material.bump (x, y);
  return track_coordinates (pos, road_index, segment_index).z + bump.z;
}

// Return WORLD_POS transformed to the track's coordinate system.
// SEGMENT_INDEX will be modified if the position on another
// segment.
Three_Vector 
Strip_Track::track_coordinates (const Three_Vector& world_pos,
								size_t& road_index,
								size_t& segment_index)
{
  // Find the distance along the track, distance from center, and elevation
  // for the world coordinates `world_pos.x' and `world_pos.y'.

  Three_Vector track_pos;
  const Segment_List* segments = &get_road (road_index).segments ();
  if (segment_index >= segments->size ())
    {
      std::cerr << segment_index << ' ' 
                << segments->size () << ' ' 
                << road_index << std::endl;
      assert (false);
    }
  Gl_Road_Segment* segment = (*segments)[segment_index];

  int direction = 0;

  size_t i = 0;
  bool found = false;
  while (i < segments->size () + 1)
	{
	  double off = segment->coordinates (world_pos, track_pos); 
	  if (off == 0.0)
		{
          found = true;
		  break;
		}

	  if ((direction == 1) || ((direction == 0) && (off > 0.0)))
		{
          direction = 1;
          // We're off the end of the current segment.  Find a new
          // candidate segment.
          if ((road_index == 0) 
              && (segment_index == size_t (m_pit_in_index))
              && (segment->on_pit_merge (track_pos.x, track_pos.y)))
            {
              // We've moved onto the pit lane.  Try the first segment
              // on the pit lane.
              road_index = 1;
              segment_index = 0;
            }
          else if ((road_index == 1) 
                   && (segment_index == mp_pit_lane->segments ().size () - 1))
            {
              // We've moved off of the pit lane.  Try the segment
              // where the pit lane joins the track.
              road_index = 0;
              segment_index = m_pit_out_index;
            }
          else
            {
              // Try the next segment.
              segment_index++;
              if ((road_index == 0) && (segment_index == segments->size ()))
                segment_index = 0;
            }
        }
	  else
		{
          direction = -1;
          // We're off the beginning of the current segment.  Find a new
          // candidate segment.
          if ((road_index == 0) 
              && (segment_index == size_t (m_pit_out_index))
              && (segment->on_pit_merge (track_pos.x, track_pos.y)))
            {
              // We've moved onto the end of the pit lane.  Try the
              // last segment on the pit lane.
              road_index = 1;
              segment_index = mp_pit_lane->segments ().size () - 1;
            }
          else if ((road_index == 1) && (segment_index == 0))
            {
              // We've moved off of the beginning of the pit lane.
              // Try the segment where the pit lane splits from the
              // track.
              road_index = 0;
              segment_index = m_pit_in_index;
            }
          else
            {
              // Try the previous segment.
              if ((road_index == 0) && (segment_index == 0))
                segment_index = segments->size ();
              segment_index--;
			}
		}
      segments = &get_road (road_index).segments ();
	  segment = (*segments)[segment_index];
	  i++;
	}

  // Throw an exception if a segment could not be found.
  if (!found)
    throw Segment_Not_Found (world_pos, segment_index);

  assert (segment_index < segments->size ());
  m_material = segment->material_at (track_pos.x, track_pos.y);
  track_pos.x += segment->start_distance ();
  return track_pos;
}

const Road&
Strip_Track::get_road (size_t road_index) const
{
  switch (road_index)
    {
    case 0: return *mp_track;
    case 1: return *mp_pit_lane;
    default: assert (false);
    }
}

Contact_Info 
Strip_Track::test_for_contact (const Three_Vector& pos, 
							   double bump_parameter,
							   size_t& road_index,
							   size_t& segment_index)
{
  const Three_Vector track_pos = track_coordinates (pos, road_index, segment_index);
  const Gl_Road_Segment* segment = 
    get_road (road_index).segments ()[segment_index];
  const double segment_distance = track_pos.x - segment->start_distance ();

  bool contact = false;
  Three_Vector normal;

  // Test for contact with the road.
  double diff = 
    elevation (pos, track_pos.x, track_pos.y, road_index, segment_index) - pos.z;
  if (diff >= 0.0)
	{
	  contact = true;
	  Three_Vector bump = m_material.bump (track_pos.x, track_pos.y);
	  normal = segment->normal (segment_distance, track_pos.y, bump);
	}

  // Test for contact with the walls.
  // Left
  if (!contact)
	{
	  const Material& material = segment->left_material (pos.z);
	  Three_Vector bump = material.bump (track_pos.x, track_pos.y);
	  diff = track_pos.y - (segment->left_width (segment_distance) + bump.z); 
	  if (diff >= 0.0)
		{
		  contact = true;
          m_material = material;
		  normal = segment->barrier_normal (segment_distance, track_pos.y, bump);
		}
	}

  // Right
  if (!contact)
	{
      const Material& material = segment->right_material (pos.z);
	  Three_Vector bump = material.bump (track_pos.x, track_pos.y);
	  diff = -track_pos.y - (segment->right_width (segment_distance) + bump.z); 
	  if (diff >= 0.0)
		{
		  contact = true;
          m_material = material;
		  normal = segment->barrier_normal (segment_distance, track_pos.y, bump);
		}
	}

  return Contact_Info (contact, diff, normal, m_material);
}

Three_Vector
Strip_Track::position (double along, double from_center) const
{
  return mp_track->position (along, from_center);
}

// Return the timing sector at the given distance.
int 
Strip_Track::sector (double distance)
{
  for (size_t i = 0; i < m_timing_lines.size (); i++)
	{
	  if (m_timing_lines [i] > distance)
		return i;
	}
  return m_timing_lines.size ();
}

void 
Strip_Track::set_start_direction (double degrees)
{
  m_start_direction = deg_to_rad (degrees);
  mp_track->set_start_direction (degrees);
}

void
Strip_Track::add_camera (const Camera& camera)
{
  m_cameras.push_back (camera);
}

double 
Strip_Track::camera_range (const Camera& camera) const
{
  double range = mp_track->segments ()[camera.segment_index]->start_distance () 
    + camera.position.x 
    - camera.range;
  return wrap (range, mp_track->length ());
}

Three_Vector
Strip_Track::camera_position (const Camera& camera) const
{
  const Gl_Road_Segment& segment = *(mp_track->segments ()[camera.segment_index]);
  return segment.position (camera.position.x, camera.position.y)
    + Three_Vector (0.0, 0.0, camera.position.z);
}

const Camera& 
Strip_Track::get_camera (double distance) const
{
  if (m_cameras.empty ())
    return s_default_camera;
  
  // See if we're near the end of the track and should be picked up by
  // the first camera.
  double first = m_cameras.begin ()->position.x - m_cameras.begin ()->range;
  if (mp_track->is_closed ()
      && (first < 0.0)
      && (distance > wrap (first, mp_track->length ())))
    return *m_cameras.begin ();

  for (std::vector <Camera>::const_reverse_iterator rit = m_cameras.rbegin ();
       rit != m_cameras.rend (); 
       rit++)
    if (distance > camera_range (*rit))
      return *rit;

  return *m_cameras.begin ();
}

Three_Vector
Strip_Track::camera_target (const Camera& camera) const
{
  double angle = 
    mp_track->segments ()[camera.segment_index]->angle (camera.position.x);
  return camera_position (camera)
    + Three_Vector (-sin (deg_to_rad (camera.direction.x) + angle),
                    cos (deg_to_rad (camera.direction.x) + angle),
                    sin (deg_to_rad (camera.direction.y)));
}

const Rectangle&
Strip_Track::bounds () const
{
  return Rectangle (mp_track->bounds ()).enclose (mp_pit_lane->bounds ());
}

Three_Vector Strip_Track::grid_position (int place, int total, bool pit) const
{
  assert (place > 0); // 1-based
  assert (place <= total);
  static const double grid_interval = (pit ? 12.0 : 8.0);
  // Put the 1st car 1 interval from the beginning of the 1st segment to avoid
  // putting off the end.
  double across = pit 
    ? 1.5 * mp_track->left_road_width (0.0) 
    : 3.0 * std::pow (-1, place);
  return Three_Vector (grid_interval * (total - place + 1), across, 0.0);
}
