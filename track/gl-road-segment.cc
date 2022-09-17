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

#include "../media/model.h"
#include "../media/texture-image.h"
#include "../geometry/three-vector.h"
#include "gl-road-segment.h"

#include <GL/glut.h>

#include <cmath>
#include <cassert>
#include <algorithm>

using namespace Vamos_Track;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

static const int s_strip_type = GL_QUAD_STRIP;
//static const int s_strip_type = GL_LINE_STRIP;
//static const int s_strip_type = GL_LINES;

static const Material s_no_material (Material::AIR, 0.0, 0.0);

Braking_Marker::Braking_Marker (std::string image_file,
                                double distance,
                                Side side,
                                double from_edge,
                                double off_ground,
                                double width,
                                double height)
  : mp_image (new Facade (image_file, true)),
    m_distance (distance),
    m_side (side),
    m_from_edge (from_edge),
    m_off_ground (off_ground)
{
  mp_image->set_width (width);
  mp_image->set_height (height);
}

Braking_Marker::~Braking_Marker ()
{
  delete mp_image;
}

double
Braking_Marker::width () const
{
  return mp_image->width ();
}

double
Braking_Marker::height () const
{
  return mp_image->height ();
}

void
Braking_Marker::draw () const
{
  mp_image->draw ();
}

//=============================================================================
namespace Vamos_Track
{
  class Segment_Iterator
  {
  public:
    Segment_Iterator (const Road_Segment& segment, double resolution);
                      
    void start (const Three_Vector& start_coords, 
                double start_angle,
                Gl_Road_Segment::Strip strip);
    Segment_Iterator& operator ++ ();

    const Three_Vector& coordinates () const { return m_coordinates; }
    const Three_Vector& normal () const { return m_normal; }
    const Two_Vector& texture_coordinates () const 
    { return m_texture_coordinates; }
    bool last_subdivision () const { return m_position == END; }

  private:
	void increment_distance ();
	void increment_kerb_distance (const Kerb& kerb);
    size_t substrips () const;

    const Road_Segment& m_segment;

    Three_Vector m_coordinates;
    Three_Vector m_normal;
    Two_Vector m_texture_coordinates;

    double m_resolution;

    Gl_Road_Segment::Strip m_strip;
    size_t m_substrip;
    double m_distance;
    Side m_side;

    enum Segment_position
      {
        BEGIN,
        BEGIN_TRANSITION,
        MIDDLE,
        END_TRANSITION,
        END
      };
    Segment_position m_position;

    bool m_connection;
    bool m_after_connection;
  };

  Segment_Iterator::Segment_Iterator (const Road_Segment& segment,
                                      double resolution)
    : m_segment (segment),
      m_texture_coordinates{0.0, 0.0},
      m_resolution (resolution),
      m_position (BEGIN),
      m_connection (false),
      m_after_connection (false)
  {
  }

  void Segment_Iterator::start(Three_Vector const & start_coords,
                               double, // start_angle,
                               Gl_Road_Segment::Strip strip)
  {
    m_coordinates = start_coords;
    m_normal = {0.0, 0.0, 1.0};
    m_distance = 0.0;
    m_strip = strip;

    m_side = Side::left;
    m_position = BEGIN;
    m_substrip = 0;
  }
}

size_t 
Segment_Iterator::substrips () const
{
  if (m_strip == Gl_Road_Segment::LEFT_KERB)
    return m_segment.mp_left_kerb->substrips ();
  if (m_strip == Gl_Road_Segment::RIGHT_KERB)
    return m_segment.mp_right_kerb->substrips ();
  return 1;
}

Segment_Iterator&
Segment_Iterator::operator ++ ()
{
  assert (m_segment.mp_elevation_curve != 0);

  // Remember the last coordinates.
  Three_Vector last_coords = m_coordinates;
  double last_tex_dist = m_texture_coordinates.y;

  // Bail out if there's no kerb.
  if (((m_strip == Gl_Road_Segment::LEFT_KERB )
       && (m_segment.mp_left_kerb->transition_end () == 0.0))
      || (( m_strip == Gl_Road_Segment::RIGHT_KERB )
          && (m_segment.mp_right_kerb->transition_end () == 0.0)))
    {
      m_position = END;
      return *this;
    }

  if (m_position == BEGIN)
	{
      // Start a new strip.
      glEnd ();
      glBegin (s_strip_type);

	  last_tex_dist = 0.0;
	  m_texture_coordinates.y = 0.0;
      m_after_connection = false;
    }

  increment_distance ();

  bool left_start_transition = false;
  bool left_end_transition = false;
  bool right_start_transition = false;
  bool right_end_transition = false;

  if (m_strip == Gl_Road_Segment::LEFT_KERB)
	{
      m_segment.mp_left_kerb->set_length (m_segment.length ());
	  if (m_distance < m_segment.mp_left_kerb->start ())
		  left_start_transition = true;
	  else if (m_distance > m_segment.mp_left_kerb->end ())
		  left_end_transition = true;
	}
  else if (m_strip == Gl_Road_Segment::RIGHT_KERB)
	{
      m_segment.mp_right_kerb->set_length (m_segment.length ());
	  if (m_distance < m_segment.mp_right_kerb->start ())
		  right_start_transition = true;
	  else if (m_distance > m_segment.mp_right_kerb->end ())
		  right_end_transition = true;
	}

  auto no_pit{m_segment.pit().active()
              && ((m_segment.pit().end() == Pit_Lane_Transition::End::in && m_after_connection)
                  || (m_segment.pit ().end() == Pit_Lane_Transition::End::out && !m_after_connection))};

  // Calculate the geometry of the strip.
  double across = 0.0;
  double up = 0.0;
  switch (m_strip)
	{
	case Gl_Road_Segment::LEFT_BARRIER:
	  across = m_segment.left_width (m_distance, no_pit);
	  up = m_side == Side::left ? m_segment.m_left_wall_height : 0.0;
	  m_texture_coordinates.x = up;
	  m_normal = m_segment.barrier_normal (m_distance, across);
      if (m_after_connection)
        glBegin (s_strip_type);
	  break;
	case Gl_Road_Segment::LEFT_SHOULDER:
	  up = 0.0;
	  across = m_side == Side::left
        ? m_segment.left_width (m_distance, no_pit)
        : m_segment.left_road_width (m_distance, no_pit);
	  m_texture_coordinates.x = across;
	  m_normal = m_segment.normal (m_distance, across, false);
	  break;
	case Gl_Road_Segment::LEFT_KERB:
        up = m_side == Side::left
            ? m_segment.mp_left_kerb->point(m_substrip + 1).y
            : m_segment.mp_left_kerb->point(m_substrip).y;
        across = m_side == Side::left
            ? m_segment.mp_left_kerb->point(m_substrip + 1).x
            : m_segment.mp_left_kerb->point(m_substrip).x;
	  if (left_start_transition)
		{
		  up = 0.0;
		  across *= (m_segment.mp_left_kerb->start_transition_width ()
					 / m_segment.mp_left_kerb->width ());
		}
	  else if (left_end_transition)
		{
		  up = 0.0;
		  across *= (m_segment.mp_left_kerb->end_transition_width ()
					 / m_segment.mp_left_kerb->width ());
		}
	  across += m_segment.left_road_width (m_distance, no_pit);
	  m_texture_coordinates.x = across;
      m_normal = m_segment.normal (m_distance, across);
	  break;
	case Gl_Road_Segment::TRACK:
	  up = 0.0;
      across = m_side == Side::left
        ? m_segment.left_road_width (m_distance, no_pit)
        : -m_segment.right_road_width (m_distance, no_pit);
	  m_texture_coordinates.x 
		= across + m_segment.left_road_width (m_distance, no_pit);
	  m_normal = m_segment.normal (m_distance, across, false);
      if (m_after_connection)
        glBegin (s_strip_type);
	  break;
	case Gl_Road_Segment::RIGHT_KERB:
	  up = 0.0;
	  across = 0.0;
	  if (m_side == Side::left)
		{
		  up = m_segment.mp_right_kerb->point (m_substrip).y;
		  across = -m_segment.mp_right_kerb->point (m_substrip).x;
		}
	  else
		{
		  up = m_segment.mp_right_kerb->point (m_substrip + 1).y;
		  across = -m_segment.mp_right_kerb->point (m_substrip + 1).x;
		}
	  if (right_start_transition)
		{
		  up = 0.0;
		  across *= (m_segment.mp_right_kerb->start_transition_width ()
					 / m_segment.mp_right_kerb->width ());
		}
	  else if (right_end_transition)
		{
		  up = 0.0;
		  across *= (m_segment.mp_right_kerb->end_transition_width ()
					 / m_segment.mp_right_kerb->width ());
		}

	  across -= m_segment.right_road_width (m_distance, no_pit);
	  m_texture_coordinates.x = across;
	  m_normal = m_segment.normal (m_distance, across);
	  break;
	case Gl_Road_Segment::RIGHT_SHOULDER:
	  up = 0.0;
	  across = m_side == Side::left
        ? -m_segment.right_road_width (m_distance, no_pit)
        : -m_segment.right_width (m_distance, no_pit);
	  m_texture_coordinates.x = across;
	  m_normal = m_segment.normal (m_distance, across, false);
	  break;
	case Gl_Road_Segment::RIGHT_BARRIER:
      across = -m_segment.right_width (m_distance, no_pit);
	  up = (m_side == Side::left) ? 0.0 : m_segment.m_right_wall_height;
	  m_texture_coordinates.x = up;
	  m_normal = m_segment.barrier_normal (m_distance, across);
      if (m_after_connection)
        glBegin (s_strip_type);
	  break;
	default:
	  assert (false);
	}

  if (((m_position == END) && m_segment.is_last_segment ())
	  && (m_strip != Gl_Road_Segment::LEFT_KERB)
	  && (m_strip != Gl_Road_Segment::RIGHT_KERB))
	{
	  // Return to the origin if we're at the end of the track.
      const double angle = m_segment.angle (m_distance);
	  m_coordinates = 
          Three_Vector{-across * std::sin(angle),
                       across * std::cos(angle),
                       up + m_segment.banking ().height(m_distance, across)};
	}
  else
    {
      m_coordinates = m_segment.position (m_distance, across);
      m_coordinates.z = m_segment.elevation (m_distance, across) + up;
    }

  if (m_position != BEGIN
	  && ((m_strip == Gl_Road_Segment::LEFT_BARRIER) 
          || (m_strip == Gl_Road_Segment::RIGHT_BARRIER)))
	{
        if (m_side == Side::left)
		{
		  double dx = m_coordinates.x - last_coords.x;
		  double dy = m_coordinates.y - last_coords.y;
		  double dist = std::sqrt (dx*dx + dy*dy);
		  m_texture_coordinates.y = last_tex_dist + dist; 
		}
	}
  else
	  m_texture_coordinates.y = m_distance;
  
  if (m_position == END && m_side == Side::right)
	{
	  m_substrip++;
      if (m_substrip != substrips ())
        m_position = BEGIN;
	}
  else if (m_position == BEGIN)
    m_position = MIDDLE;

  m_side = m_side == Side::left ? Side::right : Side::left;
  return *this;
}

void 
Segment_Iterator::increment_distance ()
{
  if (m_position == BEGIN)
	{
	  switch (m_strip)
		{
        case Gl_Road_Segment::LEFT_KERB:
          increment_kerb_distance (*(m_segment.mp_left_kerb));
          break;
        case Gl_Road_Segment::RIGHT_KERB:
          increment_kerb_distance (*(m_segment.mp_right_kerb));
          break;
        default:
		  m_distance = 0.0;
		}
      return;
	}
  if (m_side != Side::left)
    return;

  if (m_strip == Gl_Road_Segment::LEFT_KERB)
    increment_kerb_distance (*(m_segment.mp_left_kerb));
  else if (m_strip == Gl_Road_Segment::RIGHT_KERB)
    increment_kerb_distance (*(m_segment.mp_right_kerb));
  else 
    {
      if (m_connection)
        {
          m_after_connection = true;
          m_connection = false;
          if ((m_strip == Gl_Road_Segment::LEFT_BARRIER)
              || (m_strip == Gl_Road_Segment::RIGHT_BARRIER)
              || (m_strip == Gl_Road_Segment::TRACK))
              glEnd ();
        }
      else
        {
          m_distance += m_resolution;
          if ((m_segment.pit_road_connection () > 0.0)
              && !m_after_connection
              && (m_distance >= m_segment.pit_road_connection ()))
            {
              m_distance = m_segment.pit_road_connection ();
              m_connection = true;
            }
          else
            {
              //  Clamp to the end of the segment.
              if (m_distance > (m_segment.length ()))
                {
                  m_distance = m_segment.length ();
                  m_position = END;
                }
            }
        }
    }
}

void 
Segment_Iterator::increment_kerb_distance (const Kerb& kerb)
{
  switch (m_position)
    {
    case BEGIN:
      m_distance = kerb.transition_start ();
      m_position = BEGIN_TRANSITION;
      break;
    case BEGIN_TRANSITION:
      m_distance = kerb.start ();
      m_position = MIDDLE;
      break;
    case MIDDLE:
      m_distance += m_resolution;
      if (m_distance >= kerb.end ())
        {
          m_distance = kerb.end ();
          m_position = END_TRANSITION;
        }
      break;
    case END_TRANSITION:
      m_distance = kerb.transition_end ();
      m_position = END;
      break;
    case END:
      assert (false);
    }
}

//=============================================================================
Gl_Road_Segment::Gl_Road_Segment 
(double resolution, 
 double length, 
 double radius, 
 double skew,
 const TPoints& left_width, 
 const TPoints& right_width, 
 const TPoints& left_road_width, 
 const TPoints& right_road_width, 
 Kerb* left_kerb, Kerb* right_kerb,
 double left_wall_height, double right_wall_height,
 const TPoints& elevation_points,
 double end_bank, 
 double bank_pivot,
 const std::vector <Material>& materials,
 const std::vector <Braking_Marker*>& braking_markers)
  : Road_Segment (length, radius, 10.0, 10.0, 20.0, 20.0),
    m_gl_texture_name (0),
    m_gl_list_id (0),
    m_texture_offsets (N_STRIPS),
    mp_iterator (new Segment_Iterator (*this, resolution))
{
  set_widths (right_width, right_road_width, left_road_width, left_width);

  set_start_skew (skew);
  set_end_skew (skew);

  set_kerb(left_kerb, Side::left);
  set_kerb(right_kerb, Side::right);

  set_wall_heights (left_wall_height, right_wall_height);
  set_elevation_points (elevation_points);

  assert (materials.size () == N_STRIPS);
  m_materials = materials;
  set_banking (end_bank, bank_pivot);

   for (std::vector <Braking_Marker*>::const_iterator 
		 it = braking_markers.begin ();
	   it != braking_markers.end ();
	   it++)
	{
	  m_braking_markers.push_back (*it);
	}

  add_textures ();
}

Gl_Road_Segment::~Gl_Road_Segment ()
{
  delete mp_iterator;
  delete [] m_gl_texture_name;

  for (std::vector <Braking_Marker*>::iterator 
		 it = m_braking_markers.begin ();
	   it != m_braking_markers.end ();
	   it++)
	{
	  delete *it;
	}

  glDeleteLists (m_gl_list_id, 1);
  for (std::vector <GLuint>::iterator it = m_scenery_lists.begin ();
	   it != m_scenery_lists.end ();
	   it++)
	{
	  glDeleteLists (*it, 1);
	}

  for (std::vector <Texture_Image*>::iterator 
         it = m_textures.begin ();
       it != m_textures.end ();
       it++)
    {
      delete *it;
    }
}

void
Gl_Road_Segment::add_textures ()
{
  for (std::vector <Material>::iterator 
         it = m_materials.begin ();
       it != m_materials.end ();
       it++)
    {
      if (it->texture_file_name ().empty ())
        m_textures.push_back (0);
      else
        m_textures.push_back (new Texture_Image (it->texture_file_name (),
                                                 it->smooth (),
                                                 it->mip_map (),
                                                 it->width (),
                                                 it->height ()));
    }
}

const Material& 
Gl_Road_Segment::left_material (double height) const 
{ 
  if (height < left_wall_height ())
    return *(m_materials.begin ()); 
  else
    return s_no_material;
}

const Material& 
Gl_Road_Segment::right_material (double height) const 
{ 
  if (height < right_wall_height ())
    return *(m_materials.end () - 1);
  else
    return s_no_material;
}


Material const& Gl_Road_Segment::material_at(double along, double from_center) const
{
    if (from_center > left_road_width (along) + kerb_width(Side::left, along))
        return m_materials[LEFT_SHOULDER];
    if (from_center > left_road_width (along))
        return m_materials[LEFT_KERB];
    if (from_center > -right_road_width (along))
        return m_materials[TRACK];
    if (from_center > -right_road_width (along) - kerb_width(Side::right, along))
        return m_materials[RIGHT_KERB];
    return m_materials[RIGHT_SHOULDER];
}

void
Gl_Road_Segment::build ()
{
  for (std::vector <GLuint>::iterator it = m_scenery_lists.begin ();
	   it != m_scenery_lists.end ();
	   it++)
	{
	  glDeleteLists (*it, 1);
	}
  m_scenery_lists.clear ();

  for (std::vector <Model_Info>::iterator it = m_models.begin ();
	   it != m_models.end ();
	   it++)
	{
	  Ac3d model (it->file, it->scale, it->translation, 
				  deg_to_rad (1.0) * it->rotation);
	  m_scenery_lists.push_back (model.build ());
	}

  glPixelStorei (GL_UNPACK_ALIGNMENT, 1);

  if (m_gl_list_id != 0)
	{
	  glDeleteLists (m_gl_list_id, 1);
	}
  m_gl_list_id = glGenLists (1);
  glNewList (m_gl_list_id, GL_COMPILE);

  // Put the textures on a shiny white surface so we can see shading.
  glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable (GL_COLOR_MATERIAL);
  glColor3f (1.0, 1.0, 1.0);
  GLfloat specular [] = { 1.0, 1.0, 1.0, 1.0 };
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  GLfloat shininess [] = { 128.0 };
  glMaterialfv (GL_FRONT_AND_BACK, GL_SHININESS, shininess);

  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  assert (m_texture_offsets.size () == N_STRIPS);

  // Draw the left barrier.
  draw_strip (LEFT_BARRIER, m_texture_offsets [LEFT_BARRIER]);

  // Draw the left shoulder and kerb.
  glDepthMask (GL_FALSE);
  draw_strip (LEFT_SHOULDER, m_texture_offsets [LEFT_SHOULDER]);
  glDepthMask (GL_TRUE);
  draw_strip (LEFT_KERB, m_texture_offsets [LEFT_KERB]);
  glColorMask (GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  draw_strip (LEFT_SHOULDER, m_texture_offsets [LEFT_SHOULDER]);
  glColorMask (GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  // Draw the track.
  draw_strip (TRACK, m_texture_offsets [TRACK]);

  // Draw the right shoulder and kerb.
  glDepthMask (GL_FALSE);
  draw_strip (RIGHT_SHOULDER, m_texture_offsets [RIGHT_SHOULDER]);
  glDepthMask (GL_TRUE);
  draw_strip (RIGHT_KERB, m_texture_offsets [RIGHT_KERB]);
  glColorMask (GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  draw_strip (RIGHT_SHOULDER, m_texture_offsets [RIGHT_SHOULDER]);
  glColorMask (GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  // Draw the right barrier.
  draw_strip (RIGHT_BARRIER, m_texture_offsets [RIGHT_BARRIER]);
  glFlush ();

  for (std::vector <Braking_Marker*>::const_iterator 
		 it = m_braking_markers.begin ();
	   it != m_braking_markers.end ();
	   it++)
	{
	  double along = length () - (*it)->distance ();
	  double from_center = (*it)->from_edge ();
 	  if ((*it)->side() == Side::right)
 		  from_center = -(from_center + right_road_width (along));
 	  else
 		  from_center += left_road_width (along) + (*it)->width ();

      using std::sin, std::cos;
	  double angle = start_angle () + arc () * along / length ();
	  auto x{center_of_curve().x - from_center * sin(angle)};
	  auto y{center_of_curve().y + from_center * cos(angle)};
	  if (is_straight ())
		{
            x += along * cos (angle);
		  y += along * sin (angle);
		}
	  else
		{
 		  x += radius () * sin (angle);
 		  y -= radius () * cos (angle);
		}
	  double z = elevation (along, from_center) + (*it)->off_ground ();

      glPushMatrix ();
      glTranslatef (x, y, z);
      glRotatef (rad_to_deg (angle) - 90.0, 0.0, 0.0, 1.0);
      glRotatef (90.0, 1.0, 0.0, 0.0);
      (*it)->draw ();
      glPopMatrix ();
    }

  glPushMatrix ();
  glTranslatef (start_coords ().x, start_coords ().y, start_coords ().z);
  glRotatef (rad_to_deg (start_angle ()), 0.0, 0.0, 1.0);
  std::for_each (m_scenery_lists.begin (), m_scenery_lists.end (), glCallList);
  glPopMatrix ();
  glEndList ();
}

void
Gl_Road_Segment::draw_strip (Strip strip, double texture_offset)
{
  if (m_textures [strip] == 0)
    return;

  m_textures [strip]->activate ();
  
  mp_iterator->start (start_coords (), start_angle (), strip);
  
  Three_Vector vertex = (++(*mp_iterator)).coordinates ();
  glNormal3d (mp_iterator->normal ().x, 
			  mp_iterator->normal ().y, 
			  mp_iterator->normal ().z);
  m_bounds.enclose(Rectangle<double>({vertex.x, vertex.y}, {vertex.x, vertex.y}));

  double tex_width = m_textures [strip]->width (); 
  double tex_height = m_textures [strip]->height ();
  double tex_x = (tex_width > 0.0)
	? mp_iterator->texture_coordinates ().x / tex_width : 0.0;
  double tex_y = (tex_height > 0.0)
	? mp_iterator->texture_coordinates ().y / tex_height : 0.0;
  tex_y += texture_offset;
  glTexCoord2d (tex_x, tex_y);
  glVertex3d (vertex.x, vertex.y, vertex.z);

  vertex = (++(*mp_iterator)).coordinates ();
  glNormal3d (mp_iterator->normal ().x, 
			  mp_iterator->normal ().y, 
			  mp_iterator->normal ().z);
  m_bounds.enclose({{vertex.x, vertex.y}, {vertex.x, vertex.y}});

  tex_x = (tex_width > 0.0)
	? mp_iterator->texture_coordinates ().x / tex_width : 1.0;
  glTexCoord2d (tex_x, tex_y);
  glVertex3d (vertex.x, vertex.y, vertex.z);
  
  while (!mp_iterator->last_subdivision ())
	{
      vertex = (++(*mp_iterator)).coordinates ();
	  glNormal3d (mp_iterator->normal ().x, 
				  mp_iterator->normal ().y, 
				  mp_iterator->normal ().z);
      m_bounds.enclose({{vertex.x, vertex.y}, {vertex.x, vertex.y}});

	  tex_x = (tex_width > 0.0)
		? mp_iterator->texture_coordinates ().x / tex_width : 0.0;
	  tex_y = (tex_height > 0.0)
		? mp_iterator->texture_coordinates ().y / tex_height : 1.0;
	  tex_y += texture_offset;
	  glTexCoord2d (tex_x, tex_y);
	  glVertex3d (vertex.x, vertex.y, vertex.z);

      vertex = (++(*mp_iterator)).coordinates ();
	  glNormal3d (mp_iterator->normal ().x, 
				  mp_iterator->normal ().y, 
				  mp_iterator->normal ().z);
      m_bounds.enclose({{vertex.x, vertex.y}, {vertex.x, vertex.y}});

	  tex_x = (tex_width > 0.0) ?
		mp_iterator->texture_coordinates ().x / tex_width : 1.0;
	  glTexCoord2d (tex_x, tex_y);
	  glVertex3d (vertex.x, vertex.y, vertex.z);
	} 

  glEnd ();

  m_texture_offsets [strip] = tex_y;
}

void
Gl_Road_Segment::draw () const
{
  glCallList (m_gl_list_id);
}

void
Gl_Road_Segment::set_braking_marker (Braking_Marker* marker)
{
  m_braking_markers.push_back (marker);
}

void 
Gl_Road_Segment::add_model_info (const Model_Info& info)
{
  m_models.push_back (info);
}

void
Gl_Road_Segment::set_start (const Three_Vector& start_coords, 
                            double start_distance,
                            double start_angle,
                            double start_bank,
                            const std::vector <double>& texture_offsets)
{
  Road_Segment::set_start (start_coords, 
                           start_distance,
                           start_angle, 
                           start_bank,
                           texture_offsets);
  m_texture_offsets = texture_offsets;
}
