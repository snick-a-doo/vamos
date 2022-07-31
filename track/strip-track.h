//  Strip_Track.h - a constant-width track.
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

#ifndef _STRIP_TRACK_H_
#define _STRIP_TRACK_H_

#include "../geometry/contact-info.h"
#include "../geometry/material.h"
#include "../geometry/rectangle.h"
#include "../geometry/spline.h"
#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"
#include "../media/xml-parser.h"
#include "track.h"
#include "gl-road-segment.h"

#include <exception>
#include <map>
#include <string>
#include <vector>

namespace Vamos_Track
{
  class Sky_Box;
  class Map_Background;
  class Road_Segment;
  class Braking_Marker;
  class Road;

  // An exception thrown when the track can not be adjusted to make a circuit.
  class Can_Not_Close : public std::exception
  {
  public:
    Can_Not_Close (std::string reason = "")
    : m_reason (reason)
    {}

    ~Can_Not_Close () throw () {}

    virtual const char* what () const throw ()
    { return m_reason.c_str (); }

  private:
    std::string m_reason;
  };

  // Exception class
  class Segment_Not_Found : public std::exception
  {
  public:
	Segment_Not_Found (const Vamos_Geometry::Three_Vector& world_pos,
                       size_t segment_index) 
      : m_pos (world_pos),
        m_segment_index (segment_index)
    {};

    ~Segment_Not_Found () throw () {}

    virtual const char* what () const throw ()
    { 
      std::ostringstream os;
      os << m_pos << ", " << m_segment_index << "): not found";
      return os.str ().c_str ();
    }

  private:
	Vamos_Geometry::Three_Vector m_pos;
    size_t m_segment_index;
  };

  class Bad_Racing_Line_Length : public std::exception
  {
  public:
	Bad_Racing_Line_Length (double length)
      : m_length (length)
    {};

    ~Bad_Racing_Line_Length () throw () {}

    virtual const char* what () const throw ()
    { 
      std::ostringstream os;
      os << "Racing line length must be positive. (" << m_length << ")";
      return os.str ().c_str ();
    }

  private:
    double m_length;
  };

  class No_Racing_Line_Segments : public std::exception
  {
  public:
	No_Racing_Line_Segments (int n_segments)
      : m_n_segments (n_segments)
    {};

    ~No_Racing_Line_Segments () throw () {}

    virtual const char* what () const throw ()
    { 
      std::ostringstream os;
      os << "Number of segments must be positive. (" << m_n_segments << ")";
      return os.str ().c_str ();
    }

  private:
    int m_n_segments;
  };

  struct Camera
  {
    // The index of the road segment that the camera is on.
    size_t segment_index;
    // The position of the camera.
    Vamos_Geometry::Three_Vector position;
    // If true, use the specified pan and tilt angles, otherwise
    // follow the focused car.
    bool fixed;
    // The pan and tilt angles when fixed is true.
    Vamos_Geometry::Two_Vector direction;
    // The vertical field of view in degrees.
    double vertical_field_angle;
    // Activate the camera when the focused car is this distance away.
    double range;

    Camera (size_t segment_index_in,
            const Vamos_Geometry::Three_Vector& position_in,
            double range_in);

    Camera ();
  };

  struct Track_Object
  {
    Track_Object (const Vamos_Geometry::Three_Vector& position_in, 
                  const Vamos_Geometry::Material& material_in)
      : position (position_in),
        material (material_in)
    {};
    Vamos_Geometry::Three_Vector position;
    Vamos_Geometry::Material material;
  };

  class Racing_Line
  {
    friend class Strip_Track_Reader;

  public:
    Racing_Line ();
    ~Racing_Line ();

    /// @param along distance along the track
    /// @return world coordinates of the racing line at 'along'
    Vamos_Geometry::Two_Vector position (double along) const;

    /// @param along distance along the track
    /// @return vector that points from center of curvature the racing line at
    /// 'along'.  Its magnitude is 1/r to avoid infinities.
    Vamos_Geometry::Three_Vector curvature (double along, double offline_fraction) const;

    /// @param along distance along the track
    /// @return unit vector tangent to the racing line in the direction of travel.
    Vamos_Geometry::Three_Vector tangent (double along) const;

    void build (const Road& road, bool close);
    void draw () const;

      /// @return The distance from the center line to the left boundary of the racing
      /// line. I.e. the left road width minus the margin.
      double left_width(Road const& road, double along) const;
      /// @return The distance from the center line to the right boundary of the racing
      /// line. I.e. the right road width minus the margin.
      double right_width(Road const& road, double along) const;

  private:
    void build_list (const Road& road);
    void propagate (const Road& road,
                    std::vector <Vamos_Geometry::Three_Vector>& points,
                    std::vector <Vamos_Geometry::Three_Vector>& velocities,
                    double interval,
                    bool close);

    Vamos_Geometry::Three_Vector normal_curvature (const Vamos_Geometry::Three_Vector& p1,
                                                   const Vamos_Geometry::Three_Vector& p2,
                                                   const Vamos_Geometry::Three_Vector& p3) const;

    Vamos_Geometry::Three_Vector planar_curvature (const Vamos_Geometry::Three_Vector& p1,
                                                   const Vamos_Geometry::Three_Vector& p2,
                                                   const Vamos_Geometry::Three_Vector& p3) const;

    void force (const Vamos_Geometry::Three_Vector& p1,
                const Vamos_Geometry::Three_Vector& p2,
                const Vamos_Geometry::Three_Vector& p3,
                Vamos_Geometry::Three_Vector& f1,
                Vamos_Geometry::Three_Vector& f2,
                Vamos_Geometry::Three_Vector& f3);

    void load_curvature (double distance,
                         const Vamos_Geometry::Three_Vector& p1,
                         const Vamos_Geometry::Three_Vector& p2,
                         const Vamos_Geometry::Three_Vector& p3,
                         const Road& segment);

      double m_length{0.0};
      Vamos_Geometry::Parametric_Spline m_line;
      Vamos_Geometry::Vector_Spline m_curvature;
      Vamos_Geometry::Vector_Spline m_left_curvature;
      Vamos_Geometry::Vector_Spline m_right_curvature;
      Vamos_Geometry::Vector_Spline m_tangent;
      GLuint m_list_id{0};
      /// Quit propagating the line after this many step.
      size_t m_iterations;
      /// Force per unit angle of bend.
      double m_stiffness;
      /// Negative force proportional to angular rate of change. Prevents oscillation.
      double m_damping;
      /// How much distance to leave between the racing line and the edge of the road.
      double m_margin;
      /// Spacing between nodes on the line.
      double m_resolution{0.0};
  };

  typedef std::vector <Gl_Road_Segment*> Segment_List;

  class Road
  {
    friend class Strip_Track_Reader;

  public:
    Road ();
    ~Road ();

    void clear ();
    size_t add_segment (Gl_Road_Segment* segment);
    void set_length (double length);
    void set_start_direction (double degrees);
    void build (bool close, int adjusted_segments, double length);
    void draw ();

    const Vamos_Geometry::Rectangle& bounds () const { return m_bounds; }
    const Segment_List& segments () const { return m_segments; }
    const Vamos_Geometry::Spline& elevation () const { return *mp_elevation; }
    double length () const { return m_length; }
    double start_direction () const { return m_start_direction; }

    /// The distance between two positions along the track measured the shorter
    /// way around.
    double distance (double along1, double along2) const;

    double left_road_width (double along) const;
    double right_road_width (double along) const;
    double left_racing_line_width (double along) const;
    double right_racing_line_width (double along) const;

    // Return the world position for the given track position.
    Vamos_Geometry::Three_Vector position (double along, 
                                           double from_center, 
                                           const Gl_Road_Segment& segment) const;
    Vamos_Geometry::Three_Vector position (double along, 
                                           double from_center) const;

	// Return WORLD_POS transformed to the track's coordinate system.
	// SEGMENT_INDEX will be modified if the position on another
	// segment.
	Vamos_Geometry::Three_Vector 
	track_coordinates (const Vamos_Geometry::Three_Vector& world_pos,
					   size_t& segment_index,
                       bool forward_only = false) const;
    const Gl_Road_Segment* segment_at (double along) const;

    void build_racing_line ();

    const Racing_Line& racing_line () const { return m_racing_line; }

    void show_racing_line (bool show) { m_draw_racing_line = show; }

    bool is_closed () const { return m_is_closed; }

  protected:
    // Adjust the segment pointed to by the iterator and the following
    // two so that they meet the segment `joint'.
	void join (const Vamos_Geometry::Three_Vector& start_coords,
               double start_angle,
               const Vamos_Geometry::Three_Vector& end_coords,
               double end_angle,
               int adjusted_segments);

    void set_skews ();

    double build_elevation (bool periodic);

    // Fill in the elevation curve and return the length of the road.
    void build_segments (Vamos_Geometry::Three_Vector start_coords,
                         double start_angle,
                         double start_bank);

	Segment_List m_segments;
	Vamos_Geometry::Spline* mp_elevation;
	double m_length;

  private:
    void connect (Segment_List::iterator it);
    void narrow_pit_segments ();

    Vamos_Geometry::Rectangle m_bounds;
    double m_start_direction;
    Racing_Line m_racing_line;
    bool m_draw_racing_line;
    bool m_is_closed;
  };

  class Pit_Lane : public Road
  {
  public:
    void build (bool join_to_track,
                int adjusted_segments,
                Gl_Road_Segment& pit_in,
                Gl_Road_Segment& pit_out,
                const Vamos_Geometry::Spline& track_elevation);
    void set_end_direction (double degrees)
    { m_end_direction = Vamos_Geometry::deg_to_rad (degrees);}
    double end_direction () const { return m_end_direction; }

  private:
    Vamos_Geometry::Three_Vector 
    pit_in_offset (Gl_Road_Segment& pit_in) const;
    Vamos_Geometry::Three_Vector 
    pit_out_offset (Gl_Road_Segment& pit_out) const;

    double m_end_direction;
  };

  class Strip_Track : public Track
  {
    friend class Strip_Track_Reader;

    double m_start_direction;

	std::vector <double> m_timing_lines;

	std::string m_data_dir;
	std::string m_track_file;

	Vamos_Geometry::Material m_material;

    // Return the distance along the track where the camera picks up a car.
    double camera_range (const Camera& camera) const;

    std::vector <Camera> m_cameras;

    Road* mp_track;
    Pit_Lane* mp_pit_lane;

    int m_pit_in_index;
    int m_pit_out_index;

    Sky_Box* mp_sky_box;
    Map_Background* mp_map_background;

    std::vector <Track_Object> m_objects;

  public:
	Strip_Track ();
	virtual ~Strip_Track ();

	// Read the track definition file.
	virtual void read (std::string data_dir = "", std::string track_file = "");

    void set_start_direction (double degrees);
    double start_direction () const { return m_start_direction; }

    void show_racing_line (bool show);

	// Add a straight or curved segment of road to the track.
	size_t add_segment (Gl_Road_Segment* segment); 
	size_t add_pit_segment (Gl_Road_Segment* segment);
    void set_pit_in (size_t index, double angle);
    void set_pit_out (size_t index, double angle);

	// Draw the sky.
	void draw_sky (const Vamos_Geometry::Three_Vector& view) const;

    // Draw the background in map view.
    void draw_map_background () const;

	// Draw the track.
	void draw () const;

	// Make the track.
	void build (bool close, int adjusted_road_segments, 
                double track_length, 
                bool join_pit_lane, int adjusted_pit_segments);

    // Specify the images for the sky box.
	void set_sky_box (std::string sides_image, 
                      std::string top_image,
                      std::string bottom_image,
                      bool smooth);

    // Specify the background image for map view. 
    void set_map_background (std::string background_image,
                             double x_offset, double y_offset,
                             double x_size, double y_size);

	// Add a sector timing line.
	void timing_line (double dist) { m_timing_lines.push_back (dist); }

	// Return the number of timing lines.
	size_t timing_lines () const { return m_timing_lines.size (); }

    void add_camera (const Camera& camera);
    const Camera& get_camera (double distance) const;

    // Return the world coordinates for the camera.
    Vamos_Geometry::Three_Vector
    camera_position (const Camera& camera) const;

    // Return a position in the camera's line of sight.
    Vamos_Geometry::Three_Vector
    camera_target (const Camera& camera) const;

	// Return the bounds of the track.
    const Vamos_Geometry::Rectangle& bounds () const;

	// Scale the track to a particular length.
	void set_length (double length) { mp_track->set_length (length); }

	// Return the new position for a vehicle at POS when a reset is
	// performed. 
	Vamos_Geometry::Three_Vector 
	reset_position (const Vamos_Geometry::Three_Vector& pos,
					size_t& road_index,
					size_t& segment_index);

	// Return the new orientation for a vehicle at POS when a reset is
	// performed. 
	Vamos_Geometry::Three_Matrix
	reset_orientation (const Vamos_Geometry::Three_Vector& pos,
                       size_t& road_index,
					   size_t& segment_index);

	// Return the elevation of the track at the x and y components of
	// POS.
	double elevation (const Vamos_Geometry::Three_Vector& pos,
                      double x,
                      double y,
					  size_t& road_index,
					  size_t& segment_index);

	Vamos_Geometry::Contact_Info 
	test_for_contact (const Vamos_Geometry::Three_Vector& pos,
					  double bump_parameter,
					  size_t& road_index,
					  size_t& segment_index);

    const Road& get_road (size_t road_index) const;

	// Return WORLD_POS transformed to the track's coordinate system.
	// SEGMENT_INDEX will be modified if the position on another
	// segment.
	Vamos_Geometry::Three_Vector 
	track_coordinates (const Vamos_Geometry::Three_Vector& world_pos,
					   size_t& road_index,
					   size_t& segment_index);

	// Return the distance along the track for the given position.
	double from_center (const Vamos_Geometry::Three_Vector& pos,
                     size_t& road_index,
					 size_t& segment_index);

    // Return the world position for the given track position.
    Vamos_Geometry::Three_Vector position (double along, 
                                           double from_center) const;

	// Return the timing sector at the given distance.
	int sector (double distance);

    // Return object that need collision checking.
    const std::vector <Track_Object>& objects () const { return m_objects; }

    void build_racing_line () { mp_track->build_racing_line (); }

    const std::string& track_file () const { return m_track_file; }

    Vamos_Geometry::Three_Vector grid_position (int place, 
                                                int total,
                                                bool pit) const;
  };

  class Strip_Track_Reader : public Vamos_Media::XML_Parser
  {
	void on_start_tag (const Vamos_Media::XML_Tag& tag); 
	void on_end_tag (const Vamos_Media::XML_Tag& tag); 
	void on_data (std::string data_string);

	std::string m_name;

	bool m_first_road;

	std::vector <double> m_doubles;
	std::vector <bool> m_bools;
	std::vector <std::string> m_strings;
	std::vector <Vamos_Geometry::Two_Vector> m_points;
	std::vector <Vamos_Geometry::Two_Vector> m_elev_points;
	std::vector <Vamos_Geometry::Two_Vector> m_point_vectors [4];
	std::vector <Vamos_Geometry::Two_Vector> m_left_profile;
	std::vector <Vamos_Geometry::Two_Vector> m_right_profile;
	Vamos_Geometry::Two_Vector m_line_adjust;
	std::vector <Braking_Marker*> m_braking_markers;

	// Materials indexed by name.
	std::map <std::string, Vamos_Geometry::Material> m_materials;

	std::vector <Vamos_Geometry::Material> m_segment_materials;

	// Groups of materials used on the segments, indexed by name.
	std::map <std::string, 
			  std::vector <Vamos_Geometry::Material> > m_segments;

	std::string m_data_dir;
	Vamos_Track::Strip_Track* mp_road;
	
	enum Classes
	  {
		NONE,
		MATERIAL,
		ROAD,
		SEGMENT
	  };
	
	Classes m_class;
    bool m_close;
	int m_adjusted_road_segments;
	bool m_join_pit_lane;
    double m_length;

	Vamos_Geometry::Material::Material_Type m_material_type;
	Vamos_Geometry::Two_Vector m_bump_amplitude;

    std::vector <Gl_Road_Segment::Model_Info> m_model_info;
    Gl_Road_Segment::Model_Info m_current_model_info;

    Camera m_camera;

    double m_split_or_join;
    double m_merge;
    double m_angle;
    Vamos_Geometry::Direction m_pit_side;
    bool m_pit_in_active;
    bool m_pit_out_active;
    int m_adjusted_pit_segments;

  public:
	Strip_Track_Reader (std::string data_dir, 
						std::string track_file, 
						Vamos_Track::Strip_Track* road);
  };
}

#endif
