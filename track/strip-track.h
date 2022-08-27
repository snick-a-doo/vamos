//  Copyright (C) 2001-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
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
#include "../media/texture-image.h"
#include "../media/xml-parser.h"
#include "gl-road-segment.h"

#include <exception>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace Vamos_Track
{
class Road_Segment;
class Braking_Marker;
class Road;

/// An exception thrown when the track can not be adjusted to make a circuit.
class Can_Not_Close : public std::runtime_error
{
public:
    Can_Not_Close(std::string const& reason)
        : std::runtime_error(reason)
    {}
};

/// Thrown when a road segment can't be found from the given coordinates.
class Segment_Not_Found : public std::exception
{
public:
    Segment_Not_Found(Vamos_Geometry::Three_Vector const& world_pos, size_t segment_index)
    {
        std::ostringstream os;
        os << world_pos << ", " << segment_index << "): not found";
        message = os.str();
    }
    virtual const char* what() const noexcept { return message.c_str(); }

private:
    std::string message;
};

//----------------------------------------------------------------------------------------
/// A trackside view.
struct Camera
{
    Camera(size_t segment_index,
           Vamos_Geometry::Three_Vector const& position,
           double range);
    Camera();

    /// The index of the road segment that the camera is on.
    size_t segment_index{0};
    /// The position of the camera relative to the segment.
    Vamos_Geometry::Three_Vector position;
    /// Activate the camera when the focused car is this distance away.
    double range{0.0};
    // The vertical field of view in degrees.
    double vertical_field_angle{10.0};
    /// If true, use the specified pan and tilt angles, otherwise follow the focused car.
    bool fixed{false};
    /// The pan and tilt angles when @p fixed is true.
    Vamos_Geometry::Two_Vector direction;
};

//----------------------------------------------------------------------------------------
/// Pointlike solid object on the track.
struct Track_Object
{
    Vamos_Geometry::Three_Vector position;
    Vamos_Geometry::Material material;
};

//----------------------------------------------------------------------------------------
/// A calculated fast path around the track.
class Racing_Line
{
    friend class Strip_Track_Reader;

public:
    Racing_Line();
    ~Racing_Line();

    /// @param along distance along the track
    /// @return world coordinates of the racing line at 'along'
    Vamos_Geometry::Three_Vector position(double along) const;

    /// @param along distance along the track
    /// @return vector that points from center of curvature the racing line at
    /// 'along'.  Its magnitude is 1/r to avoid infinities.
    Vamos_Geometry::Three_Vector curvature(double along, double offline_fraction) const;

    /// @param along distance along the track
    /// @return unit vector tangent to the racing line in the direction of travel.
    Vamos_Geometry::Three_Vector tangent(double along) const;

    void build(const Road& road, bool close);
    void draw() const;

    /// @return The distance from the center line to the left boundary of the racing
    /// line. I.e. the left road width minus the margin.
    double left_width(Road const& road, double along) const;
    /// @return The distance from the center line to the right boundary of the racing
    /// line. I.e. the right road width minus the margin.
    double right_width(Road const& road, double along) const;

private:
    void build_list(Road const& road);
    void propagate(Road const& road,
                   std::vector<Vamos_Geometry::Three_Vector>& points,
                   std::vector<Vamos_Geometry::Three_Vector>& velocities,
                   double interval);

    Vamos_Geometry::Three_Vector normal_curvature(Vamos_Geometry::Three_Vector const& p1,
                                                  Vamos_Geometry::Three_Vector const& p2,
                                                  Vamos_Geometry::Three_Vector const& p3) const;

    Vamos_Geometry::Three_Vector planar_curvature(Vamos_Geometry::Three_Vector const& p1,
                                                  Vamos_Geometry::Three_Vector const& p2,
                                                  Vamos_Geometry::Three_Vector const& p3) const;

    void force(Vamos_Geometry::Three_Vector const& p1,
               Vamos_Geometry::Three_Vector const& p2,
               Vamos_Geometry::Three_Vector const& p3,
               Vamos_Geometry::Three_Vector& f1,
               Vamos_Geometry::Three_Vector& f2,
               Vamos_Geometry::Three_Vector& f3);

    void load_curvature(double distance,
                        Vamos_Geometry::Three_Vector const& p1,
                        Vamos_Geometry::Three_Vector const& p2,
                        Vamos_Geometry::Three_Vector const& p3,
                        Road const& segment);

    double m_length{0.0};
    Vamos_Geometry::Vector_Spline m_line;
    Vamos_Geometry::Vector_Spline m_curvature;
    Vamos_Geometry::Vector_Spline m_left_curvature;
    Vamos_Geometry::Vector_Spline m_right_curvature;
    Vamos_Geometry::Vector_Spline m_tangent;
    GLuint m_list_id{0};
    /// Quit propagating the line after this many step.
    size_t m_iterations{500};
    /// Force per unit angle of bend.
    double m_stiffness{0.5};
    /// Negative force proportional to angular rate of change. Prevents oscillation.
    double m_damping{0.01};
    /// How much distance to leave between the racing line and the edge of the road.
    double m_margin{1.6};
    /// Spacing between nodes on the line.
    double m_resolution{0.0};
};

//----------------------------------------------------------------------------------------
using Segment_List = std::vector<Gl_Road_Segment*>;

/// A set of segments. May be open, closed, or connected to another road.
class Road
{
    friend class Strip_Track_Reader;

public:
    Road();

    /// Remove all segments.
    void clear();
    /// Add a segment to the end.
    size_t add_segment(Gl_Road_Segment* segment);
    /// Adjust the total length of the road.
    void set_length(double length);
    /// Set the orientation of the beginning of the road.
    void set_start_direction(double degrees);
    /// Adjust the completed road and get it ready for rendering.
    /// @param close If true, join the last segment to the first.
    /// @param adjusted_segments Number of segments that can change when joining: 0 to 3.
    /// @param length Scale the completed road to this value.
    void build(bool close, int adjusted_segments, double length);
    /// Render the road.
    void draw();

    /// @return A rectangle that bounds the road.
    const Vamos_Geometry::Rectangle& bounds() const { return m_bounds; }
    /// @return The vector of segments.
    const Segment_List& segments() const { return m_segments; }
    /// @return The elevation curve.
    const Vamos_Geometry::Spline& elevation() const { return m_elevation; }
    /// @return The length of the road.
    double length() const { return m_length; }
    /// @return The orientation of the start of the road.
    double start_direction() const { return m_start_direction; }

    /// @return The distance between two positions along the track. If closed, it's
    /// measured the shorter way around.
    double distance(double along1, double along2) const;

    /// @return The distance from the center to the left edge at @p along.
    double left_road_width(double along) const;
    /// @return The distance from the center to the right edge at @p along.
    double right_road_width(double along) const;
    /// @return The distance from the center to the left boundary of the racing line.
    double left_racing_line_width(double along) const;
    /// @return The distance from the center to the right boundary of the racing line.
    double right_racing_line_width(double along) const;

    /// @return The world position for the given track position.
    Vamos_Geometry::Three_Vector position(double along, double from_center,
                                          const Gl_Road_Segment& segment) const;
    Vamos_Geometry::Three_Vector position(double along, double from_center) const;

    // Return WORLD_POS transformed to the track's coordinate system.
    // SEGMENT_INDEX will be modified if the position on another
    // segment.
    Vamos_Geometry::Three_Vector track_coordinates(Vamos_Geometry::Three_Vector const& world_pos,
                                                   size_t& segment_index,
                                                   bool forward_only = false) const;
    const Gl_Road_Segment* segment_at(double along) const;

    const Racing_Line& racing_line() const { return m_racing_line; }

    /// Set the presence and visibility of the racing line.
    /// @param build Calculate the racing line if true.
    /// @param show Render the racing line if true.
    void set_racing_line(bool build, bool show);

    bool is_closed() const { return m_is_closed; }

protected:
    // Adjust the segment pointed to by the iterator and the following
    // two so that they meet the segment `joint'.
    void join(Vamos_Geometry::Three_Vector const& start_coords, double start_angle,
              Vamos_Geometry::Three_Vector const& end_coords, double end_angle,
              int adjusted_segments);

    void set_skews();

    double build_elevation(bool periodic);

    // Fill in the elevation curve and return the length of the road.
    void build_segments(Vamos_Geometry::Three_Vector start_coords, double start_angle,
                        double start_bank);

    Segment_List m_segments;
    Vamos_Geometry::Spline m_elevation;
    double m_length{0.0};

private:
    void connect(Segment_List::iterator it);
    void narrow_pit_segments();

    Vamos_Geometry::Rectangle m_bounds;
    double m_start_direction{0.0};
    Racing_Line m_racing_line;
    bool m_build_racing_line{false};
    bool m_show_racing_line{false};
    bool m_is_closed{false};
};

//----------------------------------------------------------------------------------------
/// A cube around the comera where the sky is rendered.
class Sky_Box
{
public:
    Sky_Box(double side_length, std::string const& sides_image,
            std::string const& top_image, std::string const& bottom_image, bool smooth);
    ~Sky_Box();

    void draw(Vamos_Geometry::Three_Vector const& view) const;

private:
    std::unique_ptr<Vamos_Media::Texture_Image> mp_sides;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_top;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_bottom;
    GLuint m_list_id;
};

//----------------------------------------------------------------------------------------
class Map_Background
{
public:
    Map_Background(std::string const& image_file_name,
                   double dx, double dy,
                   double width, double height);
    void draw() const;

private:
    std::unique_ptr<Vamos_Media::Texture_Image> mp_image;
    double m_dx;
    double m_dy;
    double m_width;
    double m_height;
};

//----------------------------------------------------------------------------------------
class Pit_Lane : public Road
{
public:
    void build(bool join_to_track, int adjusted_segments, Gl_Road_Segment& pit_in,
               Gl_Road_Segment& pit_out, Vamos_Geometry::Spline const& track_elevation);
    void set_end_direction(double degrees)
    { m_end_direction = Vamos_Geometry::deg_to_rad(degrees); }
    double end_direction() const { return m_end_direction; }

private:
    Vamos_Geometry::Three_Vector pit_in_offset(Gl_Road_Segment& pit_in) const;
    Vamos_Geometry::Three_Vector pit_out_offset(Gl_Road_Segment& pit_out) const;

    double m_end_direction;
};

//----------------------------------------------------------------------------------------
class Strip_Track
{
    friend class Strip_Track_Reader;

public:
    Strip_Track();

    // Read the track definition file.
    void read(std::string const& data_dir = "", std::string const& track_file = "");

    void set_start_direction(double degrees);
    double start_direction() const { return m_start_direction; }

    /// Set the presence and visibility of the racing line.
    /// @param build Calculate the racing line if true.
    /// @param show Render the racing line if true.
    void set_racing_line(bool build, bool show);

    // Add a straight or curved segment of road to the track.
    size_t add_segment(Gl_Road_Segment* segment);
    size_t add_pit_segment(Gl_Road_Segment* segment);
    void set_pit_in(size_t index, double angle);
    void set_pit_out(size_t index, double angle);

    // Draw the sky.
    void draw_sky(Vamos_Geometry::Three_Vector const& view) const;

    // Draw the background in map view.
    void draw_map_background() const;

    // Draw the track.
    void draw() const;

    // Make the track.
    void build(bool close, int adjusted_road_segments, double track_length, bool join_pit_lane,
               int adjusted_pit_segments);

    // Specify the images for the sky box.
    void set_sky_box(std::string sides_image, std::string top_image, std::string bottom_image,
                     bool smooth);

    // Specify the background image for map view.
    void set_map_background(std::string background_image, double x_offset, double y_offset,
                            double x_size, double y_size);

    // Add a sector timing line.
    void timing_line(double dist) { m_timing_lines.push_back(dist); }

    // Return the number of timing lines.
    size_t timing_lines() const { return m_timing_lines.size(); }

    void add_camera(const Camera& camera);
    const Camera& get_camera(double distance) const;

    // Return the world coordinates for the camera.
    Vamos_Geometry::Three_Vector camera_position(const Camera& camera) const;

    // Return a position in the camera's line of sight.
    Vamos_Geometry::Three_Vector camera_target(const Camera& camera) const;

    // Return the bounds of the track.
    const Vamos_Geometry::Rectangle& bounds() const;

    // Scale the track to a particular length.
    void set_length(double length) { mp_track->set_length(length); }

    // Return the new position for a vehicle at POS when a reset is
    // performed.
    Vamos_Geometry::Three_Vector reset_position(Vamos_Geometry::Three_Vector const& pos,
                                                size_t& road_index, size_t& segment_index);

    // Return the new orientation for a vehicle at POS when a reset is
    // performed.
    Vamos_Geometry::Three_Matrix reset_orientation(Vamos_Geometry::Three_Vector const& pos,
                                                   size_t& road_index, size_t& segment_index);

    // Return the elevation of the track at the x and y components of
    // POS.
    double elevation(Vamos_Geometry::Three_Vector const& pos, double x, double y,
                     size_t& road_index, size_t& segment_index);

    Vamos_Geometry::Contact_Info test_for_contact(Vamos_Geometry::Three_Vector const& pos,
                                                  double bump_parameter, size_t& road_index,
                                                  size_t& segment_index);

    const Road& get_road(size_t road_index) const;

    // Return WORLD_POS transformed to the track's coordinate system.
    // SEGMENT_INDEX will be modified if the position on another
    // segment.
    Vamos_Geometry::Three_Vector track_coordinates(Vamos_Geometry::Three_Vector const& world_pos,
                                                   size_t& road_index, size_t& segment_index);

    // Return the distance along the track for the given position.
    double from_center(Vamos_Geometry::Three_Vector const& pos, size_t& road_index,
                       size_t& segment_index);

    // Return the world position for the given track position.
    Vamos_Geometry::Three_Vector position(double along, double from_center) const;

    // Return the timing sector at the given distance.
    int sector(double distance);

    /// @return Objects that need collision checking.
    const std::vector<Track_Object>& objects() const { return m_objects; }

    const std::string& track_file() const { return m_track_file; }

    Vamos_Geometry::Three_Vector grid_position(int place, int total, bool pit) const;

private:
    double m_start_direction;

    std::vector<double> m_timing_lines;

    std::string m_data_dir;
    std::string m_track_file;

    Vamos_Geometry::Material m_material;

    // Return the distance along the track where the camera picks up a car.
    double camera_range(const Camera& camera) const;

    std::vector<Camera> m_cameras;

    std::unique_ptr<Road> mp_track;
    std::unique_ptr<Pit_Lane> mp_pit_lane;
    std::unique_ptr<Sky_Box> mp_sky_box;
    std::unique_ptr<Map_Background> mp_map_background;

    int m_pit_in_index{-1};
    int m_pit_out_index{-1};

    std::vector<Track_Object> m_objects;
};

//----------------------------------------------------------------------------------------
class Strip_Track_Reader : public Vamos_Media::XML_Parser
{
public:
    Strip_Track_Reader(std::string const& data_dir, std::string const& track_file,
                       Vamos_Track::Strip_Track* road);

private:
    void on_start_tag(Vamos_Media::XML_Tag const& tag);
    void on_end_tag(Vamos_Media::XML_Tag const& tag);
    void on_data(std::string data_string);

    std::string m_name;

    bool m_first_road;

    std::vector<double> m_doubles;
    std::vector<bool> m_bools;
    std::vector<std::string> m_strings;
    std::vector<Vamos_Geometry::Two_Vector> m_points;
    std::vector<Vamos_Geometry::Two_Vector> m_elev_points;
    std::vector<Vamos_Geometry::Two_Vector> m_point_vectors[4];
    std::vector<Vamos_Geometry::Two_Vector> m_left_profile;
    std::vector<Vamos_Geometry::Two_Vector> m_right_profile;
    Vamos_Geometry::Two_Vector m_line_adjust;
    std::vector<Braking_Marker*> m_braking_markers;

    // Materials indexed by name.
    std::map<std::string, Vamos_Geometry::Material> m_materials;

    std::vector<Vamos_Geometry::Material> m_segment_materials;

    // Groups of materials used on the segments, indexed by name.
    std::map<std::string, std::vector<Vamos_Geometry::Material>> m_segments;

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

    std::vector<Gl_Road_Segment::Model_Info> m_model_info;
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
};
} // namespace Vamos_Track

#endif
