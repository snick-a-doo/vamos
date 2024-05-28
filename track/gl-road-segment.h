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
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_TRACK_GL_ROAD_SEGMENT_H_INCLUDED
#define VAMOS_TRACK_GL_ROAD_SEGMENT_H_INCLUDED

#include "road-segment.h"

#include "../geometry/rectangle.h"
#include "../geometry/two-vector.h"
#include "../media/material.h"
#include "../media/model.h"

#include <GL/glu.h>

#include <memory>
#include <string>
#include <vector>

namespace Vamos_Media
{
class Texture_Image;
class Facade;
}

namespace Vamos_Track
{
/// A sign giving distance to the start of the next corner.
struct Braking_Marker
{
    Braking_Marker(std::string const& image_file, double distance, Side side,
                   Vamos_Geometry::Point<double> offset,
                   Vamos_Geometry::Point<double> size);

    std::shared_ptr<Vamos_Media::Facade> image; ///< The image on the sign.
    double distance; ///< Distance from the sign to the end of the segment.
    Side side; ///< The side of the road the sign is on.
    /// Distance from edge of road to edge of sign, and distance from edge of road to edge
    /// of sign, and distance from ground to bottom of sign.
    Vamos_Geometry::Point<double> offset;
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

    Gl_Road_Segment(double resolution, double length, double radius, double skew,
                    std::vector<Vamos_Geometry::Point<double>> const& left_width,
                    std::vector<Vamos_Geometry::Point<double>> const& right_width,
                    std::vector<Vamos_Geometry::Point<double>> const& left_road_width,
                    std::vector<Vamos_Geometry::Point<double>> const& right_road_width,
                    std::unique_ptr<Kerb> left_kerb, std::unique_ptr<Kerb> right_kerb,
                    double left_wall_height, double right_wall_height,
                    std::vector<Vamos_Geometry::Point<double>> const& elevation_points,
                    double end_bank, double bank_pivot,
                    std::vector<Vamos_Media::Material> const& materials,
                    std::vector<Braking_Marker> const& braking_markers);

    ~Gl_Road_Segment();

    std::vector<Vamos_Media::Material> const& materials() const { return m_materials; }

    void set_materials(std::vector<Vamos_Media::Material> const& materials, double resolution);

    void set_start(Vamos_Geometry::Three_Vector const& start_corrds, double start_distance,
                   double start_angle, double start_bank,
                   std::vector<double> const& texture_offsets);

    void build();
    void draw() const;

    void add_model(Vamos_Media::Model&& model);

    std::vector<double> texture_offsets() const { return m_texture_offsets; }

    virtual Vamos_Geometry::Rectangle<double> bounds() const { return m_bounds; }

    const Vamos_Media::Material& left_material(double height) const;
    const Vamos_Media::Material& right_material(double height) const;
    Vamos_Media::Material const& material_at(double along, double from_center) const;

private:
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

    GLuint m_gl_list_id{0};

    std::vector<GLuint> m_scenery_lists;

    std::vector<double> m_texture_offsets;

    std::unique_ptr<Segment_Iterator> mp_iterator;

    std::vector<Braking_Marker> m_braking_markers;

    std::vector<Vamos_Media::Material> m_materials;

    // bounding dimensions
    Vamos_Geometry::Rectangle<double> m_bounds;

    void draw_strip(Strip strip, double texture_offset);
};
} // namespace Vamos_Track

#endif // VAMOS_TRACK_GL_ROAD_SEGMENT_H_INCLUDED
