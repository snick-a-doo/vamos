//  Copyright (C) 2001-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "strip-track.h"
#include "../geometry/numeric.h"
#include "../geometry/parameter.h"
#include "../geometry/spline.h"
#include "../media/texture-image.h"

#include <GL/glu.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;

Camera const default_camera{0, {100.0, -20.0, 10.0}, 0.0, 10.0, false, {0.0, 0.0}};

auto make_sky_tex(std::string const& image, bool smooth)
{
    return std::make_unique<Texture_Image>(image, smooth, false, 1.0, 1.0, GL_CLAMP_TO_EDGE);
}

Sky_Box::Sky_Box(double side_length, std::string const& sides_image,
                 std::string const& top_image, std::string const& bottom_image, bool smooth)
    // Clamp the textures to aviod showing seams.
    : mp_sides(std::move(make_sky_tex(sides_image, smooth))),
      mp_top(std::move(make_sky_tex(top_image, smooth))),
      mp_bottom(std::move(make_sky_tex(bottom_image, smooth))),
      m_list_id(glGenLists(1))
{
    auto const x{side_length / 2.0};

    glNewList(m_list_id, GL_COMPILE);
    glColor3f(1.0, 1.0, 1.0);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    mp_sides->activate();

    // front
    glBegin(GL_QUAD_STRIP);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(x, x, x);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(x, x, -x);
    glTexCoord2d(0.25, 0.0);
    glVertex3d(x, -x, x);
    glTexCoord2d(0.25, 1.0);
    glVertex3d(x, -x, -x);

    // right
    glTexCoord2d(0.25, 0.0);
    glVertex3d(x, -x, x);
    glTexCoord2d(0.25, 1.0);
    glVertex3d(x, -x, -x);
    glTexCoord2d(0.5, 0.0);
    glVertex3d(-x, -x, x);
    glTexCoord2d(0.5, 1.0);
    glVertex3d(-x, -x, -x);

    // back
    glTexCoord2d(0.5, 0.0);
    glVertex3d(-x, -x, x);
    glTexCoord2d(0.5, 1.0);
    glVertex3d(-x, -x, -x);
    glTexCoord2d(0.75, 0.0);
    glVertex3d(-x, x, x);
    glTexCoord2d(0.75, 1.0);
    glVertex3d(-x, x, -x);

    // left
    glTexCoord2d(0.75, 0.0);
    glVertex3d(-x, x, x);
    glTexCoord2d(0.75, 1.0);
    glVertex3d(-x, x, -x);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(x, x, x);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(x, x, -x);
    glEnd();

    // top
    mp_top->activate();

    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(-x, x, x);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(x, x, x);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(x, -x, x);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(-x, -x, x);
    glEnd();

    // bottom
    mp_bottom->activate();

    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(x, x, -x);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(-x, x, -x);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(-x, -x, -x);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(x, -x, -x);
    glEnd();

    glFlush();
    glEndList();
}

Sky_Box::~Sky_Box()
{
    glDeleteLists(m_list_id, 1);
}

void Sky_Box::draw(Three_Vector const& view) const
{
    glLoadIdentity();
    glTranslatef(view.x, view.y, view.z);
    glCallList(m_list_id);
    // Clear the depth buffer to keep the sky behind everything else.
    glClear(GL_DEPTH_BUFFER_BIT);
}

//----------------------------------------------------------------------------------------
Map_Background::Map_Background(std::string const& image_file_name,
                               Vamos_Geometry::Rectangle<int> const& size)
    : m_image{image_file_name, true, false},
      m_size{size}
{
}

void Map_Background::draw() const
{
    glColor3f(1.0, 1.0, 1.0);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    m_image.activate();

    glLoadIdentity();
    glTranslatef(m_size.left(), m_size.bottom(), 0.0);

    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(m_size.left(), m_size.bottom(), 0.0);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(m_size.left(), m_size.top(), 0.0);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(m_size.right(), m_size.top(), 0.0);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(m_size.right(), m_size.bottom(), 0.0);
    glEnd();

    // Clear the depth buffer to keep the background behind the track.
    glClear(GL_DEPTH_BUFFER_BIT);
}

//----------------------------------------------------------------------------------------
Racing_Line::Racing_Line()
    : m_iterations{500},
      m_stiffness{0.5},
      m_damping{0.01},
      m_margin{1.6}
{
}

Racing_Line::~Racing_Line()
{
    glDeleteLists(m_list_id, 1);
}

Three_Vector Racing_Line::position(double along) const
{
    return m_line.interpolate(along);
}

Three_Vector Racing_Line::curvature(double along, double offline) const
{
    along = wrap(along, m_length);
    auto c1{m_curvature.interpolate(along)};
    auto c2{(offline > 0.0 ? m_left_curvature : m_right_curvature).interpolate(along)};
    // Linearly interpolate from line to edge.
    auto f{std::abs(offline)};
    return {interpolate(f, 0.0, c1.x, 1.0, c2.x),
            interpolate(f, 0.0, c1.y, 1.0, c2.y),
            interpolate(f, 0.0, c1.z, 1.0, c2.z)};
}

Three_Vector Racing_Line::tangent(double along) const
{
    return m_tangent.interpolate(wrap(along, m_length));
}

static Three_Vector normal_curvature(Three_Vector const& p1,
                                     Three_Vector const& p2,
                                     Three_Vector const& p3)
{
    auto r21{p1 - p2};
    auto r23{p3 - p2};
    auto up{r23.cross(r21)};
    // Assume the angle is small so that sin x = x.
    return up / (r21.dot(r21) * r23.magnitude());
}

static Three_Vector planar_curvature(Three_Vector const& p1,
                                     Three_Vector const& p2,
                                     Three_Vector const& p3)
{
    return normal_curvature(p1, p2, p3).magnitude() * (p2 - (p1 + p3) / 2.0).unit();
}

/// Calculate the force on adjacent nodes.
static void force(Three_Vector const& p1, Three_Vector const& p2,
                  Three_Vector const& p3, double stiffness,
                  Three_Vector& f1, Three_Vector& f2, Three_Vector& f3)
{
    auto r21{p1 - p2};
    auto r23{p3 - p2};
    auto curvature{normal_curvature(p1, p2, p3)};
    auto df1{stiffness * curvature.cross(r21)};
    auto df3{-stiffness * curvature.cross(r23)};

    f1 += df1;
    f2 -= (df1 + df3);
    f3 += df3;
}

double Racing_Line::right_width(Road const& road, double along) const
{
    return road.right_racing_line_width(along) - m_margin;
}

double Racing_Line::left_width(Road const& road, double along) const
{
    return road.left_racing_line_width(along) - m_margin;
}

void Racing_Line::propagate(Road const& road,
                            std::vector<Three_Vector>& positions,
                            std::vector<Three_Vector>& velocites,
                            double interval)
{
    auto const points{positions.size()};
    std::vector<Three_Vector> forces(points);

    force(positions.back(), positions[0], positions[1], m_stiffness,
          forces.back(), forces[0], forces[1]);
    for (size_t i{1}; i < points - 1; ++i)
        force(positions[i - 1], positions[i], positions[i + 1], m_stiffness,
              forces[i - 1], forces[i], forces[i + 1]);
    force(positions[points - 2], positions.back(), positions[0], m_stiffness,
          forces[points - 2], forces.back(), forces[0]);

    size_t index{0};
    for (size_t i{0}; i < points; ++i)
    {
        velocites[i] += forces[i] - velocites[i] * m_damping;
        positions[i] += velocites[i];

        // Constrain the racing line to the track.
        auto along{i * interval};
        auto across{clip(road.track_coordinates(positions[i], index).y,
                         -right_width(road, along), left_width(road, along))};
        positions[i] = road.position(along, across);
    }
}

void Racing_Line::build(Road const& road, bool close)
{
    m_length = road.length();
    assert (m_length > 0.0);

    // Divide the track into the smallest number of equal intervals not longer than
    // max_interval. m_resolution may be set in the track file. If not, calculate an
    // interval from the road width minus margin.
    auto interval{m_resolution > 0.0
                  ? m_resolution
                  : 2.0 * (left_width(road, 0.0) + right_width(road, 0.0))};
    assert(interval > 0);
    auto const divisions{static_cast<int>(std::ceil(m_length / interval))};
    assert(divisions > 0);
    interval = m_length / divisions;

    // Put a node at each interval. Include one at 0, but not at m_length because that's
    // just past the end. If closed, 0 and m_length are the same. Start with the nodes in
    // the center of the track.
    std::vector<Three_Vector> positions;
    for (int node{0}; node < divisions; ++node)
        positions.push_back(road.position(node * interval, 0.0));

    std::vector<Three_Vector> velocities(positions.size());
    for (size_t i{0}; i < m_iterations; ++i)
        propagate(road, positions, velocities, interval);

    // These members are filled by load_curvature()>
    m_line.clear();
    m_curvature.clear();
    m_left_curvature.clear();
    m_right_curvature.clear();
    m_tangent.clear();

    // Find the curvature for each group of 3 consecutive nodes. We can find the curvature
    // at 0 and m_length - interval only if the track is closed.
    if (close)
        load_curvature(0, positions.back(), positions[0], positions[1], road);
    for (size_t i{0}, j{1}, k{2}; k < positions.size(); ++i, ++j, ++k)
        load_curvature(j * interval, positions[i], positions[j], positions[k], road);
    if (close)
    {
        load_curvature(m_length - interval, *std::prev(positions.end(), 2), positions.back(),
                       positions.front(), road);
        m_line.set_periodic(m_length);
        m_curvature.set_periodic(m_length);
        m_left_curvature.set_periodic(m_length);
        m_right_curvature.set_periodic(m_length);
        m_tangent.set_periodic(m_length);
    }

    build_list(road);
}

void Racing_Line::build_list(Road const& road)
{
    if (m_list_id != 0)
        glDeleteLists(m_list_id, 1);

    m_list_id = glGenLists(1);
    glNewList(m_list_id, GL_COMPILE);

    glDisable(GL_TEXTURE_2D);
    glLineWidth(2.0);

    glBegin(GL_LINE_STRIP);
    auto last_world{position(0.0)};
    for (auto along{0.0}; along < m_length; along += 0.1)
    {
        auto world{position(along)};
        auto forward{(world - last_world).unit()};
        auto curve{curvature(along, 0.0)};
        auto color{100.0 * curve.magnitude()};
        if (curve.cross(forward).z < 0.0)
            color *= -1.0;
        glColor4f(1.0 - color, 1.0 + color, 1.0, 0.5);
        // Draw the line a little above the road.
        glVertex3d(world.x, world.y, road.segment_at(along).world_elevation(world) + 0.05);
        last_world = world;
    }
    glEnd();

    glPointSize(4.0);
    glColor4f(0.8, 0.0, 0.0, 0.5);
    glBegin(GL_POINTS);
    // Vector spline does not have an iterator.
    for (size_t i{0}; i < m_line.size(); ++i)
    {
        auto const& world{m_line[i]};
        // Draw a dot a little above the line.
        glVertex3d(world.x, world.y, world.z + 0.06);
    }
    glEnd();

    glEnable(GL_TEXTURE_2D);
    glEndList();
}

void Racing_Line::load_curvature(double along, Three_Vector const& p1,
                                 Three_Vector const& p2, Three_Vector const& p3,
                                 Road const& road)
{
    auto const& segment{road.segment_at(along)};
    m_line.load(along, p2);

    m_tangent.load(along, (p3 - p1).unit());

    auto const factor{segment.racing_line_curvature_factor()};
    m_curvature.load(along, factor * planar_curvature(p1, p2, p3));

    if (segment.radius() == 0.0)
    {
        m_left_curvature.load(along, null_v);
        m_right_curvature.load(along, null_v);
    }
    else
    {
        auto across{segment.left_racing_line_width(along)};
        auto p1{road.position(along - 10, across)};
        auto p2{road.position(along, across, segment)};
        auto p3{road.position(along + 10, across)};
        m_left_curvature.load(along, planar_curvature(p1, p2, p3));

        across = segment.right_racing_line_width(along);
        p1 = road.position(along - 10, across);
        p2 = road.position(along, across, segment);
        p3 = road.position(along + 10, across);
        m_right_curvature.load(along, planar_curvature(p1, p2, p3));
    }
}

void Racing_Line::draw() const
{
    glCallList(m_list_id);
}

//----------------------------------------------------------------------------------------
Road::Road()
{
    m_elevation.load({0.0, 0.0});
}

void Road::clear()
{
    m_elevation.replace({{0.0, 0.0}});
    m_length = 0.0;
    m_bounds = Rectangle<double>();
    m_segments.clear();
}

size_t Road::add_segment(std::unique_ptr<Gl_Road_Segment> segment)
{
    if (!m_segments.empty())
    {
        auto const& last{*m_segments.back()};
        segment->set_start(last.end_coords(), last.end_distance(), last.end_angle(), 0.0,
                           last.texture_offsets());
    }
    m_length += segment->length();
    m_segments.push_back(std::move(segment));
    return m_segments.size();
}

void Road::set_length(double length)
{
    assert(!m_segments.empty());
    assert(m_length > 0.0);
    for (auto& seg : m_segments)
        seg->scale(length / m_length);
}

void Road::set_start_direction(double degrees)
{
    m_start_direction = branch(deg_to_rad(degrees), 0.0);
    if (m_segments.empty())
        return;
    m_segments.front()->set_start_angle(m_start_direction);
    connect(m_segments.begin() + 1);
}

double Road::build_elevation(bool periodic)
{
    auto length{0.0};
    for (auto& seg : m_segments)
    {
        seg->build_elevation(&m_elevation, length);
        length += seg->length();
    }
    if (periodic)
        m_elevation.set_periodic(length);
    return length;
}

void Road::build_segments(Three_Vector start_coords, double start_angle, double start_bank)
{
    auto const& first{*m_segments.front()};
    std::vector<double> texture_offsets(first.materials().size());

    m_length = 0.0;
    for (auto& seg : m_segments)
    {
        seg->set_start(start_coords, m_length, start_angle, start_bank, texture_offsets);
        seg->build();
        // Update the bounding dimensions.
        m_bounds.enclose(seg->bounds());
        m_length += seg->length();
        start_coords = seg->end_coords();
        start_angle = seg->end_angle();
        start_bank = seg->banking().end_angle();
        texture_offsets = seg->texture_offsets();
    }
}

Three_Vector Road::position(double along, double from_center,
                            Gl_Road_Segment const& segment) const
{
    return segment.position(along - segment.start_distance(), from_center);
}

Three_Vector Road::position(double along, double from_center) const
{
    along = wrap(along, m_length);
    return position(along, from_center, segment_at(along));
}

Three_Vector Road::track_coordinates(Three_Vector const& world_pos,
                                     size_t& segment_index, bool forward_only) const
{
    assert(segment_index < m_segments.size());
    // Find the distance along the track, distance from center, and elevation for the
    // world coordinates `world_pos.x' and `world_pos.y'.
    Three_Vector track_pos;
    auto seg_it{m_segments.begin() + segment_index};
    for (size_t i{0}; ; ++i)
    {
        auto off{(*seg_it)->coordinates(world_pos, track_pos)};
        if (std::abs(off) < 1.0e-6)
            break;
        if (forward_only || off > 0.0)
        {
            // We're off the end of the current segment. Find a new candidate segment.
            if (seg_it + 1 == m_segments.end())
            {
                if (!m_is_closed)
                    break;
                seg_it = m_segments.begin();
            }
            ++seg_it;
        }
        else
        {
            // Try the previous segment.
            if (seg_it == m_segments.begin())
            {
                if (!m_is_closed)
                    break;
                seg_it = m_segments.end();
            }
            --seg_it;
        }
        if (i > m_segments.size())
            throw Segment_Not_Found(world_pos, segment_index);
    }

    // Throw an exception if a segment could not be found.
    segment_index = std::distance(m_segments.begin(), seg_it);
    return track_pos + (*seg_it)->start_distance() * x_hat;
}

double Road::distance(double along1, double along2) const
{
    if (!m_is_closed)
        return along1 - along2;
    auto limit{0.5 * m_length};
    return wrap(along1 - along2, -limit, limit);
}

double Road::left_road_width(double along) const
{
    return segment_at(along).left_road_width(along);
}

double Road::right_road_width(double along) const
{
    return segment_at(along).right_road_width(along);
}

double Road::right_racing_line_width(double along) const
{
    return segment_at(along).right_racing_line_width(along);
}

double Road::left_racing_line_width(double along) const
{
    return segment_at(along).left_racing_line_width(along);
}

Gl_Road_Segment const& Road::segment_at(double along) const
{
    auto distance{0.0};
    for (auto const& seg : m_segments)
    {
        distance += seg->length();
        if (distance >= along)
            return *seg;
    }
    return *m_segments.back();
}

void Road::set_racing_line(bool build, bool show)
{
    assert((build || !show) && "The racing line must be built if it is shown.");
    m_build_racing_line = build;
    m_show_racing_line = show;
}

void Road::narrow_pit_segments()
{
    Gl_Road_Segment* last_from_out{nullptr};
    Gl_Road_Segment* last_from_in{nullptr};

    for (auto it = m_segments.begin(); it != m_segments.end(); ++it)
    {
        const auto& pit{(*it)->pit()};
        if (!pit.active())
            continue;
        if (pit.end() == Pit_Lane_Transition::End::out)
        {
            for (Segment_List::reverse_iterator rit{it};
                 rit != m_segments.rend()
                     && rit->get() != last_from_in
                     && !(*rit)->pit().active();
                 ++rit)
            {
                (*rit)->narrow(pit.side(), (*it)->pit_width());
                last_from_out = rit->get();
            }
        }
        else
        {
            for (auto it2(it + 1);
                 it2 != m_segments.end()
                     && it2->get() != last_from_out
                     && !(*it2)->pit().active();
                 ++it2)
            {
                (*it2)->narrow(pit.side(), (*it)->pit_width());
                last_from_in = it2->get();
            }
        }
    }
}

void Road::build(bool close, int adjusted_segments, double length)
{
    narrow_pit_segments();
    set_skews();

    auto& first{*m_segments.front()};
    auto& last{*m_segments.back()};

    if (close)
    {
        join(first.start_coords(), first.start_angle(),
             first.start_coords(),first.start_angle(), adjusted_segments);
        // Force the segment to end at 0, 0.
        last.set_last_segment(true);
    }
    if (length != 0.0)
        set_length(length);

    // Make sure the walls join.
    last.set_left_width(last.length(), first.left_width(0.0));
    last.set_right_width(last.length(), first.right_width(0.0));

    build_elevation(m_is_closed);
    build_segments(Three_Vector(), start_direction(), close ? last.banking().end_angle() : 0.0);
    if (m_build_racing_line)
        m_racing_line.build(*this, m_is_closed);
}

double perpendicular_distance(Three_Vector const& p1, Three_Vector const& p2, double angle)
{
    return (p2 - p1).magnitude() * sin(atan2(p1.y - p2.y, p1.x - p2.x) - angle);
}

void Road::join(Three_Vector const&, // start_coords,
                double,              // start_angle
                Three_Vector const& end_coords, double end_angle, int adjusted_segments)
{
    m_is_closed = true;

    if (adjusted_segments < 0 || adjusted_segments > 3)
    {
        std::ostringstream message;
        message << "The number of segments to be adjusted (" << adjusted_segments
                << ") is not in the range [0, 3]";
        throw Can_Not_Close(message.str());
    }
    if (m_segments.size() < size_t(adjusted_segments))
    {
        std::ostringstream message;
        message << "Track has fewer segments (" << m_segments.size()
                << ") than the number of segments to be adjusted (" << adjusted_segments << ")";
        throw Can_Not_Close(message.str());
    }
    if (adjusted_segments == 0)
        // Call it closed without adjusting anything.
        return;

    auto* last_segment{m_segments.back().get()};
    auto* last_curve{adjusted_segments > 1 ? (m_segments.end() - 2)->get()
                     : last_segment->is_straight() ? nullptr : last_segment};
    auto* other_straight{adjusted_segments == 3 ? (m_segments.end() - 3)->get() : nullptr};

    if (adjusted_segments > 1 && (last_curve->is_straight() || !last_segment->is_straight()))
        throw Can_Not_Close("Track must end with a curve followed by "
                            "a straight when more than one segment "
                            "is to be adjusted.");
    if (adjusted_segments == 3 && !other_straight->is_straight())
        throw Can_Not_Close("Track must end with a straight, a curve and a "
                            "straight when three segments are to be adjusted.");

    // Make the last segment parallel to the first by changing the length of the last
    // curve.
    auto last_arc{0.0};
    if (last_curve)
    {
        last_arc = last_curve->arc() + branch(end_angle - last_curve->end_angle(), -pi);
        last_curve->set_arc(last_arc);
        // If we're only adjusting the last curve, we're done.
        if (last_segment == last_curve)
            return;
    }
    if (adjusted_segments > 1)
    {
        // Make the last segment colinear with the first.
        const auto perp{perpendicular_distance(last_curve->end_coords(), end_coords, end_angle)};
        switch (adjusted_segments)
        {
        case 2:
            // Change the radius of the curve.
            last_curve->set_radius(last_curve->radius() + perp / (1.0 - cos(last_arc)));
            break;
        case 3:
            // Change the length of segment[-3].
            other_straight->set_length(other_straight->length() + perp / sin(last_arc));
            break;
        default:
            // 2 and 3 should be the only possibilities in this branch.
            assert(false);
        }
        // Propagate any adjustments to the end of the track.
        connect(m_segments.end() - adjusted_segments + 1);
    }

    // Extend the last segment to meet the first.  Assume they are collinear.
    // This is guaranteed for 'adjusted_segments' == 2 or 3 but not 1.
    last_segment->set_length((last_segment->start_coords() - end_coords).magnitude());
}

void Road::set_skews()
{
    assert(!m_segments.empty());
    for (auto it = m_segments.begin() + 1; it != m_segments.end(); ++it)
    {
        auto skew{(*it)->start_skew()};
        if (skew != 0.0 && (*it)->arc() != 0.0)
        {
            if ((*(it - 1))->arc() == 0.0)
                (*(it - 1))->set_end_skew(skew);
            if ((*(it + 1))->arc() == 0.0)
                (*(it + 1))->set_start_skew(-skew);
        }
    }
}

void Road::connect(Segment_List::iterator it)
{
    // There's nothing to do to the first segment.
    if (it == m_segments.begin())
        ++it;

    const auto* last{(it - 1)->get()};
    for (; it != m_segments.end(); ++it)
    {
        (*it)->set_start_angle(last->end_angle());
        (*it)->set_start_coords(last->end_coords());
        last = it->get();
    }
}

void Road::draw()
{
    for (auto const& seg : m_segments)
        seg->draw();
    if (m_show_racing_line)
        m_racing_line.draw();
}

//----------------------------------------------------------------------------------------
Three_Vector pit_offset(Gl_Road_Segment const& road_seg,
                        Gl_Road_Segment const& pit_seg,
                        double x, double direction)
{
    auto side{road_seg.pit().side()};
    auto width = [side](auto const& segment, double along){
        return side == Side::left ? segment.left_width(along) : segment.right_width(along);
    };

    auto pit_width{width(pit_seg, x) / cos(direction)};
    auto along{road_seg.pit().split_or_join()};
    auto offset{(width(road_seg, along) - pit_width) * (side == Side::left ? 1.0 : -1.0)};

    if (road_seg.radius() == 0.0)
        return rotate(Three_Vector{along, offset, 0.0},
                      road_seg.angle(along) * z_hat);
    else
        return road_seg.center_of_curve() - road_seg.start_coords()
               + Three_Vector(road_seg.radius() - offset, pit_seg.angle(along) - pi / 2);
}

Three_Vector Pit_Lane::pit_in_offset(Gl_Road_Segment& pit_in) const
{
    return pit_offset(pit_in, *segments().front(), 0.0, start_direction());
}

Three_Vector Pit_Lane::pit_out_offset(Gl_Road_Segment& pit_out) const
{
    auto const& last{*segments().back()};
    return pit_offset(pit_out, last, last.length(), start_direction());
}

void Pit_Lane::build(bool join_to_track, int adjusted_segments, Gl_Road_Segment& pit_in,
                     Gl_Road_Segment& pit_out, const Spline& track_elevation)
{
    if (m_segments.empty())
        return;

    // Skew the ends of the pit lane to meet the track.
    set_skews();
    m_segments.front()->set_start_skew(tan(start_direction()));
    m_segments.back()->set_end_skew(tan(end_direction()));

    // The ends of the pit lane attach to points on the track.  Call build_segments() to
    // transform the pit lane's coordinates to the track's orientation and pit-in
    // position.
    build_elevation(false);
    build_segments(pit_in.start_coords() + pit_in_offset(pit_in),
                   pit_in.pit_angle() + start_direction(), pit_in.banking().end_angle());

    // Make the end of the pit lane meet the track's pit-out position.
    if (join_to_track)
    {
        join(pit_in.start_coords() + pit_in_offset(pit_in),
             pit_in.pit_angle() + start_direction(),
             pit_out.start_coords() + pit_out_offset(pit_out),
             pit_out.pit_angle() + end_direction(), adjusted_segments);
    }

    // Load the pit lane with elevations from the track.
    {
        m_length = build_elevation(false);
        m_elevation.clear();
        auto in_distance{pit_in.start_distance() + pit_in.pit().split_or_join()};
        auto out_distance{pit_out.start_distance() + pit_out.pit().split_or_join()};
        auto track_length{track_elevation[track_elevation.size() - 1].x};
        auto delta{wrap(out_distance - in_distance, track_length)};

        static auto const elevations{10};
        for (auto i{0}; i < elevations; ++i)
        {
            auto along_pit{i * m_length / elevations};
            auto along_track{wrap(in_distance + i * delta / elevations, track_length)};
            auto z{track_elevation.interpolate(along_track)};
            m_elevation.load({along_pit, z});
        }
        m_elevation.load({m_length, track_elevation.interpolate(out_distance)});
    }

    // Finalize the elevations and segments.
    build_elevation(false);
    build_segments(pit_in.start_coords() + pit_in_offset(pit_in),
                   pit_in.pit_angle() + start_direction(), pit_in.banking().end_angle());
}

//----------------------------------------------------------------------------------------
Strip_Track::Strip_Track()
    : mp_track{std::make_unique<Road>()},
      mp_pit_lane{std::make_unique<Pit_Lane>()}
{
    m_timing_lines.clear();
    m_cameras.clear();
}

void Strip_Track::set_racing_line(bool build, bool show)
{
    if (mp_track)
        mp_track->set_racing_line(build, show);
}

void Strip_Track::read(std::string const& data_dir, std::string const& track_file)
{
    // Remember the file name for re-reading.
    if (!data_dir.empty() && !track_file.empty())
    {
        m_data_dir = data_dir;
        m_track_file = track_file;
    }
    mp_track->clear();
    mp_pit_lane->clear();
    m_timing_lines.clear();
    read_track_file(m_data_dir, m_track_file, this);
}

size_t Strip_Track::add_segment(std::unique_ptr<Gl_Road_Segment> segment)
{
    return mp_track->add_segment(std::move(segment));
}

size_t Strip_Track::add_pit_segment(std::unique_ptr<Gl_Road_Segment> segment)
{
    auto start{mp_pit_lane->segments().size() == 0};
    auto index{start ? m_pit_in_index : m_pit_out_index};
    auto distance{start ? 0.0 : segment->length()};
    auto width{segment->left_width(distance) + segment->right_width(distance)};
    auto left_shoulder{segment->left_width(distance) - segment->left_road_width(distance)};
    auto right_shoulder{segment->right_width(distance) - segment->right_road_width(distance)};
    mp_track->segments()[index]->set_pit_width(width, left_shoulder, right_shoulder);
    return mp_pit_lane->add_segment(std::move(segment));
}

void Strip_Track::set_pit_in(size_t index, double angle)
{
    m_pit_in_index = index;
    mp_pit_lane->set_start_direction(angle);
}

void Strip_Track::set_pit_out(size_t index, double angle)
{
    m_pit_out_index = index;
    mp_pit_lane->set_end_direction(angle);
}

void Strip_Track::build(bool close, int adjusted_road_segments, double track_length,
                        bool join_pit_lane, int adjusted_pit_segments)
{
    mp_track->build(close, adjusted_road_segments, track_length);

    if (m_pit_in_index != -1 && m_pit_out_index != -1)
    {
        auto& in{*mp_track->segments()[m_pit_in_index]};
        auto& out{*mp_track->segments()[m_pit_out_index]};
        mp_pit_lane->build(join_pit_lane, adjusted_pit_segments, in, out, mp_track->elevation());

        auto along{in.pit().split_or_join() + 1.0e-6};
        auto from_center{in.pit().side() == Side::right
                         ? -in.right_width(along)
                         : in.left_width(along)};
        m_objects.emplace_back(position(along + in.start_distance(), from_center),
                               in.pit().side() == Side::right
                               ? in.right_material(0.0)
                               : in.left_material(0.0));

        along = out.pit().split_or_join() - 1.0e-6;
        from_center = out.pit().side() == Side::right ?
            -out.right_width(along)
            : out.left_width(along);
        m_objects.emplace_back(position(along + out.start_distance(), from_center),
                               in.pit().side() == Side::right
                               ? out.right_material(0.0)
                               : out.left_material(0.0));
    }
}

void Strip_Track::set_sky_box(std::string sides_image, std::string top_image,
                              std::string bottom_image, bool smooth)
{
    mp_sky_box
        = std::make_unique<Sky_Box>(100.0, sides_image, top_image, bottom_image, smooth);
}

void Strip_Track::set_map_background(std::string const& background_image,
                                     Rectangle<int> const& size)
{
    mp_map_background = std::make_unique<Map_Background>(background_image, size);
}

void Strip_Track::draw_sky(const Three_Vector& view) const
{
    if (mp_sky_box)
        mp_sky_box->draw(view);
}

void Strip_Track::draw_map_background() const
{
    if (mp_map_background)
        mp_map_background->draw();
}

void Strip_Track::draw() const
{
    glLoadIdentity();
    mp_track->draw();
    mp_pit_lane->draw();
}

Three_Vector Strip_Track::reset_position(Three_Vector const& pos,
                                         size_t& road_index, size_t& segment_index) const
{
    return {track_coordinates(pos, road_index, segment_index).x, 0.0, 0.0};
}

Three_Matrix Strip_Track::reset_orientation(Three_Vector const& pos,
                                            size_t& road_index, size_t& segment_index) const
{
    Three_Matrix orientation{1.0};

    // Align the car's up direction with the normal.
    const auto& track_pos{track_coordinates(pos, road_index, segment_index)};
    const auto& segment{get_road(road_index).segments()[segment_index]};
    auto along{track_pos.x - segment->start_distance()};
    auto across{track_pos.y};
    auto normal{segment->normal(along, across)};
    return orientation.rotate(
        Three_Vector(-asin(normal.y), asin(normal.x), segment->angle(along)));
}

Three_Vector Strip_Track::track_coordinates(Three_Vector const& world_pos,
                                            size_t& road_index, size_t& segment_index) const
{
    // Find the distance along the track, distance from center, and elevation
    // for the world coordinates `world_pos.x' and `world_pos.y'.

    Three_Vector track_pos;
    // Use a pointer instead of reference because we may have to change the segment list
    // being referred to.
    const auto* segments{&get_road(road_index).segments()};
    if (segment_index >= segments->size())
    {
        std::cerr << segment_index << ' ' << segments->size() << ' ' << road_index << std::endl;
        assert(false);
    }
    auto* segment{(*segments)[segment_index].get()};
    auto direction{0};
    for (size_t i{0}; ; ++i)
    {
        auto off{segment->coordinates(world_pos, track_pos)};
        if (off == 0.0)
            break;

        if (direction == 1 || (direction == 0 && off > 0.0))
        {
            direction = 1;
            // We're off the end of the current segment.  Find a new candidate segment.
            if (road_index == 0
                && segment_index == size_t(m_pit_in_index)
                && segment->on_pit_merge(track_pos.x, track_pos.y))
            {
                // We've moved onto the pit lane.  Try the first segment on the pit lane.
                road_index = 1;
                segment_index = 0;
            }
            else if (road_index == 1 && segment_index == mp_pit_lane->segments().size() - 1)
            {
                // We've moved off of the pit lane.  Try the segment where the pit lane
                // joins the track.
                road_index = 0;
                segment_index = m_pit_out_index;
            }
            else
            {
                // Try the next segment.
                ++segment_index;
                if (road_index == 0 && segment_index == segments->size())
                    segment_index = 0;
            }
        }
        else
        {
            direction = -1;
            // We're off the beginning of the current segment.  Find a new candidate
            // segment.
            if (road_index == 0
                && segment_index == size_t(m_pit_out_index)
                && segment->on_pit_merge(track_pos.x, track_pos.y))
            {
                // We've moved onto the end of the pit lane.  Try the last segment on the
                // pit lane.
                road_index = 1;
                segment_index = mp_pit_lane->segments().size() - 1;
            }
            else if (road_index == 1 && segment_index == 0)
            {
                // We've moved off of the beginning of the pit lane.  Try the segment
                // where the pit lane splits from the track.
                road_index = 0;
                segment_index = m_pit_in_index;
            }
            else
            {
                // Try the previous segment.
                if (road_index == 0 && segment_index == 0)
                    segment_index = segments->size();
                --segment_index;
            }
        }
        segments = &get_road(road_index).segments();
        segment = (*segments)[segment_index].get();
        if (i == segments->size())
            throw Segment_Not_Found(world_pos, segment_index);
    }

    assert(segment_index < segments->size());
    return track_pos + segment->start_distance() * x_hat;
}

Road const& Strip_Track::get_road(size_t road_index) const
{
    assert(road_index < 2);
    return road_index == 0 ? *mp_track : *mp_pit_lane;
}

Contact_Info Strip_Track::test_for_contact(Three_Vector const& pos,
                                           double, // bump_parameter
                                           size_t& road_index, size_t& segment_index) const
{
    auto const track_pos{track_coordinates(pos, road_index, segment_index)};
    auto const& segment{get_road(road_index).segments()[segment_index]};
    auto const segment_distance{track_pos.x - segment->start_distance()};
    auto material{segment->material_at(track_pos.x, track_pos.y)};
    auto contact{false};
    Three_Vector normal;

    // Test for contact with the road.
    auto bump{material.bump(track_pos.x, track_pos.y)};
    auto elev{track_coordinates(pos, road_index, segment_index).z + bump.z};
    auto diff{elev - pos.z};
    if (diff >= 0.0)
    {
        contact = true;
        auto bump{material.bump(track_pos.x, track_pos.y)};
        normal = segment->normal(segment_distance, track_pos.y, bump);
    }
    // Test for contact with the left wall.
    if (!contact)
    {
        material = segment->left_material(pos.z);
        auto bump{material.bump(track_pos.x, track_pos.y)};
        diff = track_pos.y - (segment->left_width(segment_distance) + bump.z);
        if (diff >= 0.0)
        {
            contact = true;
            normal = segment->barrier_normal(segment_distance, track_pos.y, bump);
        }
    }
    // Test for contact with the right wall.
    if (!contact)
    {
        material = segment->right_material(pos.z);
        auto bump{material.bump(track_pos.x, track_pos.y)};
        diff = -track_pos.y - (segment->right_width(segment_distance) + bump.z);
        if (diff >= 0.0)
        {
            contact = true;
            normal = segment->barrier_normal(segment_distance, track_pos.y, bump);
        }
    }

    return Contact_Info(contact, diff, normal, material);
}

Three_Vector Strip_Track::position(double along, double from_center) const
{
    return mp_track->position(along, from_center);
}

int Strip_Track::sector(double distance) const
{
    for (size_t i{0}; i < m_timing_lines.size(); ++i)
        if (m_timing_lines[i] > distance)
            return i;
    return m_timing_lines.size();
}

void Strip_Track::set_start_direction(double degrees)
{
    m_start_direction = deg_to_rad(degrees);
    mp_track->set_start_direction(degrees);
}

void Strip_Track::add_camera(Camera const& camera)
{
    m_cameras.push_back(camera);
}

double Strip_Track::camera_range(Camera const& camera) const
{
    auto range{mp_track->segments()[camera.segment_index]->start_distance()
               + camera.position.x - camera.range};
    return wrap(range, mp_track->length());
}

Three_Vector Strip_Track::camera_position(Camera const& camera) const
{
    const auto& segment{*(mp_track->segments()[camera.segment_index])};
    return segment.position(camera.position.x, camera.position.y)
        + Three_Vector{0.0, 0.0, camera.position.z};
}

const Camera& Strip_Track::get_camera(double distance) const
{
    if (m_cameras.empty())
        return default_camera;

    // See if we're near the end of the track and should be picked up by the first camera.
    auto first{m_cameras.begin()->position.x - m_cameras.begin()->range};
    if (mp_track->is_closed() && first < 0.0 && distance > wrap(first, mp_track->length()))
        return *m_cameras.begin();

    for (auto rit = m_cameras.rbegin(); rit != m_cameras.rend(); rit++)
        if (distance > camera_range(*rit))
            return *rit;
    return *m_cameras.begin();
}

Three_Vector Strip_Track::camera_target(Camera const& camera) const
{
    auto angle{mp_track->segments()[camera.segment_index]->angle(camera.position.x)};
    return camera_position(camera)
        + Three_Vector{-sin(deg_to_rad(camera.direction.x) + angle),
                       cos(deg_to_rad(camera.direction.x) + angle),
                       sin(deg_to_rad(camera.direction.y))};
}

const Rectangle<double>& Strip_Track::bounds() const
{
    return Rectangle(mp_track->bounds()).enclose(mp_pit_lane->bounds());
}

Three_Vector Strip_Track::grid_position(int place, int total, bool pit) const
{
    assert(place > 0); // 1-based
    assert(place <= total);
    auto grid_interval = (pit ? 12.0 : 8.0);
    // Put the 1st car 1 interval from the beginning of the 1st segment to avoid putting
    // off the end.
    auto across{pit ? 1.5 * mp_track->left_road_width(0.0) : 3.0 * std::pow(-1, place)};
    return {grid_interval * (total - place + 1), across, 0.0};
}
