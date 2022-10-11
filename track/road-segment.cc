//  Copyright (C) 2001-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTYh;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "road-segment.h"

#include "../geometry/numeric.h"
#include "../media/material.h"
#include "../media/texture-image.h"

#include <cassert>
#include <cmath>
#include <complex>
#include <iostream>

using namespace Vamos_Geometry;
using namespace Vamos_Track;

//----------------------------------------------------------------------------------------
Kerb::Kerb(std::vector<Two_Vector> const& profile,
           double start, double start_transition_length, double start_transition_width,
           double end, double end_transition_length, double end_transition_width,
           bool full_length)
    : m_points{profile},
      m_profile{profile},
      m_start{start},
      m_start_transition_length{start_transition_length},
      m_start_transition_width{start_transition_width},
      m_end{end},
      m_end_transition_length{end_transition_length},
      m_end_transition_width{end_transition_width},
      m_full_length{full_length}
{
}

void Kerb::set_length(double length)
{
    if (length < m_end)
        m_full_length = true;
    if (m_full_length)
        m_end = length;
}

double Kerb::width() const
{
    return m_points.empty() ? 0.0 : m_points.back().x;
}

bool Kerb::on_kerb(double dist) const
{
    return dist >= m_start && (dist < m_end || m_full_length);
}

double Kerb::elevation(double along, double from_inside)
{
    //!!TODO: Account for tapering on the transition.
    return on_kerb(along) && from_inside > 0.0 && from_inside < width()
        ? m_profile.interpolate(from_inside)
        : 0.0;
}

double Kerb::angle(double along, double from_inside)
{
    if (!on_kerb(along) || (from_inside < 0.0) || (from_inside > width()))
        return 0.0;

    //!!TODO: Account for tapering on the transition.
    auto norm{m_profile.normal(from_inside)};
    return std::atan2(norm.y, norm.x) - pi / 2.0;
}

Two_Vector const& Kerb::point(size_t substrip) const
{
    assert(substrip < m_points.size());
    return m_points[substrip];
}

//----------------------------------------------------------------------------------------
void Banking::set(double end_angle, double pivot_from_center)
{
    m_end_angle = end_angle;
    m_pivot_from_center = pivot_from_center;
}

double Banking::angle(double along) const
{
    return Vamos_Geometry::deg_to_rad(m_bank_angle.interpolate(along));
}

double Banking::height(double along, double from_center) const
{
    return (m_pivot_from_center - from_center) * sin(angle(along));
}

void Banking::set_start(double start_angle, double length)
{
    m_start_angle = start_angle;
    m_bank_angle.clear();
    m_bank_angle.load({0.0, start_angle});
    m_bank_angle.load({length, m_end_angle});
}

//----------------------------------------------------------------------------------------
void Pit_Lane_Transition::set_merge(End end, Side side, double split_or_join, double merge,
                                    double angle)
{
    m_end = end;
    m_side = side;
    m_split_or_join = split_or_join;
    m_merge = merge;
    m_merge_is_set = true;
    m_angle = angle;
}

void Pit_Lane_Transition::set_width(double pit_width, double left_shoulder, double right_shoulder)
{
    m_pit_width = skew(pit_width);
    m_pit_shoulder_width = skew(m_side == Side::left ? left_shoulder : right_shoulder);
    m_width_is_set = true;
}

double Pit_Lane_Transition::width(Side pit_side, double distance, bool narrow) const
{
    if (pit_side != m_side)
        return 0.0;
    if (narrow)
        return m_pit_width;
    if ((m_end == End::in && distance <= m_split_or_join)
        || (m_end == End::out && distance >= m_split_or_join))
        return 0.0;
    return m_pit_width;
}

void Pit_Lane_Transition::scale(double factor)
{
    m_split_or_join *= factor;
    m_merge *= factor;
}

//----------------------------------------------------------------------------------------
Road_Segment::Road_Segment(double length, double radius,
                           double left_width, double right_width,
                           double left_road_width, double right_road_width)
    : m_length{length},
      m_radius{radius}
{
    m_left_road_width.load({0.0, left_road_width});
    m_right_road_width.load({0.0, right_road_width});
    m_left_width.load({0.0, left_width});
    m_right_width.load({0.0, right_width});
}

void Road_Segment::set_length(double length)
{
    // Expand or contract the specified road widths along with the segment.
    scale_widths(length / m_length);
    m_length = length;
}

void Road_Segment::set_radius(double radius)
{
    auto old_arc{arc()};
    m_radius = radius;
    if (old_arc != 0.0)
        set_arc(old_arc);
}

double Road_Segment::arc() const
{
    return m_radius == 0.0 ? 0.0 : m_length / m_radius;
}

void Road_Segment::set_arc(double new_arc)
{
    // Preserve length for straights.
    if (m_radius == 0.0)
        set_radius(m_length / new_arc);
    // Preserve radius for curves.
    else
        set_length(m_radius * new_arc);
}

void Road_Segment::scale(double factor)
{
    assert(factor != 0.0);
    scale_widths(factor);
    m_length *= factor;
    m_radius *= factor;
    m_pit.scale(factor);
}

void Road_Segment::set_widths(TPoints const& right, TPoints const& right_road,
                              TPoints const& left_road, TPoints const& left)
{
    m_right_width.replace(right);
    m_right_road_width.replace(right_road);
    m_left_road_width.replace(left_road);
    m_left_width.replace(left);
}

void Road_Segment::scale_widths(double factor)
{
    m_left_width.scale(factor);
    m_right_width.scale(factor);
    m_left_road_width.scale(factor);
    m_right_road_width.scale(factor);
}

void Road_Segment::set_kerb(std::unique_ptr<Kerb> kerb, Side side)
{
    (side == Side::left ? mp_left_kerb : mp_right_kerb) = std::move(kerb);
}

double Road_Segment::kerb_width(Side side, double along) const
{
    const auto& kerb{side == Side::left ? mp_left_kerb : mp_right_kerb};
    return kerb && kerb->on_kerb(along) ? kerb->width() : 0.0;
}

void Road_Segment::set_wall_heights(double left_height, double right_height)
{
    m_left_wall_height = left_height;
    m_right_wall_height = right_height;
}

void Road_Segment::build_elevation(Spline* elevation, double start_distance)
{
    mp_elevation_curve = elevation;
    for (auto p : m_elevation_points)
        if (!m_last_segment || p.x < m_length - 10.0)
            mp_elevation_curve->load({p.x + start_distance, p.y});
}

double Road_Segment::elevation(double along, double from_center) const
{
    assert(mp_elevation_curve != 0);
    auto elev{mp_elevation_curve->interpolate(along + m_start_distance)
              + banking().height(along, from_center)};

    // Add the kerb's height.
    auto diff{from_center - left_road_width(along)};
    if (mp_left_kerb)
        return elev + mp_left_kerb->elevation(along, diff);
    diff = -from_center - right_road_width(along);
    if (mp_right_kerb)
        return elev + mp_right_kerb->elevation(along, diff);
    return elev;
}

double Road_Segment::world_elevation(Three_Vector const& world_position) const
{
    Three_Vector track_position;
    coordinates(world_position, track_position);
    return track_position.z;
}

Three_Vector Road_Segment::barrier_normal(double along, double from_center,
                                          Three_Vector const& bump) const
{
    auto normal{from_center > 0.0 ? -m_left_width.normal(along) : m_right_width.normal(along)};

    // The -y direction is up, -z is to the left, x is forward.
    normal.x = bump.x;
    normal.z = -bump.y;
    return rotate(normal, angle(along) * z_hat);
}

Three_Vector Road_Segment::barrier_normal(double along, double from_center) const
{
    return barrier_normal(along, from_center, z_hat);
}

Three_Vector Road_Segment::normal(double along, double from_center, const Three_Vector& bump,
                                  bool include_kerb /* true */) const
{
    assert(mp_elevation_curve);

    Three_Vector norm{mp_elevation_curve->normal(along + m_start_distance)};
    // The z direction is up, y is to the left, x is forward.
    norm.z = norm.y;
    norm.y = bump.y;
    norm.x += bump.x;

    auto bank{m_banking.angle(along)};
    if (include_kerb && mp_left_kerb)
        bank -= mp_left_kerb->angle(along, from_center - left_road_width(along));
    if (include_kerb && mp_right_kerb)
        bank += mp_right_kerb->angle(along, -from_center - right_road_width(along));
    return rotate(rotate(norm, -bank * x_hat), angle(along) * z_hat);
}

Three_Vector Road_Segment::normal(double along, double from_center, bool include_kerb) const
{
    return normal(along, from_center, z_hat, include_kerb);
}

void Road_Segment::set_start(Three_Vector const& start_coords, double start_distance,
                             double start_angle, double start_bank,
                             std::vector<double> const&)
{
    m_start_distance = start_distance;
    m_start_coords = start_coords;
    m_start_angle = start_angle;
    m_banking.set_start(start_bank, m_length);
}

double Road_Segment::end_angle() const
{
    // Keep it within [0, 2pi).
    return branch(m_start_angle + arc(), 0.0);
}

Three_Vector Road_Segment::center_of_curve() const
{
    return m_start_coords + Three_Vector(m_radius, m_start_angle + pi / 2);
}

Three_Vector Road_Segment::end_coords() const
{
    if (is_straight())
        return m_start_coords + Three_Vector(m_length, m_start_angle);
    else
        return center_of_curve() - Three_Vector(m_radius, m_start_angle + arc() + pi / 2);
}

double Road_Segment::left_width(double distance, bool narrow) const
{
    return m_left_width.interpolate(distance) - m_pit.width(Side::left, distance, narrow);
}

double Road_Segment::right_width(double distance, bool narrow) const
{
    return m_right_width.interpolate(distance) - m_pit.width(Side::right, distance, narrow);
}

void Road_Segment::set_left_width(double distance, double width)
{
    m_left_width.load({distance, width});
}

void Road_Segment::set_right_width(double distance, double width)
{
    m_right_width.load({distance, width});
}

void Road_Segment::set_left_road_width(double distance, double width)
{
    m_left_road_width.load({distance, width});
}

void Road_Segment::set_right_road_width(double distance, double width)
{
    m_right_road_width.load({distance, width});
}

double Road_Segment::left_racing_line_width(double distance) const
{
    distance -= start_distance();
    auto width{left_road_width_no_pit(distance)};
    if (m_racing_line_adjustment_distance < 0.0 || distance < m_racing_line_adjustment_distance)
        return width + m_racing_line_adjustment;
    return width;
}

double Road_Segment::right_racing_line_width(double distance) const
{
    distance -= start_distance();
    auto width{right_road_width_no_pit(distance)};
    if (m_racing_line_adjustment_distance < 0.0 || distance < m_racing_line_adjustment_distance)
        return width - m_racing_line_adjustment;
    return width;
}

void Road_Segment::set_racing_line_adjustment(double across, double distance)
{
    m_racing_line_adjustment = across;
    m_racing_line_adjustment_distance = distance;
}

double Road_Segment::pit_width() const
{
    return m_pit.width(m_pit.side(), m_pit.split_or_join(), true);
}

double Road_Segment::pit_road_connection() const
{
    return m_pit.split_or_join();
}

double Road_Segment::extra_road_width(Side pit_side, double distance, bool narrow) const
{
    if (narrow || pit_side != m_pit.side())
        return 0.0;

    auto width{-m_pit.shoulder_width()};
    if (pit_side == Side::left)
        width += left_width(distance) - left_road_width(distance, true);
    else
        width += right_width(distance) - right_road_width(distance, true);

    auto extra{width * (distance - m_pit.merge()) / (m_pit.split_or_join() - m_pit.merge())};

    if (m_pit.end() == Pit_Lane_Transition::End::in
        && distance > m_pit.merge()
        && distance <= m_pit.split_or_join())
        return extra;
    if (m_pit.end() == Pit_Lane_Transition::End::out
        && distance < m_pit.merge()
        && distance >= m_pit.split_or_join())
        return extra;
    return 0.0;
}

double Road_Segment::left_road_width_no_pit(double distance) const
{
    return m_left_road_width.interpolate(distance);
}

double Road_Segment::right_road_width_no_pit(double distance) const
{
    return m_right_road_width.interpolate(distance);
}

double Road_Segment::left_road_width(double distance, bool narrow) const
{
    return left_road_width_no_pit(distance) + extra_road_width(Side::left, distance, narrow);
}

double Road_Segment::right_road_width(double distance, bool narrow) const
{
    return right_road_width_no_pit(distance) + extra_road_width(Side::right, distance, narrow);
}

const Pit_Lane_Transition& Road_Segment::pit() const
{
    return m_pit;
}

void Road_Segment::set_pit_width(double width, double left_shoulder, double right_shoulder)
{
    m_pit.set_width(width, left_shoulder, right_shoulder);
}

void Road_Segment::set_pit_lane(Pit_Lane_Transition::End end, Side side, double split_or_join,
                                double merge, double angle)
{
    m_pit.set_merge(end, side, split_or_join, merge, deg_to_rad(angle));
}

std::complex<double> solve_quadratic(double a, double b, double c, double solution)
{
    if (a == 0.0)
        return -c / b;
    return (-b + sign(solution) * sqrt(std::complex<double>(b * b - 4 * a * c)))
           / (2 * a);
}

double Road_Segment::get_curve_angle(const Three_Vector& position, double across) const
{
    return arc() / 2.0
           + atan2(sign(m_radius) * position.y,
                   sign(m_radius) * (position.x - across * m_start_skew / sin(arc() / 2.0)));
}

double Road_Segment::coordinates(Three_Vector const& world_position,
                                 Three_Vector& track_position) const
{
    if (!is_straight())
    {
        auto const half_angle{arc() / 2.0};
        auto const centered_position{
            rotate(world_position - center_of_curve(),
                   (pi / 2.0 - half_angle - m_start_angle) * z_hat)};

        const std::complex<double> across = solve_quadratic(
            1.0 + 2.0 * m_start_skew / tan(half_angle) - m_start_skew * m_start_skew,
            -2.0
                * (m_radius
                   + (m_start_skew / sin(half_angle)
                      * (m_radius * cos(half_angle) - centered_position.x))),
            m_radius * m_radius - centered_position.x * centered_position.x
                - centered_position.y * centered_position.y,
            -m_radius);

        track_position.y = across.real();
        if (across.imag() != 0.0)
        {
            // We're off the segment but the solution is complex.  Just indicate whether
            // we're off the beginning or end of the segement.
            if ((world_position - m_start_coords).magnitude()
                < (world_position - end_coords()).magnitude())
                track_position.x = -1.0;
            else
                track_position.x = m_length + 1.0;
        }
        else
            track_position.x = m_radius * get_curve_angle(centered_position, track_position.y);
        track_position.z = 0.0;
    }
    else
    {
        track_position
            = rotate(world_position - center_of_curve(), -m_start_angle * z_hat);
        track_position.x = (track_position.x - track_position.y * m_start_skew)
                           / (1 + track_position.y / m_length * (m_end_skew - m_start_skew));
    }
    track_position.z = elevation(track_position.x, track_position.y);
    return off_track_distance(track_position);
}

double Road_Segment::off_track_distance(const Three_Vector& track_position) const
{
    auto min{m_pit.end() == Pit_Lane_Transition::End::out
             && on_pit_merge(track_position.x, track_position.y)
             ? m_pit.split_or_join() : 0.0};
    if (track_position.x < min)
        return track_position.x - min;

    auto max{m_pit.end() == Pit_Lane_Transition::End::in
             && on_pit_merge(track_position.x, track_position.y)
             ? m_pit.split_or_join() : m_length};
    if (track_position.x > max)
        return track_position.x - max;

    return 0.0;
}

bool Road_Segment::on_pit_merge(double distance, double from_center) const
{
    if (!is_in_range(distance, 0.0, m_length))
        return false;

    auto from_split{(distance - m_pit.split_or_join())
                    * (m_pit.end() == Pit_Lane_Transition::End::in ? 1.0 : -1.0)};
    auto from_wall{m_pit.side() == Side::right ? -from_center - right_width(distance)
                                               : from_center - left_width(distance)};

    return m_pit.active() && from_split > 0.0 && from_wall > 0.0
        && std::abs(atan2(from_wall, from_split)) > std::abs(m_pit.angle()) / 2.0;
}

Three_Vector Road_Segment::position(double along, double from_center) const
{
    auto pos{elevation(along, from_center) * z_hat};
    if (is_straight())
    {
        auto extra{
            from_center * (m_start_skew + ((m_end_skew - m_start_skew) * along / m_length))};
        return pos + start_coords()
               + rotate({along + extra, from_center, 0.0}, m_start_angle * z_hat);
    }

    auto beta{arc() / 2.0};
    auto radius{m_radius - from_center * (1.0 + m_start_skew / tan(beta))};
    auto center{
        center_of_curve()
        + Three_Vector(from_center * m_start_skew / sin(beta), m_start_angle + beta - pi / 2.0)};
    auto angle{m_start_angle + along / m_radius};
    return pos + center - rotate(radius * y_hat, angle * z_hat);
}

void Road_Segment::narrow(Side side, double delta_width)
{
    (side == Side::right ? m_right_width : m_left_width).shift(-delta_width);
}
