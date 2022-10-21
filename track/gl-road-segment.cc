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

#include "gl-road-segment.h"

#include "../geometry/conversions.h"
#include "../geometry/three-vector.h"
#include "../media/model.h"
#include "../media/texture-image.h"

#include <GL/glut.h>

#include <algorithm>
#include <cassert>
#include <cmath>

using namespace Vamos_Track;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

auto constexpr s_strip_type{GL_QUAD_STRIP};
// GL_LINE_STRIP
// GL_LINES

static const Material s_no_material(Material::air, 0.0, 0.0);

//----------------------------------------------------------------------------------------
Braking_Marker::Braking_Marker(std::string const& image_file, double distance, Side side,
                               Point<double> offset, Point<double> size)
    : image{std::make_unique<Facade>(image_file, Three_Vector(), size, true)},
      distance{distance},
      side{side},
      offset{offset}
{
}

//----------------------------------------------------------------------------------------
namespace Vamos_Track
{
class Segment_Iterator
{
public:
    Segment_Iterator(Road_Segment const& segment, double resolution);

    void start(Three_Vector const& start_coords, Gl_Road_Segment::Strip strip);
    Segment_Iterator& operator++();

    Three_Vector const& coordinates() const { return m_coordinates; }
    Three_Vector const& normal() const { return m_normal; }
    Two_Vector const& texture_coordinates() const { return m_texture_coordinates; }
    bool last_subdivision() const { return m_position == Position::end; }

private:
    void increment_distance();
    void increment_kerb_distance(const Kerb& kerb);
    size_t substrips() const;

    Road_Segment const& m_segment;

    Three_Vector m_coordinates;
    Three_Vector m_normal;
    Two_Vector m_texture_coordinates;

    double m_resolution;

    Gl_Road_Segment::Strip m_strip;
    size_t m_substrip;
    double m_distance;
    Side m_side;

    enum Position{begin, begin_transition, middle, end_transition, end };
    Position m_position{Position::begin};

    bool m_connection{false};
    bool m_after_connection{false};
};
} // namespace Vamos_Track

Segment_Iterator::Segment_Iterator(const Road_Segment& segment, double resolution)
    : m_segment(segment),
      m_resolution(resolution)
{
}

void Segment_Iterator::start(Three_Vector const& start_coords, Gl_Road_Segment::Strip strip)
{
    m_coordinates = start_coords;
    m_normal = {0.0, 0.0, 1.0};
    m_distance = 0.0;
    m_strip = strip;

    m_side = Side::left;
    m_position = Position::begin;
    m_substrip = 0;
}


size_t Segment_Iterator::substrips() const
{
    if (m_strip == Gl_Road_Segment::LEFT_KERB)
        return m_segment.mp_left_kerb ? m_segment.mp_left_kerb->substrips() : 0;
    if (m_strip == Gl_Road_Segment::RIGHT_KERB)
        return m_segment.mp_right_kerb ? m_segment.mp_right_kerb->substrips() : 0;
    return 1;
}

Segment_Iterator& Segment_Iterator::operator++()
{
    assert(m_segment.mp_elevation_curve != 0);

    // Remember the last coordinates.
    Three_Vector last_coords = m_coordinates;
    double last_tex_dist = m_texture_coordinates.y;

    // Bail out if there's no kerb.
    if ((m_strip == Gl_Road_Segment::LEFT_KERB
         && (!m_segment.mp_left_kerb || m_segment.mp_left_kerb->transition_end() == 0.0))
        || (m_strip == Gl_Road_Segment::RIGHT_KERB
            && (!m_segment.mp_right_kerb || m_segment.mp_right_kerb->transition_end() == 0.0)))
    {
        m_position = Position::end;
        return *this;
    }

    if (m_position == Position::begin)
    {
        // Start a new strip.
        glEnd();
        glBegin(s_strip_type);

        last_tex_dist = 0.0;
        m_texture_coordinates.y = 0.0;
        m_after_connection = false;
    }

    increment_distance();

    auto left_start_transition{false};
    auto left_end_transition{false};
    auto right_start_transition{false};
    auto right_end_transition{false};

    if (m_strip == Gl_Road_Segment::LEFT_KERB)
    {
        m_segment.mp_left_kerb->set_length(m_segment.length());
        if (m_distance < m_segment.mp_left_kerb->start())
            left_start_transition = true;
        else if (m_distance > m_segment.mp_left_kerb->end())
            left_end_transition = true;
    }
    else if (m_strip == Gl_Road_Segment::RIGHT_KERB)
    {
        m_segment.mp_right_kerb->set_length(m_segment.length());
        if (m_distance < m_segment.mp_right_kerb->start())
            right_start_transition = true;
        else if (m_distance > m_segment.mp_right_kerb->end())
            right_end_transition = true;
    }

    auto no_pit{
        m_segment.pit().active()
        && ((m_segment.pit().end() == Pit_Lane_Transition::End::in && m_after_connection)
            || (m_segment.pit().end() == Pit_Lane_Transition::End::out && !m_after_connection))};

    // Calculate the geometry of the strip.
    auto across{0.0};
    auto up{0.0};
    switch (m_strip)
    {
    case Gl_Road_Segment::LEFT_BARRIER:
        across = m_segment.left_width(m_distance, no_pit);
        up = m_side == Side::left ? m_segment.m_left_wall_height : 0.0;
        m_texture_coordinates.x = up;
        m_normal = m_segment.barrier_normal(m_distance, across);
        if (m_after_connection)
            glBegin(s_strip_type);
        break;
    case Gl_Road_Segment::LEFT_SHOULDER:
        up = 0.0;
        across = m_side == Side::left ? m_segment.left_width(m_distance, no_pit)
                                      : m_segment.left_road_width(m_distance, no_pit);
        m_texture_coordinates.x = across;
        m_normal = m_segment.normal(m_distance, across, false);
        break;
    case Gl_Road_Segment::LEFT_KERB:
        up = m_side == Side::left ? m_segment.mp_left_kerb->point(m_substrip + 1).y
                                  : m_segment.mp_left_kerb->point(m_substrip).y;
        across = m_side == Side::left ? m_segment.mp_left_kerb->point(m_substrip + 1).x
                                      : m_segment.mp_left_kerb->point(m_substrip).x;
        if (left_start_transition)
        {
            up = 0.0;
            across *= (m_segment.mp_left_kerb->start_transition_width()
                       / m_segment.mp_left_kerb->width());
        }
        else if (left_end_transition)
        {
            up = 0.0;
            across *= (m_segment.mp_left_kerb->end_transition_width()
                       / m_segment.mp_left_kerb->width());
        }
        across += m_segment.left_road_width(m_distance, no_pit);
        m_texture_coordinates.x = across;
        m_normal = m_segment.normal(m_distance, across);
        break;
    case Gl_Road_Segment::TRACK:
        up = 0.0;
        across = m_side == Side::left ? m_segment.left_road_width(m_distance, no_pit)
                                      : -m_segment.right_road_width(m_distance, no_pit);
        m_texture_coordinates.x = across + m_segment.left_road_width(m_distance, no_pit);
        m_normal = m_segment.normal(m_distance, across, false);
        if (m_after_connection)
            glBegin(s_strip_type);
        break;
    case Gl_Road_Segment::RIGHT_KERB:
        up = 0.0;
        across = 0.0;
        if (m_side == Side::left)
        {
            up = m_segment.mp_right_kerb->point(m_substrip).y;
            across = -m_segment.mp_right_kerb->point(m_substrip).x;
        }
        else
        {
            up = m_segment.mp_right_kerb->point(m_substrip + 1).y;
            across = -m_segment.mp_right_kerb->point(m_substrip + 1).x;
        }
        if (right_start_transition)
        {
            up = 0.0;
            across *= (m_segment.mp_right_kerb->start_transition_width()
                       / m_segment.mp_right_kerb->width());
        }
        else if (right_end_transition)
        {
            up = 0.0;
            across *= (m_segment.mp_right_kerb->end_transition_width()
                       / m_segment.mp_right_kerb->width());
        }

        across -= m_segment.right_road_width(m_distance, no_pit);
        m_texture_coordinates.x = across;
        m_normal = m_segment.normal(m_distance, across);
        break;
    case Gl_Road_Segment::RIGHT_SHOULDER:
        up = 0.0;
        across = m_side == Side::left ? -m_segment.right_road_width(m_distance, no_pit)
                                      : -m_segment.right_width(m_distance, no_pit);
        m_texture_coordinates.x = across;
        m_normal = m_segment.normal(m_distance, across, false);
        break;
    case Gl_Road_Segment::RIGHT_BARRIER:
        across = -m_segment.right_width(m_distance, no_pit);
        up = (m_side == Side::left) ? 0.0 : m_segment.m_right_wall_height;
        m_texture_coordinates.x = up;
        m_normal = m_segment.barrier_normal(m_distance, across);
        if (m_after_connection)
            glBegin(s_strip_type);
        break;
    default:
        assert(false);
    }

    if (m_position == Position::end && m_segment.is_last_segment()
        && m_strip != Gl_Road_Segment::LEFT_KERB && m_strip != Gl_Road_Segment::RIGHT_KERB)
    {
        // Return to the origin if we're at the end of the track.
        auto angle{m_segment.angle(m_distance)};
        m_coordinates = {-across * std::sin(angle),
                         across * std::cos(angle),
                         up + m_segment.banking().height(m_distance, across)};
    }
    else
    {
        m_coordinates = m_segment.position(m_distance, across);
        m_coordinates.z = m_segment.elevation(m_distance, across) + up;
    }

    if (m_position != Position::begin
        && (m_strip == Gl_Road_Segment::LEFT_BARRIER
            || m_strip == Gl_Road_Segment::RIGHT_BARRIER))
    {
        if (m_side == Side::left)
        {
            Two_Vector delta{m_coordinates.x - last_coords.x,
                             m_coordinates.y - last_coords.y};
            m_texture_coordinates.y = last_tex_dist + delta.magnitude();
        }
    }
    else
        m_texture_coordinates.y = m_distance;

    if (m_position == Position::end && m_side == Side::right)
    {
        if (++m_substrip != substrips())
            m_position = Position::begin;
    }
    else if (m_position == Position::begin)
        m_position = Position::middle;

    m_side = m_side == Side::left ? Side::right : Side::left;
    return *this;
}

void Segment_Iterator::increment_distance()
{
    if (m_position == Position::begin)
    {
        switch (m_strip)
        {
        case Gl_Road_Segment::LEFT_KERB:
            increment_kerb_distance(*(m_segment.mp_left_kerb));
            break;
        case Gl_Road_Segment::RIGHT_KERB:
            increment_kerb_distance(*(m_segment.mp_right_kerb));
            break;
        default:
            m_distance = 0.0;
        }
        return;
    }
    if (m_side != Side::left)
        return;

    if (m_strip == Gl_Road_Segment::LEFT_KERB)
        increment_kerb_distance(*(m_segment.mp_left_kerb));
    else if (m_strip == Gl_Road_Segment::RIGHT_KERB)
        increment_kerb_distance(*(m_segment.mp_right_kerb));
    else
    {
        if (m_connection)
        {
            m_after_connection = true;
            m_connection = false;
            if (m_strip == Gl_Road_Segment::LEFT_BARRIER
                || m_strip == Gl_Road_Segment::RIGHT_BARRIER
                || m_strip == Gl_Road_Segment::TRACK)
                glEnd();
        }
        else
        {
            m_distance += m_resolution;
            if (m_segment.pit_road_connection() > 0.0
                && !m_after_connection
                && m_distance >= m_segment.pit_road_connection())
            {
                m_distance = m_segment.pit_road_connection();
                m_connection = true;
            }
            else if (m_distance > m_segment.length())
            {
                //  Clamp to the end of the segment.
                m_distance = m_segment.length();
                m_position = Position::end;
            }
        }
    }
}

void Segment_Iterator::increment_kerb_distance(const Kerb& kerb)
{
    switch (m_position)
    {
    case Position::begin:
        m_distance = kerb.transition_start();
        m_position = Position::begin_transition;
        break;
    case Position::begin_transition:
        m_distance = kerb.start();
        m_position = Position::middle;
        break;
    case Position::middle:
        m_distance += m_resolution;
        if (m_distance >= kerb.end())
        {
            m_distance = kerb.end();
            m_position = Position::end_transition;
        }
        break;
    case Position::end_transition:
        m_distance = kerb.transition_end();
        m_position = Position::end;
        break;
    case Position::end:
        assert(false);
    }
}

//----------------------------------------------------------------------------------------
Gl_Road_Segment::Gl_Road_Segment(double resolution, double length, double radius, double skew,
                                 TPoints const& left_width, TPoints const& right_width,
                                 TPoints const& left_road_width, TPoints const& right_road_width,
                                 std::unique_ptr<Kerb> left_kerb, std::unique_ptr<Kerb> right_kerb,
                                 double left_wall_height, double right_wall_height,
                                 TPoints const& elevation_points, double end_bank,
                                 double bank_pivot, std::vector<Material> const& materials,
                                 std::vector<Braking_Marker> const& braking_markers)
    : Road_Segment(length, radius, 10.0, 10.0, 20.0, 20.0),
      m_texture_offsets{N_STRIPS},
      mp_iterator{std::make_unique<Segment_Iterator>(*this, resolution)},
      m_braking_markers{std::move(braking_markers)},
      m_materials{materials}
{
    set_widths(right_width, right_road_width, left_road_width, left_width);
    set_start_skew(skew);
    set_end_skew(skew);
    set_kerb(std::move(left_kerb), Side::left);
    set_kerb(std::move(right_kerb), Side::right);
    set_wall_heights(left_wall_height, right_wall_height);
    set_elevation_points(elevation_points);
    assert(materials.size() == N_STRIPS);
    set_banking(end_bank, bank_pivot);
}

Gl_Road_Segment::~Gl_Road_Segment()
{
    glDeleteLists(m_gl_list_id, 1);
    for (auto id : m_scenery_lists)
        glDeleteLists(id, 1);
}

const Material& Gl_Road_Segment::left_material(double height) const
{
    return height < left_wall_height() ? m_materials.front() : s_no_material;
}

const Material& Gl_Road_Segment::right_material(double height) const
{
    return height < right_wall_height() ? m_materials.back() : s_no_material;
}

Material const& Gl_Road_Segment::material_at(double along, double from_center) const
{
    if (from_center > left_road_width(along) + kerb_width(Side::left, along))
        return m_materials[LEFT_SHOULDER];
    if (from_center > left_road_width(along))
        return m_materials[LEFT_KERB];
    if (from_center > -right_road_width(along))
        return m_materials[TRACK];
    if (from_center > -right_road_width(along) - kerb_width(Side::right, along))
        return m_materials[RIGHT_KERB];
    return m_materials[RIGHT_SHOULDER];
}

void Gl_Road_Segment::build()
{
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    if (m_gl_list_id != 0)
        glDeleteLists(m_gl_list_id, 1);
    m_gl_list_id = glGenLists(1);
    glNewList(m_gl_list_id, GL_COMPILE);

    // Put the textures on a shiny white surface so we can see shading.
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(1.0, 1.0, 1.0);
    GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    GLfloat shininess[] = {128.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    assert(m_texture_offsets.size() == N_STRIPS);

    // Draw the left barrier.
    draw_strip(LEFT_BARRIER, m_texture_offsets[LEFT_BARRIER]);

    // Draw the left shoulder and kerb.
    glDepthMask(GL_FALSE);
    draw_strip(LEFT_SHOULDER, m_texture_offsets[LEFT_SHOULDER]);
    glDepthMask(GL_TRUE);
    draw_strip(LEFT_KERB, m_texture_offsets[LEFT_KERB]);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    draw_strip(LEFT_SHOULDER, m_texture_offsets[LEFT_SHOULDER]);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Draw the track.
    draw_strip(TRACK, m_texture_offsets[TRACK]);

    // Draw the right shoulder and kerb.
    glDepthMask(GL_FALSE);
    draw_strip(RIGHT_SHOULDER, m_texture_offsets[RIGHT_SHOULDER]);
    glDepthMask(GL_TRUE);
    draw_strip(RIGHT_KERB, m_texture_offsets[RIGHT_KERB]);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    draw_strip(RIGHT_SHOULDER, m_texture_offsets[RIGHT_SHOULDER]);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Draw the right barrier.
    draw_strip(RIGHT_BARRIER, m_texture_offsets[RIGHT_BARRIER]);
    glFlush();

    // Draw braking markers.
    for (auto const& marker : m_braking_markers)
    {
        auto along{length() - marker.distance};
        auto from_center{marker.side == Side::right
                         ? -marker.offset.x - right_road_width(along)
                         : marker.offset.x + left_road_width(along) + marker.image->width()};
        auto angle{start_angle() + arc() * along / length()};
        auto r{is_straight()
               ? start_coords() + rotate(Three_Vector{along, from_center, 0.0}, angle * z_hat)
               : center_of_curve() + Three_Vector(radius() - from_center, angle - pi / 2.0)};
        r.z = elevation(along, from_center) + marker.offset.y;

        glPushMatrix();
        glTranslatef(r.x, r.y, r.z);
        glRotatef(rad_to_deg(angle) - 90.0, 0.0, 0.0, 1.0);
        glRotatef(90.0, 1.0, 0.0, 0.0);
        marker.image->draw();
        glPopMatrix();
    }

    // Draw scenery
    glPushMatrix();
    glTranslatef(start_coords().x, start_coords().y, start_coords().z);
    glRotatef(rad_to_deg(start_angle()), 0.0, 0.0, 1.0);
    std::for_each(m_scenery_lists.begin(), m_scenery_lists.end(), glCallList);
    glPopMatrix();
    glEndList();
}

void Gl_Road_Segment::draw_strip(Strip strip, double texture_offset)
{
    auto texture{m_materials[strip].texture()};
    if (!texture)
        return;

    texture->activate();
    mp_iterator->start(start_coords(), strip);
    auto vertex{(++(*mp_iterator)).coordinates()};
    glNormal3d(mp_iterator->normal().x, mp_iterator->normal().y, mp_iterator->normal().z);
    m_bounds.enclose(Rectangle<double>({vertex.x, vertex.y}, {vertex.x, vertex.y}));

    auto tex_width{texture->width()};
    auto tex_height{texture->height()};
    auto tex_x{tex_width > 0.0 ? mp_iterator->texture_coordinates().x / tex_width : 0.0};
    auto tex_y{tex_height > 0.0 ? mp_iterator->texture_coordinates().y / tex_height : 0.0};
    tex_y += texture_offset;
    glTexCoord2d(tex_x, tex_y);
    glVertex3d(vertex.x, vertex.y, vertex.z);

    vertex = (++(*mp_iterator)).coordinates();
    glNormal3d(mp_iterator->normal().x, mp_iterator->normal().y, mp_iterator->normal().z);
    m_bounds.enclose({{vertex.x, vertex.y}, {vertex.x, vertex.y}});

    tex_x = tex_width > 0.0 ? mp_iterator->texture_coordinates().x / tex_width : 1.0;
    glTexCoord2d(tex_x, tex_y);
    glVertex3d(vertex.x, vertex.y, vertex.z);

    while (!mp_iterator->last_subdivision())
    {
        vertex = (++(*mp_iterator)).coordinates();
        glNormal3d(mp_iterator->normal().x, mp_iterator->normal().y, mp_iterator->normal().z);
        m_bounds.enclose({{vertex.x, vertex.y}, {vertex.x, vertex.y}});

        tex_x = tex_width > 0.0 ? mp_iterator->texture_coordinates().x / tex_width : 0.0;
        tex_y = tex_height > 0.0 ? mp_iterator->texture_coordinates().y / tex_height : 1.0;
        tex_y += texture_offset;
        glTexCoord2d(tex_x, tex_y);
        glVertex3d(vertex.x, vertex.y, vertex.z);

        vertex = (++(*mp_iterator)).coordinates();
        glNormal3d(mp_iterator->normal().x, mp_iterator->normal().y, mp_iterator->normal().z);
        m_bounds.enclose({{vertex.x, vertex.y}, {vertex.x, vertex.y}});

        tex_x = tex_width > 0.0 ? mp_iterator->texture_coordinates().x / tex_width : 1.0;
        glTexCoord2d(tex_x, tex_y);
        glVertex3d(vertex.x, vertex.y, vertex.z);
    }
    glEnd();
    m_texture_offsets[strip] = tex_y;
}

void Gl_Road_Segment::draw() const
{
    glCallList(m_gl_list_id);
}

void Gl_Road_Segment::add_model(Ac3d&& model)
{
    m_scenery_lists.push_back(model.build());
}

void Gl_Road_Segment::set_start(Three_Vector const& start_coords, double start_distance,
                                double start_angle, double start_bank,
                                std::vector<double> const& texture_offsets)
{
    Road_Segment::set_start(start_coords, start_distance, start_angle, start_bank, texture_offsets);
    m_texture_offsets = texture_offsets;
}
