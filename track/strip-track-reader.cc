//  Copyright (C) 2005-2022 Sam Varner
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

#include "strip-track.h"

#include "../geometry/two-vector.h"
#include "../media/xml-parser.h"

#include <pugixml.hpp>

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;

using VV2 = std::vector<Two_Vector>;

auto get_side = [](auto node, auto const& tag, Side fallback) {
    auto child{node.child(tag)};
    return child
        ? std::string(child.text().as_string()) == "left" ? Side::left : Side::right
        : fallback;
};

auto get_if_present = [](auto node, auto const& tag, auto& value) {
    auto child{node.child(tag)};
    if (!child)
        return;
    std::istringstream is{child.text().as_string()};
    is >> value;
};

auto get_points = [](auto node, auto const& tag, VV2 points = {}) {
    for (auto child : node.children(tag))
    {
        std::istringstream is{child.text().as_string()};
        Point<double> p;
        is >> p;
        // Overwrite a point at the same x value.
        if (!points.empty() && p.x == points.back().x)
            points.back() = p;
        else
            points.push_back(p);
    }
    return points;
};

struct Kerb_Params
{
    double start_trn_length{0.0};
    double start_trn_width{0.0};
    double end_trn_length{0.0};
    double end_trn_width{0.0};
    VV2 profile;
};

static std::unique_ptr<Kerb> read_kerb(pugi::xml_node const& node, char const* child,
                                       Kerb_Params& param, double length)
{
    if (!node.child(child))
        return nullptr;

    auto kerb{node.child(child)};
    if (kerb.child("profile"))
    {
        param.profile.clear();
        std::istringstream is{kerb.child_value("profile")};
        Two_Vector point;
        while (is)
        {
            param.profile.push_back(point);
            is >> point;
        }
    }
    auto has_start{false};
    auto start_dist{0.0};
    if (kerb.child("start"))
    {
        has_start = true;
        auto start{kerb.child("start")};
        start_dist = get_value(start, "distance", 0.0);
        if (start.child("transition"))
        {
            auto transition{start.child("transition")};
            param.start_trn_length = get_value(transition, "length", param.start_trn_length);
            param.start_trn_width = get_value(transition, "width", param.start_trn_width);
        }
    }
    auto has_end{false};
    auto end_dist{length};
    if (kerb.child("end"))
    {
        has_end = true;
        auto end{kerb.child("end")};
        end_dist = get_value(end, "distance", length);
        if (end.child("transition"))
        {
            auto transition{end.child("transition")};
            param.end_trn_length = get_value(transition, "length", param.end_trn_length);
            param.end_trn_width = get_value(transition, "width", param.end_trn_width);
        }
    }
    return std::make_unique<Kerb>(
        param.profile, start_dist,
        has_start ? param.start_trn_length : 0.0, param.start_trn_width, end_dist,
        has_end ? param.end_trn_length : 0.0, param.end_trn_width, end_dist == length);
}

using Mats = std::array<std::string, 7>;

static void read_road(std::string const& data_dir,
                      Strip_Track* track,
                      pugi::xml_node top,
                      std::map<std::string, Material> const& materials,
                      std::map<std::string, Mats> const& seg_mats,
                      std::function<size_t(std::unique_ptr<Gl_Road_Segment>)> adder)
{
    // Most parameters are persistent from segment to segment.
    auto resolution{1.0};
    auto length{1.0};
    auto radius{0.0};
    VV2 left_width{{0.0, 25.0}};
    VV2 right_width{{0.0, 25.0}};
    VV2 left_road_width{{0.0, 8.0}};
    VV2 right_road_width{{0.0, 8.0}};
    Kerb_Params left_kerb_params;
    Kerb_Params right_kerb_params;
    auto left_wall_height{1.0};
    auto right_wall_height{1.0};
    VV2 left_profile{{}};
    VV2 right_profile{{}};
    auto bank{0.0};
    auto bank_pivot{0.0};
    Side marker_side{Side::left};
    Two_Vector marker_size;
    Two_Vector marker_offset;
    auto curve_factor{1.0};
    Camera camera;

    for (auto road_seg : top.children("road"))
    {
        auto name{road_seg.attribute("segment").value()};
        resolution = get_value(road_seg, "resolution", resolution);
        length = get_value(road_seg, "length", 0.0);
        radius = get_value(road_seg, "radius", 0.0);
        auto skew{get_value(road_seg, "skew", 0.0)}; // not persistent
        left_width = get_points(road_seg, "left-width", {{0.0, left_width.back().y}});
        right_width = get_points(road_seg, "right-width", {{0.0, right_width.back().y}});
        left_road_width
            = get_points(road_seg, "left-road-width", {{0.0, left_road_width.back().y}});
        right_road_width
            = get_points(road_seg, "right-road-width", {{0.0, right_road_width.back().y}});
        left_wall_height = get_value(road_seg, "left-wall-height", left_wall_height);
        right_wall_height = get_value(road_seg, "right-wall-height", right_wall_height);
        VV2 elevation{get_points(road_seg, "elevation")};

        bank = get_value(road_seg, "bank", bank);
        bank_pivot = get_value(road_seg, "bank-pivot", bank_pivot);

        std::vector<Material> mats;
        for (auto const& mat_name : seg_mats.at(name))
            mats.push_back(materials.at(mat_name));

        std::unique_ptr<Kerb> left_kerb{
            read_kerb(road_seg, "left-kerb", left_kerb_params, length)};
        std::unique_ptr<Kerb> right_kerb{
            read_kerb(road_seg, "right-kerb", right_kerb_params, length)};

        std::vector<Braking_Marker> markers;
        for (auto marker : road_seg.children("braking-marker"))
        {
            marker_side = get_side(marker, "side", marker_side);
            marker_size = get_value(marker, "size", marker_size);
            marker_offset = get_value(marker, "offset", marker_offset);
            markers.emplace_back(data_dir + get_value(marker, "file", std::string()),
                                 get_value(marker, "distance", 0.0),
                                 marker_side,
                                 marker_offset.x, marker_offset.y,
                                 marker_size);
        }
        auto segment{std::make_unique<Gl_Road_Segment>(
                resolution, length, radius, skew, left_width, right_width,
                left_road_width, right_road_width,
                std::move(left_kerb), std::move(right_kerb),
                left_wall_height, right_wall_height, elevation,
                bank, bank_pivot, mats, markers)};
        auto n_segments{track->get_road(0).segments().size()};

        if (road_seg.child("pit-in"))
        {
            auto pit{road_seg.child("pit-in")};
            auto side{get_side(pit, "side", Side::left)};
            auto split{get_value(pit, "split", 0.0)};
            auto merge{get_value(pit, "merge", 0.0)};
            auto angle{get_value(pit, "angle", 0.0)};
            track->set_pit_in(n_segments, angle);
            segment->set_pit_lane(Pit_Lane_Transition::End::in, side, split, merge, angle);
        }
        if (road_seg.child("pit-out"))
        {
            auto pit{road_seg.child("pit-out")};
            auto side{get_side(pit, "side", Side::left)};
            auto join{get_value(pit, "join", 0.0)};
            auto merge{get_value(pit, "merge", 0.0)};
            auto angle{get_value(pit, "angle", 0.0)};
            track->set_pit_out(n_segments, angle);
            segment->set_pit_lane(Pit_Lane_Transition::End::out, side, join, merge, angle);
        }
        segment->set_racing_line_adjustment(
            get_value(road_seg, "racing-line-adjustment", 0.0), -1.0);
        curve_factor = get_value(road_seg, "curvature-factor", curve_factor);
        segment->set_racing_line_curvature_factor(curve_factor);

        for (auto cam_node : road_seg.children("camera"))
        {
            camera.segment_index = n_segments;
            camera.position = get_value(cam_node, "position", camera.position);
            camera.range = get_value(cam_node, "range", camera.range);
            camera.vertical_field_angle
                = get_value(cam_node, "field", camera.vertical_field_angle);
            camera.fixed = cam_node.child("direction");
            camera.direction = get_value(cam_node, "direction", camera.direction);
            track->add_camera(camera);
        }
        for (auto model : road_seg.children("model"))
            segment->add_model(get_model(model, data_dir + "tracks/"));

        adder(std::move(segment));
    }
}

namespace Vamos_Track
{
void read_track_file(std::string const& data_dir,
                     std::string const& file_name,
                     Strip_Track* track)
{
    std::map<std::string, Material::Composition> mat_types{{"rubber", Material::rubber},
                                                           {"metal", Material::metal},
                                                           {"asphalt", Material::asphalt},
                                                           {"concrete", Material::concrete},
                                                           {"kerb", Material::kerb},
                                                           {"grass", Material::grass},
                                                           {"gravel", Material::gravel},
                                                           {"dirt", Material::dirt}};

    pugi::xml_document doc;
    auto result{doc.load_file(file_name.c_str())};
    auto top{doc.child("track")};
    if (top.child("racing-line"))
    {
        auto line{top.child("racing-line")};
        get_if_present(line, "iterations", track->mp_track->m_racing_line.m_iterations);
        get_if_present(line, "stiffness", track->mp_track->m_racing_line.m_stiffness);
        get_if_present(line, "damping", track->mp_track->m_racing_line.m_damping);
        get_if_present(line, "margin", track->mp_track->m_racing_line.m_margin);
    }
    if (top.child("sky"))
    {
        auto sky{top.child("sky")};
        track->set_sky_box(data_dir + sky.child_value("sides"),
                          data_dir + sky.child_value("top"),
                          data_dir + sky.child_value("bottom"),
                          sky.child("smooth"));
    }
    if (top.child("map-background"))
    {
        auto background{top.child("map-background")};
        track->set_map_background(
            background.child_value("image"),
            Rectangle<int>{get_value(background, "offset", Point{0, 0}),
                           get_value(background, "size", Point{2000, 2000})});
    }
    std::map<std::string, Material> materials;
    for (auto mat : top.children("material"))
    {
        auto name{mat.attribute("name").value()};
        auto type{mat.attribute("type").value()};
        auto composition{mat_types.count(type) ? mat_types[type] : Material::unknown};
        if (composition == Material::unknown)
            std::cerr << "Strip_Track_Reader: Warning: Unknown material \"" << type
                      << '\"' << std::endl;

        auto tex{mat.child("texture")};
        auto texture{std::make_shared<Texture_Image>(
                data_dir + tex.child_value("file"),
                tex.child("smooth"),
                tex.child("mipmap"),
                Point{get_value(tex, "width", 0.0), get_value(tex, "length", 0.0)})};
        materials.emplace(name, Material{composition,
                                         get_value(mat, "friction", 1.0),
                                         get_value(mat, "restitution", 1.0),
                                         get_value(mat, "rolling", 1.0),
                                         get_value(mat, "drag", 1.0),
                                         get_value(mat, "bump-amplitude", Two_Vector{0.0, 0.0}),
                                         get_value(mat, "bump-wavelength", 1.0),
                                         texture});
    }
    std::map<std::string, Mats> seg_mats;
    for (auto seg : top.children("segment"))
    {
        auto name{seg.attribute("name").value()};
        std::istringstream is{seg.text().as_string()};
        char delim;
        is >> delim;
        Mats mats;
        for (auto& mat : mats)
            is >> mat;
        seg_mats.emplace(name, mats);
    }
    track->set_start_direction(get_value(top, "start-direction", 0.0));

    read_road(data_dir, track, top, materials, seg_mats,
              std::bind_front(&Strip_Track::add_segment, track));
    if (track->get_road(0).segments().empty())
        throw std::runtime_error(file_name + ": No road segments read");

    auto close{false};
    auto adjusted_segs{3};
    if (top.child("circuit"))
    {
        close = true;
        auto circuit{top.child("circuit")};
        if (circuit.attribute("segments"))
            adjusted_segs = atoi(circuit.attribute("segments").value());
        if (adjusted_segs > static_cast<int>(track->get_road(0).segments().size()))
            throw std::runtime_error(
                file_name + ": Number of adjusted segments > number of segments");
    }
    auto last_dist{0.0};
    for (auto line : top.children("timing-line"))
    {
        auto dist{line.text().as_double()};
        if (dist < last_dist)
            throw std::runtime_error(file_name + ": Timing lines must be monotonic");
        track->timing_line(dist);
        last_dist = dist;
    }

    auto join_pit{false};
    auto adjusted_pit{3};
    if (top.child("pit"))
    {
        auto pit{top.child("pit")};
        read_road(data_dir, track, pit, materials, seg_mats,
                  std::bind_front(&Strip_Track::add_pit_segment, track));
        if (pit.child("join"))
        {
            join_pit = true;
            auto join{pit.child("join")};
            if (join.attribute("segments"))
                adjusted_pit = atoi(join.attribute("segments").value());
        }
    }
    track->build(close, adjusted_segs, get_value(top, "track-length", 0.0),
                join_pit, adjusted_pit);

    if (last_dist > track->get_road(0).length())
        throw std::runtime_error(file_name + ": Timing lines must be < length");
}
}
