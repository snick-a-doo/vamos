//  Copyright (C) 2008-2022 Sam Varner
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

#include "aerodynamics.h"
#include "brake.h"
#include "car.h"
#include "clutch.h"
#include "dashboard.h"
#include "differential.h"
#include "engine.h"
#include "fuel-tank.h"
#include "particle.h"
#include "suspension.h"
#include "tire.h"
#include "transmission.h"
#include "wheel.h"

#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"
#include "../media/xml.h"

#include <pugixml.hpp>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

/// Factory for gauges of various types.
static std::unique_ptr<Gauge<double>> get_gauge(auto tag, std::string const& dir)
{
    auto type{std::string(tag.attribute("type").value())};
    auto pos{get_value(tag, "position", null_v)};
    auto on_wheel{static_cast<bool>(tag.child("on-steering-wheel"))};
    if (type == "digital")
        return std::make_unique<Digital_Gauge>(
            pos, get_value(tag, "size", Point<double>{1.0, 1.0}),
            get_value(tag, "places", 1),
            dir + get_value(tag, "image", std::string{}), on_wheel);
    if (type == "LED")
        return std::make_unique<LED_Gauge>(
            pos, get_value(tag, "width", 1.0), get_value(tag, "elements", 1),
            get_value(tag, "range", Point<double>{0.0, 1.0}),
            dir + get_value(tag, "image", std::string{}), on_wheel);
    return std::make_unique<Dial>(
        pos, get_value(tag, "radius", 1.0),
        get_value(tag, "min", Point<double>{0.0, 0.0}),
        get_value(tag, "max", Point<double>{1.0, 360.0}),
        dir + get_value(tag, "face", std::string{}),
        dir + get_value(tag, "needle", std::string{}));
}

namespace Vamos_Body
{
void read_car(std::string const& data_dir, std::string const& file_name, Car* car)
{
    pugi::xml_document doc;
    auto result{doc.load_file(file_name.c_str())};
    auto top{doc.child("car")};
    if (auto robot{top.child("robot")})
        car->set_robot_parameters(get_value(robot, "slip-ratio", 0.0),
                                  get_value(robot, "deceleration", 0.0),
                                  get_value(robot, "lateral-acceleration", 0.0));
    auto model_dir{data_dir + "cars/"};
    if (auto model{top.child("exterior-model")})
        car->set_exterior_model(get_model(model, model_dir));
    if (auto model{top.child("interior-model")})
        car->set_interior_model(get_model(model, model_dir));
    if (auto init{top.child("initial-conditions")})
        car->chassis().set_initial_conditions(get_value(init, "position", null_v),
                                              get_value(init, "orientation", null_v),
                                              get_value(init, "velocity", null_v),
                                              get_value(init, "angular-velocity", null_v));
    // Views, front and rear
    {
        Point<double> size{1.0, 1.0};
        auto field_width{60.0};
        auto near_plane{0.5};
        auto far_plane{700.0};
        auto direction{180.0};
        std::string mask;
        if (auto view{top.child("view")})
            car->set_view(get_value(view, "position", null_v),
                          field_width = get_value(view, "field-width", field_width),
                          near_plane = get_value(view, "near-plane", near_plane),
                          far_plane = get_value(view, "far-plane", far_plane),
                          get_value(view, "pan-angle", 90.0));
        for (auto mirror : top.children("mirror"))
            car->add_rear_view(get_value(mirror, "position", null_v),
                               size = get_value(mirror, "size", size),
                               direction = get_value(mirror, "direction", direction),
                               field_width = get_value(mirror, "field-width", field_width),
                               near_plane = get_value(mirror, "near-plane", near_plane),
                               far_plane = get_value(mirror, "far-plane", far_plane),
                               data_dir + "textures/" + (mask = get_value(mirror, "mask", mask)));
    }
    // Dashboard
    if (auto dash{top.child("dashboard")})
    {
        auto dir{data_dir + "textures/"};
        auto dashboard{std::make_unique<Dashboard>(
                get_value(dash, "position", null_v),
                get_value(dash, "tilt", 0.0))};
        for (auto frame : dash.children("mirror-frame"))
            dashboard->add_facade(std::make_unique<Facade>(
                    dir + get_value(frame, "image", std::string{}),
                    get_value(frame, "position", null_v),
                    get_value(frame, "size", Point{1.0, 1.0}), false));
        if (auto tach = dash.child("tachometer"))
            dashboard->add_tachometer(get_gauge(tach, dir));
        if (auto speedo = dash.child("speedometer"))
            dashboard->add_speedometer(get_gauge(speedo, dir));
        if (auto fuel = dash.child("fuel-gauge"))
            dashboard->add_fuel_gauge(get_gauge(fuel, dir));
        if (auto gear = dash.child("gear-indicator"))
            dashboard->add_gear_indicator(
                std::make_unique<Gear_Indicator>(
                    get_value(gear, "position", null_v),
                    get_value(gear, "size", Point<double>{1.0, 1.0}),
                    get_value(gear, "numbers", 7),
                    dir + get_value(gear, "image", std::string{}),
                    static_cast<bool>(gear.child("on-steering-wheel"))));
        if (auto shift{dash.child("gear-shift")})
            dashboard->add_gear_shift(
                std::make_unique<Gear_Shift>(get_value(shift, "position", null_v),
                                             get_value(shift, "size", Point<double>{1.0, 1.0}),
                                             get_value(shift, "rotation", null_v),
                                             get_points(shift, "stick-positions", false),
                                             dir + get_value(shift, "gate", std::string{}),
                                             dir + get_value(shift, "stick", std::string{})));
        if (auto steer{dash.child("steering-wheel")})
            dashboard->add_steering_wheel(
                std::make_unique<Dial>(get_value(steer, "position", null_v),
                                       get_value(steer, "radius", 1.0),
                                       get_value(steer, "min", Point<double>{0.0, 0.0}),
                                       get_value(steer, "max", Point<double>{1.0, 360.0}),
                                       "", dir + get_value(steer, "image", std::string{})));
        car->set_dashboard(std::move(dashboard));
        car->show_dashboard_extras(static_cast<bool>(dash.child("extras")));
    }
    // Steering
    if (auto steer{top.child("steering")})
    {
        car->max_steer_angle(get_value(steer, "max-angle", 20.0));
        car->steer_exponent(get_value(steer, "exponent", 2.0));
        car->steer_speed_sensitivity(get_value(steer, "speed-sensitivity", 0.0));
    }
    // Drivetrain
    if (auto drive{top.child("drivetrain")})
    {
        std::shared_ptr<Engine> engine;
        std::unique_ptr<Clutch> clutch;
        std::unique_ptr<Transmission> transmission;
        std::unique_ptr<Differential> differential;
        if (auto eng_tag{drive.child("engine")})
        {
            engine = std::make_shared<Engine>(get_value(eng_tag, "mass", 1.0),
                                              get_value(eng_tag, "position", null_v),
                                              get_value(eng_tag, "max-power", 0.0),
                                              get_value(eng_tag, "peak-rpm", 0.0),
                                              get_value(eng_tag, "rpm-limit", 0.0),
                                              get_value(eng_tag, "inertia", 1.0),
                                              get_value(eng_tag, "idle", 0.0),
                                              get_value(eng_tag, "start-rpm", 0.0),
                                              get_value(eng_tag, "stall-rpm", 0.0),
                                              get_value(eng_tag, "fuel-consumption", 0.0));
            if (eng_tag.child("torque-curve"))
            {
                engine->set_torque_curve(get_points(eng_tag, "torque-curve", true));
                engine->set_friction(get_value(eng_tag, "friction", 0.0));
            }
            if (auto sound_tag{eng_tag.child("sound")})
                car->set_engine_sound(
                    data_dir + "sounds/" + get_value(sound_tag, "file", std::string{}),
                    get_value(sound_tag, "pitch", 1.0),
                    get_value(sound_tag, "volume", 1.0),
                    get_value(sound_tag, "throttle-volume-factor", 1.0),
                    get_value(sound_tag, "engine-speed-volume-factor", 1.0));
        }
        if (auto clutch_tag{drive.child("clutch")})
            clutch = std::make_unique<Clutch>(get_value(clutch_tag, "sliding", 0.0),
                                              get_value(clutch_tag, "radius", 1.0),
                                              get_value(clutch_tag, "area", 1.0),
                                              get_value(clutch_tag, "max-pressure", 1.0));
        if (auto trans_tag{drive.child("transmission")})
        {
            transmission = std::make_unique<Transmission>(
                get_value(trans_tag, "forward-gears", 1),
                get_value(trans_tag, "first-ratio", 1.0),
                get_value(trans_tag, "last-ratio", 1.0));
            car->shift_delay(get_value(trans_tag, "shift-delay", 0.2));
        }
        if (auto diff_tag{drive.child("differential")})
            differential = std::make_unique<Differential>(
                get_value(diff_tag, "final-drive", 1.0),
                get_value(diff_tag, "anti-slip", 1.0));
        car->mp_drivetrain = std::make_unique<Drivetrain>(
            engine, std::move(clutch), std::move(transmission), std::move(differential));
        car->chassis().add_particle(engine);
    }
    if (auto tank{top.child("fuel-tank")})
    {
        car->mp_fuel_tank = std::make_shared<Fuel_Tank>(get_value(tank, "position", null_v),
                                                        get_value(tank, "capacity", 1.0),
                                                        get_value(tank, "volume", 1.0),
                                                        get_value(tank, "fuel-density", 1.0));
        car->chassis().add_particle(car->mp_fuel_tank);
    }
    // Masses persist if not specified.
    auto mass{0.0};
    std::string type{"unknown"};
    auto friction{0.0};
    auto restitution{0.0};
    for (auto part : top.children("particle"))
        car->chassis().add_particle(std::make_shared<Particle>(
                                        mass = get_value(part, "mass", mass),
                                        get_value(part, "position", null_v)));
    for (auto part : top.children("contact-point"))
    {
        type = get_value(part, "material", type);
        Material material{type == "rubber" ? Material::rubber
                          : type == "metal" ? Material::metal
                          : Material::unknown,
                          friction = get_value(part, "friction", friction),
                          restitution = get_value(part, "restitution", restitution)};
        car->chassis().add_particle(std::make_shared<Particle>(
                                        mass = get_value(part, "mass", mass),
                                        get_value(part, "position", null_v),
                                        material));
    }
    for (auto part : top.children("drag"))
        car->chassis().add_particle(std::make_shared<Drag>(
                                        get_value(part, "position", null_v),
                                        get_value(part, "frontal-area", 0.0),
                                        get_value(part, "drag-coefficient", 0.0)));
    for (auto part : top.children("wing"))
        car->chassis().add_particle(std::make_shared<Wing>(
                                        get_value(part, "position", null_v),
                                        get_value(part, "frontal-area", 0.0),
                                        get_value(part, "surface-area", 0.0),
                                        get_value(part, "lift-coefficient", 0.0),
                                        get_value(part, "efficiency", 1.0)));
    // Wheels
    std::shared_ptr<Suspension> last_suspension;
    // Brake params.
    auto b_friction{1.0};
    auto b_radius{1.0};
    auto b_area{1.0};
    auto b_max_p{1.0};
    auto b_front_bias{0.5};
    // Suspension
    auto s_k{1.0};
    auto s_bounce{1.0};
    auto s_rebound{1.0};
    auto s_travel{1.0};
    auto s_max_v{1.0};
    auto s_camber{0.0};
    auto s_caster{0.0};
    auto s_toe{0.0};
    // Tire
    auto t_radius{1.0};
    Point<double> t_rolling{0.0, 0.0};
    Longi_Params t_longi{0};
    Trans_Params t_trans{0};
    Align_Params t_align{0};
    auto t_hardness{1.0};
    auto t_inertia{1.0};
    // Wheel
    auto w_mass{1.0};
    auto w_offset{0.0};
    auto w_roll_h{0.0};
    auto w_rest{1.0};
    std::string w_slow;
    std::string w_fast;
    auto w_trans_v{1.0};
    std::string w_stator;
    auto w_stator_off{0.0};
    auto w_scale{1.0};
    auto w_trans{null_v};
    auto w_rot{null_v};

    for (auto wheel_tag : top.children("wheel"))
    {
        auto side{std::string{wheel_tag.attribute("side").value()} == "left"
                  ? Side::left : Side::right};
        auto front{std::string{wheel_tag.attribute("end").value()} == "front"};
        auto susp{wheel_tag.child("suspension")};
        auto suspension{std::make_shared<Suspension>(
                get_value(susp, "position", null_v),
                get_value(susp, "hinge", null_v),
                side,
                s_k = get_value(susp, "spring-constant", s_k),
                s_bounce = get_value(susp, "bounce", s_bounce),
                s_rebound = get_value(susp, "rebound", s_rebound),
                s_travel = get_value(susp, "travel", s_travel),
                s_max_v = get_value(susp, "max-compression-velocity", s_max_v))};
        suspension->camber(s_camber = get_value(susp, "camber", s_camber));
        suspension->caster(s_caster = get_value(susp, "caster", s_caster));
        suspension->toe(s_toe = get_value(susp, "toe", s_toe));
        if (auto ar{susp.child("anti-roll")})
            if (last_suspension)
                suspension->anti_roll(last_suspension, get_value(susp, "anti-roll", 0.0));
        last_suspension = suspension;
        for (auto model : susp.children("model"))
            suspension->set_model(model_dir + get_value(model, "file", std::string()),
                                  get_value(model, "scale", 1.0),
                                  get_value(model, "translate", null_v),
                                  get_value(model, "rotate", null_v));
        car->chassis().add_particle(suspension->get_hinge());
        car->chassis().add_particle(suspension);

        auto tire_tag{wheel_tag.child("tire")};
        auto fric{tire_tag.child("friction")};
        Tire_Friction friction{t_longi = get_array(fric, "longitudinal", t_longi),
                               t_trans = get_array(fric, "transverse", t_trans),
                               t_align = get_array(fric, "aligning", t_align)};
        Tire tire{t_radius = get_value(tire_tag, "radius", t_radius),
                  t_rolling = get_value(tire_tag, "rolling-resistance", t_rolling),
                  friction,
                  t_hardness = get_value(tire_tag, "hardness", t_hardness),
                  t_inertia = get_value(tire_tag, "rotational-inertia", t_inertia)};
        auto brake_tag{wheel_tag.child("brakes")};
        b_front_bias = get_value(brake_tag, "front-bias", b_front_bias);
        Brake brake{b_friction = get_value(brake_tag, "friction", b_friction),
                    b_radius = get_value(brake_tag, "radius", b_radius),
                    b_area = get_value(brake_tag, "area", b_area),
                    b_max_p = get_value(brake_tag, "max-pressure", b_max_p),
                    front ? b_front_bias : 1.0 - b_front_bias};
        auto wheel{std::make_shared<Wheel>(
                w_mass = get_value(wheel_tag, "mass", w_mass),
                get_value(wheel_tag, "position", null_v),
                w_offset = get_value(tire_tag, "offset", w_offset),
                w_roll_h = get_value(wheel_tag, "roll-height", w_roll_h),
                w_rest = get_value(wheel_tag, "restitution", w_rest),
                suspension, tire, brake,
                get_flag(wheel_tag, "steered"),
                get_flag(wheel_tag, "driven"),
                side)};
        car->chassis().add_particle(wheel);
        car->m_wheels.push_back(wheel);
        auto model{wheel_tag.child("model")};
        wheel->set_models(w_slow = model_dir + get_value(model, "slow-file", w_slow),
                          w_fast = model_dir + get_value(model, "fast-file", w_fast),
                          w_trans_v = get_value(model, "transition-speed", w_trans_v),
                          w_stator = model_dir + get_value(model, "stator-file", w_stator),
                          w_stator_off = get_value(model, "stator-offset", w_stator_off),
                          w_scale = get_value(model, "scale", w_scale),
                          w_trans = get_value(model, "translate", w_trans),
                          w_rot = get_value(model, "rotate", w_rot));
    }
}
}
