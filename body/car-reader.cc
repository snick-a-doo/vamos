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

#include "../geometry/conversions.h"
#include "../geometry/numeric.h"
#include "../geometry/rectangle.h"
#include "../media/texture-image.h"

#include <cassert>
#include <iostream>
#include <iterator>
#include <sstream>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

Car_Reader::Car_Reader(std::string const& data_dir, std::string const& car_file, Car* car)
    : m_data_dir(data_dir),
      mp_car(car),
      m_tachometer_type("dial"),
      m_speedometer_type("dial"),
      m_fuel_gauge_type("dial")
{
    mp_car->m_chassis.m_particles.clear();
    read(car_file);
}

void Car_Reader::on_start_tag(const Vamos_Media::XML_Tag& tag)
{
    auto const& attribs{tag.get_attributes()};

    if (match("/car"))
        mp_car->m_name = attribs.at("name");
    else if (match("/car/robot"))
        m_doubles.resize(3);
    else if (match("/car/exterior-model") || match("/car/interior-model"))
    {
        m_strings.clear();
        m_strings.resize(2);
        m_doubles.resize(1);
        m_doubles[0] = 1.0;
        m_vectors.clear();
        m_vectors.resize(2);
    }
    else if (match("initial-conditions"))
    {
        m_vectors.clear();
        m_vectors.resize(4);
    }
    else if (match("view"))
    {
        m_vectors.clear();
        m_doubles.clear();
        m_doubles.resize(4);
    }
    else if (match("mirror"))
    {
        m_vectors.clear();
        m_doubles.resize(6);
        m_strings.resize(1);
    }
    else if (match("steering"))
    {
        m_doubles.clear();
        m_doubles.resize(3);
    }
    else if (match("dashboard"))
    {
        m_ints.clear();
        m_ints.resize(1);
        m_doubles.clear();
        m_doubles.resize(12);
        m_strings.clear();
        m_strings.resize(2);
        m_bools.clear();
        m_bools.resize(1);
        m_vectors.clear();
        m_vectors.resize(1);
        m_points.clear();
        m_mirrors.clear();
    }
    else if (match("on-steering-wheel"))
        m_bools[0] = true;
    else if (match("tachometer"))
        m_tachometer_type = attribs.empty() ? "dial" : attribs.at("type");
    else if (match("speedometer"))
        m_speedometer_type = attribs.empty() ? "dial" : attribs.at("type");
    else if (match("fuel-gauge"))
        m_fuel_gauge_type = attribs.empty() ? "dial" : attribs.at("type");
    else if (match("drivetrain"))
    {
        m_bools.clear();
        m_bools.resize(1);
    }
    else if (match("engine"))
    {
        m_doubles.clear();
        m_doubles.resize(14);
        m_vectors.clear();
        m_strings.clear();
        m_strings.resize(1);
        m_bools.clear();
        m_bools.resize(1);
    }
    else if (match("torque-curve"))
    {
        m_points.clear();
        m_bools[0] = true;
        m_doubles[9] = atof(attribs.at("friction").c_str());
    }
    else if (match("clutch"))
        m_doubles.clear();
    else if (match("transmission"))
    {
        m_doubles.clear();
        m_gears.clear();
    }
    else if (match("differential"))
        m_doubles.clear();
    else if (match("fuel-tank"))
    {
        m_doubles.clear();
        m_vectors.clear();
    }
    else if (match("contact-point"))
    {
        m_doubles.resize(3);
        m_strings.resize(1);
        m_vectors.clear();
    }
    else if (match("particle"))
    {
        m_doubles.resize(1);
        m_vectors.clear();
    }
    else if (match("drag"))
    {
        m_doubles.resize(2);
        m_vectors.clear();
    }
    else if (match("wing"))
    {
        m_doubles.resize(6);
        m_vectors.clear();
    }
    else if (match("wheel"))
    {
        if (m_doubles.size() != 24)
        {
            m_doubles.resize(24);
            m_doubles[8] = 0.0;
            m_doubles[20] = 0.0;
            m_doubles[23] = 1.0;
            m_strings.resize(3);
            m_vectors.resize(5);
        }
        m_strings[0] = attribs.at("side");
        m_strings[1] = attribs.at("end");
        m_bools.clear();
        m_bools.resize(2);
        m_first_model_for_this_wheel = true;
    }
    else if (match("steered"))
        m_bools[0] = true;
    else if (match("driven"))
        m_bools[1] = true;
}

void Car_Reader::on_end_tag(Vamos_Media::XML_Tag const&)
{
    if (match("/car/robot"))
        mp_car->set_robot_parameters(m_doubles[0], m_doubles[1], m_doubles[2]);
    else if (match("/car/exterior-model") && !m_strings[0].empty())
        mp_car->set_exterior_model(m_data_dir + "cars/" + m_strings[0], m_doubles[0],
                                   m_vectors[0], m_vectors[1]);
    else if (match("/car/interior-model") && !m_strings[0].empty())
        mp_car->set_interior_model(m_data_dir + "cars/" + m_strings[0], m_doubles[0],
                                   m_vectors[0], m_vectors[1]);
    else if (match("initial-conditions"))
        mp_car->chassis().set_initial_conditions(m_vectors[0], m_vectors[1], m_vectors[2],
                                                 m_vectors[3]);
    else if (match( "view"))
        mp_car->set_view(m_vectors[0], m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3]);
    else if (match("mirror"))
        mp_car->add_rear_view(m_vectors[0], m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3],
                              m_doubles[4], m_doubles[5], m_data_dir + "textures/" + m_strings[0]);
    else if (match("steering"))
    {
        mp_car->max_steer_angle(m_doubles[0]);
        mp_car->steer_exponent(m_doubles[1]);
        mp_car->steer_speed_sensitivity(m_doubles[2]);
    }
    else if (match("mirror-frame"))
    {
        auto frame{std::make_unique<Facade>(m_data_dir + "textures/" + m_strings[0], true)};
        frame->set_width(m_doubles[3]);
        frame->set_height(m_doubles[4]);
        frame->set_offset({m_doubles[0], m_doubles[1], m_doubles[2]});
        m_mirrors.push_back(std::move(frame));
    }
    else if (match("tachometer"))
    {
        if (m_tachometer_type == "LED")
        {
            mp_tachometer = std::make_unique<LED_Gauge>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_ints[0],
                m_doubles[4], m_doubles[5], m_data_dir + "textures/" + m_strings[0],
                m_bools[0]);
            m_bools[0] = false;
        }
        else if (m_tachometer_type == "digital")
        {
            mp_tachometer = std::make_unique<Digital_Gauge>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4],
                m_ints[0], m_data_dir + "textures/" + m_strings[0], m_bools[0]);
            m_bools[0] = false;
        }
        else
            mp_tachometer = std::make_unique<Dial>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4],
                m_doubles[5], m_doubles[6], m_doubles[7],
                m_data_dir + "textures/" + m_strings[0],
                m_data_dir + "textures/" + m_strings[1]);
    }
    else if (match("speedometer"))
    {
        if (m_speedometer_type == "digital")
        {
            mp_speedometer = std::make_unique<Digital_Gauge>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4],
                m_ints[0], m_data_dir + "textures/" + m_strings[0], m_bools[0]);
            m_bools[0] = false;
        }
        else
            mp_speedometer = std::make_unique<Dial>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4],
                m_doubles[5], m_doubles[6], m_doubles[7],
                m_data_dir + "textures/" + m_strings[0],
                m_data_dir + "textures/" + m_strings[1]);
    }
    else if (match("fuel-gauge"))
    {
        if (m_fuel_gauge_type == "digital")
        {
            mp_fuel_gauge = std::make_unique<Digital_Gauge>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4],
                m_ints[0], m_data_dir + "textures/" + m_strings[0], m_bools[0]);
            m_bools[0] = false;
        }
        else
            mp_fuel_gauge = std::make_unique<Dial>(
                m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4],
                m_doubles[5], m_doubles[6], m_doubles[7],
                m_data_dir + "textures/" + m_strings[0],
                m_data_dir + "textures/" + m_strings[1]);
    }
    else if (match("gear-indicator"))
    {
        mp_gear_indicator = std::make_unique<Gear_Indicator>(
            Rectangle{m_doubles[0], m_doubles[1], m_doubles[3], m_doubles[4]}, m_doubles[2],
            m_ints[0], m_data_dir + "textures/" + m_strings[0], m_bools[0]);
        m_bools[0] = false;
    }
    else if (match("gear-shift"))
        mp_gear_indicator = std::make_unique<Gear_Shift>(
            Rectangle{m_doubles[0], m_doubles[1], m_doubles[3], m_doubles[4]}, m_doubles[2],
            m_vectors[0], m_points, m_data_dir + "textures/" + m_strings[0],
            m_data_dir + "textures/" + m_strings[1]);
    else if (match("steering-wheel"))
        mp_steering_wheel = std::make_unique<Steering_Wheel>(
            m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3], m_doubles[4], m_doubles[5],
            m_doubles[6], m_doubles[7], m_data_dir + "textures/" + m_strings[0]);
    else if (match("dashboard"))
    {
        auto dash{std::make_unique<Dashboard>(
                m_doubles[8], m_doubles[9], m_doubles[10], m_doubles[11])};
        for (auto& mirror : m_mirrors)
            dash->add_facade(std::move(mirror));
        dash->add_tachometer(std::move(mp_tachometer));
        dash->add_speedometer(std::move(mp_speedometer));
        dash->add_fuel_gauge(std::move(mp_fuel_gauge));
        dash->add_gear_indicator(std::move(mp_gear_indicator));
        dash->add_steering_wheel(std::move(mp_steering_wheel));
        mp_car->set_dashboard(std::move(dash));
    }
    else if (match("/car/dashboard/extras"))
        mp_car->show_dashboard_extras(true);
    else if (match("engine"))
    {
        mp_engine = std::make_shared<Engine>(m_doubles[0], m_vectors[0], m_doubles[1], m_doubles[2],
                                             m_doubles[3], m_doubles[4], m_doubles[5], m_doubles[6],
                                             m_doubles[7], m_doubles[8]);
        if (m_bools[0])
        {
            mp_engine->set_torque_curve(m_points);
            mp_engine->set_friction(m_doubles[9]);
        }
        mp_car->set_engine_sound(m_data_dir + "sounds/" + m_strings[0], m_doubles[10],
                                 m_doubles[11], m_doubles[12], m_doubles[13]);
    }
    else if (match("clutch"))
    {
        if (m_doubles.size() != 4)
            error("clutch requires 4 arguments");
        mp_clutch
            = std::make_unique<Clutch>(m_doubles[0], m_doubles[1], m_doubles[2], m_doubles[3]);
    }
    else if (match("transmission"))
    {
        if (m_doubles.size() == 4)
        {
            mp_transmission = std::make_unique<Transmission>(static_cast<int>(m_doubles[0]),
                                                             m_doubles[1], m_doubles[2]);
            mp_car->shift_delay(m_doubles[3]);
        }
        else
        {
            mp_transmission = std::make_unique<Transmission>();
            for (auto gear : m_gears)
                mp_transmission->set_gear_ratio(gear.first, gear.second);
            mp_car->shift_delay(m_doubles[1]);
        }
    }
    else if (match("differential"))
    {
        if (m_doubles.size() != 2)
            error("differential requires 2 arguments");
        mp_differential = std::make_unique<Differential>(m_doubles[0], m_doubles[1]);
    }
    else if (match("drivetrain"))
        mp_car->mp_drivetrain = std::make_unique<Drivetrain>(mp_engine, std::move(mp_clutch),
                                                             std::move(mp_transmission),
                                                             std::move(mp_differential));
    else if (match("fuel-tank"))
    {
        if (m_doubles.size() != 3 || m_vectors.size() != 1)
            error("fuel tank requires 1 or 3 elements");
        mp_car->mp_fuel_tank
            = std::make_shared<Fuel_Tank>(m_vectors[0], m_doubles[0], m_doubles[1], m_doubles[2]);
    }
    else if (match("car"))
    {
        if (mp_engine)
            mp_car->chassis().add_particle(mp_engine);
        if (mp_car->mp_fuel_tank)
            mp_car->chassis().add_particle(mp_car->mp_fuel_tank);
    }
    else if (match("particle"))
    {
        mp_car->chassis().add_particle(std::make_shared<Particle>(m_doubles[0], m_vectors[0]));
    }
    else if (match("contact-point"))
    {
        auto material{m_strings[0] == "rubber" ? Material::RUBBER
                      : m_strings[0] == "metal" ? Material::METAL
                      : Material::UNKNOWN};
        mp_car->chassis().add_particle(std::make_shared<Particle>(
            m_doubles[0], m_vectors[0], Material{material, m_doubles[1], m_doubles[2]}));
    }
    else if (match("drag"))
    {
        mp_car->chassis().add_particle(
            std::make_shared<Drag>(m_vectors[0], m_doubles[0], m_doubles[1]));
    }
    else if (match("wing"))
    {
        mp_car->chassis().add_particle(
            std::make_shared<Wing>(m_vectors[0], m_doubles[0], m_doubles[1],
                                   m_doubles[2], m_doubles[3]));
    }
    else if (match("/car/wheel/suspension/model"))
    {
        if (m_first_model_for_this_wheel)
        {
            m_models.clear();
            m_first_model_for_this_wheel = false;
        }
        m_models.emplace_back(m_strings[2], m_doubles[21], m_vectors[3], m_vectors[4]);
    }
    else if (match("wheel"))
    {
        auto side = m_strings[0] == "left" ? Side::left : Side::right;
        auto suspension = std::make_shared<Suspension>(
            m_vectors[1], m_vectors[2], side, m_doubles[0], m_doubles[1], m_doubles[2],
            m_doubles[3], m_doubles[4]);
        suspension->camber(m_doubles[5]);
        suspension->caster(m_doubles[6]);
        suspension->toe(m_doubles[7]);
        if (m_doubles[8] != 0.0 && mp_last_suspension)
        {
            suspension->anti_roll(mp_last_suspension, m_doubles[8]);
            m_doubles[8] = 0.0;
        }
        mp_last_suspension = suspension;
        for (auto const& model : m_models)
            suspension->set_model(m_data_dir + "cars/" + model.file, model.scale,
                                  model.translate, model.rotate);
        mp_car->chassis().add_particle(suspension->get_hinge());
        mp_car->chassis().add_particle(suspension);
        Tire_Friction friction(m_longi_parameters, m_trans_parameters, m_align_parameters);
        Tire tire(m_doubles[9], m_doubles[10], m_doubles[11], friction, m_doubles[23],
                  m_doubles[12]);
        auto bias{m_doubles[17]};
        if (m_strings[1] == "rear")
            bias = 1.0 - bias;
        Brake brake(m_doubles[13], m_doubles[14], m_doubles[15], m_doubles[16], bias);
        auto wheel = std::make_shared<Wheel>(
            m_doubles[18], m_vectors[0], m_doubles[22], m_doubles[20], m_doubles[19],
            suspension, tire, brake, m_bools[0], m_bools[1], side);
        mp_car->chassis().add_particle(wheel);
        mp_car->m_wheels.push_back(wheel);
        if (!m_slow_model.empty())
        {
            auto stator_path{m_stator_model.empty() ? "" : m_data_dir + "cars/" + m_stator_model};
            wheel->set_models(m_data_dir + "cars/" + m_slow_model,
                              m_data_dir + "cars/" + m_fast_model, m_transition, stator_path,
                              m_stator_offset, m_scale, m_translation, m_rotation);
        }
    }
}

void Car_Reader::on_data(std::string const& data)
{
    if (data.empty())
        return;
    std::istringstream is(data.c_str());
    if (match("/car/robot/slip-ratio"))
        is >> m_doubles[0];
    else if (match("/car/robot/deceleration"))
        is >> m_doubles[1];
    else if (match("/car/robot/lateral-acceleration"))
        is >> m_doubles[2];
    else if (match("/car/dashboard/position"))
    {
        char delim;
        is >> delim >> m_doubles[8] >> delim >> m_doubles[9] >> delim >> m_doubles[10];
    }
    else if (match("/car/dashboard/tilt"))
        is >> m_doubles[11];
    else if (match("/car/dashboard/mirror-frame/position")
             || match("/car/dashboard/tachometer/position")
             || match("/car/dashboard/speedometer/position")
             || match("/car/dashboard/fuel-gauge/position")
             || match("/car/dashboard/gear-indicator/position")
             || match("/car/dashboard/gear-shift/position")
             || match("/car/dashboard/steering-wheel/position"))
    {
        char delim;
        is >> delim >> m_doubles[0] >> delim >> m_doubles[1] >> delim >> m_doubles[2];
    }
    else if (match("/car/dashboard/tachometer/radius")
             || match("/car/dashboard/tachometer/width")
             || match("/car/dashboard/speedometer/radius")
             || match("/car/dashboard/fuel-gauge/radius")
             || match("/car/dashboard/steering-wheel/radius"))
        is >> m_doubles[3];
    else if (match("/car/dashboard/tachometer/elements"))
        is >> m_ints[0];
    else if (match("/car/dashboard/tachometer/range"))
    {
        char delim;
        is >> delim >> m_doubles[4] >> delim >> m_doubles[5];
    }
    else if (match("/car/dashboard/tachometer/min")
             || match("/car/dashboard/speedometer/min")
             || match("/car/dashboard/fuel-gauge/min")
             || match("/car/dashboard/steering-wheel/min"))
    {
        char delim;
        is >> delim >> m_doubles[4] >> delim >> m_doubles[5];
    }
    else if (match("/car/dashboard/tachometer/max")
             || match("/car/dashboard/speedometer/max")
             || match("/car/dashboard/fuel-gauge/max")
             || match("/car/dashboard/steering-wheel/max"))
    {
        char delim;
        is >> delim >> m_doubles[6] >> delim >> m_doubles[7];
    }
    else if (match("/car/dashboard/gear-shift/rotation"))
        is >> m_vectors[0];
    else if (match("/car/dashboard/gear-shift/stick-positions"))
    {
        Two_Vector point;
        while (is >> point)
            m_points.push_back(point);
    }
    else if (match("/car/dashboard/mirror-frame/image")
             || match("/car/dashboard/tachometer/face")
             || match("/car/dashboard/tachometer/image")
             || match("/car/dashboard/speedometer/face")
             || match("/car/dashboard/speedometer/image")
             || match("/car/dashboard/fuel-gauge/face")
             || match("/car/dashboard/fuel-gauge/image")
             || match("/car/dashboard/gear-indicator/image")
             || match("/car/dashboard/gear-shift/gate")
             || match("/car/dashboard/steering-wheel/image"))
    {
        is >> m_strings[0];
    }
    else if (match("/car/dashboard/tachometer/needle")
             || match("/car/dashboard/speedometer/needle")
             || match("/car/dashboard/fuel-gauge/needle")
             || match("/car/dashboard/gear-shift/stick"))
        is >> m_strings[1];
    else if (match( "/car/dashboard/mirror-frame/size")
             || match("/car/dashboard/gear-indicator/size")
             || match("/car/dashboard/gear-shift/size")
             || match("/car/dashboard/tachometer/size")
             || match("/car/dashboard/speedometer/size")
             || match("/car/dashboard/fuel-gauge/size"))
    {
        char delim;
        is >> delim >> m_doubles[3] >> delim >> m_doubles[4];
    }
    else if (match("/car/dashboard/gear-indicator/numbers")
             || match("/car/dashboard/tachometer/places")
             || match("/car/dashboard/speedometer/places")
             || match("/car/dashboard/fuel-gauge/places"))
        is >> m_ints[0];
    // Particle
    else if (match("/car/particle/mass"))
        is >> m_doubles[0];
    // Contact Point
    else if (match("/car/contact-point/mass"))
        is >> m_doubles[0];
    else if (match("/car/contact-point/material"))
        is >> m_strings[0];
    else if (match("/car/contact-point/friction"))
        is >> m_doubles[1];
    else if (match( "/car/contact-point/restitution"))
        is >> m_doubles[2];
    // Drag
    else if (match("/car/drag/frontal-area"))
        is >> m_doubles[0];
    else if (match("/car/drag/drag-coefficient"))
        is >> m_doubles[1];
    // Wing
    else if (match("/car/wing/frontal-area"))
        is >> m_doubles[0];
    else if (match("/car/wing/surface-area"))
        is >> m_doubles[1];
    else if (match("/car/wing/lift-coefficient"))
        is >> m_doubles[2];
    else if (match("/car/wing/efficiency"))
        is >> m_doubles[3];
    // Wheel
    else if (match("/car/wheel/position"))
        is >> m_vectors[0];
    else if (match("/car/wheel/mass"))
        is >> m_doubles[18];
    else if (match("/car/wheel/restitution"))
        is >> m_doubles[19];
    else if (match("/car/wheel/roll-height"))
        is >> m_doubles[20];
    // Suspension
    else if (match("/car/wheel/suspension/model/file"))
        is >> m_strings[2];
    else if (match("/car/wheel/suspension/model/scale"))
        is >> m_doubles[21];
    else if (match("/car/wheel/suspension/model/translate"))
        is >> m_vectors[3];
    else if (match("/car/wheel/suspension/model/rotate"))
        is >> m_vectors[4];
    else if (match("/car/wheel/suspension/position"))
        is >> m_vectors[1];
    else if (match("/car/wheel/suspension/hinge"))
        is >> m_vectors[2];
    else if (match("/car/wheel/suspension/spring-constant"))
        is >> m_doubles[0];
    else if (match("/car/wheel/suspension/bounce"))
        is >> m_doubles[1];
    else if (match("/car/wheel/suspension/rebound"))
        is >> m_doubles[2];
    else if (match("/car/wheel/suspension/travel"))
        is >> m_doubles[3];
    else if (match("/car/wheel/suspension/max-compression-velocity"))
        is >> m_doubles[4];
    else if (match("/car/wheel/suspension/camber"))
        is >> m_doubles[5];
    else if (match("/car/wheel/suspension/caster"))
        is >> m_doubles[6];
    else if (match("/car/wheel/suspension/toe"))
        is >> m_doubles[7];
    else if (match("/car/wheel/suspension/anti-roll"))
        is >> m_doubles[8];
    // Tire
    else if (match( "/car/wheel/tire/radius"))
        is >> m_doubles[9];
    else if (match("/car/wheel/tire/offset"))
        is >> m_doubles[22];
    else if (match("/car/wheel/tire/rolling-resistance"))
    {
        char delim;
        is >> delim >> m_doubles[10] >> delim >> m_doubles[11];
    }
    else if (match("/car/wheel/tire/rotational-inertia"))
        is >> m_doubles[12];
    else if (match("/car/wheel/tire/friction/longitudinal"))
    {
        for (auto& param : m_longi_parameters)
        {
            char delim;
            is >> delim >> param;
        }
    }
    else if (match("/car/wheel/tire/friction/transverse"))
    {
        for (auto& param : m_trans_parameters)
        {
            char delim;
            is >> delim >> param;
        }
    }
    else if (match("/car/wheel/tire/friction/aligning"))
    {
        for (auto& param : m_align_parameters)
        {
            char delim;
            is >> delim >> param;
        }
    }
    else if (match("/car/wheel/tire/hardness"))
        is >> m_doubles[23];
    // Brakes
    else if (match("/car/wheel/brakes/friction"))
        is >> m_doubles[13];
    else if (match("/car/wheel/brakes/radius"))
        is >> m_doubles[14];
    else if (match("/car/wheel/brakes/area"))
        is >> m_doubles[15];
    else if (match("/car/wheel/brakes/max-pressure"))
        is >> m_doubles[16];
    else if (match("/car/wheel/brakes/front-bias"))
        is >> m_doubles[17];
    // Transmission
    else if (match("/car/drivetrain/transmission/gear-ratio"))
    {
        std::pair<int, double> pair;
        char delim;
        is >> delim >> pair.first >> delim >> pair.second;
        m_gears.push_back(pair);
    }
    // Initial Conditions
    else if (match("/car/initial-conditions/position"))
        is >> m_vectors[0];
    else if (match("/car/initial-conditions/orientation"))
        is >> m_vectors[1];
    else if (match("/car/initial-conditions/velocity"))
        is >> m_vectors[2];
    else if (match("/car/initial-conditions/angular-velocity"))
        is >> m_vectors[3];
    // View
    else if (match("/car/view/field-width"))
        is >> m_doubles[0];
    else if (match("/car/view/near-plane"))
        is >> m_doubles[1];
    else if (match("/car/view/far-plane"))
        is >> m_doubles[2];
    else if (match("/car/view/pan-angle"))
        is >> m_doubles[3];
    // Rear View
    else if (match("/car/mirror/size"))
    {
        char delim;
        is >> delim >> m_doubles[0] >> delim >> m_doubles[1];
    }
    else if (match("/car/mirror/direction"))
        is >> m_doubles[2];
    else if (match("/car/mirror/field-width"))
        is >> m_doubles[3];
    else if (match("/car/mirror/near-plane"))
        is >> m_doubles[4];
    else if (match("/car/mirror/far-plane"))
        is >> m_doubles[5];
    else if (match("/car/mirror/mask"))
        is >> m_strings[0];
    // Steering
    else if (match("/car/steering/max-angle"))
        is >> m_doubles[0];
    else if (match("/car/steering/exponent"))
        is >> m_doubles[1];
    else if (match("/car/steering/speed-sensitivity"))
        is >> m_doubles[2];
    // Engine
    else if (match("/car/drivetrain/engine/mass"))
        is >> m_doubles[0];
    else if (match("/car/drivetrain/engine/max-power"))
        is >> m_doubles[1];
    else if (match("/car/drivetrain/engine/peak-engine-rpm"))
        is >> m_doubles[2];
    else if (match("/car/drivetrain/engine/rpm-limit"))
        is >> m_doubles[3];
    else if (match("/car/drivetrain/engine/inertia"))
        is >> m_doubles[4];
    else if (match("/car/drivetrain/engine/idle"))
        is >> m_doubles[5];
    else if (match("/car/drivetrain/engine/start-rpm"))
        is >> m_doubles[6];
    else if (match("/car/drivetrain/engine/stall-rpm"))
        is >> m_doubles[7];
    else if (match("/car/drivetrain/engine/fuel-consumption"))
        is >> m_doubles[8];
    else if (match("/car/drivetrain/engine/torque-curve"))
    {
        Two_Vector point;
        while (is >> point)
            m_points.push_back(point);
    }
    else if (match("/car/drivetrain/engine/sound/file"))
        is >> m_strings[0];
    else if (match("/car/drivetrain/engine/sound/volume"))
        is >> m_doubles[10];
    else if (match("/car/drivetrain/engine/sound/throttle-volume-factor"))
        is >> m_doubles[11];
    else if (match("/car/drivetrain/engine/sound/engine-speed-volume-factor"))
        is >> m_doubles[12];
    else if (match("/car/drivetrain/engine/sound/pitch"))
        is >> m_doubles[13];
    else if (match("/car/exterior-model/file") || match("/car/interior-model/file"))
        is >> m_strings[0];
    else if (match("/car/exterior-model/scale") || match("/car/interior-model/scale"))
        is >> m_doubles[0];
    else if (match("/car/exterior-model/translate")
             || match("/car/interior-model/translate"))
        is >> m_vectors[0];
    else if (match("/car/exterior-model/rotate") || match("/car/interior-model/rotate"))
    {
        is >> m_vectors[1];
        m_vectors[1]
            = {deg_to_rad(m_vectors[1].x), deg_to_rad(m_vectors[1].y), deg_to_rad(m_vectors[1].z)};
    }
    else if (match("/car/wheel/model/slow-file"))
        is >> m_slow_model;
    else if (match("/car/wheel/model/fast-file"))
        is >> m_fast_model;
    else if (match("/car/wheel/model/stator-file"))
        is >> m_stator_model;
    else if (match("/car/wheel/model/transition-speed"))
        is >> m_transition;
    else if (match("/car/wheel/model/stator-offset"))
        is >> m_stator_offset;
    else if (match("/car/wheel/model/scale"))
        is >> m_scale;
    else if (match("/car/wheel/model/translate"))
        is >> m_translation;
    else if (match("/car/wheel/model/rotate"))
    {
        is >> m_rotation;
        m_rotation.y = deg_to_rad(m_rotation.y);
    }
    else if (match("position"))
    {
        Three_Vector vec;
        is >> vec;
        m_vectors.push_back(vec);
    }
    else
    {
        double arg;
        is >> arg;
        m_doubles.push_back(arg);
    }
}
