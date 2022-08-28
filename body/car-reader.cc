//  Car_Reader.cc - Reader for car definition files.
//
//  Copyright (C) 2008 Sam Varner
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
#include "../media/texture-image.h"

#include <cassert>
#include <sstream>
#include <iostream>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

std::vector <double> two_vector_to_vector (const Vamos_Geometry::Two_Vector& v)
{
  std::vector <double> v_out (2);
  v_out [0] = v.x;
  v_out [1] = v.y;
  return v_out;
}

//* Class Car_Reader

//** Constructor

Car_Reader::Car_Reader (std::string data_dir, 
                        std::string car_file, 
                        Car* car) 
 : m_first_model_for_this_wheel (true),
   m_data_dir (data_dir),
   mp_car (car),
   mp_tachometer (0),
   mp_speedometer (0),
   mp_fuel_gauge (0),
   mp_gear_indicator (0),
   mp_steering_wheel (0),
   m_tachometer_type ("dial"),
   m_speedometer_type ("dial"),
   m_fuel_gauge_type ("dial")
{
  read (car_file);
}

Car_Reader::~Car_Reader ()
{
  for (std::vector <Model_Info*>::iterator it = m_models.begin ();
       it != m_models.end ();
       it++)
    {
      delete *it;
    }
}

void 
Car_Reader::on_start_tag (const Vamos_Media::XML_Tag& tag)
{
  const Vamos_Media::XML_Tag::Attribute_List& 
    attribs = tag.get_attributes ();

  if (path () == "/car")
    {
      mp_car->m_name = attribs [0].value;
    }
  else if (path () == "/car/robot")
    {
      m_doubles.resize (3);
    }
  else if ((path () == "/car/exterior-model") || (path () == "/car/interior-model"))
    {
      m_strings.clear ();
      m_strings.resize (2);
      m_doubles.resize (1);
      m_doubles [0] = 1.0;
      m_vectors.clear ();
      m_vectors.resize (2);
    }
  else if (label () == "initial-conditions")
    {
      m_vectors.clear ();
      m_vectors.resize (4);
    }
  else if (label () == "view")
    {
      m_vectors.clear ();
      m_doubles.clear ();
      m_doubles.resize (4);
    }
  else if (label () == "mirror")
    {
      m_vectors.clear ();
      m_doubles.resize (6);
      m_strings.resize (1);
    }
  else if (label () == "steering")
    {
      m_doubles.clear ();
      m_doubles.resize (3);
    }
  else if (label () == "dashboard")
    {
      m_ints.clear ();
      m_ints.resize (1);
      m_doubles.clear ();
      m_doubles.resize (12);
      m_strings.clear ();
      m_strings.resize (2);
      m_bools.clear ();
      m_bools.resize (1);
      m_vectors.clear ();
      m_vectors.resize (1);
      m_points.clear ();
      for (std::vector <Vamos_Media::Facade*>::iterator 
             it = ma_mirrors.begin ();
           it != ma_mirrors.end ();
           it++)
        {
          delete *it;
        }
      ma_mirrors.clear ();
    }
  else if (label () == "on-steering-wheel")
    {
      m_bools [0] = true;
    }
  else if (label () == "tachometer")
    {
      if (attribs.size () == 0)
        {
          m_tachometer_type = "dial";
        }
      else
        {
          m_tachometer_type = attribs [0].value;
        }
    }
  else if (label () == "speedometer")
    {
      if (attribs.size () == 0)
        {
          m_speedometer_type = "dial";
        }
      else
        {
          m_speedometer_type = attribs [0].value;
        }
    }
  else if (label () == "fuel-gauge")
    {
      if (attribs.size () == 0)
        {
          m_fuel_gauge_type = "dial";
        }
      else
        {
          m_fuel_gauge_type = attribs [0].value;
        }
    }
  else if (label () == "drivetrain")
    {
      m_bools.clear ();
      m_bools.resize (1);
    }
  else if (label () == "engine")
    {
      m_doubles.clear ();
      m_doubles.resize (14);
      m_vectors.clear ();
      m_strings.clear ();
      m_strings.resize (1);
      m_bools.clear ();
      m_bools.resize (1);
    }
  else if (label () == "torque-curve")
    {
      m_points.clear ();
      m_bools [0] = true;
      std::istringstream is (attribs [0].value.c_str ());
      is >> m_doubles [9];
    }
  else if (label () == "clutch")
    {
      m_doubles.clear ();
    }
  else if (label () == "transmission")
    {
      m_doubles.clear ();
      m_gears.clear ();
    }
  else if (label () == "differential")
    {
      m_doubles.clear ();
    }
  else if (label () == "fuel-tank")
    {
      m_doubles.clear ();
      m_vectors.clear ();
    }
  else if (label () == "contact-point")
    {
      m_doubles.resize (3);
      m_strings.resize (1);
      m_vectors.clear ();
    }
  else if (label () == "particle")
    {
      m_doubles.resize (1);
      m_vectors.clear ();
    }
  else if (label () == "drag")
    {
      m_doubles.resize (2);
      m_vectors.clear ();
    }
  else if (label () == "wing")
    {
      m_doubles.resize (6);
      m_vectors.clear ();
    }
  else if (label () == "wheel")
    {
      if (m_doubles.size () != 24)
        {
          m_doubles.resize (24);
          m_doubles [8] = 0.0;
          m_doubles [20] = 0.0;
          m_doubles [23] = 1.0;
          m_long_parameters.resize (11);
          m_trans_parameters.resize (15);
          m_align_parameters.resize (18);
          m_strings.resize (3);
          m_vectors.resize (5);
        }
      m_strings [0] = attribs [0].value;
      m_strings [1] = attribs [1].value;
      m_bools.clear ();
      m_bools.resize (2);
      m_first_model_for_this_wheel = true;
    }
  else if (label () == "steered")
    {
      m_bools [0] = true;
    }
  else if (label () == "driven")
    {
      m_bools [1] = true;
    }
}

void Car_Reader::on_end_tag(Vamos_Media::XML_Tag const&)
{
  if (path () == "/car/robot")
    {
      mp_car->set_robot_parameters (m_doubles [0], m_doubles [1], m_doubles [2]);
    }
  else if ((path () == "/car/exterior-model") 
      && (m_strings [0] != ""))
    {
      mp_car->exterior_model (m_data_dir + "cars/" + m_strings [0],
                              m_doubles [0], m_vectors [0], m_vectors [1]);
    }
  else if ((path () == "/car/interior-model") 
           && (m_strings [0] != ""))
    {
      mp_car->interior_model (m_data_dir + "cars/" + m_strings [0],
                              m_doubles [0], m_vectors [0], m_vectors [1]);
    }
  else if (label () == "initial-conditions")
    {
      mp_car->chassis ().set_initial_conditions (m_vectors [0], m_vectors [1],
                                                 m_vectors [2], m_vectors [3]);
    }
  else if (label () == "view")
    {
      mp_car->set_view (m_vectors [0], m_doubles [0], 
                        m_doubles [1], m_doubles [2], m_doubles [3]);
    }
  else if (label () == "mirror")
    {
      mp_car->add_rear_view (m_vectors [0], m_doubles [0], m_doubles [1],
                             m_doubles [2], m_doubles [3], 
                             m_doubles [4], m_doubles [5],
                             m_data_dir + "textures/" + m_strings [0]);
    }
  else if (label () == "steering")
    {
      mp_car->max_steer_angle (m_doubles [0]);
      mp_car->steer_exponent (m_doubles [1]);
      mp_car->steer_speed_sensitivity (m_doubles [2]);
    }
  else if (label () == "mirror-frame")
    {
      Facade* frame = new Facade (m_data_dir + "textures/" + m_strings [0]);
      frame->set_width (m_doubles [3]);
      frame->set_height (m_doubles [4]);
      frame->set_x_offset (m_doubles [0]);
      frame->set_y_offset (m_doubles [1]);
      frame->set_z_offset (m_doubles [2]);
      ma_mirrors.push_back (frame);
    }
  else if (label () == "tachometer")
    {
      if (m_tachometer_type == "LED")
        {
          mp_tachometer = 
            new LED_Gauge (m_doubles [0], m_doubles [1], m_doubles [2],
                           m_doubles [3], m_ints [0], 
                           m_doubles [4], m_doubles [5],
                           m_data_dir + "textures/" + m_strings [0],
                           m_bools [0]);
          m_bools [0] = false;
        }
      else if (m_tachometer_type == "digital")
        {
          mp_tachometer = 
            new Digital_Gauge (m_doubles [0], m_doubles [1],
                               m_doubles [2], m_doubles [3],
                               m_doubles [4], m_ints [0],
                               m_data_dir + "textures/" + m_strings [0], 
                               m_bools [0]);
          m_bools [0] = false;
        }
      else
        {
          mp_tachometer = new Dial (m_doubles [0], m_doubles [1],
                                    m_doubles [2], m_doubles [3],
                                    m_doubles [4], m_doubles [5],
                                    m_doubles [6], m_doubles [7],
                                    m_data_dir + "textures/" + m_strings [0], 
                                    m_data_dir + "textures/" + m_strings [1]);
        }
    }
  else if (label () == "speedometer")
    {
      if (m_speedometer_type == "digital")
        {
          mp_speedometer = 
            new Digital_Gauge (m_doubles [0], m_doubles [1],
                               m_doubles [2], m_doubles [3],
                               m_doubles [4], m_ints [0],
                               m_data_dir + "textures/" + m_strings [0], 
                               m_bools [0]);
          m_bools [0] = false;
        }
      else
        {
          mp_speedometer = new Dial (m_doubles [0], m_doubles [1],
                                     m_doubles [2], m_doubles [3],
                                     m_doubles [4], m_doubles [5],
                                     m_doubles [6], m_doubles [7],
                                     m_data_dir + "textures/" + m_strings [0], 
                                     m_data_dir + "textures/" + m_strings [1]);
        }
    }
  else if (label () == "fuel-gauge")
    {
      if (m_fuel_gauge_type == "digital")
        {
          mp_fuel_gauge = 
            new Digital_Gauge (m_doubles [0], m_doubles [1],
                               m_doubles [2], m_doubles [3],
                               m_doubles [4], m_ints [0],
                               m_data_dir + "textures/" + m_strings [0], 
                               m_bools [0]);
          m_bools [0] = false;
        }
      else
        {
          mp_fuel_gauge = new Dial (m_doubles [0], m_doubles [1],
                                    m_doubles [2], m_doubles [3],
                                    m_doubles [4], m_doubles [5],
                                    m_doubles [6], m_doubles [7],
                                    m_data_dir + "textures/" + m_strings [0], 
                                    m_data_dir + "textures/" + m_strings [1]);
        }
    }
  else if (label () == "gear-indicator")
    {
      mp_gear_indicator = 
        new Gear_Indicator (m_doubles [0], m_doubles [1],
                            m_doubles [2], m_doubles [3],
                            m_doubles [4], m_ints [0],
                            m_data_dir + "textures/" + m_strings [0],
                            m_bools [0]);
      m_bools [0] = false;
    }
  else if (label () == "gear-shift")
    {
      mp_gear_indicator = 
        new Gear_Shift (m_doubles [0], m_doubles [1],
                        m_doubles [2], m_doubles [3],
                        m_doubles [4], m_vectors [0],
                        m_points,
                        m_data_dir + "textures/" + m_strings [0],
                        m_data_dir + "textures/" + m_strings [1]);
    }
  else if (label () == "steering-wheel")
    {
      mp_steering_wheel = 
        new Steering_Wheel (m_doubles [0], m_doubles [1],
                            m_doubles [2], m_doubles [3],
                            m_doubles [4], m_doubles [5],
                            m_doubles [6], m_doubles [7],
                            m_data_dir + "textures/" + m_strings [0]);
    }
  else if (label () == "dashboard")
    {
      Dashboard* dash = new Dashboard (m_doubles [8], m_doubles [9],
                                       m_doubles [10], m_doubles [11]);
      for (std::vector <Facade*>::iterator it = ma_mirrors.begin ();
           it != ma_mirrors.end ();
           it++)
        {
          dash->add_facade (*it);
        }
      dash->add_tachometer (mp_tachometer);
      dash->add_speedometer (mp_speedometer);
      dash->add_fuel_gauge (mp_fuel_gauge);
      dash->add_gear_indicator (mp_gear_indicator);
      dash->add_steering_wheel (mp_steering_wheel);
      mp_car->dashboard (dash);
    }
  else if (path () ==  "/car/dashboard/extras")
    {
      mp_car->show_dashboard_extras (true);
    }
  else if (label () == "engine")
    {
      mp_engine = new Engine (m_doubles [0], m_vectors [0], m_doubles [1], 
                              m_doubles [2], m_doubles [3], m_doubles [4], 
                              m_doubles [5], m_doubles [6], m_doubles [7],
                              m_doubles [8]);
      if (m_bools [0])
        {
          mp_engine->set_torque_curve (m_points);
          mp_engine->set_friction (m_doubles [9]);
        }
      mp_car->engine_sound (m_data_dir + "sounds/" + m_strings [0], 
                            m_doubles [10], m_doubles [11],
                            m_doubles [12], m_doubles [13]);
    }
  else if (label () == "clutch")
    {
      if (m_doubles.size () != 4)
        {
          error ("clutch requires 4 arguments");
        }
      mp_clutch = new Clutch (m_doubles [0], m_doubles [1], m_doubles [2], 
                              m_doubles [3]);
    }

  else if (label() == "transmission")
  {
      if (m_doubles.size() == 4)
      {
            mp_transmission = new Transmission(static_cast<int>(m_doubles[0]),
                                               m_doubles[1], m_doubles[2]);
            mp_car->shift_delay(m_doubles[3]);
      }
      else
      {
          mp_transmission = new Transmission;
          for (auto gear : m_gears)
              mp_transmission->set_gear_ratio(gear.first, gear.second);
          mp_car->shift_delay(m_doubles[1]);
      }
  }

  else if (label () == "differential")
    {
      if (m_doubles.size () != 2)
        {
          error ("differential requires 2 arguments");
        }
      mp_differential = new Differential (m_doubles [0], m_doubles [1]);
    }

  else if (label () == "drivetrain")
    {
      delete mp_car->mp_drivetrain;
      mp_car->mp_drivetrain = 
        new Drivetrain (mp_engine, mp_clutch, 
                        mp_transmission, mp_differential);
    }

  else if (label () == "fuel-tank")
    {
      if ((m_doubles.size () != 3) || (m_vectors.size () != 1))
        {
          error ("fuel tank requires 1 or 3 elements");
        }
      mp_car->mp_fuel_tank = new Fuel_Tank (m_vectors [0], m_doubles [0],
                                            m_doubles [1], m_doubles [2]);
    }

  else if (label () == "car")
    {
      if (mp_car->mp_drivetrain)
        mp_car->chassis ().particles ().push_back (mp_car->mp_drivetrain->engine ());
      if (mp_car->mp_fuel_tank)
        mp_car->chassis ().particles ().push_back (mp_car->mp_fuel_tank);
    }

  else if (label () == "particle")
    {
      mp_car->chassis ().particles ().
        push_back (new Particle (m_doubles [0], m_vectors [0]));
    }
  else if (label () == "contact-point")
    {
      Material::Composition material = Material::UNKNOWN;
      if (m_strings [0] == "rubber")
        material = Material::RUBBER;
      else if (m_strings [0] == "metal")
        material = Material::METAL;

      mp_car->chassis ().
        add_particle (new Contact_Point (m_doubles [0], m_vectors [0], material,
                                         m_doubles [1], m_doubles [2]));
    }
  else if (label () == "drag")
    {
      mp_car->chassis ().add_drag_particle (new Drag (m_vectors [0], 
                                                      m_doubles [0], 
                                                      m_doubles [1]));
    }
  else if (label () == "wing")
    {
      mp_car->chassis ().
        add_drag_particle (new Wing (m_vectors [0], m_doubles [0], 
                                     m_doubles [1], m_doubles [2], 
                                     m_doubles [3]));
    }
  else if (path () == "/car/wheel/suspension/model")
    {
      if (m_first_model_for_this_wheel)
        {
          for (std::vector <Model_Info*>::iterator it = m_models.begin ();
               it != m_models.end ();
               it++)
            {
              delete *it;
            }
          m_models.clear ();
          m_first_model_for_this_wheel = false;
        }
      m_models.push_back (new Model_Info (m_strings [2], m_doubles [21],
                                          m_vectors [3], m_vectors [4]));
    }
  else if (label () == "wheel")
    {
        auto side = m_strings[0] == "left" ? Side::left : Side::right;
      Suspension* suspension =
        new Suspension (m_vectors [1], m_vectors [2], side, m_doubles [0],
                        m_doubles [1], m_doubles [2], m_doubles [3],
                        m_doubles [4]);
      suspension->camber (m_doubles [5]);
      suspension->caster (m_doubles [6]);
      suspension->toe (m_doubles [7]);
      if (m_doubles [8] != 0.0)
        {
          Suspension* other = static_cast <Suspension*> 
            (*(mp_car->chassis ().particles ().end () - 2));
          assert (other != 0);
          suspension->anti_roll (other, m_doubles [8]);
          m_doubles [8] = 0.0;
        }
      for (std::vector <Model_Info*>::iterator it = m_models.begin ();
           it != m_models.end ();
           it++)
        {
          suspension->set_model (m_data_dir + "cars/" + (*it)->file,
                                 (*it)->scale, 
                                 (*it)->translate,
                                 (*it)->rotate);
        }
      mp_car->chassis ().particles ().push_back (suspension->hinge ());
      mp_car->chassis ().particles ().push_back (suspension);

      Tire_Friction friction (m_long_parameters, m_trans_parameters,
                              m_align_parameters);
      Tire tire (m_doubles [9], m_doubles [10], m_doubles [11], friction,
                 m_doubles [23], m_doubles [12]);

      double bias = m_doubles [17];
      if (m_strings [1] == "rear")
          bias = 1.0 - bias;
      Brake brake (m_doubles [13], m_doubles [14], m_doubles [15], 
                   m_doubles [16], bias);
 
      Wheel* wheel = new Wheel (m_doubles [18], m_vectors [0], 
                                m_doubles [22], m_doubles [20], m_doubles [19],
                                suspension, tire, brake,
                                m_bools [0], m_bools [1], side);
      mp_car->chassis ().particles ().push_back (wheel);
      mp_car->m_wheels.push_back (wheel);
      if (m_slow_model != "")
        {
          std::string stator_path;
          if (m_stator_model != "")
            {
              stator_path = m_data_dir + "cars/" + m_stator_model;
            }

          wheel->set_models (m_data_dir + "cars/" + m_slow_model,
                             m_data_dir + "cars/" + m_fast_model, 
                             m_transition,
                             stator_path, m_stator_offset, 
                             m_scale, m_translation, m_rotation);
        }
    }
}

void 
Car_Reader::on_data (std::string data)
{
  if (data.size () == 0)
    {
      return;
    }
  std::istringstream is (data.c_str ());

  if (path () == "/car/robot/slip-ratio")
    {
      is >> m_doubles [0];
    }
  else if (path () == "/car/robot/deceleration")
    {
      is >> m_doubles [1];
    }
  else if (path () == "/car/robot/lateral-acceleration")
    {
      is >> m_doubles [2];
    }
  else if (path () == "/car/dashboard/position")
    {
      char delim;
      is >> delim >> m_doubles [8] >> delim >> m_doubles [9] 
         >> delim >> m_doubles [10];
    }
  else if (path () == "/car/dashboard/tilt")
    {
      is >> m_doubles [11];
    }
  else if ((path () == "/car/dashboard/mirror-frame/position")
           || (path () == "/car/dashboard/tachometer/position")
           || (path () == "/car/dashboard/speedometer/position")
           || (path () == "/car/dashboard/fuel-gauge/position")
           || (path () == "/car/dashboard/gear-indicator/position")
           || (path () == "/car/dashboard/gear-shift/position")
           || (path () == "/car/dashboard/steering-wheel/position"))
    {
      char delim;
      is >> delim >> m_doubles [0] >> delim >> m_doubles [1] 
         >> delim >> m_doubles [2];
    }
  else if ((path () == "/car/dashboard/tachometer/radius")
           || (path () == "/car/dashboard/tachometer/width")
           || (path () == "/car/dashboard/speedometer/radius")
           || (path () == "/car/dashboard/fuel-gauge/radius")
           || (path () == "/car/dashboard/steering-wheel/radius"))
    {
      is >> m_doubles [3];
    }
  else if (path () == "/car/dashboard/tachometer/elements")
    {
      is >> m_ints [0];
    }
  else if (path () == "/car/dashboard/tachometer/range")
    {
      char delim;
      is >> delim >> m_doubles [4] >> delim >> m_doubles [5];
    }
  else if ((path () == "/car/dashboard/tachometer/min")
           || (path () == "/car/dashboard/speedometer/min")
           || (path () == "/car/dashboard/fuel-gauge/min")
           || (path () == "/car/dashboard/steering-wheel/min"))
    {
      char delim;
      is >> delim >> m_doubles [4] >> delim >> m_doubles [5];
    }
  else if ((path () == "/car/dashboard/tachometer/max")
           || (path () == "/car/dashboard/speedometer/max")
           || (path () == "/car/dashboard/fuel-gauge/max")
           || (path () == "/car/dashboard/steering-wheel/max"))
    {
      char delim;
      is >> delim >> m_doubles [6] >> delim >> m_doubles [7];
    }
  else if (path () == "/car/dashboard/gear-shift/rotation")
    {
      is >> m_vectors [0];
    }
  else if (path () == "/car/dashboard/gear-shift/stick-positions")
    {
      Two_Vector point;
      while (is >> point)
        {
          m_points.push_back (point);
        }
    }
  else if ((path () == "/car/dashboard/mirror-frame/image")
           || (path () == "/car/dashboard/tachometer/face")
           || (path () == "/car/dashboard/tachometer/image")
           || (path () == "/car/dashboard/speedometer/face")
           || (path () == "/car/dashboard/speedometer/image")
           || (path () == "/car/dashboard/fuel-gauge/face")
           || (path () == "/car/dashboard/fuel-gauge/image")
           || (path () == "/car/dashboard/gear-indicator/image")
           || (path () == "/car/dashboard/gear-shift/gate")
           || (path () == "/car/dashboard/steering-wheel/image"))
    {
      is >> m_strings [0];
    }
  else if ((path () == "/car/dashboard/tachometer/needle")
           || (path () == "/car/dashboard/speedometer/needle")
           || (path () == "/car/dashboard/fuel-gauge/needle")
           || (path () == "/car/dashboard/gear-shift/stick"))
    {
      is >> m_strings [1];
    }
  else if ((path () == "/car/dashboard/mirror-frame/size")
           || (path () == "/car/dashboard/gear-indicator/size")
           || (path () == "/car/dashboard/gear-shift/size")
           || (path () == "/car/dashboard/tachometer/size")
           || (path () == "/car/dashboard/speedometer/size")
           || (path () == "/car/dashboard/fuel-gauge/size"))
    {
      char delim;
      is >> delim >> m_doubles [3] >> delim >> m_doubles [4];
    }
  else if ((path () == "/car/dashboard/gear-indicator/numbers")
           || (path () == "/car/dashboard/tachometer/places")
           || (path () == "/car/dashboard/speedometer/places")
           || (path () == "/car/dashboard/fuel-gauge/places"))
    {
      is >> m_ints [0];
    }

  // Particle
  else if (path () == "/car/particle/mass")
    is >> m_doubles [0];

  // Contact Point
  else if (path () == "/car/contact-point/mass")
    is >> m_doubles [0];
  else if (path () == "/car/contact-point/material")
    is >> m_strings [0];
  else if (path () == "/car/contact-point/friction")
    is >> m_doubles [1];
  else if (path () == "/car/contact-point/restitution")
    is >> m_doubles [2];

  // Drag
  else if (path () == "/car/drag/frontal-area")
    is >> m_doubles [0];
  else if (path () == "/car/drag/drag-coefficient")
    is >> m_doubles [1];

  // Wing
  else if (path () == "/car/wing/frontal-area")
    is >> m_doubles [0];
  else if (path () == "/car/wing/surface-area")
    is >> m_doubles [1];
  else if (path () == "/car/wing/lift-coefficient")
    is >> m_doubles [2];
  else if (path () == "/car/wing/efficiency")
    is >> m_doubles [3];

  // Wheel
  else if (path () == "/car/wheel/position")
    is >> m_vectors [0];
  else if (path () == "/car/wheel/mass")
    is >> m_doubles [18];
  else if (path () == "/car/wheel/restitution")
    is >> m_doubles [19];
  else if (path () == "/car/wheel/roll-height")
    is >> m_doubles [20];

  // Suspension
  else if (path () == "/car/wheel/suspension/model/file")
    is >> m_strings [2];
  else if (path () == "/car/wheel/suspension/model/scale")
    is >> m_doubles [21];
  else if (path () == "/car/wheel/suspension/model/translate")
    is >> m_vectors [3];
  else if (path () == "/car/wheel/suspension/model/rotate")
    is >> m_vectors [4];
  else if (path () == "/car/wheel/suspension/position")
    is >> m_vectors [1];
  else if (path () == "/car/wheel/suspension/hinge")
    is >> m_vectors [2];
  else if (path () == "/car/wheel/suspension/spring-constant")
    is >> m_doubles [0];
  else if (path () == "/car/wheel/suspension/bounce")
    is >> m_doubles [1];
  else if (path () == "/car/wheel/suspension/rebound")
    is >> m_doubles [2];
  else if (path () == "/car/wheel/suspension/travel")
    is >> m_doubles [3];
  else if (path () == "/car/wheel/suspension/max-compression-velocity")
    is >> m_doubles [4];
  else if (path () == "/car/wheel/suspension/camber")
    is >> m_doubles [5];
  else if (path () == "/car/wheel/suspension/caster")
    is >> m_doubles [6];
  else if (path () == "/car/wheel/suspension/toe")
    is >> m_doubles [7];
  else if (path () == "/car/wheel/suspension/anti-roll")
    is >> m_doubles [8];

  // Tire
  else if (path () == "/car/wheel/tire/radius")
    is >> m_doubles [9];
  else if (path () == "/car/wheel/tire/offset")
    is >> m_doubles [22];
  else if (path () == "/car/wheel/tire/rolling-resistance")
    {
      char delim;
      is >> delim >> m_doubles [10] >> delim >> m_doubles [11];
    }
  else if (path () == "/car/wheel/tire/rotational-inertia")
    is >> m_doubles [12];
  else if (path () == "/car/wheel/tire/friction/longitudinal")
    {
      for (std::vector <double>::iterator it = m_long_parameters.begin ();
           it != m_long_parameters.end ();
           it++)
        {
          char delim;
          is >> delim >> *it;
        }
    }
  else if (path () == "/car/wheel/tire/friction/transverse")
    {
      for (std::vector <double>::iterator it = m_trans_parameters.begin ();
           it != m_trans_parameters.end ();
           it++)
        {
          char delim;
          is >> delim >> *it;
        }
    }
  else if (path () == "/car/wheel/tire/friction/aligning")
    {
      for (std::vector <double>::iterator it = m_align_parameters.begin ();
           it != m_align_parameters.end ();
           it++)
        {
          char delim;
          is >> delim >> *it;
        }
    }
  else if (path () == "/car/wheel/tire/hardness")
    is >> m_doubles [23];

  // Brakes
  else if (path () == "/car/wheel/brakes/friction")
    is >> m_doubles [13];
  else if (path () == "/car/wheel/brakes/radius")
    is >> m_doubles [14];
  else if (path () == "/car/wheel/brakes/area")
    is >> m_doubles [15];
  else if (path () == "/car/wheel/brakes/max-pressure")
    is >> m_doubles [16];
  else if (path () == "/car/wheel/brakes/front-bias")
    is >> m_doubles [17];

  // Transmission
  else if (path () == "/car/drivetrain/transmission/gear-ratio")
    {
      std::pair <int, double> pair;
      char delim;
      is >> delim >> pair.first >> delim >> pair.second;
      m_gears.push_back (pair);
    }

  // Initial Conditions
  else if (path () == "/car/initial-conditions/position")
    is >> m_vectors [0];
  else if (path () == "/car/initial-conditions/orientation")
    is >> m_vectors [1];
  else if (path () == "/car/initial-conditions/velocity")
    is >> m_vectors [2];
  else if (path () == "/car/initial-conditions/angular-velocity")
    is >> m_vectors [3];

  // View
  else if (path () == "/car/view/field-width")
    {
      is >> m_doubles [0];
    }
  else if (path () == "/car/view/near-plane")
    {
      is >> m_doubles [1];
    }
  else if (path () == "/car/view/far-plane")
    {
      is >> m_doubles [2];
    }
  else if (path () == "/car/view/pan-angle")
    {
      is >> m_doubles [3];
    }
  // Rear View
  else if (path () == "/car/mirror/size")
    {
      char delim;
      is >> delim >> m_doubles [0] >> delim >> m_doubles [1];
    }
  else if (path () == "/car/mirror/direction")
    {
      is >> m_doubles [2];
    }
  else if (path () == "/car/mirror/field-width")
    {
      is >> m_doubles [3];
    }
  else if (path () == "/car/mirror/near-plane")
    {
      is >> m_doubles [4];
    }
  else if (path () == "/car/mirror/far-plane")
    {
      is >> m_doubles [5];
    }
  else if (path () == "/car/mirror/mask")
    {
      is >> m_strings [0];
    }

  // Steering
  else if (path () == "/car/steering/max-angle")
    {
      is >> m_doubles [0];
    }
  else if (path () == "/car/steering/exponent")
    {
      is >> m_doubles [1];
    }
  else if (path () == "/car/steering/speed-sensitivity")
    {
      is >> m_doubles [2];
    }
  // Engine
  else if (path () == "/car/drivetrain/engine/mass")
    {
      is >> m_doubles [0];
    }
  else if (path () == "/car/drivetrain/engine/max-power")
    {
      is >> m_doubles [1];
    }
  else if (path () == "/car/drivetrain/engine/peak-engine-rpm")
    {
      is >> m_doubles [2];
    }
  else if (path () == "/car/drivetrain/engine/rpm-limit")
    {
      is >> m_doubles [3];
    }
  else if (path () == "/car/drivetrain/engine/inertia")
    {
      is >> m_doubles [4];
    }
  else if (path () == "/car/drivetrain/engine/idle")
    {
      is >> m_doubles [5];
    }
  else if (path () == "/car/drivetrain/engine/start-rpm")
    {
      is >> m_doubles [6];
    }
  else if (path () == "/car/drivetrain/engine/stall-rpm")
    {
      is >> m_doubles [7];
    }
  else if (path () == "/car/drivetrain/engine/fuel-consumption")
    {
      is >> m_doubles [8];
    }
  else if (path () == "/car/drivetrain/engine/torque-curve")
    {
      Two_Vector point;
      while (is >> point)
        {
          m_points.push_back (point);
        }
    }
  else if (path () == "/car/drivetrain/engine/sound/file")
    {
      is >> m_strings [0];
    }
  else if (path () == "/car/drivetrain/engine/sound/volume")
    {
      is >> m_doubles [10];
    }
  else if (path () == "/car/drivetrain/engine/sound/throttle-volume-factor")
    {
      is >> m_doubles [11];
    }
  else if (path () == "/car/drivetrain/engine/sound/engine-speed-volume-factor")
    {
      is >> m_doubles [12];
    }
  else if (path () == "/car/drivetrain/engine/sound/pitch")
    {
      is >> m_doubles [13];
    }
  else if ((path () == "/car/exterior-model/file") 
           || (path () == "/car/interior-model/file"))
    {
      is >> m_strings [0];
    }
  else if ((path () == "/car/exterior-model/scale")
           || (path () == "/car/interior-model/scale"))
    {
      is >> m_doubles [0];
    }
  else if ((path () == "/car/exterior-model/translate")
           || (path () == "/car/interior-model/translate"))
    {
      is >> m_vectors [0];
    }
    else if (path() == "/car/exterior-model/rotate"
             || path() == "/car/interior-model/rotate")
    {
        is >> m_vectors[1];
        m_vectors[1] = {deg_to_rad(m_vectors[1].x),
                        deg_to_rad(m_vectors[1].y),
                        deg_to_rad(m_vectors[1].z)};
    }
  else if (path () == "/car/wheel/model/slow-file")
    is >> m_slow_model;
  else if (path () == "/car/wheel/model/fast-file")
    is >> m_fast_model;
  else if (path () == "/car/wheel/model/stator-file")
    is >> m_stator_model;
  else if (path () == "/car/wheel/model/transition-speed")
      is >> m_transition;
  else if (path () == "/car/wheel/model/stator-offset")
      is >> m_stator_offset;
  else if (path () == "/car/wheel/model/scale")
      is >> m_scale;
  else if (path () == "/car/wheel/model/translate")
      is >> m_translation;
  else if (path () == "/car/wheel/model/rotate")
    {
      is >> m_rotation;
      m_rotation.y = deg_to_rad (m_rotation.y);
    }
  
  else if (label () == "position")
    {
      Three_Vector vec;
      is >> vec;
      m_vectors.push_back (vec);
    }
  else
    {
      double arg;
      is >> arg;
      m_doubles.push_back (arg);
    }
}
