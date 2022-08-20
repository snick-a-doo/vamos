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

#include "drivetrain.h"

#include <cassert>

using namespace Vamos_Body;

Drivetrain::Drivetrain(Engine* engine,
                       Clutch* clutch,
                       Transmission* transmission,
                       Differential* differential)
    : mp_engine{engine},
      mp_clutch{clutch},
      mp_transmission{transmission},
      mp_differential{differential}
{
}

// Process the input parameters.
void Drivetrain::input(double gas,
                       double clutch,
                       double left_wheel_speed,
                       double right_wheel_speed)
{
    m_gas = gas;
    mp_clutch->set_position(clutch);
    auto shaft_speed{mp_differential->get_driveshaft_speed(left_wheel_speed, right_wheel_speed)};
    mp_transmission->set_driveshaft_speed(shaft_speed);
}

void Drivetrain::find_forces()
{
    // Find the drag due to friction in the clutch if the clutch is not fully engaged, and
    // the torque on the driveshaft.
    auto torque{0.0};
    if (mp_transmission->is_in_neutral())
    {
        // We're in neutral. Just update the engine.
        mp_engine->input(m_gas, 0.0, 0.0, false);
    }
    else if (mp_clutch->is_engaged())
    {
        // If the clutch is fully engaged, the engine speed is forced to match the
        // transmission speed. There is no clutch drag.
        mp_engine->input(m_gas, 0.0, mp_transmission->clutch_speed(), true);
        torque = mp_transmission->torque(mp_engine->drive_torque());
    }
    else
    {
        auto drag{mp_clutch->drag(mp_engine->rotational_speed(),
                                  mp_transmission->clutch_speed())};
        // Find out how much torque we get from the drivetrain.
        torque = mp_transmission->torque(drag);
        // Update the Engine
        mp_engine->input(m_gas, drag, 0.0, false);
    }
    // Apply the torque to the differential.
    mp_differential->find_torques(torque);
    mp_engine->find_forces();
}

void Drivetrain::propagate(double time)
{
    mp_engine->propagate(time);
}

void Drivetrain::reset()
{
    mp_clutch->set_position(0.0);
    mp_transmission->shift(0);
}

double Drivetrain::torque(Vamos_Geometry::Direction side) const
{
    return mp_differential->get_torque(side);
}
