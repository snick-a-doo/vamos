//  Copyright (C) 2022 Sam Varner
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

#include "numeric.h"

#include <random>

std::random_device random_device;
std::mt19937 random_generator(random_device());

namespace Vamos_Geometry
{
double random_in_range(double low, double high)
{
    std::uniform_real_distribution<> distrib(low, high);
    return distrib(random_generator);
}
}
