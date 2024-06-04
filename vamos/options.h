//  Copyright (C) 2014-2022 Sam Varner
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

#ifndef VAMOS_VAMOS_OPTIONS_H_INCLUDED
#define VAMOS_VAMOS_OPTIONS_H_INCLUDED

#include <iosfwd>
#include <string>
#include <vector>

class Options
{
public:
    Options(int argc, char* argv[]);

    int exit_status() const { return m_exit_status; }
    operator bool() const { return !m_exit; }

    std::string data_dir{"../../data/"};
    std::string car_file{"GT"};
    std::string track_file{"Peanut"};
    std::string world_file{"earth"};
    std::string controls_file{"stick"};
    size_t number_of_cars{1};
    size_t laps{0};
    double performance{0.0};
    double volume{1.0};
    bool map_mode{false};
    bool full_screen{false};
    bool interact{true};
    bool demo{false};
    bool qualifying{false};
    bool show_line{false};
    std::string input_file;
    std::vector<double> parameter;
    std::string view;

private:
    int m_exit_status{EXIT_SUCCESS};
    bool m_exit{false};
};

std::ostream& operator<<(std::ostream& os, const Options& opt);

#endif // VAMOS_VAMOS_OPTIONS_H_INCLUDED
