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

#include "options.h"

#include "../geometry/parameter.h"

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <string_view>

constexpr std::string_view version = "1.0.0";

static void show_usage()
{
    std::cerr << "Usage: vamos "
              << "[-m|--map] "
              << "[[-t|--track=] TRACK_FILE] "
              << "[[-c|--car=] CAR_FILE] "
              << "[[-w|--world=] WORLD_FILE] "
              << "[[-a|--controls=] CONTROLS_FILE] "
              << "[[-o|--opponents=] NUMBER_OF_OPPONENTS] "
              << "[[-p|--laps=] NUMBER_OF_LAPS] "
              << "[[-z|--performance=] FACTOR] "
              << "[[-s|--volume=] VOLUME_PERCENT] "
              << "[-f|--full-screen] "
              << "[-b|--view=] body|map|world|chase"
              << "[-n|--no-interaction] "
              << "[-d|--demo] "
              << "[-q|--qualifying] "
              << "[-l|--show-line[=ARG]  draw the racing line on the track [ARG=yes]]" << std::endl;
}

Options::Options(int argc, char* argv[])
{
    option long_options[] = {{"car", required_argument, 0, 'c'},
                             {"track", required_argument, 0, 't'},
                             {"world", required_argument, 0, 'w'},
                             {"controls", required_argument, 0, 'a'},
                             {"opponents", required_argument, 0, 'o'},
                             {"laps", required_argument, 0, 'p'},
                             {"performance", required_argument, 0, 'z'},
                             {"volume", required_argument, 0, 's'},
                             {"show-line", optional_argument, 0, 'l'},
                             {"map", no_argument, 0, 'm'},
                             {"view", required_argument, 0, 'b'},
                             {"demo", no_argument, 0, 'd'},
                             {"qualifying", no_argument, 0, 'q'},
                             {"full-screen", no_argument, 0, 'f'},
                             {"no-interaction", no_argument, 0, 'n'},
                             {"version", no_argument, 0, 'v'},
                             {0, 0, 0, 0}};

    while (true)
    {
        auto option_index{0};
        auto c{getopt_long(argc, argv, "c:t:w:a:o:p:z:s:l::mdqfnv", long_options, &option_index)};
        if (c == -1)
            break;

        switch (c)
        {
        case 0:
            break;
        case 'c':
            car_file = optarg;
            break;
        case 't':
            track_file = optarg;
            break;
        case 'w':
            world_file = optarg;
            break;
        case 'a':
            controls_file = optarg;
            break;
        case 'o':
            number_of_cars = atoi(optarg) + 1;
            break;
        case 'p':
            laps = atoi(optarg);
            break;
        case 'z':
            performance = atoi(optarg) / 100.0 - 1.0;
            break;
        case 's':
            volume = atoi(optarg) / 100.0;
            break;
        case 'l':
            show_line = !optarg || std::strcmp(optarg, "yes") == 0;
            break;
        case 'm':
            map_mode = true;
            break;
        case 'b':
            view = optarg;
            break;
        case 'd':
            demo = true;
            break;
        case 'q':
            qualifying = true;
            break;
        case 'f':
            full_screen = true;
            break;
        case 'n':
            interact = false;
            break;
        case 'v':
            std::cout << version << std::endl;
            m_exit = true;
            break;
        default:
            show_usage();
            m_exit_status = EXIT_FAILURE;
            m_exit = true;
            break;
        }
    }
    // Default to 5 laps for races, 5 minutes for qualifying. Leave laps at 0 for
    // single-car practice.
    if ((number_of_cars > 1 || qualifying) && laps == 0)
        laps = 5;
    // Assume the rest of the arguments are values of adjustable parameters.
    if (optind < argc)
        input_file = argv[optind++];
    while (optind < argc)
        Vamos_Geometry::Parameter::set(atof(argv[optind++]));

    // Subtract a car for demo mode.
    if (demo && (number_of_cars > 1))
        --number_of_cars;

    namespace fs = std::filesystem;
    if (fs::exists(data_dir))
        return;

    std::cerr << "Couldn't find the data direcory, ../../data or " << data_dir << std::endl;
    data_dir.clear();
    m_exit_status = EXIT_FAILURE;
    m_exit = true;
}

std::ostream& operator<<(std::ostream& os, Options const& opt)
{
    os << "data_dir\t" << opt.data_dir << '\n'
       << "car_file\t" << opt.car_file << '\n'
       << "track_file\t" << opt.track_file << '\n'
       << "world_file\t" << opt.world_file << '\n'
       << "controls_file\t" << opt.controls_file << '\n'
       << "number_of_cars\t" << opt.number_of_cars << '\n'
       << "laps\t" << opt.laps << '\n'
       << "performance\t" << opt.performance << '\n'
       << "volume\t" << opt.volume << '\n'
       << "map_mode\t" << opt.map_mode << '\n'
       << "full_screen\t" << opt.full_screen << '\n'
       << "view\t" << opt.view << '\n'
       << "interact\t" << opt.interact << '\n'
       << "demo\t" << opt.demo << '\n'
       << "qualifying\t" << opt.qualifying << '\n'
       << "show_line\t" << opt.show_line << '\n'
       << "input_file\t" << opt.input_file << '\n'
       << "parameters\t[ ";
    for (auto param : opt.parameter)
        os << param << ' ';
    os << "]\n";
    return os;
}
