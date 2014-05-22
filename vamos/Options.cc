//  Options.h - runtime option handling.
//
//  Copyright (C) 2014 Sam Varner
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

#include <cstdlib>
#include <getopt.h>
#include <iostream>

#include <boost/filesystem.hpp>

#include "Options.h"

#include "../geometry/Parameter.h"

Options::Options (int argc, char* argv [])
  : data_dir ("../data/"),
    car_file ("GT"),
    track_file ("Peanut"),
    world_file ("earth"),
    controls_file ("stick"),
    number_of_cars (1),
    laps (5),
    performance (0.0),
    volume (1.0),
    map_mode (false),
    full_screen (false),
    interact (true),
    demo (false),
    qualifying (false),
    show_line (false),
    m_exit_status (EXIT_SUCCESS),
    m_exit (false)
{
  int option_index = 0;

  while (true)
    {
      static struct option long_options [] =
        {
          { "car", required_argument, 0, 'c' },
          { "track", required_argument, 0, 't' },
          { "world", required_argument, 0, 'w' },
          { "controls", required_argument, 0, 'a' },
          { "opponents", required_argument, 0, 'o' },
          { "laps", required_argument, 0, 'p' },
          { "performance", required_argument, 0, 'z' },
          { "volume", required_argument, 0, 's' },
          { "show-line", optional_argument, 0, 'l' },
          { "map", no_argument, 0, 'm' },
          { "demo", no_argument, 0, 'd' },
          { "qualifying", no_argument, 0, 'q' },
          { "full-screen", no_argument, 0, 'f' },
          { "no-interaction", no_argument, 0, 'n' },
          { "version", no_argument, 0, 'v' },
          { 0, 0, 0, 0 }
        };

      int c = getopt_long (argc, argv, "c:t:w:a:o:p:z:s:l::mdqfnv", 
                           long_options, &option_index);
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
          number_of_cars = atoi (optarg) + 1;
          break;
        case 'p':
          laps = atoi (optarg);
          break;
        case 'z':
          performance = atoi (optarg) / 100.0 - 1.0;
          break;
        case 's':
          volume = atoi (optarg) / 100.0;
          break;
        case 'l':
          show_line = (optarg == NULL) ? true : (strcmp (optarg, "yes") == 0);
          break;
        case 'm':
          map_mode = true;
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
          std::cout << PACKAGE_STRING << std::endl;
          m_exit = true;
          break;
        default:
          show_usage ();
          m_exit_status = EXIT_FAILURE;
          m_exit = true;
          break;
        }
    }

  // Assume the rest of the arguments are values of adjustable
  // parameters.
  if (optind < argc)
    input_file = argv [optind++];
  while (optind < argc)
    Vamos_Geometry::Parameter::set (atof (argv [optind++]));

  // Subtract a car for demo mode.
  if (demo && (number_of_cars > 1))
    --number_of_cars;

  find_data_directory ();
}

void Options::find_data_directory ()
{
  namespace fs = boost::filesystem;

  if (!fs::exists (data_dir))
    {
      data_dir = DATADIR "/";
      if (!fs::exists (data_dir))
        {
          std::cerr << "Couldn't find the data direcory, ../data or "
                    << data_dir 
                    << std::endl;

          data_dir.clear ();
          m_exit_status = EXIT_FAILURE;
          m_exit = true;
        }
    }
}

void Options::show_usage () const
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
            << "[-n|--no-interaction] "
            << "[-d|--demo] "
            << "[-q|--qualifying] "
            << "[-l|--show-line[=ARG]  draw the racing line on the track [ARG=yes]]"
            << std::endl;
}

std::ostream& operator << (std::ostream& os, const Options& opt)
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
     << "interact\t" << opt.interact << '\n'
     << "demo\t" << opt.demo << '\n'
     << "qualifying\t" << opt.qualifying << '\n'
     << "show_line\t" << opt.show_line << '\n'
     << "input_file\t" << opt.input_file << '\n'
     << "parameters\t[ ";
  for (std::vector <double>::const_iterator it = opt.parameter.begin ();
       it != opt.parameter.end ();
       ++it)
    {
      os << *it << ' ';
    }
  os << "]\n";
  return os;
}

