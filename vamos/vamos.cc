//  Copyright (C) 2001--2014 Sam Varner
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

#include "../body/gl-car.h"
#include "../media/model.h"
#include "../track/strip-track.h"
#include "../world/gl-world.h"
#include "../world/sounds.h"
#include "../world/interactive-driver.h"
#include "../world/robot-driver.h"
#include "options.h"

#include <filesystem>
#include <iostream>

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;
using namespace Vamos_World;

namespace fs = std::filesystem;

struct Entry
{
  Entry (const std::string& file_in, bool interactive_in, double time_in)
    : file (file_in),
      interactive (interactive_in),
      time (time_in)
  {}

  std::string file;
  bool interactive;
  double time;

  static bool quicker (const Entry& e1, const Entry& e2) { return (e1.time < e2.time); }
};

typedef std::vector <Entry> Entry_List;

fs::path get_path(std::string const& data_dir,
                  std::string const& section,
                  std::string const& file,
                  bool extend)
{
    fs::path path(data_dir);
    path /= section;
    path /= file + (extend ? ".xml" : "");
    if (fs::exists(path))
        return path;
    return file;
}

void read_input (Options& opt, Entry_List& entries)
{
  std::ifstream in (opt.input_file.c_str ());
  in >> opt.track_file;
  int lap;
  double time;
  in >> lap >> lap >> time;

  while (true)
    {
      std::string file, name, type;
      in >> file >> name >> type >> lap >> time;
      if (!in)
        break;
      entries.push_back (Entry (file, type == "interactive", time));
    }

  std::sort (entries.begin (), entries.end (), Entry::quicker);
  opt.number_of_cars = entries.size ();
}

void get_entries (Options& opt, Entry_List& entries)
{
  if (!opt.input_file.empty ())
    return read_input (opt, entries);

  opt.track_file = get_path(opt.data_dir, "tracks", opt.track_file,true);

  std::vector <std::string> car_files;

  auto car_path = get_path(opt.data_dir, "cars", opt.car_file, false);
  if (fs::is_directory (car_path))
    {
      for (fs::directory_iterator it = fs::directory_iterator (car_path);
           (it != fs::directory_iterator ())
             && (car_files.size () < opt.number_of_cars);
           it++)
        {
          if (it->path ().extension () == ".xml")
            car_files.push_back (it->path ().native ());
        }
      std::sort (car_files.begin (), car_files.end ());
    }
  else
      car_files.push_back(get_path(opt.data_dir, "cars", opt.car_file,true));

  std::sort (car_files.begin (), car_files.end ());
  std::vector <std::string>::const_iterator it = car_files.begin ();
  while (entries.size () < opt.number_of_cars)
    {
      entries.push_back (Entry (*it, false, 0));

      if (++it == car_files.end ())
        it = car_files.begin ();
    }

  if (!opt.demo)
    entries.back ().interactive = true;
}

int main (int argc, char* argv [])
{
  Options opt (argc, argv);
  if (!opt)
    std::exit (opt.exit_status ());

  Strip_Track track;
  track.set_racing_line(opt.show_line || opt.demo || opt.number_of_cars > 1,
                        opt.show_line);
  Atmosphere air (1.2, Three_Vector (0.0, 0.0, 0.0));
  Sounds sounds (opt.volume);
  Gl_World world (track, air, sounds, opt.full_screen);

  try
    {
      Entry_List entries;
      get_entries (opt, entries);
      track.read (opt.data_dir, opt.track_file);

      if (!opt.map_mode)
        {
          bool robots = false;
          bool interactive = false;
          Three_Matrix orientation{1.0};

          for (Entry_List::const_iterator itEntry = entries.begin ();
               itEntry != entries.end ();
               ++itEntry)
            {
              const int place = itEntry - entries.begin () + 1;
              Vamos_Body::Gl_Car* car 
                = new Vamos_Body::Gl_Car (track.grid_position (place, 
                                                               opt.number_of_cars,
                                                               opt.qualifying),
                                          orientation);
              car->read (opt.data_dir, itEntry->file);
              car->start_engine ();
              if (itEntry->interactive)
                {
                  interactive = true;
                  Interactive_Driver* driver = new Interactive_Driver (*car);
                  world.add_car (*car, *driver);
                  world.set_focused_car (place - 1);
                }
              else
                {
                  robots = true;
                  car->adjust_robot_parameters (opt.performance, 
                                                opt.performance,
                                                opt.performance);
                  Robot_Driver* driver
                    = new Robot_Driver (*car, track, world.get_gravity ());
                  if (opt.qualifying)
                    driver->qualify ();
                  driver->interact (opt.interact);
                  driver->show_steering_target (opt.show_line);
                  world.add_car (*car, *driver);
                }
            }

          if (robots && !interactive)
            world.set_focused_car (0);
          world.cars_can_interact (opt.interact);
        }

      world.read(get_path(opt.data_dir, "worlds", opt.world_file, true),
                 get_path(opt.data_dir, "controls", opt.controls_file, true));
      if (!opt.map_mode)
          sounds.read (opt.data_dir + "sounds/", "default-sounds.xml");
    }
  catch (XML_Exception& error)
    {
      std::cerr << error.message () << std::endl;
      std::exit (EXIT_FAILURE);
    }
  catch (Vamos_Media::Ac3d_Exception& error)
    {
      std::cerr << error.message () << std::endl;
      std::exit (EXIT_FAILURE);
    }

  world.start (opt.qualifying, opt.laps);
  std::ostringstream name;
  name << (opt.qualifying ? "qualifying" : "race") << "-results";
  world.write_results (name.str ());

  return EXIT_SUCCESS;
}
