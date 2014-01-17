//  Vamos Automotive Simulator
//  Copyright (C) 2001--2014 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include <boost/filesystem.hpp>

#include "../body/Gl_Car.h"
#include "../media/Texture_Image.h"
#include "../media/Ac3d.h"
#include "../track/Strip_Track.h"
#include "../world/Gl_World.h"
#include "../world/Sounds.h"
#include "../world/Interactive_Driver.h"
#include "../world/Robot_Driver.h"
#include "Options.h"

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;
using namespace Vamos_World;

namespace fs = boost::filesystem;

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

std::string
get_path (std::string file, std::string section, bool extend)
{
  std::string path ("../data/" + section + "/" + file + (extend ? ".xml" : ""));
  if (fs::exists (path))
    return path;

  path = DATADIR "/" + section + "/" + file + (extend ? ".xml" : "");
  if (fs::exists (path))
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
}

void get_entries (Options& opt, Entry_List& entries)
{
  if (!opt.input_file.empty ())
    return read_input (opt, entries);

  opt.track_file = get_path (opt.track_file, "tracks", true);

  std::vector <std::string> car_files;

  fs::path car_path (get_path (opt.car_file, "cars", false));
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
      car_files.push_back (get_path (opt.car_file, "cars", true));

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
  Atmosphere air (1.2, Three_Vector (0.0, 0.0, 0.0));
  Sounds sounds (opt.volume);
  Gl_World world (argc, argv, &track, &air, &sounds, opt.full_screen);

  try
    {
      Entry_List entries;
      get_entries (opt, entries);
      track.read (opt.data_dir, opt.track_file);

      if (!opt.map_mode)
        {
          const Vamos_Track::Road& road = track.get_road (0);
          bool robots = false;
          bool interactive = false;
          Three_Matrix orientation;

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
                  world.add_car (car, new Interactive_Driver (car), road, true);
                  world.set_focused_car (place - 1);
                }
              else
                {
                  robots = true;
                  car->adjust_robot_parameters (opt.performance, 
                                                opt.performance,
                                                opt.performance);
                  Robot_Driver* driver = new Robot_Driver (car, 
                                                           &track,
                                                           world.get_gravity (),
                                                           opt.qualifying);
                  driver->interact (opt.interact);
                  driver->show_steering_target (opt.show_line);
                  world.add_car (car, driver, road, false);
                }
            }

          if (robots && !interactive)
            world.set_focused_car (0);
          world.cars_can_interact (opt.interact);
        }

      world.read (get_path (opt.world_file, "worlds", true), 
                  get_path (opt.controls_file, "controls", true));
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

  if (opt.show_line || opt.demo || (opt.number_of_cars > 1))
    {
      track.build_racing_line ();
      track.show_racing_line (opt.show_line);
    }
  world.do_start_sequence (!opt.qualifying);
  world.start (opt.laps);

  return EXIT_SUCCESS;
}
