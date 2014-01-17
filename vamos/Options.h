//  Options.cc - runtime option handling.
//
//  Vamos Automotive Simulator
//  Copyright (C) 2014 Sam Varner
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

#include <string>
#include <vector>
#include <iosfwd>

class Options
{
public:
  Options (int argc, char* argv []);

  int exit_status () const { return m_exit_status; }
  operator bool () const { return !m_exit; }

  std::string data_dir;
  std::string car_file;
  std::string track_file;
  std::string world_file;
  std::string controls_file;
  size_t number_of_cars;
  size_t laps;
  double performance;
  double volume;
  bool map_mode;
  bool full_screen;
  bool interact;
  bool demo;
  bool qualifying;
  bool show_line;
  std::string input_file;
  std::vector <double> parameter;

private:
  void find_data_directory ();
  void show_usage () const;

  int m_exit_status;
  bool m_exit;
};

std::ostream& operator << (std::ostream& os, const Options& opt);

