//  Sounds.cc - sound management
//
//  Vamos Automotive Simulator
//  Copyright (C) 2003--2004 Sam Varner
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

#include "Sounds.h"
#include "../geometry/Numeric.h"

#include <AL/alut.h>

#include <sstream>
#include <cassert>

using Vamos_Media::Sample;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

Vamos_World::
Sounds::Sounds () :
  mp_tire_squeal_sound (0),
  mp_kerb_sound (0),
  mp_grass_sound (0),
  mp_gravel_sound (0),
  mp_scrape_sound (0),
  mp_wind_sound (0),
  mp_soft_crash_sound (0),
  mp_hard_crash_sound (0)
{
  alutInit (0, 0);
  alDistanceModel (AL_INVERSE_DISTANCE);
  master_volume (1.0);
}

Vamos_World::
Sounds::~Sounds ()
{
  delete mp_soft_crash_sound;
  delete mp_hard_crash_sound;
  delete mp_wind_sound;
  delete mp_scrape_sound;
  delete mp_gravel_sound;
  delete mp_grass_sound;
  delete mp_kerb_sound;
  delete mp_tire_squeal_sound;
}

void Vamos_World::
Sounds::master_volume (double volume)
{
  alListenerf (AL_GAIN, volume);
}

void Vamos_World::
Sounds::add_sample (std::string file, Sound_Type type, 
                    double volume, double pitch)
{
  Sample* sample = 0;
  file = m_data_dir + file;
  switch (type)
    {
    case TIRE_SQUEAL:
      sample = mp_tire_squeal_sound = new Sample (file, volume, pitch, true);
      break;
    case KERB:
      sample = mp_kerb_sound = new Sample (file, volume, pitch, true);
      break;
    case GRASS:
      sample = mp_grass_sound = new Sample (file, volume, pitch, true);
      break;
    case GRAVEL:
      sample = mp_gravel_sound = new Sample (file, volume, pitch, true);
      break;
    case SCRAPE:
      sample = mp_scrape_sound = new Sample (file, volume, pitch, true);
      break;
    case WIND:
      sample = mp_wind_sound = new Sample (file, volume, pitch, true);
      break;
    case SOFT_CRASH:
      sample = mp_soft_crash_sound = new Sample (file, volume, pitch, false);
      break;
    case HARD_CRASH:
      sample = mp_hard_crash_sound = new Sample (file, volume, pitch, false);
      break;
    default:
      assert (false);
    }
}

void Vamos_World::
Sounds::read (std::string data_dir, std::string sounds_file)
{
  delete mp_soft_crash_sound;
  delete mp_hard_crash_sound;
  delete mp_wind_sound;
  delete mp_scrape_sound;
  delete mp_gravel_sound;
  delete mp_grass_sound;
  delete mp_kerb_sound;
  delete mp_tire_squeal_sound;
    
  mp_soft_crash_sound = 0;
  mp_hard_crash_sound = 0;
  mp_wind_sound = 0;
  mp_scrape_sound = 0;
  mp_gravel_sound = 0;
  mp_grass_sound = 0;
  mp_kerb_sound = 0;
  mp_tire_squeal_sound = 0;

  if (data_dir != "")
      m_data_dir = data_dir;

  if (sounds_file != "")
      m_sounds_file = sounds_file;

  Sounds_Reader reader (m_data_dir + m_sounds_file, this);
}

void Vamos_World::
Sounds::play_tire_squeal_sound (double slide, const Three_Vector& position)
{
  double volume = slide;
  if (volume > 0.5)
    {
      const double pitch = clip (2.0 * (1.0 - slide), 0.8, 4.0);
      mp_tire_squeal_sound->pitch (pitch);
      mp_tire_squeal_sound->volume (0.1 * volume);
      mp_tire_squeal_sound->position (position);

      mp_grass_sound->pause ();
      mp_gravel_sound->pause ();
      mp_scrape_sound->pause ();
      mp_tire_squeal_sound->play ();
    }
  else
    {
      mp_tire_squeal_sound->pause ();
    }
}

void Vamos_World::
Sounds::play_kerb_sound (double speed, const Three_Vector& position)
{
  if (speed > 0.0)
    {
      mp_kerb_sound->volume (1.0);
      mp_kerb_sound->pitch (speed);
      mp_kerb_sound->position (position);
      mp_kerb_sound->play ();
    }
  else
    {
      mp_kerb_sound->pause ();
    }
}

void Vamos_World::
Sounds::play_grass_sound (double speed, const Three_Vector& position)
{
  double volume = clip (0.05 * speed, 0.0, 1.0);
  if (speed > 0.0)
    {
      mp_grass_sound->volume (volume);
      mp_grass_sound->position (position);

      mp_gravel_sound->pause ();
      mp_grass_sound->play ();
    }
  else
    {
      mp_grass_sound->pause ();
    }
}

void Vamos_World::
Sounds::play_gravel_sound (double speed, const Three_Vector& position)
{
  double volume = clip (0.08 * speed, 0.0, 1.0);
  if (speed > 0.0)
    {
      mp_gravel_sound->volume (volume);
      mp_gravel_sound->position (position);

      mp_grass_sound->pause ();
      mp_gravel_sound->play ();
    }
  else
    {
      mp_gravel_sound->pause ();
    }
}

void Vamos_World::
Sounds::play_scrape_sound (double speed, const Three_Vector& position)
{
  double volume = clip (0.1 * speed, 0.0, 1.0);
  if (speed > 0.0)
    {
      mp_scrape_sound->volume (volume);
      mp_scrape_sound->position (position);

      mp_tire_squeal_sound->pause ();
      mp_grass_sound->pause ();
      mp_gravel_sound->pause ();
      mp_scrape_sound->play ();
    }
  else
    {
      mp_scrape_sound->pause();
    }
}

void Vamos_World::
Sounds::play_wind_sound (double speed, const Three_Vector& position)
{
  double volume = clip (0.005 * speed, 0.0, 1.0);
  if (volume > 0.0)
    {
      mp_wind_sound->volume (volume);
      mp_wind_sound->position (position);
      mp_wind_sound->play ();
    }
  else
    {
      mp_wind_sound->pause ();
    }
}

void Vamos_World::
Sounds::play_hard_crash_sound (double speed, const Three_Vector& position)
{
  double volume = clip (0.1 * (speed - 1.0), 0.0, 1.0);
  if (volume > 0.0)
    {
      mp_hard_crash_sound->volume (volume);
      mp_hard_crash_sound->position (position);
      mp_hard_crash_sound->play ();
    }
}

void Vamos_World::
Sounds::play_soft_crash_sound (double speed, const Three_Vector& position)
{
  double volume = clip (0.1 * (speed - 1.0), 0.0, 1.0);
  if (volume > 0.0)
    {
      mp_soft_crash_sound->volume (volume);
      mp_soft_crash_sound->position (position);
      mp_soft_crash_sound->play ();
    }
}

// Pause all sounds.
void Vamos_World::
Sounds::pause ()
{
  //! Might be good to have a registration system to make it easier
  //! to do something to all sounds.
  if (mp_tire_squeal_sound)
    mp_tire_squeal_sound->pause ();
  if (mp_kerb_sound)
    mp_kerb_sound->pause ();
  if (mp_grass_sound)
    mp_grass_sound->pause ();
  if (mp_gravel_sound)
    mp_gravel_sound->pause ();
  if (mp_scrape_sound)
    mp_scrape_sound->pause ();
  if (mp_wind_sound)
    mp_wind_sound->pause ();
  if (mp_soft_crash_sound)
    mp_soft_crash_sound->pause ();
  if (mp_hard_crash_sound)
    mp_hard_crash_sound->pause ();
}

//* Class Sounds_Reader

//** Constructor
Vamos_World::
Sounds_Reader::Sounds_Reader (std::string file_name, Sounds* sounds) 
  : mp_sounds (sounds),
    m_rate (8000), // The default sample rate is 8 kHz.
    m_buffer_duration (0.2)
{
  read (file_name);
}

void Vamos_World::
Sounds_Reader::on_start_tag (const XML_Tag& tag)
{
}

void Vamos_World::
Sounds_Reader::on_end_tag (const XML_Tag& tag)
{
  Sounds::Sound_Type type = Sounds::NONE;

  if (label () == "tire-squeal")
    {
      type = Sounds::TIRE_SQUEAL;
    }
  else if (label () == "kerb-sound")
    {
      type = Sounds::KERB;
    }
  else if (label () == "grass-sound")
    {
      type = Sounds::GRASS;
    }
  else if (label () == "gravel-sound")
    {
      type = Sounds::GRAVEL;
    }
  else if (label () == "scrape-sound")
    {
      type = Sounds::SCRAPE;
    }
  else if (label () == "wind-sound")
    {
      type = Sounds::WIND;
    }
  else if (label () == "soft-crash-sound")
    {
      type = Sounds::SOFT_CRASH;
    }
  else if (label () == "hard-crash-sound")
    {
      type = Sounds::HARD_CRASH;
    }
  if (type != Sounds::NONE)
    {
      mp_sounds->add_sample (m_file, type, m_volume, m_pitch);
    }
}

void Vamos_World::
Sounds_Reader::on_data (std::string data)
{
  if (data.size () == 0)
    {
      return;
    }
  std::istringstream is (data.c_str ());

  if (label () == "file")
    {
      m_file = data;
    }
  else if (label () == "pitch")
    {
      is >> m_pitch;
    }
  else if (label () == "volume")
    {
      is >> m_volume;
    }
  else if (label () == "sample-rate")
    {
      is >> m_rate;
    }
  else if (label () == "buffer-duration")
    {
      is >> m_buffer_duration;
    }
}
