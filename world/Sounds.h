//  Sounds.h - sound management
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

#ifndef _SOUNDS_H_
#define _SOUNDS_H_

#include "../media/Sample.h"
#include "../media/XML_Parser.h"

#include <string>

namespace Vamos_World
{
  class Souds_Reader;

  class Sounds
  {
    friend class Sounds_Reader;

    enum Sound_Type
      {
        TIRE_SQUEAL,
        KERB,
        GRASS,
        GRAVEL,
        SCRAPE,
        WIND,
        SOFT_CRASH,
        HARD_CRASH,
        NONE
      };

  private:
    std::string m_data_dir;
    std::string m_sounds_file;

    // The sound for tires slipping on a hard surface.
    Vamos_Media::Sample* mp_tire_squeal_sound;
    // The sound for driving on kerbs.
    Vamos_Media::Sample* mp_kerb_sound;
    // The sound for driving or sliding on grass.
    Vamos_Media::Sample* mp_grass_sound;
    // The sound for driving or sliding on gravel.
    Vamos_Media::Sample* mp_gravel_sound;
    // The sound for hard surfaces sliding.
    Vamos_Media::Sample* mp_scrape_sound;
    // Wind noise.
    Vamos_Media::Sample* mp_wind_sound;
    // The sound for a crash into a soft surface.
    Vamos_Media::Sample* mp_soft_crash_sound;
    // The sound for a crash into a hard surface.
    Vamos_Media::Sample* mp_hard_crash_sound;

    void add_sample (std::string file, Sound_Type type, 
                     double volume, double pitch);

  public:
    Sounds ();

    ~Sounds ();

    // Set the master volume level 0.0 -- 1.0.
    void master_volume (double volume);

    // Read the sound definition file.
    void read (std::string data_dir = "", 
                       std::string sounds_file = "");

    // The other sounds are adjusted by these functions.
    void play_tire_squeal_sound (double slide,
                                 const Vamos_Geometry::Three_Vector& position);    
    void play_kerb_sound (double speed,
                           const Vamos_Geometry::Three_Vector& position);
    void play_grass_sound (double speed,
                           const Vamos_Geometry::Three_Vector& position);
    void play_gravel_sound (double speed,
                            const Vamos_Geometry::Three_Vector& position);
    void play_scrape_sound (double speed,
                            const Vamos_Geometry::Three_Vector& position);
    void play_wind_sound (double speed,
                          const Vamos_Geometry::Three_Vector& position);
    void play_hard_crash_sound (double speed,
                                const Vamos_Geometry::Three_Vector& position);
    void play_soft_crash_sound (double speed,
                                const Vamos_Geometry::Three_Vector& position);

    // Pause all sounds.
    void pause ();
  };

  class Sounds_Reader : public Vamos_Media::XML_Parser
  {
    void on_start_tag (const Vamos_Media::XML_Tag& tag); 
    void on_end_tag (const Vamos_Media::XML_Tag& tag); 
    void on_data (std::string data_string);

    Sounds* mp_sounds;

    std::string m_file;
    double m_pitch;
    double m_volume;
    int m_rate;

    // The number of seconds of sound buffered for playing.  Delay in
    // responding to a change in a sound may be as long as this. 
    double m_buffer_duration;

  public:
    Sounds_Reader (std::string file_name, Sounds* sounds);
  };
}

#endif // not _SOUNDS_H_
