//  Copyright (C) 2003-2022 Sam Varner
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

#ifndef VAMOS_WORLD_SOUNDS_H_INCLUDED
#define VAMOS_WORLD_SOUNDS_H_INCLUDED

#include "../media/sample.h"
#include "../media/xml-parser.h"

#include <map>
#include <memory>
#include <string>

namespace Vamos_World
{
/// The names of the unique sounds.
enum class Sound{tire_squeal, kerb, grass, gravel, scrape, wind, soft_crash, hard_crash};

class Sounds
{
public:
    Sounds(double volume);

    /// Set the overall volume level 0.0 -- 1.0.
    void overall_volume(double volume);
    /// Read the sound definition file.
    void read(std::string const& data_dir = "", std::string const& sounds_file = "");
    /// Play a sound.
    void play_tire_squeal(double slide, Vamos_Geometry::Three_Vector const& position);
    void play_kerb(double speed, Vamos_Geometry::Three_Vector const& position);
    void play_grass(double speed, Vamos_Geometry::Three_Vector const& position);
    void play_gravel(double speed, Vamos_Geometry::Three_Vector const& position);
    void play_scrape(double speed, Vamos_Geometry::Three_Vector const& position);
    void play_wind(double speed, Vamos_Geometry::Three_Vector const& position);
    void play_hard_crash(double speed, Vamos_Geometry::Three_Vector const& position);
    void play_soft_crash(double speed, Vamos_Geometry::Three_Vector const& position);
    /// Pause all sounds.
    void pause();

private:
    /// Play a sound and pause others.
    /// @param playing The sound to play.
    /// @param paused A list of sounds to pause.
    /// @param volume If <= 0.0, @p playing is paused and @p paused are not affected.
    void play(Sound playing, std::initializer_list<Sound> const& paused,
              double volume, double pitch, Vamos_Geometry::Three_Vector const& pos);
    /// Sound names and their associated samples.
    using Sample_Ptr = std::unique_ptr<Vamos_Media::Sample>;
    std::map<Sound, Sample_Ptr> m_samples;

    friend class Sounds_Reader;
    std::string m_data_dir; ///< Saved for possible re-reading.
    std::string m_sounds_file; ///< Saved for possible re-reading.
};
} // namespace Vamos_World

#endif // VAMOS_WORLD_SOUNDS_H_INCLUDED
