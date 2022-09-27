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

#include "sounds.h"
#include "../geometry/numeric.h"

#include <AL/alut.h>

#include <cassert>
#include <sstream>

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_World;

namespace Vamos_World
{
class Sounds_Reader : public XML_Parser
{
public:
    /// Modify the Sounds object according to the definition file.
    Sounds_Reader(std::string const& file_name, Sounds& sounds);

private:
    void on_start_tag(XML_Tag const&) override {};
    void on_end_tag(XML_Tag const& tag) override;
    void on_data(std::string const& data_string) override;

    std::string m_file; ///< The name of the WAV file with the sample.
    Sounds& m_sounds; ///< The sound object to be modified.
    double m_pitch{1.0};
    double m_volume{1.0};
    int m_rate{8000}; ///< The sample rate in Hz.
    /// The number of seconds of sound buffered for playing. Delay in responding to a
    /// change in a sound may be as long as this.
    double m_buffer_duration{0.2};
};
}

//----------------------------------------------------------------------------------------
/// OpenAL exit handler to ensure resources are freed when the sound object is destroyed.
void exit_alut()
{
    alutExit();
}

Sounds::Sounds(double volume)
{
    alutInit(0, 0);
    alDistanceModel(AL_INVERSE_DISTANCE);
    overall_volume(volume);
    // Register the exit handler.
    atexit(exit_alut);
}

void Sounds::overall_volume(double volume)
{
    alListenerf(AL_GAIN, volume);
}

void Sounds::read(std::string const& data_dir, std::string const& sounds_file)
{
    if (!data_dir.empty())
        m_data_dir = data_dir;
    if (!sounds_file.empty())
        m_sounds_file = sounds_file;

    Sounds_Reader(m_data_dir + m_sounds_file, *this);
}

void Sounds::play(Sound playing, std::initializer_list<Sound> const& paused,
                  double volume, double pitch, Three_Vector const& pos)
{
    auto& sample{*m_samples[playing]};
    if (volume <= 0.0)
        return sample.pause();

    sample.volume(volume);
    sample.pitch(pitch);
    sample.position(pos);
    sample.play();
    for (auto s : paused)
        m_samples[s]->pause();
}

void Sounds::play_tire_squeal(double slide, Three_Vector const& position)
{
    play(Sound::tire_squeal, {Sound::grass, Sound::gravel, Sound::scrape},
         slide > 0.5 ? 0.1 * slide : 0.0,
         clip(2.0 * (1.0 - slide), 0.8, 4.0),
         position);
}

void Sounds::play_kerb(double speed, Three_Vector const& position)
{
    play(Sound::kerb, {}, speed > 0.0 ? 1.0 : 0.0, speed, position);
}

void Sounds::play_grass(double speed, Three_Vector const& position)
{
    play(Sound::grass, {Sound::gravel, Sound::scrape},
         clip(0.05 * speed, 0.0, 1.0), 1.0, position);
}

void Sounds::play_gravel(double speed, Three_Vector const& position)
{
    play(Sound::gravel, {Sound::grass, Sound::scrape},
         clip(0.08 * speed, 0.0, 1.0), 1.0, position);
}

void Sounds::play_scrape(double speed, Three_Vector const& position)
{
    play(Sound::scrape, {Sound::grass, Sound::gravel},
         clip(0.1 * speed, 0.0, 1.0), 1.0, position);
}

void Sounds::play_wind(double speed, Three_Vector const& position)
{
    play(Sound::wind, {}, clip(0.005 * speed, 0.0, 1.0), 1.0, position);
}

#include <iostream>
void Sounds::play_hard_crash(double speed, Three_Vector const& position)
{
    play(Sound::hard_crash, {}, clip(0.1 * (speed - 1.0), 0.0, 1.0), 1.0, position);
}

void Sounds::play_soft_crash(double speed, Three_Vector const& position)
{
    play(Sound::soft_crash, {}, clip(0.1 * (speed - 1.0), 0.0, 1.0), 1.0, position);
}

void Sounds::pause()
{
    for (auto& s : m_samples)
        s.second->pause();
}

//----------------------------------------------------------------------------------------
Sounds_Reader::Sounds_Reader(std::string const& file_name, Sounds& sounds)
    : m_sounds{sounds}
{
    read(file_name);
}

void Sounds_Reader::on_end_tag(XML_Tag const&)
{
    Sound type;
    if (match("tire-squeal"))
        type = Sound::tire_squeal;
    else if (match("kerb-sound"))
        type = Sound::kerb;
    else if (match("grass-sound"))
        type = Sound::grass;
    else if (match("gravel-sound"))
        type = Sound::gravel;
    else if (match("scrape-sound"))
        type = Sound::scrape;
    else if (match("wind-sound"))
        type = Sound::wind;
    else if (match("soft-crash-sound"))
        type = Sound::soft_crash;
    else if (match("hard-crash-sound"))
        type = Sound::hard_crash;
    else
        return;

    auto loop{type != Sound::soft_crash && type != Sound::hard_crash};
    m_sounds.m_samples[type] = std::make_unique<Sample>(
        m_sounds.m_data_dir + m_file, m_volume, m_pitch, loop);

    // Don't carry over pitch and volume to the next sound.
    m_pitch = 1.0;
    m_volume = 1.0;
}

void Sounds_Reader::on_data(std::string const& data)
{
    if (data.empty())
        return;

    std::istringstream is{data};
    if (match("file"))
        m_file = data;
    else if (match("pitch"))
        is >> m_pitch;
    else if (match("volume"))
        is >> m_volume;
    else if (match("sample-rate"))
        is >> m_rate;
    else if (match("buffer-duration"))
        is >> m_buffer_duration;
}
