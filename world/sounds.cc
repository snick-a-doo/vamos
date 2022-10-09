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
#include <pugixml.hpp>

#include <cassert>
#include <sstream>

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_World;

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

    read_sounds_file(m_data_dir + m_sounds_file);
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

void Sounds::read_sounds_file(std::string const& file_name)
{
    pugi::xml_document doc;
    auto result{doc.load_file(file_name.c_str())};
    auto top{doc.child("sounds")};

    auto get_value = [](auto const& node, auto tag, double fallback) {
        auto child{node.child(tag)};
        return child.text().empty() ? fallback : child.text().as_double();
    };
    auto set_sound = [&](auto tag, auto type, bool loop) {
        auto node{top.child(tag)};
        auto file{m_data_dir + node.child_value("file")};
        auto volume{get_value(node, "volume", 1.0)};
        auto pitch{get_value(node, "pitch", 1.0)};
        m_samples[type] = std::make_unique<Sample>(file, volume, pitch, loop);
    };

    set_sound("tire-squeal", Sound::tire_squeal, true);
    set_sound("kerb-sound", Sound::kerb, true);
    set_sound("grass-sound", Sound::grass, true);
    set_sound("gravel-sound", Sound::gravel, true);
    set_sound("scrape-sound", Sound::scrape, true);
    set_sound("wind-sound", Sound::wind, true);
    set_sound("soft-crash-sound", Sound::hard_crash, false);
    set_sound("hard-crash-sound", Sound::soft_crash, false);
}
