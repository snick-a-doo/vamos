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

#ifndef VAMOS_MEDIA_SAMPLE_H_INCLUDED
#define VAMOS_MEDIA_SAMPLE_H_INCLUDED

#include "../geometry/three-vector.h"

#include <AL/alut.h>

#include <stdexcept>
#include <string>

namespace Vamos_Media
{
class Missing_Sound_File : public std::runtime_error
{
public:
    Missing_Sound_File(std::string const& file)
        : std::runtime_error{"Can't find the sound file \"" + file + "\""}
    {}
};

/// A sound clip.
class Sample
{
public:
    /// @param file The WAV file with the sample.
    /// @param base_volume The relative volume of this sample whet its volume is 1.0.
    /// @param base_pitch The relative pitch of this sample whet its pitch is 1.0.
    Sample(std::string const& file, double base_volume, double base_pitch, bool loop);
    ~Sample();

    /// Set the sample's volume relative to @p base_volume passed to the constructor.
    void volume(double volume_factor);
    /// Set the sample's pitch relative to @p base_pitch passed to the constructor.
    void pitch(double pitch_factor);
    /// Set the location of the source.
    void position(Vamos_Geometry::Three_Vector const& v);
    /// Set the velocity of the source.
    void velocity(Vamos_Geometry::Three_Vector v);

    void play();
    void pause();
    void stop();

private:
    double m_base_volume{1.0};
    double m_base_pitch{1.0};
    bool m_loop{false}; ///< If true, play from the beginning after finishing.
    ALuint m_buffer{};
    ALuint m_source{};
};
} // namespace Vamos_Media

#endif // VAMOS_MEDIA_SAMPLE_H_INCLUDED
