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

#include "sample.h"

#include <iostream>

using namespace Vamos_Media;

/// Wrap an OpenAL function calls in error checks.
auto al_check = [](auto& al_func, auto name, auto&&... args) {
    alGetError(); // Clear existing errors.
    al_func(args...);
    auto error{alGetError()};
    if (error != AL_NO_ERROR)
        std::cerr << "OpenAL error in " << name << ": " << alGetString(error) << std::endl;
};

//-----------------------------------------------------------------------------
Sample::Sample(std::string const& file, double base_volume, double base_pitch, bool loop)
    : m_base_volume(base_volume),
      m_base_pitch(base_pitch),
      m_buffer{alutCreateBufferFromFile(file.c_str())}
{
    if (m_buffer == AL_NONE)
        throw(Missing_Sound_File(file));

    al_check(alGenSources, "Sample() - generate source", 1, &m_source);
    al_check(alSourcei, "Sample() - attach source", m_source, AL_BUFFER, m_buffer);
    al_check(alSourcei, "Sample() - loop", m_source, AL_LOOPING, loop);
    al_check(alSourcef, "Sample() - reference distance",
             m_source, AL_REFERENCE_DISTANCE, 10.0);
}

Sample::~Sample()
{
    stop();
    al_check(alSourcei, "Sample() - detach buffer", m_source, AL_BUFFER, 0);
    al_check(alDeleteBuffers, "~Sample() - delete buffer", 1, &m_buffer);
    al_check(alDeleteSources, "~Sample() - delete source", 1, &m_source);
}

void Sample::volume(double volume_factor)
{
    al_check(alSourcef, "volume()", m_source, AL_GAIN, m_base_volume * volume_factor);
}

void Sample::pitch(double pitch_factor)
{
    al_check(alSourcef, "pitch()", m_source, AL_PITCH, m_base_pitch * pitch_factor);
}

void Sample::position(Vamos_Geometry::Three_Vector const& v)
{
    al_check(alSource3f, "position()", m_source, AL_POSITION, v.x, v.y, v.z);
}

void Sample::velocity(Vamos_Geometry::Three_Vector v)
{
    v /= alGetDouble(AL_SPEED_OF_SOUND);
    al_check(alSource3f, "velocity()", m_source, AL_VELOCITY, v.x, v.y, v.z);
}

static bool state_is(ALuint source, ALint state)
{
    ALint current;
    alGetSourcei(source, AL_SOURCE_STATE, &current);
    return state == current;
}

void Sample::play()
{
    if (!state_is(m_source, AL_PLAYING))
        al_check(alSourcePlay, "play()", m_source);
}

void Sample::pause()
{
    if (!state_is(m_source, AL_PAUSED))
        al_check(alSourcePause, "pause()", m_source);
}

void Sample::stop()
{
    if (!state_is(m_source, AL_STOPPED))
        al_check(alSourceStop, "stop()", m_source);
}
