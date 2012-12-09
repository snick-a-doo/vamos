//  Sample.cc - an audio sample class.
//
//  Vamos Automotive Simulator
//  Copyright (C) 2003 Sam Varner
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

#include "Sample.h"

#include <iostream>

using namespace Vamos_Media;

//-----------------------------------------------------------------------------
//* A helper class for checking errors from OpenAL calls.
class AL_Error_Check
{
public:
  // Store passed-in information and clear errors.
  AL_Error_Check (std::string name);
  AL_Error_Check (std::string name, double value);
  // Check for errors and report.
  ~AL_Error_Check ();

private:
  std::string m_name;
  double m_value;
  bool m_has_value;
};

// Store passed-in information and clear errors.
AL_Error_Check::AL_Error_Check (std::string name)
  : m_name (name),
    m_has_value (false)
{
  alGetError (); 
}

// Store passed-in information and clear errors.
AL_Error_Check::AL_Error_Check (std::string name, double value)
  : m_name (name),
    m_value (value),
    m_has_value (true)
{
  alGetError (); 
}

// Check for errors and report.
AL_Error_Check::~AL_Error_Check ()
{
  ALenum error = alGetError ();
  if (error != AL_NO_ERROR)
    {
      std::cerr << "OpenAL error in " << m_name << ": " 
                << alGetString (error);
      if (m_has_value)
        std::cerr << ": " << m_value;
      std::cerr << std::endl;
    }
}

//-----------------------------------------------------------------------------
Sample::Sample (std::string file, 
                double base_volume, 
                double base_pitch,
                bool loop)
  : m_base_volume (base_volume),
    m_base_pitch (base_pitch)
{
  m_buffer = alutCreateBufferFromFile (file.c_str ());
  if (m_buffer == AL_NONE)
    throw (Missing_Sound_File (file));

  {
    AL_Error_Check error ("Sample() - generate source");
    alGenSources (1, &m_source);
  }{
    AL_Error_Check error ("Sample() - attach source");
    alSourcei (m_source, AL_BUFFER, m_buffer);
  }{
    AL_Error_Check error ("Sample() - loop");
    alSourcei (m_source, AL_LOOPING, loop);
  }{
    AL_Error_Check error ("Sample() - reference distance");
    alSourcef (m_source, AL_REFERENCE_DISTANCE, 10.0);
  }
}

Sample::~Sample ()
{
  stop ();
  {
    AL_Error_Check error ("~Sample() - detach buffer");
    alSourcei (m_source, AL_BUFFER, 0);
  }{
    AL_Error_Check error ("~Sample() - delete buffer");
    alDeleteBuffers (1, &m_buffer);
  }{
    AL_Error_Check error ("~Sample() - delete source");
    alDeleteSources (1, &m_source);
  }
}

void
Sample::volume (double volume_factor)
{
  const double gain = volume_factor * m_base_volume;
  AL_Error_Check error ("volume()", gain);
  alSourcef (m_source, AL_GAIN, gain);
}

void
Sample::pitch (double pitch_factor)
{
  const double pitch = pitch_factor * m_base_pitch;
  AL_Error_Check error ("pitch()", pitch);
  alSourcef (m_source, AL_PITCH, pitch);
}

void
Sample::position (const Vamos_Geometry::Three_Vector& v)
{
  AL_Error_Check error ("position()");
  alSource3f (m_source, AL_POSITION, v.x, v.y, v.z);
}

void
Sample::velocity (const Vamos_Geometry::Three_Vector& v, bool relative)
{
  const double v_s = alGetDouble (AL_SPEED_OF_SOUND);
  AL_Error_Check error ("velocity()");
  alSource3f (m_source, AL_VELOCITY, v.x/v_s, v.y/v_s, v.z/v_s);
}

static bool 
state_is_not (ALuint source, ALint state)
{
  ALint current;
  alGetSourcei (source, AL_SOURCE_STATE, &current);
  return state != current;
}

void
Sample::play ()
{
  if (state_is_not (m_source, AL_PLAYING))
    {
      AL_Error_Check error ("play()");
      alSourcePlay (m_source);
    }
}

void
Sample::pause ()
{
  if (state_is_not (m_source, AL_PAUSED))
    {
      AL_Error_Check error ("pause()");
      alSourcePause (m_source);
    }
}

void
Sample::stop ()
{
  if (state_is_not (m_source, AL_STOPPED))
    {
      AL_Error_Check error ("stop()");
      alSourceStop (m_source);
    }
}
