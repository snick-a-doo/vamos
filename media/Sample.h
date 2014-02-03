//  Sample.h - an audio sample class.
//
//  Copyright (C) 2003 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _SAMPLE_H_
#define _SAMPLE_H_

#include "../geometry/Three_Vector.h"

#include <AL/alut.h>

#include <string>

namespace Vamos_Media
{
  class Missing_Sound_File
  {
  private:
	std::string m_file;
  public:
	Missing_Sound_File (std::string file) : m_file (file) {};
	std::string message () { return "Can't find the sound file \"" 
							   + m_file + '\"'; }
  };

  class Sample
  {
  public:
	Sample (std::string file, double base_volume, double base_pitch, bool loop);
	~Sample ();
	
	// Set the sample's volume as volume_factor * m_base_volume.
	void volume (double volume_factor);
	// Set the sample's pitch as pitch_factor * m_base_pitch.
	void pitch (double pitch_factor);

    void position (const Vamos_Geometry::Three_Vector& v);
    void velocity (const Vamos_Geometry::Three_Vector& v, 
                   bool relative = false);

	void play ();
	void pause ();
	void stop ();

  private:
	double m_base_volume;
	double m_base_pitch;
	bool m_loop;
    ALuint m_buffer;
    ALuint m_source;
  };
}

#endif // not _SAMPLE_H_
