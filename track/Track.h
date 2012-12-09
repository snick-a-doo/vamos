//  Track.h - the interface for tracks
//
//	Vamos Automotive Simulator
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

#ifndef _TRACK_H_
#define _TRACK_H_

#include "../geometry/Contact_Info.h"
#include "../geometry/Rectangle.h"
#include "../geometry/Three_Vector.h"
#include "../geometry/Three_Matrix.h"

#include <string>

namespace Vamos_Track
{
  class Track
  {
  public:
    virtual ~Track () {};

	// Read the track definition file.
	virtual void read (std::string data_dir = "", 
					   std::string track_file = "") = 0;

	// Draw the sky.
	virtual void draw_sky (const Vamos_Geometry::Three_Vector& view) const = 0;

	// Draw the track.
	virtual void draw () const = 0;

	// Make the track.
	virtual void build (bool close,
                        int adjusted_segments, 
                        double length, 
                        bool join_pit_lane,
                        int adjusted_pit_segments) = 0;

	// Add a sector timing line.
	virtual void timing_line (double dist) = 0;

	// Return the number of timing lines.
	virtual size_t timing_lines () const = 0;

	// Return the bounds of the track.
	virtual const Vamos_Geometry::Rectangle& bounds () const = 0;

	// Scale the track to a particular length.
	virtual void set_length (double length) = 0;

	//!! Most of the rest should be const.

	// Return the new position for a vehicle at POS when a reset is
	// performed. 
	virtual Vamos_Geometry::Three_Vector 
	reset_position (const Vamos_Geometry::Three_Vector& pos,
                    size_t& road_index,
					size_t& segment_index) = 0;

	// Return the new orientation for a vehicle at POS when a reset is
	// performed. 
	virtual Vamos_Geometry::Three_Matrix
	reset_orientation (const Vamos_Geometry::Three_Vector& pos,
                       size_t& road_index,
					   size_t& segment_index) = 0;

	// Return the elevation of the track at the x and y components of
	// POS.
	virtual double elevation (const Vamos_Geometry::Three_Vector& pos,
							  double x,
                              double y,
                              size_t& road_index,
							  size_t& segment_index) = 0;

	virtual Vamos_Geometry::Contact_Info 
	test_for_contact (const Vamos_Geometry::Three_Vector& pos,
					  double bump_parameter,
					  size_t& road_index,
					  size_t& segment_index) = 0;

	// Return WORLD_POS transformed to the track's coordinate system.
	// SEGMENT_INDEX will be modified if the position on another
	// segment.
	virtual Vamos_Geometry::Three_Vector 
	track_coordinates (const Vamos_Geometry::Three_Vector& pos,
                       size_t& road_index,
                       size_t& segment_index) = 0;

	// Return the timing sector at the given distance.
	virtual int sector (double distance) = 0;
  };
}

#endif
