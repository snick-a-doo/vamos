//  Copyright (C) 2003-2024 Sam Varner
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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_MEDIA_MODEL_H_INCLUDED
#define VAMOS_MEDIA_MODEL_H_INCLUDED

#include "../geometry/three-vector.h"

#include <GL/gl.h>

#include <stdexcept>
#include <string>

namespace Vamos_Media
{
class Model_Exception : public std::runtime_error
{
public:
    Model_Exception(std::string const& message)
        : std::runtime_error(message)
    {}
};

/// A 3D model read by the Assimp library.
class Model
{
public:
    Model(std::string const& file, double scale,
         Vamos_Geometry::Three_Vector const& translation,
         Vamos_Geometry::Three_Vector const& rotation);
    ~Model() = default;
    /// Get the GL list ID of the model.
    GLuint build() const;

private:
    GLuint m_id{0}; ///< The GL list ID of the model.
};
} // namespace Vamos_Media

#endif // VAMOS_MEDIA_MODEL_H_INCLUDED
