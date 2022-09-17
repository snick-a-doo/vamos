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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_MEDIA_MODEL_H_INCLUDED
#define VAMOS_MEDIA_MODEL_H_INCLUDED

#include "../geometry/three-vector.h"

#include <GL/gl.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace Vamos_Media
{
class Ac3d_Exception : public std::runtime_error
{
public:
    Ac3d_Exception(std::string const& message)
        : std::runtime_error(message)
    {};
};

class Ac3d_Material;
class Ac3d_Surface;
class Ac3d_Object;

class Ac3d
{
public:
    Ac3d(std::string const& file, double scale,
         Vamos_Geometry::Three_Vector const& rotation,
         Vamos_Geometry::Three_Vector const& translation);
    ~Ac3d();

    GLuint build();

private:
    using Material_Ptr = std::unique_ptr<Ac3d_Material const>;
    using Object_Ptr = std::unique_ptr<Ac3d_Object const>;
    using Surface_Ptr = Ac3d_Surface*; //std::unique_ptr<Ac3d_Surface>;

    Material_Ptr read_material(std::ifstream& is);
    Object_Ptr read_object(std::ifstream& is, double scale,
                           Vamos_Geometry::Three_Vector const& translation,
                           Vamos_Geometry::Three_Vector const& rotation);
    Surface_Ptr read_surface(std::ifstream& is, Ac3d_Object& obj);

    std::string m_file;
    int m_version;

    std::vector<Material_Ptr> m_materials;
    std::vector<Object_Ptr> m_objects;

    double m_scale;
    Vamos_Geometry::Three_Vector m_translation;
    Vamos_Geometry::Three_Vector m_rotation; // angle-axis representation
};
} // namespace Vamos_Media

#endif // VAMOS_MEDIA_MODEL_H_INCLUDED
