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

#include "model.h"

#include "texture-image.h"

#include "../geometry/three-matrix.h"
#include "../geometry/two-vector.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <numbers>
#include <sstream>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

using namespace std::numbers;

using Object_Ptr = std::unique_ptr<Ac3d_Object const>;

namespace Vamos_Media
{
// The material properties for a surface.  The names of their
// members and accessors mean the same things as the OpenGL
// material parameters.
struct Ac3d_Material
{
    std::string m_name;
    std::array<GLfloat, 3> color;
    std::array<GLfloat, 3> ambient;
    std::array<GLfloat, 3> emission;
    std::array<GLfloat, 3> specular;
    GLfloat shininess;
    GLfloat transparency;

    void set_gl_properties(GLenum face) const;
};

//!!! Unify?
struct Surface_Vertex
{
    Three_Vector const* coordinates;
    Three_Vector const* normal;
    Two_Vector texture_coord;
};

struct Object_Vertex
{
    Object_Vertex(std::unique_ptr<Three_Vector> coords)
        : coordinates(std::move(coords))
    {}
    std::unique_ptr<Three_Vector> coordinates;
    Three_Vector normal;
};

/// A surface is a rendered figure such as a triangle, or a strip of quadrilaterals.
/// Vertices, normals, and material properties are stored in a surface object.
class Ac3d_Surface
{
public:
    // The first three are defined by the AC3D file format. The POLYGON type may be
    // changed to one of the triangle or quadrilateral types.
    enum Figure_Type
    {
        POLYGON,
        LINE,
        CLOSED_LINE,
        TRIANGLE,
        TRIANGLE_STRIP,
        TRIANGLE_FAN,
        QUADRILATERAL,
        QUADRILATERAL_STRIP
    };

    Ac3d_Surface(std::string figure_type_code, double scale, Three_Vector const& offset,
                 Three_Matrix const& rotation);

    virtual ~Ac3d_Surface();

    void set_material(Ac3d_Material const* mat) { mp_material = mat; }
    const Ac3d_Material* get_material() const { return mp_material; }

    void set_figure_type(Figure_Type type) { m_figure_type = type; }
    Figure_Type get_figure_type() const { return m_figure_type; }

    void add_vertex(const Surface_Vertex* vert) { m_vertices.push_back(vert); }
    void set_vertices(const std::vector<const Surface_Vertex*>& verts) { m_vertices = verts; }
    const std::vector<const Surface_Vertex*> get_vertices() const { return m_vertices; }

    /// Change the order of the vertices when converting a polygon to a strip.
    void rearrange_vertices(size_t i, size_t j, size_t k, size_t l);

    void set_normal(Three_Vector const& norm) { m_normal = norm; }

    bool is_shaded() const { return m_shaded; }
    bool is_two_sided() const { return m_two_sided; }

    // Add the OpenGL calls for rendering this surface to the display list.
    void build() const;


private:
    // Set the appropriate OpenGL attributes.
    void set_attributes() const;
    // Return the appropriate argument for glBegin().
    GLenum get_gl_figure_type() const;
    void set_material_properties() const;
    void draw_figure() const;

    // Materials and vertices are handled by pointer. Vertices on different surfaces may
    // point to the same vertex object.
    Ac3d_Material const* mp_material;
    std::vector<Surface_Vertex const*> m_vertices;
    /// The normal vector for a flat-shaded surface. Smooth shading uses the `normal'
    /// member of the Vertex structure.
    Three_Vector m_normal;

    // See enum Figure_Type above.
    Figure_Type m_figure_type;
    bool m_shaded;
    bool m_two_sided;

    // Transformations
    double m_scale;
    Three_Vector m_offset;
    Three_Matrix m_rotation;
};

using Surface_Ptr = Ac3d_Surface*; //std::unique_ptr<Ac3d_Surface>;

/// An array of surfaces for an object. This class turns individual polygons into strips
/// if it can.
class Surface_List : public std::vector<Surface_Ptr>
{
public:
    // Override vector's push_back. We may join the surface instead of adding it to the
    // vector.
    void push_back(Surface_Ptr surface);

private:
    using Surface_Vertices = std::vector<Surface_Vertex const*>;

    bool join_surface(Ac3d_Surface const& surface);

    bool join_quadrilateral_to_edge(size_t index1, size_t index2,
                                    const Surface_Vertices& old_vertices,
                                    const Surface_Vertices& new_vertices);
    bool join_triangle_to_edge(size_t index1, size_t index2, const Surface_Vertices& old_vertices,
                               const Surface_Vertices& new_vertices);
    bool join_quadrilateral(const Surface_Vertices& new_vertices, size_t old_index1,
                            size_t old_index2, size_t new_index1, size_t new_index2);
    bool join_triangle(const Surface_Vertices& new_vertices, size_t new_index1, size_t new_index2,
                       Ac3d_Surface::Figure_Type new_figure_type);

    // Vertex indices used to see if a quadrilateral candidate for joining would join at
    // the same side as the others.
    size_t m_last_vertex1;
    size_t m_last_vertex2;
};

// An object is a collection of surfaces.  Objects store the vertices that the surfaces'
// vertices point to.
class Ac3d_Object
{
public:

public:
    Ac3d_Object(std::string type, double scale, Three_Vector const& translation,
                Three_Vector const& rotation);
    ~Ac3d_Object();

    void read_data(std::ifstream& is);

    void set_name(std::string name) { m_name = name; }
    void set_texture_image(std::string file);
    void set_texture_repeat(Two_Vector const& p);

    double get_scale() const { return m_scale; }
    void set_rotation(Three_Matrix const& rot);
    Three_Matrix const& get_rotation() const { return m_rotation; }
    void set_location(Three_Vector const& loc);
    Three_Vector const& get_location() const { return m_location; }

    void set_url(std::string url) { m_url = url; }
    void add_kid(Object_Ptr obj) { m_kids.push_back(std::move(obj)); }
    void add_vertex(double x, double y, double z);
    Three_Vector const* get_vertex(size_t index) const;
    void add_normal(size_t index, Three_Vector const& norm);
    Three_Vector const* get_normal(size_t index) const;
    void add_surface(Surface_Ptr surf);

    void build() const;

private:
    std::string m_type;
    std::string m_name;
    std::string m_data;
    std::string m_url;

    double m_scale;
    Three_Matrix m_rotation{1.0};
    Three_Vector m_location;

    std::unique_ptr<Texture_Image> mp_texture;
    Two_Vector m_texture_repeat;

    std::vector<Object_Ptr> m_kids;
    std::vector<Object_Vertex*> m_vertices;
    Surface_List m_surfaces;
};
} // namespace Vamos_Media

void Ac3d_Material::set_gl_properties(GLenum face) const
{
    glColor4f(color[0], color[1], color[2], 1.0 - transparency);
    glMaterialfv(face, GL_AMBIENT, ambient.data());
    glMaterialfv(face, GL_EMISSION, emission.data());
    glMaterialfv(face, GL_SPECULAR, specular.data());
    glMaterialfv(face, GL_SHININESS, &shininess);
}

Ac3d_Surface::Ac3d_Surface(std::string figure_type_code, double scale, Three_Vector const& offset,
                           Three_Matrix const& rotation)
    : m_scale(scale),
      m_offset(offset),
      m_rotation(rotation)
{
    m_normal.z = 1.0;

    std::istringstream is(figure_type_code);
    is.setf(std::ios_base::hex, std::ios_base::basefield);
    int code;
    is >> code;

    int figure_type = Figure_Type(code & 0x0f);
    if (figure_type != POLYGON && figure_type != CLOSED_LINE && figure_type != LINE)
        throw Ac3d_Exception("Unrecognized figure type");

    m_figure_type = Figure_Type(figure_type);
    m_shaded = bool(code & 0x10);
    m_two_sided = bool(code & 0x20);
}

Ac3d_Surface::~Ac3d_Surface()
{
    for (std::vector<const Surface_Vertex*>::iterator it = m_vertices.begin(); it != m_vertices.end(); it++)
    {
        delete *it;
    }
}

void Ac3d_Surface::build() const
{
    if (m_vertices.empty())
        return;

    glPushAttrib(GL_ENABLE_BIT);
    {
        set_attributes();
        glBegin(get_gl_figure_type());
        {
            set_material_properties();
            draw_figure();
        }
        glEnd();
    }
    glPopAttrib();
}

void Ac3d_Surface::set_attributes() const
{
    if (m_two_sided)
        glDisable(GL_CULL_FACE);
    else
        glEnable(GL_CULL_FACE);
}

void Ac3d_Surface::rearrange_vertices(size_t i, size_t j, size_t k, size_t l)
{
    m_vertices = {m_vertices[i], m_vertices[j], m_vertices[k], m_vertices[l]};
}

GLenum Ac3d_Surface::get_gl_figure_type() const
{
    auto const n_vertices{m_vertices.size()};
    switch (m_figure_type)
    {
    case LINE:
        return GL_LINE_STRIP;
    case CLOSED_LINE:
        return GL_LINE_LOOP;
    case TRIANGLE:
        assert(n_vertices == 3);
        return GL_TRIANGLES;
    case TRIANGLE_STRIP:
        assert(n_vertices > 3);
        return GL_TRIANGLE_STRIP;
    case TRIANGLE_FAN:
        assert(n_vertices > 3);
        return GL_TRIANGLE_FAN;
    case QUADRILATERAL:
        assert(n_vertices == 4);
        return GL_QUADS;
    case QUADRILATERAL_STRIP:
        assert(n_vertices >= 4);
        assert(n_vertices % 2 == 0);
        return GL_QUAD_STRIP;
    case POLYGON:
        assert(n_vertices > 4);
        return GL_POLYGON;
    default:
        throw Ac3d_Exception("Unrecognized figure type");
    }
}

void Ac3d_Surface::set_material_properties() const
{
    auto face{m_two_sided ? GL_FRONT_AND_BACK : GL_FRONT};
    glColorMaterial(face, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    mp_material->set_gl_properties(face);
}

void Ac3d_Surface::draw_figure() const
{
    auto norm{m_rotation * m_normal};
    for (auto vertex : m_vertices)
    {
        glTexCoord2f(vertex->texture_coord.x, vertex->texture_coord.y);
        if (m_shaded)
            norm = (m_rotation * *(vertex->normal)).unit();
        glNormal3d(norm.x, norm.y, norm.z);
        // Rotate and translate the vertex.
        auto const& pos = m_offset + m_scale * (m_rotation * *(vertex->coordinates));
        glVertex3f(pos.x, pos.y, pos.z);
    }
}

void Surface_List::push_back(Surface_Ptr surface)
{
    if (!surface->is_shaded() || !join_surface(*surface))
        std::vector<Surface_Ptr>::push_back(std::move(surface));
}

bool Surface_List::join_surface(Ac3d_Surface const& surface)
{
    if (empty())
        return false;
    if (surface.get_material() != back()->get_material())
        return false;

    if (surface.get_figure_type() != Ac3d_Surface::QUADRILATERAL
        && surface.get_figure_type() != Ac3d_Surface::TRIANGLE)
        return false;

    auto const& new_verts{surface.get_vertices()};
    auto const& old_verts{back()->get_vertices()};
    auto const n_old_verts{old_verts.size()};

    auto const new_figure_type{surface.get_figure_type()};
    auto const old_figure_type{back()->get_figure_type()};

    if (new_figure_type == Ac3d_Surface::QUADRILATERAL)
    {
        if (old_figure_type == Ac3d_Surface::QUADRILATERAL)
        {
            for (size_t i{0}; i < n_old_verts; ++i)
            {
                auto j{(i + 1) % n_old_verts};
                if (join_quadrilateral_to_edge(i, j, old_verts, new_verts))
                    return true;
            }
        }
        else if (old_figure_type == Ac3d_Surface::QUADRILATERAL_STRIP)
            return join_quadrilateral_to_edge(n_old_verts - 1, n_old_verts - 2,
                                              old_verts, new_verts);
    }
    if (new_figure_type == Ac3d_Surface::TRIANGLE)
    {
        if (old_figure_type == Ac3d_Surface::TRIANGLE)
        {
            for (size_t i{0}; i < n_old_verts; ++i)
            {
                auto j{(i + 1) % n_old_verts};
                if (join_triangle_to_edge(i, j, old_verts, new_verts))
                    return true;
            }
        }
        else if (old_figure_type == Ac3d_Surface::TRIANGLE_STRIP)
            return join_triangle_to_edge(n_old_verts - 2, n_old_verts - 1,
                                         old_verts, new_verts);
        else if (old_figure_type == Ac3d_Surface::TRIANGLE_FAN)
            return join_triangle_to_edge(0, n_old_verts - 1, old_verts, new_verts);
    }
    return false;
}

bool Surface_List::join_quadrilateral_to_edge(size_t index1, size_t index2,
                                              Surface_Vertices const& old_vertices,
                                              Surface_Vertices const& new_vertices)
{
    auto const n_new_verts{new_vertices.size()};
    auto const* v1{old_vertices[index1]};
    auto const* v2{old_vertices[index2]};

    for (size_t i{0}; i < n_new_verts; ++i)
    {
        auto j{(i + 1) % n_new_verts};
        // The common vertices are on opposite sides; j1 connects with i2 and j2 with i1.
        if (new_vertices[i]->coordinates == v2->coordinates
            && new_vertices[j]->coordinates == v1->coordinates)
            return join_quadrilateral(new_vertices, index1, index2, i, j);
    }
    return false;
}

bool Surface_List::join_triangle_to_edge(size_t index1, size_t index2,
                                         Surface_Vertices const& old_vertices,
                                         Surface_Vertices const& new_vertices)
{
    auto const n_new_verts{new_vertices.size()};
    auto const* v1{old_vertices[index1]};
    auto const* v2{old_vertices[index2]};
    auto const old_figure_type{back()->get_figure_type()};

    for (size_t i{0}; i < n_new_verts; ++i)
    {
        auto j{(i + 1) % n_new_verts};
        auto even{old_vertices.size() % 2 == 0};
        auto match{false};
        if ((even && old_figure_type == Ac3d_Surface::TRIANGLE_STRIP)
            || old_figure_type == Ac3d_Surface::TRIANGLE_FAN)
            match = new_vertices[i]->coordinates == v1->coordinates
                && new_vertices[j]->coordinates == v2->coordinates;
        else
            // Common vertices are on opposite sides; j1 connects with i2 and j2 with i1.
            match = new_vertices[i]->coordinates == v2->coordinates
                && new_vertices[j]->coordinates == v1->coordinates;
        if (match)
        {
            auto new_figure_type{Ac3d_Surface::TRIANGLE_STRIP};
            if ((old_figure_type == Ac3d_Surface::TRIANGLE && index2 == 0)
                || (old_figure_type == Ac3d_Surface::TRIANGLE_FAN && index1 == 0))
                new_figure_type = Ac3d_Surface::TRIANGLE_FAN;
            return join_triangle(new_vertices, i, j, new_figure_type);
        }
    }
    return false;
}

bool Surface_List::join_quadrilateral(Surface_Vertices const& new_vertices,
                                      size_t old_index1, size_t old_index2,
                                      size_t new_index1, size_t) // new_index2
{
    auto n_new_vertices{new_vertices.size()};
    auto new_index3{(new_index1 + 2) % n_new_vertices};
    auto new_index4{(new_index1 + 3) % n_new_vertices};

    if (back()->get_figure_type() == Ac3d_Surface::QUADRILATERAL)
    {
        auto old_index3{(old_index1 + 2) % n_new_vertices};
        auto old_index4{(old_index1 + 3) % n_new_vertices};
        back()->rearrange_vertices(old_index3, old_index4, old_index2, old_index1);
        back()->set_figure_type(Ac3d_Surface::QUADRILATERAL_STRIP);
        m_last_vertex1 = new_index3;
        m_last_vertex2 = new_index4;
    }
    else if (m_last_vertex1 != new_index3 || m_last_vertex2 != new_index4)
        return false;

    back()->add_vertex(new_vertices[new_index4]);
    back()->add_vertex(new_vertices[new_index3]);
    return true;
}

bool Surface_List::join_triangle(Surface_Vertices const& new_vertices, size_t new_index1,
                                 size_t, // new_index2
                                 Ac3d_Surface::Figure_Type new_figure_type)
{
    auto n_new_vertices{new_vertices.size()};
    auto new_index3{(new_index1 + 2) % n_new_vertices};
    auto old_figure_type{back()->get_figure_type()};

    if (old_figure_type == Ac3d_Surface::TRIANGLE)
    {
        back()->set_figure_type(new_figure_type);
        back()->add_vertex(new_vertices[new_index3]);
        return true;
    }
    if (new_figure_type != old_figure_type)
        return false;

    back()->add_vertex(new_vertices[new_index3]);
    return true;
}

int get_version_number(char ver)
{
    if (ver >= '0' && ver <= '9')
        return ver - '0';
    else if (ver >= 'a' && ver <= 'f')
        return ver - 'a' + 10;
    else if (ver >= 'A' && ver <= 'F')
        return ver - 'A' + 10;

     std::ostringstream message;
     message << "The version number " << ver << "is not a hexadecimal character.";
     throw Ac3d_Exception(message.str());
}

std::string get_quoted(std::ifstream& is)
{
    // Read the material's name.
    std::string word;
    is >> word;
    if (word[0] != '\"')
        return word;

    // Get multi-word names.
    while (word[word.size() - 1] != '\"')
    {
        std::string more;
        is >> more;
        word = word + ' ' + more;
    }
    // Strip the quotes.
    return word.substr(1, word.size() - 2);
}

Ac3d_Object::Ac3d_Object(std::string type, double scale, Three_Vector const& translation,
                         Three_Vector const& rotation)
    : m_type(type),
      m_scale(scale),
      m_location(translation)
{
    m_texture_repeat = {1.0, 1.0};
    m_rotation.identity();

    // Conversion from Blender to AC3D puts the z-axis in the
    // y-direction.  Compensate by...
    // ...adding 90 to the x-rotation...
    m_rotation.rotate(Three_Vector(rotation.x + pi / 2.0, 0.0, 0.0));
    // ...using the z rotation for y...
    m_rotation.rotate(Three_Vector(0.0, rotation.z, 0.0));
    // ...using the -y rotation for z.
    m_rotation.rotate(Three_Vector(0.0, 0.0, -rotation.y));
}

Ac3d_Object::~Ac3d_Object()
{
    for (std::vector<Object_Vertex*>::iterator it = m_vertices.begin(); it != m_vertices.end(); it++)
    {
        delete *it;
    }
}

void Ac3d_Object::read_data(std::ifstream& is)
{
    size_t length;
    is >> length;
    m_data.resize(length);
    is.read(m_data.data(), length);
}

void Ac3d_Object::build() const
{
    if (mp_texture)
    {
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        mp_texture->activate();
    }
    else
        glDisable(GL_TEXTURE_2D);

    for (auto& surface : m_surfaces)
        surface->build();
    glDisable(GL_TEXTURE_2D);
    for (auto& kid : m_kids)
        kid->build();
    glEnable(GL_TEXTURE_2D);
}

void Ac3d_Object::set_texture_repeat(Two_Vector const& p)
{
    m_texture_repeat = p;
}

void Ac3d_Object::set_texture_image(std::string file)
{
    mp_texture = std::make_unique<Texture_Image>(file, false, false);
}

void Ac3d_Object::set_rotation(Three_Matrix const& rot)
{
    m_rotation *= rot;
}

void Ac3d_Object::set_location(Three_Vector const& loc)
{
    m_location += loc;
}

void Ac3d_Object::add_vertex(double x, double y, double z)
{
    m_vertices.push_back(new Object_Vertex(std::make_unique<Three_Vector>(x, y, z)));
}

Three_Vector const* Ac3d_Object::get_vertex(size_t index) const
{
    assert(index < m_vertices.size());
    return m_vertices[index]->coordinates.get();
}

void Ac3d_Object::add_normal(size_t index, Three_Vector const& norm)
{
    m_vertices[index]->normal += norm;
}

Three_Vector const* Ac3d_Object::get_normal(size_t index) const
{
    assert(index < m_vertices.size());
    return &(m_vertices[index]->normal);
}

void Ac3d_Object::add_surface(Surface_Ptr surf)
{
    m_surfaces.push_back(std::move(surf));
}

//----------------------------------------------------------------------------------------
static int read_header(std::ifstream& is)
{
    std::string header;
    is >> header;
    if (header.size() < 5 || header.substr(0, 4) != "AC3D")
        throw Ac3d_Exception("AC3D header is missing");
    return get_version_number(header[4]);
}

static Three_Matrix read_matrix(std::ifstream& is)
{
    Three_Matrix mat{0.0};
    for (size_t i{0}; i < 3; ++i)
        for (size_t j{0}; j < 3; ++j)
            is >> mat[i][j];
    return mat;
}

static Three_Vector read_vector(std::ifstream& is)
{
    Three_Vector vec;
    is >> vec.x >> vec.y >> vec.z;
    return vec;
}

Ac3d::Ac3d(std::string const& file, double scale, Three_Vector const& translation,
           Three_Vector const& rotation)
    : m_file{file},
      m_scale{scale},
      m_translation{translation},
      m_rotation{rotation}
{
}

Ac3d::~Ac3d()
{
    // Trivial destructor instead of = default because of managed pointers.
}

GLuint Ac3d::build()
{
    read();
    auto id{glGenLists(1)};
    glNewList(id, GL_COMPILE);
    for (auto& obj : m_objects)
        obj->build();
    glEndList();
    return id;
}

void Ac3d::read()
{
    std::ifstream is{m_file.c_str()};
    if (!is)
        throw Ac3d_Exception("Couldn't open file \"" + m_file + '"');

    m_version = read_header(is);
    std::string label;
    while (is >> label)
    {
        if (label == "MATERIAL")
            m_materials.push_back(read_material(is));
        else if (label == "OBJECT")
            m_objects.push_back(read_object(is, m_scale, m_translation, m_rotation));
        else if (label[0] == '#')
            continue;
        else
            throw Ac3d_Exception("Not part of an object definition");
    }
    is.close();
}

Ac3d::Object_Ptr Ac3d::read_object(std::ifstream& is, double scale,
                                   Three_Vector const& translation,
                                   Three_Vector const& rotation)
{
    std::string type;
    is >> type;
    auto obj = std::make_unique<Ac3d_Object>(type, scale, translation, rotation);

    std::string label;
    while (is >> label)
    {
        if (label == "name")
            obj->set_name(get_quoted(is));
        else if (label == "data")
            obj->read_data(is);
        else if (label == "texture")
        {
            std::string dir = m_file.substr(0, m_file.find_last_of("/") + 1);
            std::string file = get_quoted(is);
            obj->set_texture_image(dir + file);
        }
        else if (label == "texrep")
        {
            Two_Vector p;
            is >> p.x >> p.y;
            obj->set_texture_repeat(p);
        }
        else if (label == "rot")
            obj->set_rotation(read_matrix(is));
        else if (label == "loc")
            obj->set_location(read_vector(is));
        else if (label == "url")
            obj->set_url(get_quoted(is));
        else if (label == "numvert")
        {
            size_t num;
            is >> num;
            for (size_t i = 0; i < num; i++)
            {
                double x, y, z;
                // The label is the x-value.
                is >> x >> y >> z;
                obj->add_vertex(x, y, z);
            }
        }
        else if (label == "numsurf")
        {
            size_t num;
            is >> num;
            for (size_t i{0}; i < num; ++i)
                obj->add_surface(read_surface(is, *obj));
        }
        else if (label == "kids")
        {
            size_t n_kids;
            is >> n_kids;
            for (size_t i{0}; i < n_kids; ++i)
            {
                is >> label;
                if (label != "OBJECT")
                    throw Ac3d_Exception("An OBJECT line must follow a kids line.");
                obj->add_kid(read_object(is, scale, translation, rotation));
            }
            break;
        }
        else
            std::cerr << "Ac3d::read_object(): Unrecognized OBJECT data: " << label << std::endl;
    }
    return obj;
}

Surface_Ptr Ac3d::read_surface(std::ifstream& is, Ac3d_Object& obj)
{
    std::string label;
    is >> label;
    if (label != "SURF")
        throw Ac3d_Exception("Expected A SURF section.");

    std::string surf_type;
    is >> surf_type;
    auto surf{new Ac3d_Surface( //std::make_unique<Ac3d_Surface>(
            surf_type, obj.get_scale(), obj.get_location(), obj.get_rotation())};

    auto mat{-1};
    is >> label;
    // Read the surface.
    if (label == "mat")
    {
        is >> mat;
        surf->set_material(m_materials[mat].get());
        is >> label;
    }
    if (label == "refs")
    {
        size_t num;
        is >> num;
        if (num == 3)
            surf->set_figure_type(Ac3d_Surface::TRIANGLE);
        else if (num == 4)
            surf->set_figure_type(Ac3d_Surface::QUADRILATERAL);

        std::vector<const Surface_Vertex*> verts(num);
        std::vector<size_t> refs(num);
        for (size_t i{0}; i < num; ++i)
        {
            Two_Vector tex;
            is >> refs[i] >> tex.x >> tex.y;
            verts[i] = new Surface_Vertex(
                obj.get_vertex(refs[i]), obj.get_normal(refs[i]), tex);
        }
        surf->set_vertices(verts);

        Three_Vector normal;
        if (num >= 3)
        {
            auto const& p0{*(verts[0]->coordinates)};
            auto const& p1{*(verts[1]->coordinates)};
            auto const& p2{*(verts[num - 1]->coordinates)};
            auto r1{p1 - p0};
            auto r2{p2 - p0};
            for (size_t i{0}; i < num; ++i)
            {
                normal = (r1.cross(r2)).unit();
                obj.add_normal(refs[i], normal);
            }
        }
        surf->set_normal(normal);
    }
    else
        throw Ac3d_Exception("Expected a mat or refs section.");

    if (mat == -1)
        throw Ac3d_Exception("Expected a mat section.");

    return surf;
}

Ac3d::Material_Ptr Ac3d::read_material(std::ifstream& is)
{
    auto read = [&is](std::string const& label){
        std::string actual_label;
        is >> actual_label;
        if (actual_label != label)
            throw Ac3d_Exception("Expected \"" + label + "\".");
    };
    auto read3 = [read, &is](std::string const& label){
        read(label);
        std::array<GLfloat, 3> values;
        for (size_t i{0}; i < 3; ++i)
            is >> values[i];
        return values;
    };
    auto read1 = [read, &is](std::string const& label){
        read(label);
        GLfloat value;
        is >> value;
        return value;
    };

    std::string name{get_quoted(is)};
    auto color{read3("rgb")};
    auto ambient{read3("amb")};
    auto emission{read3("emis")};
    auto specular{read3("spec")};
    auto shininess{read1("shi")};
    auto transparency{read1("trans")};
    return std::make_unique<Ac3d_Material>(name, color, ambient, emission, specular,
                                           shininess, transparency);
}
