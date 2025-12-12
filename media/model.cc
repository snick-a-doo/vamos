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
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "model.h"
#include "texture-image.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/quaternion.h>
#include <assimp/matrix4x4.h>
#include <assimp/vector3.h>

#include <cassert>
#include <cstdlib>
#include <numbers>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

using namespace std::numbers;

namespace Vamos_Media
{
Model::Model(std::string const& file, double scale,
             Three_Vector const& translation,
             Three_Vector const& rotation)
    : m_id{glGenLists(1)}
{
    Assimp::Importer importer;
    auto const* scene{importer.ReadFile(file.c_str(),
                                        aiProcessPreset_TargetRealtime_Quality)};
    // Assimp quaternions are defined by pitch, yaw, and roll, in that order. In Vamos
    // coordinates, z is up and x is forward. y is left to make it right handed. The 90°
    // rotation about x is needed to orient existing models properly.
    if (!scene)
        throw Model_Exception("Could not read model file " + file);
    aiMatrix4x4t<double> xform{aiVector3t<double>{scale},
                               aiQuaterniont<double>{-rotation.y,
                                                     rotation.z,
                                                     rotation.x + 0.5 * std::numbers::pi},
                               aiVector3t<double>{translation.x, translation.y, translation.z}};

    glNewList(m_id, GL_COMPILE);

    for (size_t i{0}; i < scene->mNumMeshes; ++i)
    {
        auto const* mesh{scene->mMeshes[i]};
        auto const* mat{scene->mMaterials[mesh->mMaterialIndex]};
        aiColor3D color(0.0, 0.0, 0.0);
        // Assimp's overloaded Get() methods take 4 arguments. The AI_MATKEY macros expand
        // to the first 3 arguments. The last is a reference to the value.
        mat->Get(AI_MATKEY_COLOR_DIFFUSE, color);

        auto alpha{1.0};
        mat->Get(AI_MATKEY_OPACITY, alpha);
        auto two_sided{0};
        mat->Get(AI_MATKEY_TWOSIDED, two_sided);
        auto shading{0};
        mat->Get(AI_MATKEY_SHADING_MODEL, shading);
        auto shine{0.0};
        mat->Get(AI_MATKEY_SHININESS_STRENGTH, shine);

        glPushAttrib(GL_ENABLE_BIT);
        if (!two_sided)
            glEnable(GL_CULL_FACE);

        aiString path;
        auto const* tex{mesh->mTextureCoords[0]};
        if (tex && mat->GetTexture(aiTextureType_DIFFUSE, 0, &path) == aiReturn_SUCCESS)
        {
            Texture_Image{std::string(path.C_Str())}.activate();
            glEnable(GL_TEXTURE_2D);
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        }
        else
            glDisable(GL_TEXTURE_2D);
        glBegin(GL_TRIANGLES);
        glColor4f(color[0], color[1], color[2], alpha);
        for (size_t j{0}; j < mesh->mNumFaces; ++j)
        {
            auto face{mesh->mFaces[j]};
            if (face.mNumIndices != 3)
                continue;
            for (size_t vert{0}; vert < 3; ++vert)
            {
                auto index{face.mIndices[vert]};
                auto normal{mesh->mNormals[index]};
                normal *= xform;
                auto vertex{mesh->mVertices[index]};
                vertex *= xform;
                // I don't know why textures are inverted. This is also seen in the
                // Facade::draw().
                if (tex)
                    glTexCoord2f(tex[index][0], 1.0 - tex[index][1]);
                glNormal3f(normal[0], normal[1], normal[2]);
                glVertex3f(vertex[0], vertex[1], vertex[2]);
            }
        }
        glEnd();
        glPopAttrib();
    }
    glEndList();
}

GLuint Model::build() const
{
    return m_id;
}
}
