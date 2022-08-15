//  Copyright (C) 2001--2022 Sam Varner
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

#include "three-matrix.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace Vamos_Geometry;

using Vec = Three_Matrix::Vec;
using Mat = Three_Matrix::Mat;
auto constexpr N = Three_Matrix::N;

Mat diagonal(double diag)
{
    return {std::array{diag, 0.0, 0.0},
            std::array{0.0, diag, 0.0},
            std::array{0.0, 0.0, diag}};
}

Three_Matrix::Three_Matrix(double diag)
    : m_mat{diagonal(diag)}
{
}

Vec& Three_Matrix::operator [] (size_t index)
{
    return m_mat[index];
}

const Vec& Three_Matrix::operator [] (size_t index) const
{
    return m_mat[index];
}

Three_Matrix& Three_Matrix::identity()
{
    m_mat = diagonal(1.0);
    return *this;
}

Three_Matrix& Three_Matrix::zero()
{
    m_mat = diagonal(0.0);
    return *this;
}

Three_Matrix& Three_Matrix::rotate(Three_Vector const& delta_theta)
{
    auto const angle{0.5 * delta_theta.magnitude()}; // in radians
    if (angle == 0.0)
        return *this;

    auto const unit{delta_theta.unit() * sin(angle)};
    auto const w{cos(angle)};

    // This tranformation matrix is derived from quaternion analysis.
    auto const x{unit.x};
    auto const y{unit.y};
    auto const z{unit.z};

    auto const wx{w * x};
    auto const wy{w * y};
    auto const wz{w * z};

    auto const xx{x * x};
    auto const xy{x * y};
    auto const xz{x * z};

    auto const yy{y * y};
    auto const yz{y * z};

    auto const zz{z * z};

    Three_Matrix q_rot(0.0);
    q_rot[0] = {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)};
    q_rot[1] = {2.0 * (xy + wz),       1.0 - 2.0 * (zz + xx), 2.0 * (yz - wx)};
    q_rot[2] = {2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)};
    // Note that the matrix is not symmetric.  However, since x, y, and
    // z are imaginary (although we treat them as real here) and w is
    // real, the transformation matrix is Hermetian (equal to its
    // complex conjugate transpose).

    return *this *= q_rot;
}

Three_Matrix& Three_Matrix::operator *= (Three_Matrix const& mat2)
{
    Mat mult{0.0};
    for (size_t i{0}; i < N; ++i)
        for (size_t j{0}; j < N; ++j)
            for (size_t k{0}; k < N; ++k)
                mult[i][j] += m_mat[i][k] * mat2[k][j];
    m_mat = mult;
    return *this;
}

namespace Vamos_Geometry
{
Three_Matrix transpose(Three_Matrix const& mat)
{
    Three_Matrix t_mat = mat;
    std::swap(t_mat[0][1], t_mat[1][0]);
    std::swap(t_mat[0][2], t_mat[2][0]);
    std::swap(t_mat[1][2], t_mat[2][1]);
    return t_mat;
}

Three_Matrix invert(Three_Matrix const& mat)
{
    auto det{mat[0][0] * mat[1][1] * mat[2][2]
             + mat[0][1] * mat[1][2] * mat[2][0]
             + mat[0][2] * mat[1][0] * mat[2][1]
             - mat[0][2] * mat[1][1] * mat[2][0]
             - mat[0][1] * mat[1][0] * mat[2][2]
             - mat[0][0] * mat[1][2] * mat[2][1]};

    if (det == 0.0)
        throw Singular_Matrix();

    // A^{-1} = C^T / det(A)
    // A^{-1}_{ij} = C_{ji} / det(A)
    // where C_{ji} is the cofactor of the A_{ji}.
    // See Boas p.122-123.
    Three_Matrix out_mat{0.0};
    out_mat[0][0] = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
    out_mat[1][0] = mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2];
    out_mat[2][0] = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];
    out_mat[0][1] = mat[2][1] * mat[0][2] - mat[2][2] * mat[0][1];
    out_mat[1][1] = mat[2][2] * mat[0][0] - mat[2][0] * mat[0][2];
    out_mat[2][1] = mat[2][0] * mat[0][1] - mat[2][1] * mat[0][0];
    out_mat[0][2] = mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1];
    out_mat[1][2] = mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2];
    out_mat[2][2] = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    return out_mat * (1.0 / det);
}

Three_Matrix operator * (double factor, Three_Matrix const& mat)
{
    Three_Matrix out_mat{0.0};
    for (size_t i{0}; i < N; ++i)
        for (size_t j{0}; j < N; ++j)
            out_mat[i][j] = factor * mat[i][j];
    return out_mat;
}

Three_Matrix operator * (Three_Matrix const& mat, double factor)
{
    return factor * mat;
}

Three_Vector operator * (Three_Vector const& vec, Three_Matrix const& mat)
{
    return {vec.x * mat[0][0] + vec.y * mat[1][0] + vec.z * mat[2][0],
            vec.x * mat[0][1] + vec.y * mat[1][1] + vec.z * mat[2][1],
            vec.x * mat[0][2] + vec.y * mat[1][2] + vec.z * mat[2][2]};
}

Three_Vector operator * (Three_Matrix const& mat, Three_Vector const& vec)
{
    return {vec.x * mat[0][0] + vec.y * mat[0][1] + vec.z * mat[0][2],
            vec.x * mat[1][0] + vec.y * mat[1][1] + vec.z * mat[1][2],
            vec.x * mat[2][0] + vec.y * mat[2][1] + vec.z * mat[2][2]};
}

Three_Matrix operator * (Three_Matrix const& mat1, Three_Matrix const& mat2)
{
    Three_Matrix out_mat{mat1};
    return out_mat *= mat2;
}

std::ostream& operator << (std::ostream& os, Three_Matrix const& mat)
{
    os << "[[" << mat[0][0] << ",\t" << mat[0][1] << ",\t" << mat[0][2] << "]\n"
       << " [" << mat[1][0] << ",\t" << mat[1][1] << ",\t" << mat[1][2] << "]\n"
       << " [" << mat[2][0] << ",\t" << mat[2][1] << ",\t" << mat[2][2] << "]]";
    return os;
}
}
