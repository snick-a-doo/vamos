//  Vamos Automotive Simulator
//  Copyright (C) 2001-2022 Sam Varner
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

#ifndef VAMOS_GEOMETRY_THREE_MATRIX_H_INCLUDED
#define VAMOS_GEOMETRY_THREE_MATRIX_H_INCLUDED

#include "three-vector.h"

#include <array>
#include <stdexcept>

namespace Vamos_Geometry
{
/// Exception thrown when trying to invert a singular matrix.
class Singular_Matrix : public std::runtime_error
{
public:
    Singular_Matrix()
        : std::runtime_error("Matrix is singular. Can't invert.")
    {}
};

/// Class representing a 3×3 matrix.
class Three_Matrix
{
public:
    static constexpr size_t N{3};
    using Vec = std::array<double, N>;
    using Mat = std::array<Vec, N>;

    /// Initialize to @p diag times the identity matrix.
    Three_Matrix(double diag);

    /// Element access
    /// @{
    Vec& operator[](size_t index);
    Vec const& operator[](size_t index) const;
    /// @}

    /// Set this matrix to the identity matrix.
    /// @return A reference to this matrix to allow chaining.
    Three_Matrix& identity();
    /// Set all elements to zero.
    /// @return A reference to this matrix to allow chaining.
    Three_Matrix& zero();
    /// Treat this matrix as a rotation matrix. Represent it in a reference frame rotated
    /// about @p delta_theta by an angle equal to the magnitude of @p delta_theta.
    Three_Matrix& rotate(Three_Vector const& delta_theta);
    /// Multiply by another matrix.
    Three_Matrix& operator *= (Three_Matrix const& mat);

    /// Test for equality.
    friend bool operator == (Three_Matrix const& m1, Three_Matrix const& m2) = default;

private:
    /// The matrix elements.
    Mat m_mat;
};

/// @return The transpose of a matrix.
Three_Matrix transpose(Three_Matrix const& mat);
/// @return the inverse of a matrix.
Three_Matrix invert(Three_Matrix const& mat);
/// Multiplication by a scalar.
/// @{
Three_Matrix operator * (double, Three_Matrix const&);
Three_Matrix operator * (Three_Matrix const&, double);
/// @}
/// Multiplication with a vector.
/// @{
Three_Vector operator * (Three_Vector const& vec, Three_Matrix const& mat);
Three_Vector operator * (Three_Matrix const& mat, Three_Vector const& vec);
/// @}
/// Multiplication with a matrix.
Three_Matrix operator * (Three_Matrix const&, Three_Matrix const&);
/// Stream operator.
std::ostream& operator << (std::ostream& os, Three_Matrix const& mat);
}

#endif // VAMOS_GEOMETRY_THREE_MATRIX_H_INCLUDED
