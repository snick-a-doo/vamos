//	Vamos Automotive Simulator
//  Copyright (C) 2001--2004 Sam Varner
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

#ifndef _THREE_MATRIX_H_
#define _THREE_MATRIX_H_

#include "Three_Vector.h"

namespace Vamos_Geometry
{
  // Exception.
  class Singular_Matrix {};

  class Three_Matrix
  {
	double m_mat [3][3];
	double m_e_vec [3][3];
	double m_e_val [3];
  
	// The diagonalization routine used by eigen ().
	void diagonalize ();

	// Put DIAG in each diagonal element.
	void set_diagonal (double diag);

	// Copy the elements of MAT into this matrix.
	inline void copy_in (const Three_Matrix& mat);

  public:
	// Initialize to identity matrix.
	Three_Matrix ();
	// Initialize elements with a C-style 2-D array.
	Three_Matrix (const double [3][3]);
	// Copy constructor.
	Three_Matrix (const Three_Matrix& mat);
	// Copy assignment.
	Three_Matrix& operator = (const Three_Matrix& mat);

	double* operator [] (int index) { return m_mat [index]; }
	const double* operator [] (int index) const { return m_mat [index]; }

    bool operator == (const Three_Matrix& mat) const;
  
	// Return the unit vector for a given axis.
	Three_Vector unit (int index) const; 

	// Set this matrix to the identity matrix.
	void identity ();
	// Set all elements to zero.
	void zero ();
	// Rotate the frame about the vector delta_theta, by an angle equal to
	// the magnitude of delta_theta.
	const Three_Matrix& rotate (const Three_Vector& delta_theta);
  
	// Return the transpose of this matrix.
	Three_Matrix transpose () const;

	// Retrun the inverse of this matrix.
	Three_Matrix invert () const;

	// Return the eigen vectors of this matrix.  
	// Set `e_val' to the eigenvalues.
	Three_Matrix eigen (Three_Vector* e_val = 0);

	// Return the product of this matrix and `mat'.
	inline Three_Matrix& operator *= (const Three_Matrix& mat);
	inline Three_Matrix& operator *= (double);
  };

  void 
  Three_Matrix::copy_in (const Three_Matrix& mat)
  {
    for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
          {
            m_mat [i][j] = mat [i][j];
          }
      }
  }

  Three_Matrix& 
  Three_Matrix::operator *= (double factor)
  {
    for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
          {
            m_mat [i][j] *= factor;
          }
      }
    return *this;
  }

  Three_Matrix& 
  Three_Matrix::operator *= (const Three_Matrix& mat2)
  {
    double temp_mat [3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
          {
            for (int k = 0; k < 3; k++)
              {
                temp_mat [i][j] += m_mat [i][k] * mat2 [k][j];
              }
          }
      }
    copy_in (temp_mat);
    return *this;
  }

  // Multiplication with a matrix.
  Three_Matrix operator * (const Three_Matrix&, const Three_Matrix&);
  // Multiplication with a vector.
  inline Three_Vector operator * (const Three_Matrix& mat,
                                   const Three_Vector& vec)
  {
    return Three_Vector 
      (vec.x * mat [0][0] + vec.y * mat [0][1] + vec.z * mat [0][2],
       vec.x * mat [1][0] + vec.y * mat [1][1] + vec.z * mat [1][2],
       vec.x * mat [2][0] + vec.y * mat [2][1] + vec.z * mat [2][2]);
  }
  Three_Vector operator * (const Three_Vector&, const Three_Matrix&);
  // Multiplication by a scalar.
  Three_Matrix operator * (double, const Three_Matrix&);
  Three_Matrix operator * (const Three_Matrix&, double);

  // Stream operator.
  std::ostream& operator << (std::ostream& os, Three_Matrix mat);

  // Return the Euler angles about the x (PHI), y (THETA), and z (PSI)
  // axes for the orientation matrix MAT.
  void euler_angles (const Three_Matrix& mat, 
					 double* phi, double* theta, double* psi);
}

#endif
