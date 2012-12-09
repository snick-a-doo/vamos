//  Vamos Automotive Simulator
//  Copyright (C) 2001--2002 Sam Varner
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

#include "Three_Matrix.h"

#include <cmath>

using namespace Vamos_Geometry;

static void rotate_elements (double mat [3][3], 
                             int i, int j, int k, int l, 
                             double h, double s, double tau);

Three_Matrix::Three_Matrix ()
{
  identity ();
}


Three_Matrix::Three_Matrix (const double mat_in [3][3])
{
  for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
        {
          m_mat [i][j] = mat_in [i][j];
        }
    }
}


Three_Matrix::Three_Matrix (const Three_Matrix& mat)
{
  copy_in (mat);
}

Three_Matrix& 
Three_Matrix::operator = (const Three_Matrix& mat)
{
  if (&mat != this)
    {
      copy_in (mat);
    }
  return *this;
}

bool
Three_Matrix::operator == (const Three_Matrix& mat) const
{
  for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
        {
          if (m_mat [i][j] != mat [i][j])
            return false;
        }
    }
  return true;
}

Three_Vector 
Three_Matrix::unit (int index) const
{
  return Three_Vector (m_mat [0][index], m_mat [1][index], m_mat [2][index]);
}

void 
Three_Matrix::set_diagonal (double diag)
{
  for (int i = 0; i < 3; i++)
    {
      m_mat [i][i] = diag;
    }
}

void 
Three_Matrix::identity ()
{
  zero ();
  set_diagonal (1.0);
}

void 
Three_Matrix::zero ()
{
  for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
        {
          m_mat [i][j] = 0.0;
        }
    }
}

const Three_Matrix& 
Three_Matrix::rotate (const Three_Vector& delta_theta)
{
  const double angle = 0.5 * delta_theta.magnitude (); // in radians
  if (angle == 0.0)
    return *this;

  const Three_Vector unit = delta_theta.unit () * sin (angle);
  const double w = cos (angle);

  // This tranformation matrix is derived from quaternion analysis.
  Three_Matrix q_rot;
  const double& x = unit.x;
  const double& y = unit.y;
  const double& z = unit.z;

  const double wx = w*x;
  const double wy = w*y;
  const double wz = w*z;

  const double xx = x*x;
  const double xy = x*y;
  const double xz = x*z;

  const double yy = y*y;
  const double yz = y*z;

  const double zz = z*z;

  q_rot [0][0] = 1.0 - 2.0 * (yy + zz);
  q_rot [0][1] = 2.0 * (xy - wz);
  q_rot [0][2] = 2.0 * (xz + wy);

  q_rot [1][0] = 2.0 * (xy + wz);
  q_rot [1][1] = 1.0 - 2.0 * (xx + zz);
  q_rot [1][2] = 2.0 * (yz - wx);

  q_rot [2][0] = 2.0 * (xz - wy);
  q_rot [2][1] = 2.0 * (yz + wx);
  q_rot [2][2] = 1.0 - 2.0 * (xx + yy);
  // Note that the matrix is not symmetric.  However, since x, y, and
  // z are imaginary (although we treat them as real here) and w is
  // real, the transformation matrix is Hermetian (equal to its
  // complex conjugate transpose).

  return *this *= q_rot;
}

void 
Three_Matrix::diagonalize ()
{
  const int dim = 3; 

  int rotations = 0;
  double array_z [dim] = {0.0};
  double array_b [dim];
  double mat [dim][dim];
  
  for (int i = 0; i < dim; i++)
    {
      for (int j = 0; j < dim; j++)
        {
          mat [i][j] = m_mat [i][j];
          m_e_vec [i][j] = 0.0;
        }
      array_b [i] = m_mat [i][i];
      m_e_val [i] = m_mat [i][i];
      m_e_vec [i][i] = 1.0;
    }

  for (int iter = 0; iter < 50; iter++)
    {
      double off_diag = 0.0;
      for (int i = 0; i < (dim - 1); i++)
        {
          for (int j = i + 1; j < dim; j++)
            {
              off_diag += mat [i][j];
            }
        }
      if (off_diag == 0.0)
        break;

      double thresh = (iter < 4) ? (0.2 * off_diag / (dim * dim)) : 0.0;

      for (int row = 0; row < (dim - 1); row++)
        {
          for (int col = row + 1; col < dim; col++) 
            {
              double small = 100.0 * std::abs (mat [row][col]);
              if ((iter > 4) 
                  && (std::abs (m_e_val [row] + small) 
                      == std::abs (m_e_val [row]))
                  && (std::abs (m_e_val [col] + small) 
                      == std::abs (m_e_val [col])))
                {
                  mat [row][col] = 0.0;
                }
              else if (std::abs (mat [row][col]) > thresh)
                {
                  double param_h = m_e_val [col] - m_e_val [row];
                  double param_t;
                  if (std::abs (param_h) + small == std::abs (param_h))
                    {
                      param_t = mat [row][col] / param_h;
                    }
                  else
                    {
                      double theta = 0.5 * param_h / mat [row][col];
                      param_t = 1.0 / (std::abs (theta) + 
                                       sqrt (1.0 + theta * theta));
                      if (theta < 0.0)
                        param_t = -param_t;
                    }
                  double param_c = 1.0 / sqrt (1.0 + param_t * param_t);
                  double param_s = param_t * param_c;
                  double tau = param_s / (1.0 + param_c);

                  param_h = param_t * mat [row][col];
                  array_z [row] -= param_h;
                  array_z [col] += param_h;
                  m_e_val [row] -= param_h;
                  m_e_val [col] += param_h;
                  mat [row][col] = 0.0;

                  // Rotate the matrix.
                  for (int j = 0; j < row; j++)
                    rotate_elements (mat, j, row, j, col, 
                                     param_h, param_s, tau);
                  for (int j = row + 1; j < col; j++)
                    rotate_elements (mat, row, j, j, col, 
                                     param_h, param_s, tau);
                  for (int j = col + 1; j < dim; j++)
                    rotate_elements (mat, row, j, col, j, 
                                     param_h, param_s, tau);
                  // Rotate the eigenvectors.
                  for (int j = 0; j < dim; j++)
                    rotate_elements (m_e_vec, j, row, j, col, 
                                     param_h, param_s, tau);

                  rotations++;
                }
            }
        }
      for (int i = 0; i < dim; i++)
        {
          array_b [i] += array_z [i];
          m_e_val [i] = array_b [i];
          array_z [i] = 0.0;
        }
    }
}
    
void 
rotate_elements (double mat [3][3], int i, int j, int k, int l, 
                                 double param_h, double param_s, double tau)
{
  double param_g = mat [i][j];
  param_h = mat [k][l];
  mat [i][j] = param_g - param_s * (param_h + param_g * tau);
  mat [k][l] = param_h + param_s * (param_g - param_h * tau);
}

Three_Matrix 
Three_Matrix::eigen (Three_Vector* out_vec)
{
  diagonalize ();
  if (out_vec)
    *out_vec = Three_Vector (m_e_val [0],
                             m_e_val [1],
                             m_e_val [2]);
  return Three_Matrix (m_e_vec).transpose ();
}

Three_Matrix 
Three_Matrix::transpose () const
{
  Three_Matrix out_mat (m_mat);
  out_mat [0][1] = m_mat [1][0];
  out_mat [1][0] = m_mat [0][1];
  out_mat [0][2] = m_mat [2][0];
  out_mat [2][0] = m_mat [0][2];
  out_mat [1][2] = m_mat [2][1];
  out_mat [2][1] = m_mat [1][2];
  return out_mat;
}

Three_Matrix 
Three_Matrix::invert () const
{
  double det
    = m_mat [0][0] * m_mat [1][1] * m_mat [2][2]
    + m_mat [0][1] * m_mat [1][2] * m_mat [2][0]
    + m_mat [0][2] * m_mat [1][0] * m_mat [2][1]
    - m_mat [0][2] * m_mat [1][1] * m_mat [2][0]
    - m_mat [0][1] * m_mat [1][0] * m_mat [2][2]
    - m_mat [0][0] * m_mat [1][2] * m_mat [2][1];
  
  if (det == 0.0)
    {
      throw Singular_Matrix ();
    }
  
  // A^{-1} = C^T / det(A)
  // A^{-1}_{ij} = C_{ji} / det(A)
  // where C_{ji} is the cofactor of the A_{ji}.
  // See Boas p.122-123.
  Three_Matrix out_mat;
  out_mat [0][0] =
    (m_mat [1][1] * m_mat [2][2] - m_mat [1][2] * m_mat [2][1]) / det;
  out_mat [1][0] =
    (m_mat [1][2] * m_mat [2][0] - m_mat [1][0] * m_mat [2][2]) / det;
  out_mat [2][0] =
    (m_mat [1][0] * m_mat [2][1] - m_mat [1][1] * m_mat [2][0]) / det;
  out_mat [0][1] =
    (m_mat [2][1] * m_mat [0][2] - m_mat [2][2] * m_mat [0][1]) / det;
  out_mat [1][1] =
    (m_mat [2][2] * m_mat [0][0] - m_mat [2][0] * m_mat [0][2]) / det;
  out_mat [2][1] =
    (m_mat [2][0] * m_mat [0][1] - m_mat [2][1] * m_mat [0][0]) / det;
  out_mat [0][2] =
    (m_mat [0][1] * m_mat [1][2] - m_mat [0][2] * m_mat [1][1]) / det;
  out_mat [1][2] =
    (m_mat [0][2] * m_mat [1][0] - m_mat [0][0] * m_mat [1][2]) / det;
  out_mat [2][2] = 
    (m_mat [0][0] * m_mat [1][1] - m_mat [0][1] * m_mat [1][0]) / det;
  
  return out_mat;
}

namespace Vamos_Geometry
{
  Three_Vector 
  operator * (const Three_Vector& vec, const Three_Matrix& mat)
  {
    return Three_Vector 
      (vec.x * mat [0][0] + vec.y * mat [1][0] + vec.z * mat [2][0],
       vec.x * mat [0][1] + vec.y * mat [1][1] + vec.z * mat [2][1],
       vec.x * mat [0][2] + vec.y * mat [1][2] + vec.z * mat [2][2]);
  }

  Three_Matrix 
  operator * (const Three_Matrix& mat1, const Three_Matrix& mat2)
  {
    double out_mat [3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
          {
            for (int k = 0; k < 3; k++)
              {
                out_mat [i][j] += mat1 [i][k] * mat2 [k][j];
              }
          }
      }
    return Three_Matrix (out_mat);
  }

  Three_Matrix 
  operator * (double factor, const Three_Matrix& mat)
  {
    Three_Matrix out_mat = mat;
    return out_mat *= factor;
  }
  
  Three_Matrix 
  operator * (const Three_Matrix& mat, double factor)
  {
    return factor * mat;
  }

  // Stream operators.
  std::ostream& 
  operator << (std::ostream& os, Three_Matrix mat)
  {
    os << "[[ " << mat [0][0] 
       << ",\t" << mat [0][1]
       << ",\t" << mat [0][2] << "]\n"
       << " [ " << mat [1][0] 
       << ",\t" << mat [1][1]
       << ",\t" << mat [1][2] << "]\n"
       << " [ " << mat [2][0] 
       << ",\t" << mat [2][1]
       << ",\t" << mat [2][2] << "]]";
    return os;
  }

  // Return the Euler angles about the x (PHI), y (THETA), and z (PSI)
  // axes for the orientation matrix MAT.
  void
  euler_angles (const Three_Matrix& mat, 
                double* phi, double* theta, double* psi)
  {
    *theta = asin (mat [2][0]);

    if (std::abs (*theta) > 0.00001)
      {
        double cos_theta = cos (*theta);
        double tr_x = mat [2][2] / cos_theta;
        double tr_y = -mat [2][1] / cos_theta;
      
        *phi = atan2 (tr_y, tr_x);
      
        tr_x = mat [0][0] / cos_theta;
        tr_y = mat [1][0] / cos_theta;
      
        *psi = atan2 (tr_y, tr_x);
      }
    else
      {
        *phi = 0.0;
      
        double tr_x = mat [1][1];
        double tr_y = -mat [0][1];
        
        *psi = atan2 (tr_y, tr_x);
      }
  }
}
