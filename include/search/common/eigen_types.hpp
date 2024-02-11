//  * Copyright (c) 2024, Ramkumar Natarajan
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  *     * Redistributions of source code must retain the above copyright
//  *       notice, this list of conditions and the following disclaimer.
//  *     * Redistributions in binary form must reproduce the above copyright
//  *       notice, this list of conditions and the following disclaimer in the
//  *       documentation and/or other materials provided with the distribution.
//  *     * Neither the name of the Carnegie Mellon University nor the names of its
//  *       contributors may be used to endorse or promote products derived from
//  *       this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//

/*!
 * \file eigen_types.hpp 
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/11/24
*/

#include <stdio.h>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

/// Set red font in printf funtion
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
/// Set green font in printf funtion
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
/// Set yellow font in printf funtion
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
/// Set blue font in printf funtion
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
/// Set magenta font in printf funtion
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
/// Set cyan font in printf funtion
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
/// Reset font color in printf funtion
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#ifndef SEARCH_EIGEN_TYPES_HPP
#define SEARCH_EIGEN_TYPES_HPP

/*! \brief Rename the float type used in lib

    Default is set to be double, but user can change it to float.
*/

typedef double decimal_t;

typedef std::vector<int> vec_int;
typedef std::vector<double> vec_double;

/// Pre-allocated std::vector for Eigen using vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
/// Eigen 1D float vector of size N
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
/// Eigen 1D int vector of size N
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;
/// Eigen 1D int vector of dynamic size
using VecDi = Eigen::Matrix<int, Eigen::Dynamic, 1>;
/// Eigen 1D float vector of dynamic size
using VecDf = Eigen::Matrix<decimal_t, Eigen::Dynamic, 1>;
/// MxN Eigen matrix
template <int M, int N>
using Mati = Eigen::Matrix<int, M, N>;
/// MxN Eigen matrix
template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;
/// MxN Eigen matrix with M unknown
template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
/// Vector of Eigen 1D float vector
template <int N>
using vec_Vecf = vec_E<Vecf<N>>;
/// Vector of Eigen 1D int vector
template <int N>
using vec_Veci = vec_E<Veci<N>>;

/// Eigen 1D float vector of size 2
typedef Vecf<2> Vec2f;
/// Eigen 1D int vector of size 2
typedef Veci<2> Vec2i;
/// Eigen 1D float vector of size 3
typedef Vecf<3> Vec3f;
/// Eigen 1D int vector of size 3
typedef Veci<3> Vec3i;
/// Eigen 1D float vector of size 4
typedef Vecf<4> Vec4f;
/// Eigen 1D int vector of size 4
typedef Veci<4> Vec4i;
/// Column vector in float of size 6
typedef Vecf<6> Vec6f;
/// Column vector in int of size 6
typedef Veci<6> Vec6i;


/// Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
/// Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
/// Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
/// Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;
/// Vector of type Vec4f.
typedef vec_E<Vec4f> vec_Vec4f;
/// Vector of type Vec4i.
typedef vec_E<Vec4i> vec_Vec4i;

/// 2x2 Matrix in float
typedef Matf<2, 2> Mat2f;
/// 3x3 Matrix in float
typedef Matf<3, 3> Mat3f;
/// 4x4 Matrix in float
typedef Matf<4, 4> Mat4f;
/// 6x6 Matrix in float
typedef Matf<6, 6> Mat6f;

/// Dynamic Nx1 Eigen float vector
typedef Vecf<Eigen::Dynamic> VecDf;
/// Mx3 Eigen float matrix
typedef MatDNf<3> MatD3f;
/// Dynamic MxN Eigen float matrix
typedef Mati<Eigen::Dynamic, Eigen::Dynamic> MatDi;
/// Dynamic MxN Eigen int matrix
typedef Matf<Eigen::Dynamic, Eigen::Dynamic> MatDf;

/// Allias of Eigen::Affine2d
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
/// Allias of Eigen::Affine3d
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;

template<typename M>
static M loadEigenFromFile (const std::string & path, char delim= ' ') {
  using namespace Eigen;
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, delim)) {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
static void writeEigenToFile(std::string& fname, const MatDf& matrix)
{
  std::ofstream file(fname);
  if (file.is_open())
  {
    file << matrix.format(CSVFormat);
    file.close();
  }
}

#endif //SEARCH_EIGEN_TYPES_HPP
