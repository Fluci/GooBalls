#pragma once

#include <Eigen/Core>

namespace GooBalls {

/**
 * A bunch of typedef for general use.
 * */

typedef double Float;
typedef Eigen::Matrix<Float, Eigen::Dynamic, 1, Eigen::ColMajor> Coordinates1d;
/// Each row is a coordinate tuple: (x,y)
typedef Eigen::Matrix<Float, Eigen::Dynamic, 2, Eigen::RowMajor> Coordinates2d;
/// Each row is a coordiante tuple: (x,y,z)
typedef Eigen::Matrix<Float, Eigen::Dynamic, 3, Eigen::RowMajor> Coordinates3d;

typedef Eigen::Matrix<Float, 2, 2, Eigen::RowMajor> RotationMatrix;
typedef Eigen::Matrix<Float, 1, 2, Eigen::RowMajor> TranslationVector;

/// Colors in the range [0,1]
typedef double ColorFloat;

/// Holds a list of intensity values
typedef Eigen::Matrix<ColorFloat, Eigen::Dynamic, 1, Eigen::RowMajor> ColorsFloatGrey;

/// Holds a list of (intensity, alpha) values 
typedef Eigen::Matrix<ColorFloat, Eigen::Dynamic, 2, Eigen::RowMajor> ColorsFloatGreyA;

/// Holds a list of (red, green, blue) values
typedef Eigen::Matrix<ColorFloat, Eigen::Dynamic, 3, Eigen::RowMajor> ColorsFloatRGB;

/// Holds a list of (red, green, blue, alpha) values
typedef Eigen::Matrix<ColorFloat, Eigen::Dynamic, 4, Eigen::RowMajor> ColorsFloatRGBA;


/// Colors in the range [0,255]
typedef char ColorInt;

/// Holds a list of intensity values
typedef Eigen::Matrix<ColorInt, Eigen::Dynamic, 1, Eigen::RowMajor> ColorsIntGrey;

/// Holds a list of (intensity, alpha) values 
typedef Eigen::Matrix<ColorInt, Eigen::Dynamic, 2, Eigen::RowMajor> ColorsIntGreyA;

/// Holds a list of (red, green, blue) values
typedef Eigen::Matrix<ColorInt, Eigen::Dynamic, 3, Eigen::RowMajor> ColorsIntRGB;

/// Holds a list of (red, green, blue, alpha) values
typedef Eigen::Matrix<ColorInt, Eigen::Dynamic, 4, Eigen::RowMajor> ColorsIntRGBA;

typedef int VertexIndex;
typedef int TriangleIndex;

/// A list of triangles where each triangle corresponds to one row entry consisting of three indices to vertices.
typedef Eigen::Matrix<VertexIndex, Eigen::Dynamic, 3, Eigen::RowMajor> TriangleList;

} // GooBalls
