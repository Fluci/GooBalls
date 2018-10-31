#include <Eigen/Core>


/**
 * A bunch of typedef for general use.
 * */

typedef double CoordinatePrecision;
typedef Eigen::Matrix<CoordinatePrecision, Eigen::Dynamic, 2, Eigen::RowMajor> Coordinates2d;
typedef Eigen::Matrix<CoordinatePrecision, Eigen::Dynamic, 3, Eigen::RowMajor> Coordinates3d;


/// Colors in the range [0,1]
typedef double ColorFloatPrecision;

/// Holds a list of intensity values
typedef Eigen::Matrix<ColorFloatPrecision, Eigen::Dynamic, 1, Eigen::RowMajor> ColorsFloatGrey;

/// Holds a list of (intensity, alpha) values 
typedef Eigen::Matrix<ColorFloatPrecision, Eigen::Dynamic, 2, Eigen::RowMajor> ColorsFloatGreyA;

/// Holds a list of (red, green, blue) values
typedef Eigen::Matrix<ColorFloatPrecision, Eigen::Dynamic, 3, Eigen::RowMajor> ColorsFloatRGB;

/// Holds a list of (red, green, blue, alpha) values
typedef Eigen::Matrix<ColorFloatPrecision, Eigen::Dynamic, 4, Eigen::RowMajor> ColorsFloatRGBA;


/// Colors in the range [0,255]
typedef char ColorIntPrecision;

/// Holds a list of intensity values
typedef Eigen::Matrix<ColorIntPrecision, Eigen::Dynamic, 1, Eigen::RowMajor> ColorsIntGrey;

/// Holds a list of (intensity, alpha) values 
typedef Eigen::Matrix<ColorIntPrecision, Eigen::Dynamic, 2, Eigen::RowMajor> ColorsIntGreyA;

/// Holds a list of (red, green, blue) values
typedef Eigen::Matrix<ColorIntPrecision, Eigen::Dynamic, 3, Eigen::RowMajor> ColorsIntRGB;

/// Holds a list of (red, green, blue, alpha) values
typedef Eigen::Matrix<ColorIntPrecision, Eigen::Dynamic, 4, Eigen::RowMajor> ColorsIntRGBA;

typedef int VertexIndex;
typedef int TriangleIndex;

/// A list of triangles where each triangle corresponds to one row entry consisting of three indices to vertices.
typedef Eigen::Matrix<VertexIndex, Eigen::Dynamic, 3, Eigen::RowMajor> TriangleList;

