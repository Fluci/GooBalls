#include <Eigen/Core>


/**
 * A bunch of typedef for general use.
 * */

typedef double CoordPrecision;
typedef Eigen::Matrix<CoordPrecision, Eigen::Dynamic, 2, Eigen::RowMajor> Coordinates2d;
typedef Eigen::Matrix<CoordPrecision, Eigen::Dynamic, 3, Eigen::RowMajor> Coordinates3d;

/// Colors in the range [0,1]
typedef double ColorFloatPrecision;
typedef Eigen::Matrix<ColorFloatPrecision, Eigen::Dynamic, 1, Eigen::RowMajor> ColorsFloatGrey;
typedef Eigen::Matrix<ColorFloatPrecision, Eigen::Dynamic, 3, Eigen::RowMajor> ColorsFloatRGB;

/// Colors in the range [0,255]
typedef char ColorIntPrecision;
typedef Eigen::Matrix<ColorIntPrecision, Eigen::Dynamic, 1, Eigen::RowMajor> ColorsIntGrey;
typedef Eigen::Matrix<ColorIntPrecision, Eigen::Dynamic, 3, Eigen::RowMajor> ColorsIntRGB;

typedef int VertexIndex;
typedef int TriangleIndex;

/// A list of triangles where each triangle corresponds to one row entry consisting of three indices to vertices.
typedef Eigen::Matrix<VertexIndex, Eigen::Dynamic, 3, Eigen::RowMajor> TriangleList;

