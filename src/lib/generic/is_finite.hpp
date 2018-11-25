#pragma once

#include <Eigen/Core>

namespace GooBalls {

template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
    return ( (x - x).array() == (x - x).array()).all();
}



} // GooBalls
