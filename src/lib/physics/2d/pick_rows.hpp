#pragma once

#include <Eigen/Core>
#include <vector>

namespace GooBalls{

template <typename Derived1, typename Derived2>
void pickRows(
        const Eigen::DenseBase<Derived1>& data,
        const std::vector<int> indexes,
        Derived2& out
        ) {
    out.resize(indexes.size(), data.cols());
    for(size_t i = 0; i < indexes.size(); ++i){
        out.row(i) = data.row(indexes[i]);
    }
}

}
