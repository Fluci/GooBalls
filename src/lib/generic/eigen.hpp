#pragma once

#include <Eigen/Core>
#include <vector>

namespace GooBalls{


template <typename Derived>
void minSize(Eigen::DenseBase<Derived>& a, int N){
    if(a.rows() >= N){
        return;
    }
    a.derived().resize(N, Eigen::NoChange);
}

template <typename Derived>
void minSize(Eigen::DenseBase<Derived>& a, int N, int M){
    if(a.rows() >= N && a.cols() >= M){
        return;
    }
    a.derived().resize(N, M);
}

template <typename Derived1, typename Derived2>
void pickRows(
        const Eigen::DenseBase<Derived1>& data,
        const std::vector<int> indexes,
        Eigen::DenseBase<Derived2> & out
        ) {
    minSize(out, indexes.size(), data.cols());
    for(size_t i = 0; i < indexes.size(); ++i){
        assert(indexes[i] < data.rows());
        out.row(i) = data.row(indexes[i]);
    }
}

}
