#include "kernel_test.hpp"

#include <boost/test/unit_test.hpp>
#include <random>
#include <iostream>

namespace GooBalls {
namespace d2 {
namespace Physics {
/**
 * Properties a kernel should have:
 * C^0, C^1 continuous
 * evaluated at r = h, the value should be zero
 * Kernel must be normalized: int_X W(|x - x_i|) dx = 1
 */

/// Create `samples` 2d points on unit disk
Coordinates2d randomUnitDisk(int samples){
    std::mt19937 rnd;
    rnd.seed(23);
    std::uniform_real_distribution<> dist(0.0, 1.0);
    Coordinates2d Xs(samples, 2);
    for(int i = 0; i < samples; ++i){
        do {
            Xs(i, 0) = dist(rnd);
            Xs(i, 1) = dist(rnd);
            // rejection sampling
        } while (Xs.row(i).norm() > 1.0);
    }
    return Xs;
}

void testZeroBorder(Kernel& k, int experiments){
    std::mt19937 rnd;
    rnd.seed(17); // we use a fixed seed for repeatability
    std::uniform_real_distribution<> dist(-10.0, 10.0);
    for(int i = 0; i < experiments; ++i){
        Coordinates2d x(1,2);
        x(0,0) = dist(rnd);
        x(0,1) = dist(rnd);
        auto h = x.norm();
        k.setH(h);
        Coordinates1d w;
        k.compute(x, &w, nullptr, nullptr);
        BOOST_TEST(w[0] == 0.0);
    }
}

void testNonNegativity(Kernel& k, int experiments){
    Coordinates2d Xs = randomUnitDisk(experiments);
    Coordinates1d W;
    k.setH(1.0);
    k.compute(Xs, &W, nullptr, nullptr);
    for(int i = 0; i < experiments; ++i){
        BOOST_TEST(0 <= W[i]);
    }
}


void testMonotonicity(Kernel& k, int experiments){
    Coordinates2d Xs(experiments, 2);
    Xs.setZero();
    FloatPrecision h = 1.0;
    for(int i = 0; i < experiments; ++i){
        Xs(i, 0) = i/double(experiments-1)*h;
    }
    k.setH(h);
    Coordinates1d w;
    k.compute(Xs, &w, nullptr, nullptr);
    for(int i = 1; i < experiments; ++i){
        BOOST_TEST(w[i-1] > w[i]);
    }
}

void testNormalization(Kernel& k, int experiments){
    FloatPrecision h = 1.0;
    int special = 7;
    Coordinates2d Xs = randomUnitDisk(experiments+special);
    Xs(0, 0) = 0.0; Xs(0, 0) = 0.0;
    Xs(1, 0) = 0.0; Xs(1, 1) = 1.0;
    Xs(2, 0) = 1.0; Xs(2, 1) = 0.0;
    Xs(3, 0) = 0.1; Xs(3, 1) = 0.1;
    Xs(4, 0) = 0.2; Xs(4, 1) = 0.2;
    Xs(5, 0) = 0.3; Xs(5, 1) = 0.3;
    Xs(6, 0) = 0.4; Xs(6, 1) = 0.4;
    Coordinates1d W;
    k.setH(h);
    k.compute(Xs, &W, nullptr, nullptr);
    for(int i = 0; i < experiments; ++i){
       // std::cout << W[i] << std::endl;
    }
    auto integral = W.sum() / experiments * 4.0;
    BOOST_TEST(integral == 1.0);
}

} // Physics
} // d2
} // GooBalls
