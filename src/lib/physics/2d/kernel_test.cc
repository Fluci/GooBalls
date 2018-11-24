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

std::vector<FloatPrecision> getHs(){
    std::vector<FloatPrecision> hs;
    hs.push_back(0.01);
    hs.push_back(0.1);
    hs.push_back(0.5);
    hs.push_back(1.0);
    hs.push_back(5.0);
    hs.push_back(10.0);
    hs.push_back(100.0);
    return hs;
}

void testRadialSymmetry(Kernel& k, int experiments){
    int radiusSamples = std::sqrt(experiments);
    int angleSamples = radiusSamples;
    FloatPrecision h = 1.0;
    k.setH(h);
    Coordinates2d Xs(angleSamples, 2);
    Coordinates1d w, wgradN, wlap;
    Coordinates2d wgrad;
    Coordinates2d dirs(angleSamples, 2);
    for(int i = 0; i < radiusSamples; ++i){
        FloatPrecision radius = i/double(radiusSamples) * h;
        for(int j = 0; j < angleSamples; ++j){
            FloatPrecision angle = i/double(angleSamples) * 2*M_PI;
            dirs(j, 0) = std::sin(angle);
            dirs(j, 1) = std::cos(angle);
        }
        Xs = dirs * radius;
        k.compute(Xs, &w, &wgrad, &wlap);
        wgradN = wgrad.rowwise().norm();
        BOOST_TEST(w.maxCoeff() - w.minCoeff() < 0.0001);
        BOOST_TEST(wlap.maxCoeff() - wlap.minCoeff() < 0.0001);
        BOOST_TEST(wgradN.maxCoeff() - wgradN.minCoeff() < 0.0001);
        // TODO: check gradient direction
    }
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

void testZeroBorderGradient(Kernel& k, int experiments){
    Coordinates2d Xorig = randomUnitDisk(experiments).rowwise().normalized();
    auto hs = getHs();
    for(auto h : hs){
        Coordinates2d Xs = h*Xorig;
        Coordinates2d Wgrad;
        k.setH(h);
        k.compute(Xs, nullptr, &Wgrad, nullptr);
        for(int i = 0; i < Xs.rows(); ++i){
            BOOST_TEST(Wgrad.row(i).norm() == 0.0);
            BOOST_TEST(Wgrad(i, 0) == 0.0);
            BOOST_TEST(Wgrad(i, 1) == 0.0);
        }
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
        BOOST_TEST(w[i-1] >= w[i], std::to_string(i) + ": " + std::to_string(w[i-1]) + " >=! " + std::to_string(w[i]));
    }
}

void testGradientFiniteDifference(Kernel& k, int experiments){
    const Coordinates2d Xorig = randomUnitDisk(experiments);
    auto hs = getHs();
    for(auto h : hs){
        FloatPrecision dx = h * 0.00000001;
        k.setH(h);
        Coordinates2d Xs = 0.99 * h * Xorig;
        Coordinates2d Xsx = Xs;
        Xsx.col(0).array() += dx;
        Coordinates2d Xsy = Xs;
        Xsy.col(1).array() += dx;
        Coordinates1d W0, Wx, Wy;
        Coordinates2d WgradReceived;
        k.compute(Xs, &W0, &WgradReceived, nullptr);
        k.compute(Xsx, &Wx, nullptr, nullptr);
        k.compute(Xsy, &Wy, nullptr, nullptr);
        Coordinates2d gradExpected(Xs.rows(), 2);
        gradExpected.col(0) = (Wx - W0)/dx;
        gradExpected.col(1) = (Wy - W0)/dx;
        for(int i = 0; i < Xs.rows(); ++i){
            BOOST_TEST(gradExpected(i, 0) == WgradReceived(i, 0));
            BOOST_TEST(gradExpected(i, 1) == WgradReceived(i, 1));
        }
    }
}

void testLaplacianFiniteDifference(Kernel& k, int experiments){
    const Coordinates2d Xorig = randomUnitDisk(experiments);
    auto hs = getHs();
    for(auto h : hs){
        FloatPrecision dx = h * 0.001;
        k.setH(h);
        Coordinates2d Xs = 0.99 * h * Xorig;
        Coordinates2d Xsx1, Xsx2, Xsy1, Xsy2;
        Xsx1 = Xs; Xsx1.col(0).array() -= dx;
        Xsx2 = Xs; Xsx2.col(0).array() += dx;
        Xsy1 = Xs; Xsy1.col(1).array() -= dx;
        Xsy2 = Xs; Xsy2.col(1).array() += dx;

        Coordinates1d W0, Wx1, Wx2, Wy1, Wy2;
        Coordinates1d Wlap;
        k.compute(Xs, &W0, nullptr, &Wlap);
        k.compute(Xsx1, &Wx1, nullptr, nullptr);
        k.compute(Xsx2, &Wx2, nullptr, nullptr);
        k.compute(Xsy1, &Wy1, nullptr, nullptr);
        k.compute(Xsy1, &Wy2, nullptr, nullptr);
        Coordinates1d expectedLaplacian = (Wx1 + Wx2 + Wy1 + Wy2 - 4*W0) / (dx*dx);
        for(int i = 0; i < Xs.rows(); ++i){
//            BOOST_TEST(expectedLaplacian[i] == Wlap[i]);
        }
    }
}

void testNormalization(Kernel& k, int experiments){
    auto hs = getHs();
    int n = std::sqrt(experiments);
    n = 100;
    for(auto h : hs){
        Coordinates2d Xs(n*n, 2);
        int inserted = 0;
        for(int i = 0; i < n; ++i){
            for(int j = 0; j < n; ++j){
                TranslationVector pos;
                pos[0] = h*(2*i/double(n)-1);
                pos[1] = h*(2*j/double(n)-1);
                if(pos.norm() > h){
                    continue;
                }
                Xs.row(inserted++) = pos;
            }
        }
        Xs.conservativeResize(inserted, Eigen::NoChange);
        FloatPrecision dA;
        dA = 1.0/(n*n); // area of one test square
        Coordinates1d W;
        k.setH(h);
        k.compute(Xs, &W, nullptr, nullptr);
        FloatPrecision totalWeight = W.sum()*dA;
        BOOST_TEST(totalWeight == 1.0);
    }
}

} // Physics
} // d2
} // GooBalls
