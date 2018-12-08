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

Coordinates1d increasingToOne(int n){
    Coordinates1d Xorig(n);
    for(int i = 0; i  < n; ++i){
        Xorig[i] = double(i)/n;
    }
    assert(Xorig.maxCoeff() < 1.0);
    return Xorig;
}

std::vector<Float> getHs(){
    std::vector<Float> hs;
    hs.push_back(0.1);
    hs.push_back(0.5);
    hs.push_back(1.0);
    hs.push_back(5.0);
    hs.push_back(10.0);
    hs.push_back(50.0);
    hs.push_back(100.0);
    return hs;
}


void testRadialSymmetry(Kernel& k, int experiments){
    int radiusSamples = std::sqrt(experiments);
    int angleSamples = radiusSamples;
    auto hs = getHs();
    for(auto h : hs){
        k.setH(h);
        Coordinates2d Xs(angleSamples, 2);
        Coordinates1d w, wgradN, wlap;
        Coordinates2d wgrad;
        Coordinates2d dirs(angleSamples, 2);
        for(int i = 1; i < radiusSamples; ++i){
            Float radius = i/double(radiusSamples) * h;
            for(int j = 0; j < angleSamples; ++j){
                Float angle = i/double(angleSamples) * 2*M_PI;
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
}

void testZeroBorder1d(Kernel& k){
    auto hs = getHs();
    Coordinates1d ws(hs.size());
    for(size_t i = 0; i < hs.size(); ++i){
        Float h = hs[i];
        k.setH(h);
        Coordinates1d Xs(1);
        Xs[0] = h;
        Coordinates1d w;
        k.compute1d(Xs.array()*Xs.array(), &w, nullptr, nullptr);
        ws[i] = w[0];
        // BOOST_TEST(w[0] == 0.0);
    }
    ws = ws.array().abs();
    BOOST_TEST(ws.maxCoeff() == 0.0);
}

void testZeroBorder(Kernel& k, int experiments){
    std::mt19937 rnd;
    rnd.seed(17); // we use a fixed seed for repeatability
    std::uniform_real_distribution<> dist(-10.0, 10.0);
    Coordinates1d ws(experiments);
    for(int i = 0; i < experiments; ++i){
        Coordinates2d x(1,2);
        x(0,0) = dist(rnd);
        x(0,1) = dist(rnd);
        auto h = x.norm();
        k.setH(h);
        Coordinates1d w;
        k.compute(x, &w, nullptr, nullptr);
        ws[i] = w[0];
        //BOOST_TEST(w[0] == 0.0);
    }
    ws = ws.array().abs();
    BOOST_TEST(ws.maxCoeff() == 0.0);
}

void testZeroBorderGradient1d(Kernel& k){
    auto hs = getHs();
    for(auto h : hs){
        k.setH(h);
        Coordinates1d Xs(1);
        Xs[0] = h;
        Coordinates1d w;
        k.compute1d(Xs.array()*Xs.array(), nullptr, &w, nullptr);
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
        int i;
        auto extrema = Wgrad.rowwise().norm();
        extrema.maxCoeff(&i);
        BOOST_TEST(Wgrad.row(i).norm() == 0.0);
        BOOST_TEST(Wgrad(i, 0) == 0.0);
        BOOST_TEST(Wgrad(i, 1) == 0.0);
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
    Float h = 1.0;
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
        Float dx = h * 0.00000001;
        k.setH(h);
        Coordinates2d Xs = 0.99 * h * Xorig.array();
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
        auto diff = (gradExpected - WgradReceived).array().abs();
        int i, j;
        diff.maxCoeff(&i,&j);
        BOOST_TEST(gradExpected(i, j) == WgradReceived(i, j));
    }
}


void testGradientFiniteDifference1d(Kernel& k, int experiments){
    auto hs = getHs();
    Coordinates1d Xorig = increasingToOne(experiments);
    for(auto h : hs){
        Float dx = h * 0.000001;
        k.setH(h);
        Coordinates1d Xs = 0.99 * h * Xorig;
        Coordinates1d Xsx = Xs.array() + dx;
        Coordinates1d W0, Wx;
        Coordinates1d WgradReceived;
        k.compute1d(Xs.array()*Xs.array(), &W0, &WgradReceived, nullptr);
        k.compute1d(Xsx.array()*Xsx.array(), &Wx, nullptr, nullptr);
        Coordinates1d gradExpected = (Wx - W0)/dx;
        auto diff = (gradExpected - WgradReceived).array().abs();
        int i;
        diff.maxCoeff(&i);
        BOOST_TEST(gradExpected[i] == WgradReceived[i]);
    }
}

void testLaplacianFromGradientFiniteDifferences1d(Kernel& k, int experiments){
    const Coordinates1d Xorig = increasingToOne(experiments);
    auto hs = getHs();
    for(auto h : hs){
        Float dx = h * 0.000001;
        k.setH(h);
        Coordinates1d Xs = 0.99*h*Xorig.array();
        Coordinates1d Xsx = Xs.array() + dx;

        Coordinates1d G0, Gx;
        Coordinates1d Wlap;
        k.compute1d(Xs.array()*Xs.array(), nullptr, &G0, &Wlap);
        k.compute1d(Xsx.array()*Xsx.array(), nullptr, &Gx, nullptr);
        Coordinates1d expectedLaplacian = (Gx - G0)/dx;
        
        auto diff = (expectedLaplacian - Wlap).array().abs();
        int i;
        diff.maxCoeff(&i);
        BOOST_TEST(expectedLaplacian[i] == Wlap[i]);
    }
}

void testLaplacianFromGradientFiniteDifferences(Kernel& k, int experiments){
    const Coordinates2d Xorig = randomUnitDisk(experiments);
    auto hs = getHs();
    for(auto h : hs){
        Float dx = h * 0.000001;
        k.setH(h);
        Coordinates2d Xs = 0.99*h*Xorig;
        Coordinates2d Xsx, Xsy;
        Xsx = Xs; Xsx.col(0).array() += dx;
        Xsy = Xs; Xsy.col(1).array() += dx;

        Coordinates2d G0, Gx, Gy;
        Coordinates1d Wlap;
        k.compute(Xs, nullptr, &G0, &Wlap);
        k.compute(Xsx, nullptr, &Gx, nullptr);
        k.compute(Xsy, nullptr, &Gy, nullptr);
        Coordinates1d expectedLaplacian = (Gx - G0).col(0)/dx + (Gy - G0).col(1)/dx;
        
        auto diff = (expectedLaplacian - Wlap).array().abs();
        int i;
        diff.maxCoeff(&i);
        BOOST_TEST(expectedLaplacian[i] == Wlap[i]);
    }
}

void testLaplacianFiniteDifference1d(Kernel& k, int experiments){
    const Coordinates1d Xorig = increasingToOne(experiments);
    auto hs = getHs();
    for(auto h : hs){
        Float dx = h * 0.0001;
        k.setH(h);
        Coordinates1d Xs = 0.99 * h * Xorig.array() + dx;
        Coordinates1d Xsx1 = Xs.array() - dx;
        Coordinates1d Xsx2 = Xs.array() + dx;

        Coordinates1d W0, Wx1, Wx2;
        Coordinates1d Wlap;
        k.compute1d(Xs.array()*Xs.array(), &W0, nullptr, &Wlap);
        k.compute1d(Xsx1.array()*Xsx1.array(), &Wx1, nullptr, nullptr);
        k.compute1d(Xsx2.array()*Xsx2.array(), &Wx2, nullptr, nullptr);
        Coordinates1d expectedLaplacian = (Wx1 + Wx2 - 2*W0) / (dx*dx);
        
        auto diff = (expectedLaplacian - Wlap).array().abs();
        int i;
        diff.maxCoeff(&i);
        BOOST_TEST(expectedLaplacian[i] == Wlap[i]);
    }
}


void testLaplacianFiniteDifference(Kernel& k, int experiments){
    const Coordinates2d Xorig = randomUnitDisk(experiments);
    auto hs = getHs();
    for(auto h : hs){
        Float dx = h * 0.00001;
        k.setH(h);
        Coordinates2d Xs = 0.99 * h * Xorig.array() + dx;
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
        k.compute(Xsy2, &Wy2, nullptr, nullptr);
        Coordinates1d expectedLaplacian = (Wx1 + Wx2 + Wy1 + Wy2 - 4*W0) / (dx*dx);
        
        auto diff = (expectedLaplacian - Wlap).array().abs();
        int i;
        diff.maxCoeff(&i);
        BOOST_TEST(expectedLaplacian[i] == Wlap[i]);
    }
}


void testNormalization1d(Kernel& k, int experiments){
    auto hs = getHs();
    Coordinates1d Xorig = increasingToOne(experiments);
    // we integrate over [-h, h] with `experiments` steps of size (2*h/experiments)
    for(auto h : hs){
        Coordinates1d Xs = h*(Xorig.array()*2 - 1);
        Float dA = h*2/experiments; // area of one test segment
        Coordinates1d W;
        k.setH(h);
        k.compute1d(Xs.array()*Xs.array(), &W, nullptr, nullptr);
        Float totalWeight = W.sum()*dA;
        BOOST_TEST(totalWeight == 1.0);
    }
}

void testNormalization(Kernel& k, int experiments){
    auto hs = getHs();
    int n = std::max(100.0, std::sqrt(experiments));
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
        Float dA;
        dA = 2*h/n; // area of one test square
        dA = dA*dA;
        Coordinates1d W;
        k.setH(h);
        k.compute(Xs, &W, nullptr, nullptr);
        Float totalWeight = W.sum()*dA;
        BOOST_TEST(totalWeight == 1.0);
    }
}

} // Physics
} // d2
} // GooBalls
