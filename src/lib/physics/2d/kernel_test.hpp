#pragma once

#include "kernel.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {
/**
 * Properties a kernel should have:
 * C^0, C^1 continuous
 * evaluated at r = h, the value should be zero
 * Kernel must be normalized: int_X W(|x - x_i|) dx = 1
 */

void testZeroBorder1d(Kernel& k);
void testZeroBorder(Kernel& k, int experiments);

void testZeroBorderGradient1d(Kernel& k);
void testZeroBorderGradient(Kernel& k, int experiments);

void testMonotonicity(Kernel& k, int experiments);

void testNonNegativity(Kernel& k, int experiments);

void testNormalization1d(Kernel& k, int experiments);
void testNormalization(Kernel& k, int experiments);

void testRadialSymmetry(Kernel& k, int experiments);

void testGradientFiniteDifference1d(Kernel& k, int experiments);
void testGradientFiniteDifference(Kernel& k, int experiments);

void testLaplacianFromGradientFiniteDifferences1d(Kernel& k, int experiments);
void testLaplacianFromGradientFiniteDifferences(Kernel& k, int experiments);

void testLaplacianFiniteDifference1d(Kernel& k, int experiments);
void testLaplacianFiniteDifference(Kernel& k, int experiments);

} // Physics
} // d2
} // GooBalls
