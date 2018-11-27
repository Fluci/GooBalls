#include <iostream>
#include "generic/types.hpp"
#include "spatial/2d/neighborhood_brute_force.hpp"
#include "spatial/2d/neighborhood_spatial_hashing.hpp"
#include <memory>

using namespace GooBalls;
using namespace Spatial;

std::vector<std::vector<std::vector<double>>> getTimings(
        const std::vector<int>& PNs, 
        const std::vector<double>& hs, 
        std::vector<std::unique_ptr<Neighborhood>>& algos, 
        double minRunDuration = 1.0){

    // that's where our result will be stored (Clock-Ticks / runs)
    std::vector<std::vector<std::vector<double> >> timings (PNs.size(), std::vector<std::vector<double>> (algos.size(), std::vector<double>(hs.size())));

    const clock_t min_clocks = minRunDuration * CLOCKS_PER_SEC;
    for(int pi = 0; pi < PNs.size(); ++pi){
        // create test points
        Coordinates2d points = Coordinates2d::Random(PNs[pi], 2);
        for(int hi = 0; hi < hs.size(); ++hi){
            double h = hs[hi];
            for(int ai = 0; ai < algos.size(); ++ai){
                auto& algo = *algos[ai];
                clock_t tick = clock();
                int runs = 0;
                std::cout << "PN=" << PNs[pi] << ", h=" << h << ", algo=" << ai << ": " << std::flush;
                do{
                    algo.inRange(points, h);
                    runs++;
                }while(clock() - tick < min_clocks);
                double perRun = double(clock() - tick)/CLOCKS_PER_SEC/runs;
                std::cout << perRun << " s" << std::endl;
                timings[pi][ai][hi] = perRun;
            }
        }
    }
    return timings;
}

int main() {
    // How many points do you want?
    std::vector<int> PNs;
    PNs.push_back(1000);
    PNs.push_back(2000);
    // How many seconds should an algorithm be run at least?
    const double minRunDuration = 0.1;

    // whish radius do you want to test?
    std::vector<double> hs;
    hs.push_back(0.01);
    hs.push_back(0.05);
    hs.push_back(0.1);
    hs.push_back(0.2);
    hs.push_back(0.5);

    // which algorithms should be tested?
    std::vector<std::unique_ptr<Neighborhood>> algos;
    algos.push_back(std::make_unique<NeighborhoodBruteForce>());
    algos.push_back(std::make_unique<NeighborhoodSpatialHashing>());
    
    getTimings(PNs, hs, algos, minRunDuration);
}
