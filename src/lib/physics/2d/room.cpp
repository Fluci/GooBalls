#include "room.hpp"
#include <numeric>

namespace GooBalls {
namespace d2 {
namespace Physics {


void Room::restrictFluid(Fluid& fluid) const {
    // floor: y = 0
    auto& vs = fluid.particles_velocity();
    auto& pos = fluid.particles_position();
    if(!std::isnan(floor)){
        for(int i = 0; i < vs.rows(); ++i){
            if(pos(i, 1) < floor){
                vs(i, 1) = std::abs(vs(i, 1));
            }
        }
    }
    if(!std::isnan(ceiling)){
        for(int i = 0; i < vs.rows(); ++i){
            if(pos(i, 1) > ceiling){
                vs(i, 1) = -std::abs(vs(i, 1));
            }
        }
    }
    if(!std::isnan(left_wall)){
        for(int i = 0; i < vs.rows(); ++i){
            if(pos(i, 0) < left_wall) {
                vs(i, 0) = std::abs(vs(i, 0));
            }
        }
    }
    if(!std::isnan(right_wall)){
        for(int i = 0; i < vs.rows(); ++i){
            if(pos(i, 0) > right_wall){
                vs(i, 0) = -std::abs(vs(i, 0));
            }
        }
    }
}

} // Physics
} // d2
} // GooBalls
