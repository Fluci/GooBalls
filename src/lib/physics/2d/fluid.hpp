#pragma once

#include "types.hpp"
#include <memory>

namespace GooBalls {
namespace d2 {
namespace Physics {

class Fluid {
public:
    typedef Coordinates2d Coordinates;
    typedef Eigen::Matrix<FloatPrecision, Eigen::Dynamic, 1> MassList; 
    // constructors
    Fluid(std::shared_ptr<Coordinates> ptr);
    // Field access
    const Coordinates& particles_position() const;
    Coordinates& particles_position();

    const Coordinates& particles_velocity() const;
    Coordinates& particles_velocity();
    
    const MassList& particles_mass() const;
    MassList& particles_mass();
    
private:
    std::shared_ptr<Coordinates> m_particles_position;
    Coordinates m_particles_velocity;
    MassList m_particles_mass;
};


} // Physics
} // d2
} // GooBalls
