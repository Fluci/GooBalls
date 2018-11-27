#pragma once

#include "types.hpp"
#include "scene.hpp"
#include "observing/interfaces.hpp"
#include "observing/abstractSubject.hpp"
#include "fluid_solver.hpp"

#include <memory>

namespace GooBalls {
namespace d2 {
namespace Physics {

class Engine : public Observing::AbstractSubject, public b2ContactListener {
    std::unique_ptr<FluidSolver> m_fluidSolver;
public:
    Engine();
    void initScene(Scene& scene);
    /**
    * Modifies the objects in `scene` s.t. they move as if `dt` time passed
    **/
    void advance(Scene& scene, TimeStep dt);
    /// Box2D's velocity iteration count
    int velocity_iterations = 6;
    /// Box2D's position iteration count
    int position_iterations = 2;

    void fluidSolver(std::unique_ptr<FluidSolver>&& solver);

    void BeginContact(b2Contact* contact);
    void EndContact(b2Contact* contact);
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
};

} // Physics
} // d2
} // GooBalls
