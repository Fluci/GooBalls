#pragma once

#include "types.hpp"
#include "scene.hpp"
#include "observing/interfaces.hpp"
#include "observing/abstractSubject.hpp"

namespace GooBalls {
namespace d2 {
namespace Physics {

class Engine : public Observing::AbstractSubject, public b2ContactListener {
public:
    /**
    * Modifies the objects in `scene` s.t. they move as if `dt` time passed
    **/
    void advance(Scene& scene, TimeStep dt);
    /// Box2D's velocity iteration count
    int velocity_iterations = 6;
    /// Box2D's position iteration count
    int position_iterations = 2;

    void BeginContact(b2Contact* contact);
    void EndContact(b2Contact* contact);
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
};

} // Physics
} // d2
} // GooBalls
