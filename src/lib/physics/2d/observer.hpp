#pragma once

#include "observing/interfaces.hpp"
#include "observing/abstractObserver.hpp"
#include <Box2D/Box2D.h>

namespace GooBalls {

namespace d2 {

namespace Physics {

class Observer;
class Message;

class Message : public Observing::Message {
public:
    /// this has a default implementation of casting the Observer to a Physics::Observer and then calls `visit(Physics::Observer&)`
    virtual void visit(Observing::Observer& obs);

    /// Implement this when inheriting from this class
    virtual void visit(Observer& obs) = 0;
};

class Observer : public Observing::AbstractObserver, public b2ContactListener {
public:
    virtual void BeginContact(b2Contact* contact) = 0;
    virtual void EndContact(b2Contact* contact) = 0;
    virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) = 0;
    virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) = 0;

};

} // Physics

} // d2

} // GooBalls
