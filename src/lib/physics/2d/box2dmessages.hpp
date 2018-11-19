#include "observer.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

/**
 * every Box2d-message inherits from this class
 * */
class Box2DMessage : public Message {
    // empty, only for casting
};

class Box2DBeginContactMessage : public Box2DMessage {
public:
    b2Contact* contact;
    virtual void visit(Observer& obs);
};

class Box2DEndContactMessage : public Box2DMessage {
public:
    b2Contact* contact;
    virtual void visit(Observer& obs);
};

class Box2DPreSolveMessage : public Box2DMessage {
public:
    b2Contact* contact;
    const b2Manifold* oldManifold;
    virtual void visit(Observer& obs);
};

class Box2DPostSolveMessage : public Box2DMessage {
public:
    b2Contact* contact;
    const b2ContactImpulse* impulse;
    virtual void visit(Observer& obs);
};

} // Physics

} // d2

} // GooBalls

