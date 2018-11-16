#include "box2dmessages.hpp"


namespace GooBalls {

namespace d2 {

namespace Physics {

void Box2DBeginContactMessage::visit(Observer& obs){
    obs.BeginContact(contact);
}

void Box2DEndContactMessage::visit(Observer& obs){
    obs.EndContact(contact);
}

void Box2DPreSolveMessage::visit(Observer& obs){
    obs.PreSolve(contact, oldManifold);
}

void Box2DPostSolveMessage::visit(Observer& obs){
    obs.PostSolve(contact, impulse);
}

} // Physics

} // d2

} // GooBalls
