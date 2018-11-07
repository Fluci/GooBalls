#pragma once

#include "observer.hpp"

namespace GooBalls {

namespace d2 {

namespace Physics {

void Message::visit(Observer::Observer& obs) {
    Observer* physObs = std::dynamic_cast<Observer>(&obs);
    if(physObs == nullptr){
        // we don't care about this observer, it is not interested in physics messages
        return;
    }
    // physics message handling
    visit(*physObs);
}

} // Physics

} // d2

} // GooBalls
