#pragma once

#include "interfaces.hpp"


namespace GooBalls {

namespace Observing {

class AbstractObserver : public Observer {

void observe(std::shared_ptr<Message> msg);

};

} // Observer

} // GooBalls
