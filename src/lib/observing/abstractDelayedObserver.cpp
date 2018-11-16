#include "abstractDelayedObserver.hpp"

namespace GooBalls {

namespace Observing {

void AbstractDelayedObserver::observe(std::shared_ptr<Message> msg) {
    m_msgs.push_back(msg);
}

void AbstractDelayedObserver::forgetMessages() {
    m_msgs.clear();
}

} // Observer

} // GooBalls

