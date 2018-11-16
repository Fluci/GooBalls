#pragma once

#include "interfaces.hpp"
#include <vector>

namespace GooBalls {

namespace Observing {

/***
 * Handles as much as possible of the message maintenance, such that a child class
 * only has to implement the actual message processing.
 * */
class AbstractDelayedObserver : public DelayedObserver {
private:
    std::vector<std::shared_ptr<Message> > m_msgs;
public:
    virtual void observe(std::shared_ptr<Message> msg);
    virtual void forgetMessages();
};

} // GooBalls

} // Observer
