#pragma once

#include "interfaces.hpp"

#include <set>

namespace GooBalls {

namespace Observing {

/**
 * This class is not really abstract, as a child class doesn't have to implement anything.
 * But it provides basic observer handling and message sending ready for reuse.
 * */
class AbstractSubject : public Subject {
private:
    std::set<Observer*> m_observers;
protected:
    /**
     * Sends the message to all observers
     * */
    virtual void sendMessage(std::shared_ptr<Message> msg);

public:
    virtual ~AbstractSubject() = default;
    virtual void addObserver(Observer* obs);

    virtual void removeObserver(Observer* obs);
};

}

} // GooBalls
