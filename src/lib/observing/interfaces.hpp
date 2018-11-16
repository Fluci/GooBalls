#pragma once

#include <memory>

namespace GooBalls {

namespace Observing {

class Subject;
class Message;
class Observer;

/// Highest level interface for messages
class Message {
public:
    virtual ~Message() = default;
    /// An observer visits a message which in turn shuld call the observer with the correct notify method
    virtual void visit(Observer& observer) = 0;
};

class Observer {
public:
    virtual ~Observer() = default;

    /// Observer receives a message object
    virtual void observe(std::shared_ptr<Message> msg) = 0;

};

class DelayedObserver : public Observer {
public:
    /// Processes all stored messages
    virtual void consumeMessages() = 0;
    
    /// Removes all messages and pretends nothing ever happened
    virtual void forgetMessages() = 0;
};

class Subject {
public:
    virtual ~Subject() = default;

    /// Adds `obs` to the list of observers that will get a message
    virtual void addObserver(Observer* obs) = 0;
    
    /// Removes `obs` from the list of observers
    virtual void removeObserver(Observer* obs) = 0;
};

}

} // GooBalls
