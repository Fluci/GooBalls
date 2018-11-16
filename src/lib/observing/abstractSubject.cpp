#include "abstractSubject.hpp"


namespace GooBalls {

namespace Observing {

void AbstractSubject::sendMessage(std::shared_ptr<Message> msg) {
    for(Observer* obs : m_observers){
        obs->observe(msg);
    }
}

void AbstractSubject::addObserver(Observer* obs){
    m_observers.insert(obs);
}

void AbstractSubject::removeObserver(Observer* obs) {
    m_observers.erase(obs);
}

}

} // GooBAlls
