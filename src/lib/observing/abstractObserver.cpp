#include "abstractObserver.hpp"


namespace GooBalls {

namespace Observing {


void AbstractObserver::observe(std::shared_ptr<Message> msg){
    msg->visit(*this);
}

} // Observer

} // GooBalls
