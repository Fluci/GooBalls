#include "abstractSubject.hpp"
#include "abstractObserver.hpp"

#include <boost/test/unit_test.hpp>

using namespace GooBalls;
using namespace Observing;

class SubjectTestMessage : public Message {
public:
    int seen;
    virtual void visit(Observer& obs) {
        // nothing
        seen++;
    }
};

class SubjectTestObserver: public AbstractObserver {
    // empty
};

class TestSubject: public AbstractSubject {
public:
    void sendAMessage(std::shared_ptr<Message> msg) {
        sendMessage(msg);
    }
};


BOOST_AUTO_TEST_CASE(send_some_messages) {
    TestSubject sender;
    std::shared_ptr<SubjectTestMessage> msg = std::make_shared<SubjectTestMessage>();
    SubjectTestObserver ob1, ob2;
    sender.addObserver(&ob1);
    sender.addObserver(&ob2);
    // test
    sender.sendAMessage(msg);
    BOOST_CHECK_EQUAL(2, msg->seen);
    // clean up
    sender.removeObserver(&ob1);
    sender.removeObserver(&ob2);
}

