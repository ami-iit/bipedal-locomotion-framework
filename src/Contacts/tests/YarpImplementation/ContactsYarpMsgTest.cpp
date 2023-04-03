/**
 * @file ContactsYarpMsgTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <memory>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

#include <manif/SE3.h>

#include <catch2/catch.hpp>

#include <BipedalLocomotion/Contacts/ContactListMapYarpMsg.h>
#include <BipedalLocomotion/Contacts/ContactListYarpMsg.h>
#include <BipedalLocomotion/Contacts/PlannedContactYarpMsg.h>

using namespace BipedalLocomotion::Contacts;

template <typename Msg> class Sender : public yarp::os::Thread
{
public:
    Sender(std::shared_ptr<yarp::os::BufferedPort<Msg>> p, const typename Msg::MsgType& msg)
    {
        this->portOut = p;
        this->msg = msg;
    }

    bool threadInit() override
    {
        this->success = false;
        return true;
    }

    void run() override
    {
        int times = 10;

        while (times--)
        {
            Msg& temp = portOut->prepare();
            temp = this->msg;
            this->portOut->write();
        }

        this->success = true;
    }

    std::shared_ptr<yarp::os::BufferedPort<Msg>> portOut;
    bool success;
    typename Msg::MsgType msg;
};

template <typename Msg> class Receiver : public yarp::os::Thread
{
public:
    Receiver(std::shared_ptr<yarp::os::BufferedPort<Msg>> p,
             const typename Msg::MsgType& expectedMsg)
    {
        this->portIn = p;
        this->expectedMsg = expectedMsg;
    }

    bool threadInit() override
    {
        this->success = false;
        return true;
    }

    void run() override
    {
        int times = 10;
        Msg* temp = this->portIn->read(true);
        if (temp != nullptr)
        {
            this->success = (*temp == this->expectedMsg);
        }
    }

    std::shared_ptr<yarp::os::BufferedPort<Msg>> portIn;
    bool success;
    typename Msg::MsgType expectedMsg;
};

template <typename Msg> void test(const typename Msg::MsgType& msg)
{
    auto portIn = std::make_shared<yarp::os::BufferedPort<Msg>>();
    auto portOut = std::make_shared<yarp::os::BufferedPort<Msg>>();

    auto receiverThread = Receiver(portIn, msg);
    auto senderThread = Sender(portOut, msg);

    REQUIRE(portOut->open("/blf/test/contact:o"));
    REQUIRE(portIn->open("/blf/test/contact:i"));

    yarp::os::Network::connect("/blf/test/contact:o", "/blf/test/contact:i");

    receiverThread.start();
    senderThread.start();

    receiverThread.stop();
    senderThread.stop();

    portOut->close();
    portIn->close();

    REQUIRE(senderThread.success); // Send test
    REQUIRE(receiverThread.success); // Receive test
}

TEST_CASE("Planned Contact")
{
    using namespace std::chrono_literals;

    yarp::os::NetworkBase::setLocalMode(true);
    yarp::os::Network yarp;

    PlannedContact p1;
    p1.activationTime = 100ms;
    p1.deactivationTime = 500ms;
    p1.pose = manif::SE3d::Random();

    test<PlannedContactYarpMsg>(p1);

    yarp::os::NetworkBase::setLocalMode(false);
}

TEST_CASE("Contact list")
{
    using namespace std::chrono_literals;

    yarp::os::NetworkBase::setLocalMode(true);
    yarp::os::Network yarp;

    ContactList list;
    PlannedContact p1, p2;
    p1.activationTime = 100ms;
    p1.deactivationTime = 500ms;
    p1.pose = manif::SE3d::Random();

    p2.activationTime = 1s;
    p2.deactivationTime = 1s + 500ms;
    p2.pose = manif::SE3d::Random();

    REQUIRE(list.addContact(p2));
    REQUIRE(list.addContact(p1));

    test<ContactListYarpMsg>(list);

    yarp::os::NetworkBase::setLocalMode(false);
}

TEST_CASE("ContactList")
{
    using namespace std::chrono_literals;

    yarp::os::NetworkBase::setLocalMode(true);
    yarp::os::Network yarp;

    ContactList list;
    PlannedContact p1, p2;
    p1.activationTime = 100ms;
    p1.deactivationTime = 500ms;
    p1.pose = manif::SE3d::Random();

    p2.activationTime = 1s;
    p2.deactivationTime = 1s + 500ms;
    p2.pose = manif::SE3d::Random();

    REQUIRE(list.addContact(p2));
    REQUIRE(list.addContact(p1));

    test<ContactListYarpMsg>(list);

    yarp::os::NetworkBase::setLocalMode(false);
}

TEST_CASE("ContactListMap")
{
    using namespace std::chrono_literals;

    yarp::os::NetworkBase::setLocalMode(true);
    yarp::os::Network yarp;

    ContactListMap contactListMap;
    contactListMap["left"].setDefaultName("left_foot");
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Random(), 0s, 1s));
    REQUIRE(contactListMap["left"].addContact(manif::SE3d::Random(), 2s, 5s));

    contactListMap["right"].setDefaultName("right_foot");
    REQUIRE(contactListMap["right"].addContact(manif::SE3d::Random(), 0s, 3s));

    test<ContactListMapYarpMsg>(contactListMap);

    yarp::os::NetworkBase::setLocalMode(false);
}
