/**
 * @file ContactListMapYarpMsg.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactListMapYarpMsg.h>
#include <BipedalLocomotion/Contacts/ContactListYarpMsg.h>
#include <BipedalLocomotion/Contacts/PlannedContactYarpMsg.h>

#include <utility>

using namespace BipedalLocomotion::Contacts;

// Constructor with field values
ContactListMapYarpMsg::ContactListMapYarpMsg(const ContactListMap& contactListMap)
    : WirePortable()
    , ContactListMap(contactListMap)
{
}

void ContactListMapYarpMsg::fromContactListMap(const ContactListMap& map)
{
    ContactListMap* tmp = static_cast<ContactListMap*>(this);
    if ((*tmp) != map)
    {
        (*tmp) = map;
    }
}

// Read structure on a Wire
bool ContactListMapYarpMsg::read(yarp::os::idl::WireReader& reader)
{
    if (!this->readContactListMap(reader))
    {
        return false;
    }
    return !reader.isError();
}

// Read structure on a Connection
bool ContactListMapYarpMsg::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1))
    {
        return false;
    }
    return read(reader);
}

// Write structure on a Wire
bool ContactListMapYarpMsg::write(const yarp::os::idl::WireWriter& writer) const
{
    if (!this->writeContactListMap(writer))
    {
        return false;
    }
    return !writer.isError();
}

// Write structure on a Connection
bool ContactListMapYarpMsg::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1))
    {
        return false;
    }
    return write(writer);
}

// Convert to a printable string
std::string ContactListMapYarpMsg::toString() const
{
    yarp::os::Bottle b;
    if (!yarp::os::Portable::copyPortable(*this, b))
    {
        return {};
    }
    return b.toString();
}

// read vectors field
bool ContactListMapYarpMsg::readContactListMap(yarp::os::idl::WireReader& reader)
{
    if (reader.noMore())
    {
        reader.fail();
        return false;
    }

    size_t _csize;
    yarp::os::idl::WireState _ktype;
    yarp::os::idl::WireState _vtype;
    reader.readMapBegin(_ktype, _vtype, _csize);
    for (size_t _i = 0; _i < _csize; ++_i)
    {
        size_t _msize;
        yarp::os::idl::WireState _lst;
        reader.readListBegin(_lst, _msize);
        std::string _key;
        if (reader.noMore())
        {
            reader.fail();
            return false;
        }
        if (!reader.readString(_key))
        {
            reader.fail();
            return false;
        }
        ContactList& _val = this->operator[](_key);
        if (reader.noMore())
        {
            reader.fail();
            return false;
        }

        BipedalLocomotion::Contacts::ContactListYarpMsg tempContactList;
        if (!reader.readNested(tempContactList))
        {
            reader.fail();
            return false;
        }
        _val = tempContactList;

        reader.readListEnd();
    }
    reader.readMapEnd();
    return true;
}

// write vectors field
bool ContactListMapYarpMsg::writeContactListMap(const yarp::os::idl::WireWriter& writer) const
{
    if (!writer.writeMapBegin(BOTTLE_TAG_STRING,
                              BOTTLE_TAG_LIST | BOTTLE_TAG_LIST,
                              static_cast<const ContactListMap*>(this)->size()))
    {
        return false;
    }

    for (const auto& _item : *static_cast<const ContactListMap*>(this))
    {
        if (!writer.writeListBegin(0, 2))
        {
            return false;
        }
        if (!writer.writeString(_item.first))
        {
            return false;
        }

        if (!writer.writeNested(ContactListYarpMsg(_item.second)))
        {
            return false;
        }

        if (!writer.writeListEnd())
        {
            return false;
        }
    }

    if (!writer.writeMapEnd())
    {
        return false;
    }
    return true;
}
