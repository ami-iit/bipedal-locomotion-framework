/**
 * @file ContactListYarpMsg.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactListYarpMsg.h>
#include <BipedalLocomotion/Contacts/PlannedContactYarpMsg.h>

#include <utility>

using namespace BipedalLocomotion::Contacts;

// Constructor with field values
ContactListYarpMsg::ContactListYarpMsg(const ContactList& contactList)
    : WirePortable()
    , ContactList(contactList)
{
}

void ContactListYarpMsg::fromContactList(const ContactList& list)
{
    ContactList* tmp = static_cast<ContactList*>(this);
    if ((*tmp) != list)
    {
        (*tmp) = list;
    }
}

// Read structure on a Wire
bool ContactListYarpMsg::read(yarp::os::idl::WireReader& reader)
{
    bool ok = this->readContactList(reader);
    ok = ok && this->readDefaultName(reader);
    ok = ok && this->readDefaultIndex(reader);
    ok = ok && !reader.isError();

    return ok;
}

// Read structure on a Connection
bool ContactListYarpMsg::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3))
    {
        return false;
    }
    return read(reader);
}

// Write structure on a Wire
bool ContactListYarpMsg::write(const yarp::os::idl::WireWriter& writer) const
{
    bool ok = this->writeContactList(writer);
    ok = ok && this->writeDefaultName(writer);
    ok = ok && this->writeDefaultIndex(writer);
    ok = ok && !writer.isError();
    return ok;
}

// Write structure on a Connection
bool ContactListYarpMsg::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3))
    {
        return false;
    }
    return write(writer);
}

// Convert to a printable string
std::string ContactListYarpMsg::toString() const
{
    yarp::os::Bottle b;
    if (!yarp::os::Portable::copyPortable(*this, b))
    {
        return {};
    }
    return b.toString();
}

// read vectors field
bool ContactListYarpMsg::readContactList(yarp::os::idl::WireReader& reader)
{
    if (reader.noMore())
    {
        reader.fail();
        return false;
    }

    size_t _csize;
    yarp::os::idl::WireState _etype;
    reader.readListBegin(_etype, _csize);

    // WireReader removes BOTTLE_TAG_LIST from the tag
    constexpr int expected_tag = ((BOTTLE_TAG_LIST) & (~BOTTLE_TAG_LIST));
    if constexpr (expected_tag != 0)
    {
        if (_csize != 0 && _etype.code != expected_tag)
        {
            return false;
        }
    }

    BipedalLocomotion::Contacts::PlannedContactYarpMsg tempContact;
    for (size_t _i = 0; _i < _csize; ++_i)
    {
        if (reader.noMore())
        {
            reader.fail();
            return false;
        }

        if (!reader.readNested(tempContact))
        {
            reader.fail();
            return false;
        }

        // TODO(Giulio) use MOVE if implemented
        static_cast<ContactList*>(this)->addContact(tempContact);
    }

    reader.readListEnd();
    return true;
}

// write vectors field
bool ContactListYarpMsg::writeContactList(const yarp::os::idl::WireWriter& writer) const
{
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<const ContactList*>(this)->size()))
    {
        return false;
    }

    for (const auto& _item : *static_cast<const ContactList*>(this))
    {
        if (!writer.writeNested(PlannedContactYarpMsg(_item)))
        {
            return false;
        }
    }

    if (!writer.writeListEnd())
    {
        return false;
    }

    return true;
}

// read/write name field
bool ContactListYarpMsg::readDefaultName(yarp::os::idl::WireReader& reader)
{
    std::string temp;
    if (reader.noMore())
    {
        reader.fail();
        return false;
    }
    if (!reader.readString(temp))
    {
        reader.fail();
        return false;
    }

    this->setDefaultName(temp);

    return true;
}

bool ContactListYarpMsg::writeDefaultName(const yarp::os::idl::WireWriter& writer) const
{
    return writer.writeString(this->defaultName());
}

// read/write name field
bool ContactListYarpMsg::readDefaultIndex(yarp::os::idl::WireReader& reader)
{
    std::int32_t temp;
    if (reader.noMore())
    {
        reader.fail();
        return false;
    }
    if (!reader.readI32(temp))
    {
        reader.fail();
        return false;
    }

    this->setDefaultIndex(temp);

    return true;
}

bool ContactListYarpMsg::writeDefaultIndex(const yarp::os::idl::WireWriter& writer) const
{
    return writer.writeI32(this->defaultIndex());
}
