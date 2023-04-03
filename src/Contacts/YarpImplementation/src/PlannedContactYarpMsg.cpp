/**
 * @file PlannedContactYarpMsg.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/PlannedContactYarpMsg.h>

#include <chrono>

using namespace BipedalLocomotion::Contacts;

PlannedContactYarpMsg::PlannedContactYarpMsg(const PlannedContact& contact)
    : WirePortable()
    , PlannedContact(contact)
{
}

// Build from an existing planned contact
void PlannedContactYarpMsg::fromPlannedContact(const PlannedContact& contact)
{
    PlannedContact* tmp = static_cast<PlannedContact*>(this);
    if ((*tmp) != contact)
    {
        (*tmp) = contact;
    }
}

// Read structure on a Wire
bool PlannedContactYarpMsg::read(yarp::os::idl::WireReader& reader)
{
    bool ok = this->readPose(reader);
    ok = ok && this->readName(reader);
    ok = ok && this->readIndex(reader);
    ok = ok && this->readTime(reader, this->activationTime);
    ok = ok && this->readTime(reader, this->deactivationTime);
    ok = ok && !reader.isError();
    return ok;
}

bool PlannedContactYarpMsg::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(5))
    {
        return false;
    }
    return read(reader);
}

// Write structure on a Wire
bool PlannedContactYarpMsg::write(const yarp::os::idl::WireWriter& writer) const
{
    bool ok = this->writePose(writer);
    ok = ok && this->writeName(writer);
    ok = ok && this->writeIndex(writer);
    ok = ok && this->writeTime(writer, this->activationTime);
    ok = ok && this->writeTime(writer, this->deactivationTime);
    ok = ok && !writer.isError();

    return ok;
}

// Write structure on a Connection
bool PlannedContactYarpMsg::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(5))
    {
        return false;
    }
    return write(writer);
}

std::string PlannedContactYarpMsg::toString() const
{
    yarp::os::Bottle b;
    if (!yarp::os::Portable::copyPortable(*this, b))
    {
        return {};
    }
    return b.toString();
}

bool PlannedContactYarpMsg::readPose(yarp::os::idl::WireReader& reader)
{
    // the pose is stored as position plus quaternion
    constexpr std::size_t poseDim = 7;

    if (reader.noMore())
    {
        reader.fail();
        return false;
    }
    size_t _csize;
    yarp::os::idl::WireState _etype;
    reader.readListBegin(_etype, _csize);
    // WireReader removes BOTTLE_TAG_LIST from the tag
    constexpr int expected_tag = ((BOTTLE_TAG_FLOAT64) & (~BOTTLE_TAG_LIST));
    if constexpr (expected_tag != 0)
    {
        if (_csize != 0 && _etype.code != expected_tag)
        {
            return false;
        }
    }

    // the pose is stored as position plus quaternion
    if (_csize != poseDim)
    {
        return false;
    }

    if (_csize != 0
        && !reader.readBlock(reinterpret_cast<char*>(this->pose.coeffs().data()),
                             poseDim * sizeof(double)))
    {
        return false;
    }
    reader.readListEnd();
    return true;
}

// write pose field
bool PlannedContactYarpMsg::writePose(const yarp::os::idl::WireWriter& writer) const
{
    constexpr std::size_t poseDim = 7;

    if (!writer.writeListBegin(BOTTLE_TAG_FLOAT64, poseDim))
    {
        return false;
    }
    if (!writer.writeBlock(reinterpret_cast<const char*>(this->pose.coeffs().data()),
                           poseDim * sizeof(double)))
    {
        return false;
    }

    if (!writer.writeListEnd())
    {
        return false;
    }
    return true;
}

bool PlannedContactYarpMsg::readName(yarp::os::idl::WireReader& reader)
{
    if (reader.noMore())
    {
        reader.fail();
        return false;
    }
    if (!reader.readString(this->name))
    {
        reader.fail();
        return false;
    }
    return true;
}

bool PlannedContactYarpMsg::writeName(const yarp::os::idl::WireWriter& writer) const
{
    return writer.writeString(this->name);
}

// read index field
bool PlannedContactYarpMsg::readIndex(yarp::os::idl::WireReader& reader)
{
    if (reader.noMore())
    {
        reader.fail();
        return false;
    }
    if (!reader.readI32(this->index))
    {
        reader.fail();
        return false;
    }
    return true;
}

bool PlannedContactYarpMsg::writeIndex(const yarp::os::idl::WireWriter& writer) const
{
    return writer.writeI32(this->index);
}

bool PlannedContactYarpMsg::readTime(yarp::os::idl::WireReader& reader,
                                     std::chrono::nanoseconds& time)
{
    std::int64_t temp;

    if (reader.noMore())
    {
        reader.fail();
        return false;
    }
    if (!reader.readI64(temp))
    {
        reader.fail();
        return false;
    }

    time = std::chrono::nanoseconds(temp);

    return true;
}

bool PlannedContactYarpMsg::writeTime(const yarp::os::idl::WireWriter& writer,
                                      const std::chrono::nanoseconds& time) const
{
    return writer.writeI64(time.count());
}
