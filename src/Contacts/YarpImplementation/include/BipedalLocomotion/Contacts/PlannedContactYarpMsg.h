/**
 * @file PlannedContactYarpMsg.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_PLANNED_CONTACT_YARP_MSG_H
#define BIPEDAL_LOCOMOTION_CONTACTS_PLANNED_CONTACT_YARP_MSG_H

#include <BipedalLocomotion/Contacts/Contact.h>

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace BipedalLocomotion::Contacts
{

class PlannedContactYarpMsg : public yarp::os::idl::WirePortable,
                              public BipedalLocomotion::Contacts::PlannedContact
{
public:

    using MsgType = BipedalLocomotion::Contacts::PlannedContact;

    // Default constructor
    PlannedContactYarpMsg() = default;

    // Constructor with field values
    PlannedContactYarpMsg(const PlannedContact& contact);

    // Build from an existing planned contact
    void fromPlannedContact(const PlannedContact& contact);

    // Read structure on a Wire
    bool read(yarp::os::idl::WireReader& reader) override;

    // Read structure on a Connection
    bool read(yarp::os::ConnectionReader& connection) override;

    // Write structure on a Wire
    bool write(const yarp::os::idl::WireWriter& writer) const override;

    // Write structure on a Connection
    bool write(yarp::os::ConnectionWriter& connection) const override;

    // Convert to a printable string
    std::string toString() const;

    // If you want to serialize this class without nesting, use this helper
    typedef yarp::os::idl::Unwrapped<PlannedContact> unwrapped;

private:
    // read/write pose field
    bool readPose(yarp::os::idl::WireReader& reader);
    bool writePose(const yarp::os::idl::WireWriter& writer) const;
    // bool nested_read_pose(yarp::os::idl::WireReader& reader);
    // bool nested_write_pose(const yarp::os::idl::WireWriter& writer) const;

    // read/write name field
    bool readName(yarp::os::idl::WireReader& reader);
    bool writeName(const yarp::os::idl::WireWriter& writer) const;
    // bool nested_read_name(yarp::os::idl::WireReader& reader);
    // bool nested_write_name(const yarp::os::idl::WireWriter& writer) const;

    // read/write index field
    bool readIndex(yarp::os::idl::WireReader& reader);
    bool writeIndex(const yarp::os::idl::WireWriter& writer) const;
    // bool nested_read_index(yarp::os::idl::WireReader& reader);
    // bool nested_write_index(const yarp::os::idl::WireWriter& writer) const;

    // read/write activationTime field
    bool readTime(yarp::os::idl::WireReader& reader, std::chrono::nanoseconds& time);
    bool writeTime(const yarp::os::idl::WireWriter& writer, const std::chrono::nanoseconds& time) const;
    // bool nested_read_activationTime(yarp::os::idl::WireReader& reader);
    // bool nested_write_activationTime(const yarp::os::idl::WireWriter& writer) const;
};

} // namespace BipedalLocomotion::Contacts

#endif // BIPEDAL_LOCOMOTION_CONTACTS_PLANNED_CONTACT_YARP_MSG_H
