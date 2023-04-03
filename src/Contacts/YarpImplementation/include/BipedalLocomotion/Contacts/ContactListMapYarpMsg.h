/**
 * @file ContactListMapYarpMsg.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_LIST_MAP_YARP_MSG_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_LIST_MAP_YARP_MSG_H

#include <BipedalLocomotion/Contacts/ContactList.h>

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace BipedalLocomotion::Contacts
{

class ContactListMapYarpMsg : public yarp::os::idl::WirePortable,
                              public BipedalLocomotion::Contacts::ContactListMap
{
public:

    using MsgType = BipedalLocomotion::Contacts::ContactListMap;

    // Default constructor
    ContactListMapYarpMsg() = default;

    // Constructor with field values
    ContactListMapYarpMsg(const ContactListMap& map);

    // Build from an existing planned contact
    void fromContactListMap(const ContactListMap& map);

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
    // typedef yarp::os::idl::Unwrapped<VectorsCollection> unwrapped;

private:
    // read/write vectors field
    bool readContactListMap(yarp::os::idl::WireReader& reader);
    bool writeContactListMap(const yarp::os::idl::WireWriter& writer) const;
    // bool nested_read_vectors(yarp::os::idl::WireReader& reader);
    // bool nested_write_vectors(const yarp::os::idl::WireWriter& writer) const;
};

} // namespace BipedalLocomotion::Contacts

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACT_LIST_MAP_YARP_MSG_H
