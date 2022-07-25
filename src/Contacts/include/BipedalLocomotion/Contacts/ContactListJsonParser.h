/**
 * @file ContactListJsonParser.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTACTS_CONTACTLIST_JSON_PARSER_H
#define BIPEDAL_LOCOMOTION_CONTACTS_CONTACTLIST_JSON_PARSER_H

#include <string>

// BipedalLocomotion
#include <BipedalLocomotion/Contacts/ContactList.h>

namespace BipedalLocomotion
{
namespace Contacts
{

/**
 * Parse a ContactListMap from a JSON file.
 * @param filename the name of the file that should be loaded.
 * @return return a ContactListMap containing all the contacts.
 * @note The JSON file should have a structure similar to the following example.
 * ```json
 * {
 *  "my_left_foot": [
 *    {
 *      "activation_time": 0.0,
 *      "deactivation_time": 1.0,
 *      "index": 42,
 *      "name": "left_foot",
 *      "position": [
 *        0.5968800669521466,
 *        0.8232947158735686,
 *        -0.6048972614132321
 *      ],
 *      "quaternion": [
 *        -0.3149241877569222,
 *        -0.8968666065772184,
 *        0.18925645870451685,
 *        0.24624182993742819
 *      ]
 *    },
 *    {
 *      "activation_time": 2.0,
 *      "deactivation_time": 5.0,
 *      "index": 42,
 *      "name": "left_foot",
 *      "position": [
 *        0.10793991159086103,
 *        -0.0452058962756795,
 *        0.2577418495238488
 *      ],
 *      "quaternion": [
 *        0.09318476783085411,
 *        0.5701901104695768,
 *        -0.10052857125007243,
 *        -0.8099961995771466
 *      ]
 *    }
 *  ],
 *  "my_right_foot": [
 *    {
 *      "activation_time": 0.0,
 *      "deactivation_time": 3.0,
 *      "index": 33,
 *      "name": "right_foot",
 *      "position": [
 *        0.8323901360074013,
 *        0.2714234559198019,
 *        0.43459385886536617
 *      ],
 *      "quaternion": [
 *        -0.794180500610814,
 *        -0.17857274481750596,
 *        0.5769716176729885,
 *        -0.06702879722711332
 *      ]
 *    }
 *  ]
 *}
 * ```
 * @note The structure of each step replicates the attributes of PlannedContact
 */
ContactListMap contactListMapFromJson(const std::string& filename);

/**
 * Store a ContactListMap to a JSON file.
 * @param map a ContactListMap containing the list of the contacts.
 * @param filename the name of the file that should be loaded.
 * @return return true in case of success, failure otherwise.
 * @note The function generates a JSON file readable by contactListMapFromJson()
 */
bool contactListMapToJson(const ContactListMap& map, const std::string& filename);

} // namespace Contacts
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTACTS_CONTACTLIST_JSON_PARSER_H
