/**
 * @file walkingCommands.thrift
 * @authors Ines Sorrentino <ines.sorrentino@iit.it>
 * @copyright 2024 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2024
 */

service JointTorqueControlCommands
{
    bool setKpJtcvc(1:string jointName, 2:double kp);

    double getKpJtcvc(1:string jointName);

    bool setKfcJtcvc(1:string jointName, 2:double kfc);

    double getKfcJtcvc(1:string jointName);

    bool setFrictionModel(1:string jointName, 2:string model);

    string getFrictionModel(1:string jointName);
}
