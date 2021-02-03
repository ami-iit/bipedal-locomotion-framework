/**
 * @file Commands.thrift
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


service CartesianTrajectoryPlayerCommands
{
    /**
     * Call this method to home the robot.
     * @return true/false in case of success/failure;
     */
    bool homing();

    bool start();
}
