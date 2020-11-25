/**
 * @file MasImuTestCommands.thrift
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

service MasImuTestCommands
{
    /**
     * Quits the module.
     */
    oneway void quit();

    /**
     * Call this method to start the test.
     * @return true/false in case of success/failure (for example if the preparation phase was not successfull);
     */
    bool startTest();

    /**
     * Stop the test
     */
    bool stopTest();

    /**
     * Print the results. This works only if the test has already been stopped.
     */
    bool printResults();
}
