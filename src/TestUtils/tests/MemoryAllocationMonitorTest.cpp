/**
 * @file MemoryAllocationMonitorTest.cpp
 * @authors Silvio Traversaro
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/TestUtils/MemoryAllocationMonitor.h>

using namespace BipedalLocomotion::TestUtils;

#include <vector>
#include <Eigen/Dense>

TEST_CASE("MemoryAllocationMonitor test")
{
    SECTION("Check that memory monitor is enabled")
    {
        // If the memory monitor is not enabled, this test should not be compiled
        REQUIRE(MemoryAllocationMonitor::monitorIsEnabled());
    }

    SECTION("Check with no memory allocation")
    {
        MemoryAllocationMonitor::startMonitor();
        {
            double c = 1.2 + 3.5;
        }
        REQUIRE(MemoryAllocationMonitor::endMonitorAndCheckNoMemoryAllocationInLastMonitor());
    }

    SECTION("Check with simple malloc")
    {
        MemoryAllocationMonitor::startMonitor();
        {
            double* p = static_cast<double*>(malloc(100*sizeof(double)));
            REQUIRE(p);
            free(static_cast<void*>(p));
        }
        MemoryAllocationMonitor::endMonitor();
        REQUIRE(!MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor());
        REQUIRE(MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor() == 2);
    }

    SECTION("Check with simple new")
    {
        MemoryAllocationMonitor::startMonitor();
        {
            double* p = new double;
            REQUIRE(p);
            delete p;
        }
        MemoryAllocationMonitor::endMonitor();
        REQUIRE(!MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor());
        REQUIRE(MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor() == 2);
    }

    SECTION("Check with array new")
    {
        MemoryAllocationMonitor::startMonitor();
        {
            double* p = new double[100];
            REQUIRE(p);
            delete [] p;
        }
        MemoryAllocationMonitor::endMonitor();
        REQUIRE(!MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor());
        REQUIRE(MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor() == 2);
    }

    SECTION("Check with std::vector constructor")
    {
        MemoryAllocationMonitor::startMonitor();
        {
            std::vector<double> vec;
            vec.resize(100);
        }
        MemoryAllocationMonitor::endMonitor();
        REQUIRE(!MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor());
        REQUIRE(MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor() == 2);
    }

    SECTION("Check with Eigen::MatrixXd constructor")
    {
        MemoryAllocationMonitor::startMonitor();
        {
            Eigen::MatrixXd mat;
            mat.resize(100, 100);
        }
        MemoryAllocationMonitor::endMonitor();
        REQUIRE(!MemoryAllocationMonitor::checkNoMemoryAllocationInLastMonitor());
        REQUIRE(MemoryAllocationMonitor::getNumberOfDynamicMemoryOperationsInLastMonitor() == 2);
    }

}
