/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TestSinTrajectoryTask.hpp"

using namespace trajectory_follower;

TestSinTrajectoryTask::TestSinTrajectoryTask(std::string const& name)
    : TestSinTrajectoryTaskBase(name)
{
}

TestSinTrajectoryTask::TestSinTrajectoryTask(std::string const& name, RTT::ExecutionEngine* engine)
    : TestSinTrajectoryTaskBase(name, engine)
{
}

TestSinTrajectoryTask::~TestSinTrajectoryTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TestSinTrajectoryTask.hpp for more detailed
// documentation about them.

bool TestSinTrajectoryTask::configureHook()
{
    if (! TestSinTrajectoryTaskBase::configureHook())
        return false;
    return true;
}
bool TestSinTrajectoryTask::startHook()
{
    if (! TestSinTrajectoryTaskBase::startHook())
        return false;
    return true;
}
void TestSinTrajectoryTask::updateHook()
{
    TestSinTrajectoryTaskBase::updateHook();
}
void TestSinTrajectoryTask::errorHook()
{
    TestSinTrajectoryTaskBase::errorHook();
}
void TestSinTrajectoryTask::stopHook()
{
    TestSinTrajectoryTaskBase::stopHook();
}
void TestSinTrajectoryTask::cleanupHook()
{
    TestSinTrajectoryTaskBase::cleanupHook();
}
