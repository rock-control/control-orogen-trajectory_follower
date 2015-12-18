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
    a = _amp.value();
    f = _freq.value();
    l = _length.value();

    trajs.resize( 2 );
    std::vector< double > points;
    for( double t = 0; t <= l; t += l/100.0 )
    {
        points.push_back( a * sin( f * t ) );
        points.push_back( 0 );
        points.push_back( 0 );
    }

    trajs[0].spline.interpolate( points );
    trajs[0].speed = 0.2;
    points.clear();
    for( double t = l; t >= 0; t -= l/100.0 )
    {
        points.push_back( a * sin( f * t ) );
        points.push_back( 0 );
        points.push_back( 0 );
    }
    trajs[1].spline.interpolate( points );
    trajs[0].speed = -0.2;

    if (! TestSinTrajectoryTaskBase::configureHook())
        return false;
    return true;
}
bool TestSinTrajectoryTask::startHook()
{
    if (! TestSinTrajectoryTaskBase::startHook())
        return false;
    return true;

    _trajectory.write( trajs );
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
