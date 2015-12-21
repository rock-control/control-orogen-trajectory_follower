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
    h = _height.value();
    double aLast;

    trajs.resize( 2 );
    std::vector< double > pf;
    for( double t = 0; t <= l; t += l/100.0 )
    {
        pf.push_back( t );
        pf.push_back( a * cos( f * 2.0 * M_PI * t ) );
        pf.push_back( h );

        aLast = a * cos( f * 2.0 * M_PI * t ) - a;
    }
    trajs[0].spline.interpolate( pf );
    trajs[0].speed = 0.2;

    std::vector< double > pb;
    for( double t = l; t >= 0; t -= l/100.0 )
    {
        pb.push_back( t );
        pb.push_back( aLast - a * cos( f * 2.0 * M_PI * t ) );
        pb.push_back( h );
    }
    trajs[1].spline.interpolate( pb );
    trajs[1].speed = -0.2;

    written = false;

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
    if( !written ) 
    {
        _trajectory.write( trajs );
        written = true;
    }
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
