/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TurnVelocityToSteerAngleTask.hpp"

using namespace trajectory_follower;

TurnVelocityToSteerAngleTask::TurnVelocityToSteerAngleTask(std::string const& name)
    : TurnVelocityToSteerAngleTaskBase(name)
{
}

TurnVelocityToSteerAngleTask::TurnVelocityToSteerAngleTask(std::string const& name, RTT::ExecutionEngine* engine)
    : TurnVelocityToSteerAngleTaskBase(name, engine)
{
}

TurnVelocityToSteerAngleTask::~TurnVelocityToSteerAngleTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TurnVelocityToSteerAngleTask.hpp for more detailed
// documentation about them.

bool TurnVelocityToSteerAngleTask::configureHook()
{
    ackermanRatio = _ackerman_ratio.value();
    wheelBase = _wheel_base.value();
    if (! TurnVelocityToSteerAngleTaskBase::configureHook())
        return false;
    return true;
}
bool TurnVelocityToSteerAngleTask::startHook()
{
    if (! TurnVelocityToSteerAngleTaskBase::startHook())
        return false;
    return true;
}
void TurnVelocityToSteerAngleTask::updateHook()
{
    TurnVelocityToSteerAngleTaskBase::updateHook();
    base::commands::Motion2D mc;
    if( _motion_command_in.readNewest( mc ) == RTT::NewData )
    {
        mc.rotation = atan( ackermanRatio * wheelBase / mc.translation 
                * mc.rotation );

        _motion_command.write( mc );
    }
}
void TurnVelocityToSteerAngleTask::errorHook()
{
    TurnVelocityToSteerAngleTaskBase::errorHook();
}
void TurnVelocityToSteerAngleTask::stopHook()
{
    TurnVelocityToSteerAngleTaskBase::stopHook();
}
void TurnVelocityToSteerAngleTask::cleanupHook()
{
    TurnVelocityToSteerAngleTaskBase::cleanupHook();
}
