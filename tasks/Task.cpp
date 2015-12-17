/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>

using namespace trajectory_follower;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    trajectoryFollower = TrajectoryFollower( _controller_type.value(),
        _no_orientation_controller_config.value(),
        _chained_controller_config.value(),
        base::Pose( _pose_transform.value() ) );

    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    motionCommand.translation = 0;
    motionCommand.rotation    = 0;

    if( _robot_pose.readNewest( rbpose ) == RTT::NoData || 
            _trajectory.readNewest( trajectories ) == RTT::NoData )
    {
	if( !trajectories.empty() )
	{
	    LOG_WARN_S << "Clearing old trajectories, since there is no pose data.";
	}

        trajectories.clear();
        trajectoryFollower.removeTrajectory();
        _motion_command.write( motionCommand );

        return;
    }

    base::Pose robotPose = base::Pose( rbpose.position, rbpose.orientation );
    FollowerStatus status = 
            trajectoryFollower.traverseTrajectory( motionCommand, robotPose );
    
    switch( status )
    {
        case TRAJECTORY_FINISHED:
	    if( !trajectories.empty() )
	    {
                trajectoryFollower.setNewTrajectory( trajectories.front(), 
                      robotPose );
                trajectories.erase( trajectories.begin() );
	    } 
            else if( state() != FINISHED_TRAJECTORY )
            {
                state( FINISHED_TRAJECTORY );
	    }
	    break;

        case TRAJECTORY_FOLLOWING:
	    if( state() != FOLLOWING_TRAJECTORY )
            {
		state( FOLLOWING_TRAJECTORY );
            }
	    break;

        case INITIAL_STABILITY_FAILED:
	    if( state() != STABILITY_FAILED )
            {
		state( STABILITY_FAILED );
            }
	    break;

        default:
            std::runtime_error("Unknown TrajectoryFollower state");
    }
    
    _motion_command.write( motionCommand );
    _follower_data.write( trajectoryFollower.getData() );
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    motionCommand.translation = 0;
    motionCommand.rotation    = 0;
    _motion_command.write( motionCommand );

    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
