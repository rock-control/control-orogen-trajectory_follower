/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>
#include <random>

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

std::string Task::printState(const TaskBase::States& state)
{
    switch(state)
    {
        case FINISHED_TRAJECTORIES:
            return "FINISHED_TRAJECTORIES";
        case FOLLOWING_TRAJECTORY:
            return "FOLLOWING_TRAJECTORY";
        case SLAM_POSE_INVALID:
            return "SLAM_POSE_INVALID";
        case LATERAL:
            return "LATERAL";
        case TURN_ON_SPOT:
            return "TURN_ON_SPOT";
        case STABILITY_FAILED:
            return "STABILITY_FAILED";
        default:
            return "UNKNOWN_STATE";
    }
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
    trajectoryFollower = TrajectoryFollower( _follower_config.value() );

    current_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    motionCommand.translation = 0;
    motionCommand.rotation    = 0;
    motionCommand.heading     = 0;

    if( _robot_pose.readNewest( rbpose ) == RTT::NoData)
    {
        if( !trajectories.empty() )
        {
            LOG_WARN_S << "Clearing old trajectories, since there is no "
                       "trajectory or pose data.";
        }

        trajectoryFollower.removeTrajectory();
        _motion_command.write(motionCommand.toBaseMotion2D());

        return;
    }

    base::Pose robotPose = base::Pose( rbpose.position, rbpose.orientation );

    if (_trajectory.readNewest(trajectories, false) == RTT::NewData && !trajectories.empty()) {
        trajectoryFollower.setNewTrajectory(trajectories.front(), robotPose);
        trajectories.erase(trajectories.begin());
        //emit following once, to let the outside know we got the trajectory
        state(FOLLOWING_TRAJECTORY);
    }
    
    SubTrajectory subTrajectory;
    if (_holonomic_trajectory.readNewest(subTrajectory, false) == RTT::NewData) {
        trajectoryFollower.setNewTrajectory(subTrajectory, robotPose);
        //emit following once, to let the outside know we got the trajectory
        state(FOLLOWING_TRAJECTORY);
    }

    FollowerStatus status = trajectoryFollower.traverseTrajectory(motionCommand, robotPose);

    switch(status)
    {
    case TRAJECTORY_FINISHED:
        if(!trajectories.empty())
        {
            trajectoryFollower.setNewTrajectory(trajectories.front(), robotPose);
            trajectories.erase(trajectories.begin());
        }
        else
            new_state = FINISHED_TRAJECTORIES;
        break;
    case TRAJECTORY_FOLLOWING:
        new_state = FOLLOWING_TRAJECTORY;
        break;
    case SLAM_POSE_CHECK_FAILED:
        new_state = SLAM_POSE_INVALID;
        break;
    case EXEC_TURN_ON_SPOT:
        new_state = TURN_ON_SPOT;
        break;
    case EXEC_LATERAL:
        new_state = LATERAL;
        break;
    case INITIAL_STABILITY_FAILED:
        if(current_state != new_state)
        {
            LOG_ERROR_S << "update TrajectoryFollowerTask state to STABILITY_FAILED.";
        }
        new_state = STABILITY_FAILED;
        break;
    default:
        std::runtime_error("Unknown TrajectoryFollower state");
    }
    
    _follower_data.write(trajectoryFollower.getData());
    _motion_command.write(motionCommand.toBaseMotion2D());

    // update task state
    if(current_state != new_state)
    {
        LOG_INFO_S << "update TrajectoryFollowerTask state to " << printState(new_state);
        current_state = new_state;
        state(new_state);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    motionCommand.translation = 0;
    motionCommand.rotation    = 0;
    motionCommand.heading     = 0;
    _motion_command.write(motionCommand.toBaseMotion2D());

    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}