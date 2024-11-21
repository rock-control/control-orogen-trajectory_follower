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
    lastMotionCommand.translation = 0;
    lastMotionCommand.rotation    = 0;
    lastMotionCommand.heading     = 0;
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
        case NO_POSITION_UPDATES:
            return "NO_POSITION_UPDATES";
        default:
            return "UNKNOWN_STATE";
    }
}

bool Task::isMotionCommandZero(const Motion2D& mc)
{
    return ( mc.translation == 0 &&
             mc.rotation    == 0 &&
             mc.heading     == 0 );
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
    _robot2map.registerUpdateCallback([this](const base::Time& time){lastPoseUpdate = time;});
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    Motion2D motionCommand{};

    base::Time now;
    if(_simulated_time.connected())
    {
        if(_simulated_time.readNewest(now)==RTT::NoData)
        {
            LOG_ERROR_S << "Connected to simulated time, but got no time input. Aborting update.\n";
            return;
        }
    }
    else
    {
        now = base::Time::now();
    }

    Eigen::Affine3d robot2map;
    if(!_robot2map.get(lastPoseUpdate, robot2map, false))
    {
        LOG_ERROR_S << "Could not get robot pose!" << std::endl;
        trajectoryFollower.removeTrajectory();
        _motion_command.write(motionCommand.toBaseMotion2D());
        state(SLAM_POSE_INVALID);
        return;
    }

    base::Pose robotPose(robot2map);
    if (_trajectory.readNewest(trajectories, false) == RTT::NewData && !trajectories.empty()) {
        trajectoryFollower.setNewTrajectory(trajectories.front(), robotPose);
        _current_trajectory.write(trajectoryFollower.getData().currentTrajectory);
        trajectories.erase(trajectories.begin());
        //emit following once, to let the outside know we got the trajectory
        state(FOLLOWING_TRAJECTORY);
    }
    
    SubTrajectory subTrajectory;
    if (_holonomic_trajectory.readNewest(subTrajectory, false) == RTT::NewData) {
        trajectoryFollower.setNewTrajectory(subTrajectory, robotPose);
        _current_trajectory.write(trajectoryFollower.getData().currentTrajectory);
        //emit following once, to let the outside know we got the trajectory
        state(FOLLOWING_TRAJECTORY);
    }

    FollowerStatus status = trajectoryFollower.traverseTrajectory(motionCommand, robotPose);

    switch(status)
    {
    case TRAJECTORY_FINISHED:
        //check if next spline is just a point
        while(!trajectories.empty() && trajectories.front().posSpline.isSingleton())
        {
            LOG_ERROR_S << "Ignoring degenerate trajectory!" << std::endl;
            trajectories.erase(trajectories.begin());
        }
        
        // check if there is a next trajectory
        if(trajectories.empty())
        {
            new_state = FINISHED_TRAJECTORIES;
        }else
        {
            trajectoryFollower.setNewTrajectory(SubTrajectory(trajectories.front()), robotPose);
            _current_trajectory.write(trajectoryFollower.getData().currentTrajectory);
            trajectories.erase(trajectories.begin());
            new_state = FOLLOWING_TRAJECTORY;
        }
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

    base::Time const latency = now - lastPoseUpdate;
    if(latency > base::Time::fromSeconds(_transformer_max_latency.value())){
        LOG_ERROR_S << "No position updates since " << lastPoseUpdate << " (" << latency.toSeconds() << "s ago)\n";
        new_state = NO_POSITION_UPDATES;
        if(_send_zero_cmd_on_timeout)
        {
            motionCommand = Motion2D{};
        }
    }


    if ( not ( isMotionCommandZero(lastMotionCommand) &&
               isMotionCommandZero(motionCommand)     &&
               _send_zero_cmd_once.value() )
       )
    {
        lastMotionCommand = motionCommand;
        _motion_command.write(motionCommand.toBaseMotion2D());
    }

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
    Motion2D motionCommand{};
    _motion_command.write(motionCommand.toBaseMotion2D());

    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

bool Task::cancelCurrentTrajectory()
{
    LOG_INFO_S << "Current trajectory cancellation requested";
    // try to remove the trajectory and if no exception occurs, return true
    try {
        trajectoryFollower.removeTrajectory();
    } catch (std::runtime_error& e) {
        LOG_ERROR_S << "Could not remove trajectory: " << e.what();
        return false;
    }
    trajectories.clear();
    return true;
}
