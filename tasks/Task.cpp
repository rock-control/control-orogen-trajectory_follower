/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
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

    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::commands::Motion2D oldMotionCommand;
    oldMotionCommand = motionCommand;

    motionCommand.translation = 0;
    motionCommand.rotation    = 0;

    if( _robot_pose.readNewest( rbpose ) == RTT::NoData)
    {
        if( !trajectories.empty() )
        {
            LOG_WARN_S << "Clearing old trajectories, since there is no "
                       "trajectory or pose data.";
        }

        trajectoryFollower.removeTrajectory();
        _motion_command.write( motionCommand );

        return;
    }

    base::Pose robotPose = base::Pose( rbpose.position, rbpose.orientation );

    if (_trajectory.readNewest( trajectories, false ) == RTT::NewData && !trajectories.empty()) {
        trajectoryFollower.setNewTrajectory( trajectories.front(), robotPose );
        trajectories.erase( trajectories.begin() );
        _trajectories.write(trajectories);
    }

    FollowerStatus status = trajectoryFollower.traverseTrajectory(
                                motionCommand, robotPose );

    switch( status )
    {
    case TRAJECTORY_FINISHED:
        if( !trajectories.empty() )
        {
            trajectoryFollower.setNewTrajectory( trajectories.front(),
                                                 robotPose );
            trajectories.erase( trajectories.begin() );
        }
        else if( state() != FINISHED_TRAJECTORIES )
        {
            LOG_INFO_S << "TrajectoryFollowerTask Finished Trajectories.";
            state( FINISHED_TRAJECTORIES );
        }
        break;

    case TRAJECTORY_FOLLOWING:
        if( state() != FOLLOWING_TRAJECTORY )
        {
            LOG_INFO_S << "TrajectoryFollowerTask Following Trajectory.";
            state( FOLLOWING_TRAJECTORY );
        }
        break;

    case INITIAL_STABILITY_FAILED:
        if( state() != STABILITY_FAILED )
        {
            LOG_ERROR_S << "TrajectoryFollowerTask Stability Failed.";
            state( STABILITY_FAILED );
        }
        break;

    default:
        std::runtime_error("Unknown TrajectoryFollower state");
    }

    // debug output
    _motion_command.write( motionCommand );
    _follower_data.write( trajectoryFollower.getData() );
    base::samples::RigidBodyState splineReferencePose;
    splineReferencePose.position = trajectoryFollower.getData().referencePose.position;
    splineReferencePose.orientation = trajectoryFollower.getData().referencePose.orientation;
    _spline_reference_pose.write( splineReferencePose );

    base::Pose currentPose;
    currentPose.fromTransform( robotPose.toTransform() * base::Pose( _follower_config.value().poseTransform ).toTransform() );
    base::samples::RigidBodyState poseTransformed;
    poseTransformed.position = currentPose.position;
    poseTransformed.orientation = currentPose.orientation;
    _transformed_pose.write(poseTransformed);

    base::samples::RigidBodyState movementDirection;
    movementDirection.position = currentPose.position;
    movementDirection.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(atan2(trajectoryFollower.getData().movementVector.y(), trajectoryFollower.getData().movementVector.x()), Eigen::Vector3d::UnitZ()));
    _movement_direction.write(movementDirection);

    base::samples::RigidBodyState mCommand;
    mCommand.position = currentPose.position;
    mCommand.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(motionCommand.rotation, Eigen::Vector3d::UnitZ()));
    _motion_command_viz.write(mCommand);

    base::samples::RigidBodyState mCommandDiff;
    mCommandDiff.position = currentPose.position;
    mCommandDiff.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(currentPose.getYaw() + (motionCommand.rotation - oldMotionCommand.rotation), Eigen::Vector3d::UnitZ()));
    _motion_command_diff_viz.write(mCommandDiff);
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
