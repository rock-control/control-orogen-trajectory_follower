#include "Task.hpp"

#include <iostream>
#include <math.h>

#define SAMPLING_TIME 0.01
#define SEARCH_DIST   0.5  // Distance that will be searched along the curve for the closest point

using namespace trajectory_follower;
using namespace base::geometry;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name), trFollower(NULL)
{
    /*
    _controllerType.set(0);
    _forwardVelocity.set(0.2);
    _forwardLength.set(0.1);
    _gpsCenterofRotationOffset.set(0.4);

    _K0_nO.set(5.0);
    _K2_P.set(150.0);
    _K3_P.set(150.0);
    _K0_PI.set(0.0);
    _K2_PI.set(150.0);
    _K3_PI.set(150.0);
    */
}

Task::~Task() {}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

//bool Task::configureHook()
//{
//  
// return true;
//}

bool Task::startHook()
{
    if(trFollower)
	delete trFollower;
    
    trFollower = new TrajectoryFollower(_forwardLength.get(), _gpsCenterofRotationOffset.get(), _controllerType.get());
    
    trFollower->getNoOrientationController().setConstants( _forwardLength.get(), _K0_nO.get() );
    trFollower->getNoOrientationController().setPointTurnSpeed( _pointTurnSpeed.get() );
    trFollower->getPController().setConstants( _K2_P.get(), _K3_P.get() );
    trFollower->getPIController().setConstants( _K0_PI.get(), _K2_PI.get(), _K3_PI.get(), SAMPLING_TIME);
    trFollower->setAddPoseErrorY(_addPoseErrorY.get()); 
    trFollower->setNoOrientationPointTurnUpperLimit(_noOrientationPointTurnUpperLimit.get());
    trFollower->setNoOrientationPointTurnLowerLimit(_noOrientationPointTurnLowerLimit.get());   
    trFollower->setNoOrientationRotationalVelocity(_noOrientationRotationalVelocity.get());

    driveSpeed = _forwardVelocity.get();
    
    return true;
}

void overwriteTrajectorySpeed(base::Trajectory &tr, double speed)
{
    if(speed<=0)
	return;
    
    if(tr.speed < 0)
        tr.speed = -speed;
    else
        tr.speed = speed;
}

void Task::updateHook()
{
    base::MotionCommand2D mc;
    mc.translation = 0;
    mc.rotation    = 0;

    driveSpeed = _forwardVelocity.get();

    base::samples::RigidBodyState rbpose;
    if(_pose.readNewest(rbpose) == RTT::NoData)
    {
        trajectories.clear();
        trFollower->removeTrajectory();
        _motion_command.write(mc);
        return;
    }

    RTT::FlowStatus trajectory_status = _trajectory.readNewest(trajectories, false);
    if (trajectory_status == RTT::NoData)
    {
        trajectories.clear();
        trFollower->removeTrajectory();
        _motion_command.write(mc);
        return;
    }
    else if (trajectory_status == RTT::NewData)
        {
        if(!trajectories.empty())
        {
            base::Trajectory curTr(trajectories.front());
            overwriteTrajectorySpeed(curTr, driveSpeed);
            trFollower->setNewTrajectory(curTr);
            trajectories.erase(trajectories.begin());
        }
        else
        {
            trFollower->removeTrajectory();
        }
    }

    Eigen::Vector2d motionCmd;    
    TrajectoryFollower::FOLLOWER_STATUS status = 
            trFollower->traverseTrajectory(motionCmd, base::Pose(rbpose.position, rbpose.orientation));
    
    switch(status)
    {
	case TrajectoryFollower::REACHED_TRAJECTORY_END:
	    if(!trajectories.empty())
	    {
		    if(state() != RUNNING)
		        state(RUNNING);
		    base::Trajectory curTr(trajectories.front());
		    overwriteTrajectorySpeed(curTr, driveSpeed);
		    trFollower->setNewTrajectory(curTr);
		    trajectories.erase(trajectories.begin());
	    } else
	    {
		if(state() != REACHED_THE_END)
		    state(REACHED_THE_END);
	    }
	    RTT::log(RTT::Info) << "End of the trajectory reached" << RTT::endlog();
	    break;
	case TrajectoryFollower::RUNNING:
	    if(state() != RUNNING)
		state(RUNNING);
	    break;
	case TrajectoryFollower::INITIAL_STABILITY_FAILED:
	    if(state() != INITIAL_STABILITY_FAILED)
		state(INITIAL_STABILITY_FAILED);
		RTT::log(RTT::Error) << "Trajectory follower failed" << RTT::endlog();
	    break;
    }
    
    mc.translation = motionCmd(0);
    mc.rotation    = motionCmd(1);
    
    _motion_command.write(mc);
    _currentCurvePoint.write(trFollower->getCurvePoint());
    _poseError.write(trFollower->getControlError());
    _currentPose.write(trFollower->getPose());
}

// void Task::errorHook()
// {
// }

void Task::stopHook()
{
    if(trFollower)
    {
	delete trFollower;
	trFollower = NULL;
    }
    
    base::MotionCommand2D mc;
    mc.translation = 0;
    mc.rotation    = 0;
    _motion_command.write(mc);
}
// void Task::cleanupHook()
// {
// }

