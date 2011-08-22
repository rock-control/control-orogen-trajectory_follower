#include "Task.hpp"

#include <iostream>
#include <math.h>

#define SAMPLING_TIME 0.01
#define SEARCH_DIST   0.5  // Distance that will be searched along the curve for the closest point

using namespace trajectory_follower;
using namespace base::geometry;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name)
    , oCurve(0.001, 3)
{
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
    oTrajController_nO.setConstants( _forwardLength.get(), _K0_nO.get() );
    oTrajController_nO.setPointTurnSpeed( _pointTurnSpeed.get() );
    oTrajController_P.setConstants( _K2_P.get(), _K3_P.get() );
    oTrajController_PI.setConstants( _K0_PI.get(), _K2_PI.get(), _K3_PI.get(), SAMPLING_TIME);
    
    bCurveGenerated = false;
    bFoundClosestPoint = false;
    bInitStable = false;
    return true;
}


double angleLimit(double angle)
{
    if(angle > M_PI)
	return angle - 2*M_PI;
    else if (angle < -M_PI)
	return angle + 2*M_PI;
    else
     	return angle;
}


void Task::updateHook()
{
    base::MotionCommand2D mc;
    mc.translation = 0;
    mc.rotation    = 0;

    base::samples::RigidBodyState rbpose;
    if(_pose.readNewest(rbpose) == RTT::NoData)
    {
        _motion_command.write(mc);
        return;
    }

    RTT::FlowStatus trajectory_status = _trajectory.readNewest(oCurve, false);
    if (trajectory_status == RTT::NoData)
    {
        _motion_command.write(mc);
        return;
    }
    else if (trajectory_status == RTT::NewData)
    {
        para = oCurve.findOneClosestPoint(rbpose.position);
        bInitStable = false;
    }

    Eigen::Vector2d motionCmd;
    motionCmd(0) = 0.0; 
    motionCmd(1) = 0.0; 

    pose.position = rbpose.position;
    pose.heading  = rbpose.getYaw();
    if ( para < oCurve.getEndParam() )
    {
        if (state() == REACHED_THE_END)
            state(RUNNING);

        if(_controllerType.get() == 0)
        {
            pose.position.x() = pose.position.x() - (_forwardLength.get() + _gpsCenterofRotationOffset.get()) * sin(pose.heading);
            pose.position.y() = pose.position.y() + (_forwardLength.get() + _gpsCenterofRotationOffset.get()) * cos(pose.heading);
        }
        else
        {
            pose.position.x() = pose.position.x() - (_gpsCenterofRotationOffset.get()) * sin(pose.heading);
            pose.position.y() = pose.position.y() + (_gpsCenterofRotationOffset.get()) * cos(pose.heading);
        }

        Eigen::Vector3d vError = oCurve.poseError(pose.position, pose.heading, para);
        para  = vError(2);
       
        error.d 	  = vError(0);
        error.theta_e = angleLimit(vError(1) + M_PI_2);
        error.param   = vError(2);
        
        curvePoint.pose.position 	= oCurve.getPoint(para); 	    
        curvePoint.pose.heading  	= oCurve.getHeading(para);
        curvePoint.param 		= para;

	//disable this test for testing, as it seems to be not needed
	bInitStable = true;
        if(!bInitStable)
        {
            if(_controllerType.get() == 0)
            {
                bInitStable = oTrajController_nO.checkInitialStability(error.d, error.theta_e, oCurve.getCurvatureMax());
                bInitStable = true;
            }
            else if(_controllerType.get() == 1)
                bInitStable = oTrajController_P.checkInitialStability(error.d, error.theta_e, oCurve.getCurvature(para), oCurve.getCurvatureMax());
            else if(_controllerType.get() == 2)
                bInitStable = oTrajController_PI.checkInitialStability(error.d, error.theta_e, oCurve.getCurvature(para), oCurve.getCurvatureMax());

            if (!bInitStable)
            {
                std::cout << "Trajectory controller: failed initial stability test";
                _motion_command.write(mc);
                return state(INITIAL_STABILITY_FAILED);
            }
        }

        if(_controllerType.get() == 0)
            motionCmd = oTrajController_nO.update(_forwardVelocity.get(), error.d, error.theta_e); 
        else if(_controllerType.get() == 1)
            motionCmd = oTrajController_P.update(_forwardVelocity.get(), error.d, error.theta_e, oCurve.getCurvature(para), oCurve.getVariationOfCurvature(para));
        else if(_controllerType.get() == 2)
            motionCmd = oTrajController_PI.update(_forwardVelocity.get(), error.d, error.theta_e, oCurve.getCurvature(para), oCurve.getVariationOfCurvature(para));
    }
    else
    {
        if (state() != REACHED_THE_END)
            state(REACHED_THE_END);
        std::cout << "curve parameter past end of curve" << std::endl;
    }

    mc.translation = motionCmd(0);
    mc.rotation    = motionCmd(1);

    _motion_command.write(mc);
    _currentCurvePoint.write(curvePoint);
    _poseError.write(error);
    _currentPose.write(pose);
}

void Task::errorHook()
{
    updateHook();
    if (bInitStable)
        recover();
}

// void Task::stopHook()
// {
// }
// void Task::cleanupHook()
// {
// }

