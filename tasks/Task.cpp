#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>

#include <iostream>
#include <math.h>

#define SAMPLING_TIME 0.005
#define SEARCH_DIST 0.05  // Distance that will be searched along the curve for the closest point

using namespace trajectory_controller;
using namespace base::geometry;
using namespace Eigen;

RTT::NonPeriodicActivity* Task::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


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
    oTrajController_nO.setConstants( _forwardLength.get(), _K0_nO.get(), ROBOT.TRACK, ROBOT.WHEEL_RADIUS_EFF);
    oTrajController_P.setConstants( _K2_P.get(), _K3_P.get(), ROBOT.TRACK, ROBOT.WHEEL_RADIUS_EFF);
    oTrajController_PI.setConstants( _K0_PI.get(), _K2_PI.get(), _K3_PI.get(), ROBOT.TRACK, ROBOT.WHEEL_RADIUS_EFF, SAMPLING_TIME);
    
    bCurveGenerated = false;
    bFoundClosestPoint = false;
    bInitStable = false;
    return true;
}


double heading(Eigen::Quaterniond q)
{
    // Hope this works ... needs a better fix
    return atan2(2*q.x()*q.y()+2*q.w()*q.z() , 1-2*(q.y()*q.y() + q.z()*q.z()));
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


void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    wrappers::samples::RigidBodyState rbpose;
    std::vector<wrappers::Waypoint> trajectory;

    if(_trajectory.read(trajectory)) 
    {
	oCurve.clear();
        for(std::vector<wrappers::Waypoint>::iterator it = trajectory.begin(); it != trajectory.end(); it++) 
        {
            oCurve.addPoint(it->position);
        }
        oCurve.update();	    
        bCurveGenerated = true; 
//        para = oCurve.getStartParam();
    }

    if(bCurveGenerated && !bFoundClosestPoint)
    {
	if(_pose.read(rbpose))
	{
	    pose.position = rbpose.position;
	    para = oCurve.findOneClosestPoint(rbpose.position);
	    bFoundClosestPoint = true;
	}
	else 
	    return;
    }

    if(_pose.read(rbpose) && bCurveGenerated) 
    {
	pose.position = rbpose.position;
	pose.heading  = heading(rbpose.orientation);
	if ( para < oCurve.getEndParam() )
	{
	    if(_controllerType.get() == 0)
	    {
		pose.position.x() = pose.position.x() - (_forwardLength.get() + _gpsCenterofRotationOffset.get()) * sin(pose.heading);
		pose.position.y() = pose.position.y() + (_forwardLength.get() + _gpsCenterofRotationOffset.get()) * cos(pose.heading);
	    }

	    Eigen::Vector3d vError = oCurve.poseError(pose.position, pose.heading, para, SEARCH_DIST);
	    para  = vError(2);
	   
	    error.d 	  = vError(0);
	    error.theta_e = angleLimit(vError(1) + M_PI_2);
	    error.param   = vError(2);
	    
 	    curvePoint.pose.position 	= oCurve.getPoint(para); 	    
	    curvePoint.pose.heading  	= oCurve.getHeading(para);
	    curvePoint.param 		= para;
	
	    if(!bInitStable)
	    {
		if(_controllerType.get() == 0)
		{
		    if(!oTrajController_nO.checkInitialStability(error.d, error.theta_e, oCurve.getCurvatureMax()))	    
		    {
			std::cout << "Trajectory controller: no orientation ...failed Initial stability test";
//			return;
		    }
		}
		else if(_controllerType.get() == 1)
		{
		    if(!oTrajController_P.checkInitialStability(error.d, error.theta_e, oCurve.getCurvature(para), oCurve.getCurvatureMax()))	    
		    {
			std::cout << "Trajectory controller: Proportional ...failed Initial stability test";
//			return;
		    }
		}	
		else if(_controllerType.get() == 2)
		{
		    if(!oTrajController_PI.checkInitialStability(error.d, error.theta_e, oCurve.getCurvature(para), oCurve.getCurvatureMax()))	    
		    {
			std::cout << "Trajectory controller: Proportional integral ...failed Initial stability test";
//			return;
		    }	
		}
		bInitStable = true;	
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
	    motionCmd(0) = 0.0; 
	    motionCmd(1) = 0.0; 
	}	    

	controldev::MotionCommand mc;
	mc.translation = motionCmd(0);
	mc.rotation    = motionCmd(1);

        _motion_command.write(mc);
	_currentCurvePoint.write(curvePoint);
	_poseError.write(error);
	_currentPose.write(pose);
    }
}

// void Task::errorHook()
// {
// }
// void Task::stopHook()
// {
// }
// void Task::cleanupHook()
// {
// }

