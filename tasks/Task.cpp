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
    _forwardVelocity.set(0.5);
    _forwardLength.set(0.1);

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
    bFirstPose = false;
    bInitStable = false;
    return true;
}


double heading(Eigen::Quaterniond q)
{
    double test = q.x()*q.y() + q.z()*q.w();
    if (test > 0.499) // singularity at north pole
    { 
	return 2 * atan2(q.x(),q.w());
    }
    if (test < -0.499)  // singularity at south pole
    {
	return -2 * atan2(q.x(),q.w());
    }
    double sqy = q.y()*q.y();
    double sqz = q.z()*q.z();
    return atan2(2*q.y()*q.w()-2*q.x()*q.z() , 1 - 2*sqy - 2*sqz);
}


void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    base::samples::RigidBodyState rbs;
    wrappers::samples::RigidBodyState pose;
    std::vector<wrappers::Waypoint> trajectory;

    if(!bFirstPose)
    {
	if(_pose.read(pose))
	{
	    rbs = pose;
	    oCurve.addPoint(rbs.position);
	    bFirstPose = true;
	}
	else 
	    return;
    }

    if(_trajectory.read(trajectory)) 
    {
        oCurve.clear();
        for(std::vector<wrappers::Waypoint>::iterator it = trajectory.begin(); it != trajectory.end(); it++) 
        {
            oCurve.addPoint(it->position);
        }
        oCurve.update();	    
        bCurveGenerated = true; 
        para = oCurve.getStartParam();
    }

    if(_pose.read(pose) && bCurveGenerated) 
    {
	rbs = pose;
	if ( para < oCurve.getEndParam() )
	{
	    rbs.position.x() = rbs.position.x() + _forwardLength.get() * cos(heading(rbs.orientation));
	    rbs.position.y() = rbs.position.y() + _forwardLength.get() * sin(heading(rbs.orientation));

	    error = oCurve.poseError(rbs.position, heading(rbs.orientation), para, SEARCH_DIST);
	    para  = error(2);

	    if(!bInitStable)
	    {
		if(_controllerType.get() == 0)
		{
		    if(!oTrajController_nO.checkInitialStability(error(0), error(1), oCurve.getCurvatureMax()))	    
		    {
			std::cout << "Trajectory controller: no orientation ...failed Initial stability test";
			return;
		    }
		}
		else if(_controllerType.get() == 1)
		{
		    if(!oTrajController_P.checkInitialStability(error(0), error(1), oCurve.getCurvature(para), oCurve.getCurvatureMax()))	    
		    {
			std::cout << "Trajectory controller: Proportional ...failed Initial stability test";
			return;
		    }
		}	
		else if(_controllerType.get() == 2)
		{
		    if(!oTrajController_PI.checkInitialStability(error(0), error(1), oCurve.getCurvature(para), oCurve.getCurvatureMax()))	    
		    {
			std::cout << "Trajectory controller: Proportional integral ...failed Initial stability test";
			return;
		    }	
		}
		bInitStable = true;	
	    }


	    if(_controllerType.get() == 0)
		motionCmd = oTrajController_nO.update(_forwardVelocity.get(), error(0), error(1)); 
	    else if(_controllerType.get() == 1)
		motionCmd = oTrajController_P.update(_forwardVelocity.get(), error(0), error(1), oCurve.getCurvature(para), oCurve.getVariationOfCurvature(para));
	    else if(_controllerType.get() == 2)
		motionCmd = oTrajController_PI.update(_forwardVelocity.get(), error(0), error(1), oCurve.getCurvature(para), oCurve.getVariationOfCurvature(para));
	    
	    wrappers::Waypoint wp;	   
	    wp.position = oCurve.getPoint(para);
	    wp.heading  = oCurve.getHeading(para);
	    _currentCurvePoint.write(wp);
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

