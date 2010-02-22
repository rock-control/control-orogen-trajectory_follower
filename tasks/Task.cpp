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
{
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
    forwardVelocity = 0.05;  // forward velocity
    l1 = 0.00;  // 10 cm infront of CoG
    
    double K0 = 5.0;
    oTrajController_nO.setConstants( l1, K0, ROBOT.TRACK, ROBOT.WHEEL_RADIUS_EFF);

//    double K2_P=150.0, K3_P=150.0;
//    oTrajController_P.setConstants( K2_P, K3_P, ROBOT.TRACK, ROBOT.WHEEL_RADIUS_EFF);
//
//    double K0_PI=0.0, K2_PI=150.0, K3_PI=150.0;
//    oTrajController_PI.setConstants( K0_PI, K2_PI, K3_PI, ROBOT.TRACK, ROBOT.WHEEL_RADIUS_EFF, SAMPLING_TIME);
    
    bCurveGenerated = false;
    bFirstPose = false;
    newCurve = false;
    return true;
}


// QUICK FIX...  extracting heading from quaternion
double heading(Eigen::Quaterniond q)
{
    return atan( 2*(q.x()*q.w()+q.y()*q.z())/(1-2*(q.z()*q.z()+q.w()*q.w())) );
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
	    rbs.position.x() = rbs.position.x() + l1 * cos(heading(rbs.orientation));
	    rbs.position.y() = rbs.position.y() + l1 * sin(heading(rbs.orientation));

	    error = oCurve.poseError(rbs.position, heading(rbs.orientation), para, SEARCH_DIST);
	    para  = error(2);

	    if(newCurve)
	    {
	    	if(!oTrajController_nO.checkInitialStability(error(0), error(1), oCurve.getCurvatureMax()))	    
	    	{
		    std::cout << "Trajectory controller: no orientation ...failed Initial stability test";
		    return;
		}
	    	
//		if(!oTrajController_P.checkInitialStability(error(0), error(1), oCurve.getCurvature(para), oCurve.getCurvatureMax()))	    
//	    	{
//		    std::cout << "Trajectory controller: Proportional ...failed Initial stability test";
//		    return;
//		}
//
//	    	if(!oTrajController_PI.checkInitialStability(error(0), error(1), oCurve.getCurvature(para), oCurve.getCurvatureMax()))	    
//	    	{
//		    std::cout << "Trajectory controller: Proposrtional integral ...failed Initial stability test";
//		    return;
//		}
	        newCurve = false;	
	    }
	    motionCmd = oTrajController_nO.update(forwardVelocity, error(0), error(1)); 

//	    motionCmd = oTrajController_P->update(forwardVelocity, error(0), error(1), oCurve.getCurvature(para), oCurve.getVoC(para));
//	    motionCmd = oTrajController_PI->update(forwardVelocity, error(0), error(1), oCurve.getCurvature(para), oCurve.getVoC(para));
	    
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

