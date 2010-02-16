#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>

#include <iostream>
#include <math.h>
#include <HBridge.hpp>

#define SAMPLING_TIME 0.005


// already defines in drivers/hbridge/HBridge.hpp
#define FL MOTOR_FRONT_LEFT // Front Left
#define FR MOTOR_FRONT_RIGHT  // Front Right
#define RL MOTOR_REAR_LEFT  // Rear Left
#define RR MOTOR_REAR_RIGHT  // Rear Right

#define SEARCH_DIST 0.05  // Distance that will be searched along the curve for the closest point

using namespace trajectory_controller;
using namespace base::geometry;
using namespace hbridge;
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
    u1 = 0.05;  // forward velocity
    l1 = 0.00;  // 10 cm infront of CoG
    R  = 0.45;   // distance between wheels
    r  = 0.018;  // wheel radius
    K0 = 5.0;
    oTrajController_NO.setConstants( l1, K0, R, r);

    K2_P=150.0, K3_P=150.0;
    oTrajController_P.setConstants( K2_P, K3_P, R, r);

    K0_PI=0.0, K2_PI=150.0, K3_PI=150.0;
    oTrajController_PI.setConstants( K0_PI, K2_PI, K3_PI, R, r, SAMPLING_TIME);
    
    velLeftWheel = 0.0;
    velRightWheel = 0.0;

    bCurveGenerated = false;
    return true;
}

double heading(Eigen::Quaterniond q)
{
    return atan( 2*(q.x()*q.w()+q.y()*q.z())/(1-2*(q.z()*q.z()+q.w()*q.w())) );
}


void Task::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    wrappers::samples::RigidBodyState pose;
    std::vector<wrappers::Waypoint> trajectory;

    if(isPortUpdated(_trajectory)) 
    {
	if(_trajectory.read(trajectory)) 
	{
	    for(std::vector<wrappers::Waypoint>::iterator it = trajectory.begin(); it != trajectory.end(); it++) 
	    {
		oCurve.addPoint(it->position);
	    }
	    oCurve.update();	    
	    bCurveGenerated = true; 
	    para = oCurve.getStartParam();
	}
    }

    if(_pose.read(pose) && bCurveGenerated) 
    {
	base::samples::RigidBodyState rbs = pose;	
	if ( para < oCurve.getEndParam() )
	{
	    error = oCurve.poseError(rbs.position, heading(rbs.orientation), para, SEARCH_DIST);
	    para  = error(2);
	    
	    motionCmd = oTrajController_NO.update(u1, error(0), error(1)); 
	    velRightWheel = oTrajController_NO.get_vel_right();
	    velLeftWheel = oTrajController_NO.get_vel_left();

//	    motionCmd = oTrajController_P->update(u1, error(0), error(1), oCurve.getCurvature(para), oCurve.getVoC(para));
//	    velRightWheel = oTrajController_P.get_vel_right();
//	    velLeftWheel = oTrajController_P.get_vel_left();
//
//	    motionCmd = oTrajController_PI->update(u1, error(0), error(1), oCurve.getCurvature(para), oCurve.getVoC(para));
//	    velRightWheel = oTrajController_PI.get_vel_right();
//	    velLeftWheel = oTrajController_PI.get_vel_left();
	    
	    wrappers::Waypoint wp;	   
	    wp.position = oCurve.getPoint(para);
	    wp.heading  = oCurve.getHeading(para);
	    _currentCurvePoint.write(wp);
	}
	else 
	{
	    motionCmd(0) = 0.0; 
	    motionCmd(1) = 0.0; 
	    velRightWheel = 0.0;
	    velLeftWheel = 0.0;
	}	    

	controldev::MotionCommand mc;
	mc.translation = motionCmd(0);
	mc.rotation    = motionCmd(1);

	controldev::FourWheelCommand refVel;

	refVel.mode[0] = refVel.mode[1] =
	    refVel.mode[2] = refVel.mode[3] = controldev::MODE_SPEED;
	refVel.target[FL] = velLeftWheel;
	refVel.target[RL] = velLeftWheel;
	refVel.target[FR] = velRightWheel;
	refVel.target[RR] = velRightWheel;

	if(_motionCommand.connected())
	    _motionCommand.write(mc);
	else
	    _four_wheel_command.write(refVel);
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

