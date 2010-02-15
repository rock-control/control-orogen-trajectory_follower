#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>


using namespace trajectory_controller;


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
    float u1 = 0.05;  // forward velocity
    float l1 = 0.00;  // 10 cm infront of CoG
    float R  = 0.45;   // distance between wheels
    float r  = 0.018;  // wheel radius
    float K0 = 5.0;

    TrajectoryController::TrajectoryController_NoOrientation oTrajController_NO( u1, l1, K0, R, r);

    // Trajectory controller P
    float K1=0.0, K2=150.0, K3=150.0;
    TrajectoryController::TrajectoryController_PI oTrajController_PI( K1, K2, K3, R, r);

    bCurveGenerated = false;
    return true;
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
		oCurve.add_point(it->position);
	    }
	    oCurve.generate_curve();	    
	    bCurveGenerated = true; 
	}
    }
    
    if(_pose.read(pose) && bCurveGenerated) 
    {
	base::samples::RigidBodyState rbs = pose;	
	
	
	controldev::MotionCommand mc;
	std::cout << "DTF: New Movement command tv " << mc.translation << " rv " << mc.rotation << std::endl;
	

	_motionCommand.write(mc);
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

