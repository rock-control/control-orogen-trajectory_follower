/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseWatchdog.hpp"
#include <base-logging/Logging.hpp>

using namespace trajectory_follower;

PoseWatchdog::PoseWatchdog(std::string const& name)
    : PoseWatchdogBase(name)
{
}

PoseWatchdog::PoseWatchdog(std::string const& name, RTT::ExecutionEngine* engine)
    : PoseWatchdogBase(name, engine)
{
}

PoseWatchdog::~PoseWatchdog()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseWatchdog.hpp for more detailed
// documentation about them.

bool PoseWatchdog::configureHook()
{
    if (! PoseWatchdogBase::configureHook())
        return false;
    gotMap = false;
    gotPose = false;
    gotTraj = false;
    
    std::cout << "configured" << std::endl;
    return true;
}
bool PoseWatchdog::startHook()
{
    if (! PoseWatchdogBase::startHook())
        return false;
    std::cout << "started" << std::endl;
    return true;
}
void PoseWatchdog::updateHook()
{
    switch(state())
    {
        case RUNNING:
            //first call to updateHook() after startHook was executed
            state(WAITING_FOR_DATA);
        case WAITING_FOR_DATA:
                gotTraj |= _currentTrajectory.readNewest(currentTrajectory, false) == RTT::NewData;
                gotPose |= _robot_pose.readNewest(pose, false) == RTT::NewData;
                gotMap |= _tr_map.readNewest(map, false) == RTT::NewData;
                std::cout << "waiting... traj:" << gotTraj << ", pose:" << gotPose << ", map:" << gotMap << std::endl;
            if(gotTraj && gotMap && gotPose)
            {
                std::cout << "Got data, starting to watch for pose error" << std::endl;
                state(WATCHING);
            }
            break;
        case WATCHING:
            std::cout << "watching..." << std::endl;
            //check pose whenever we get a new pose or a new map
            if(_robot_pose.readNewest(pose, false) == RTT::NewData ||
               _tr_map.readNewest(map, false) == RTT::NewData)
            {
                std::cout << "checking..." << std::endl;
                if(!checkPose())
                {
                    //pose error, abort trajectory
                    std::cout << "Pose is leaving map. Stopping robot." << std::endl;
                    state(TRAJECTORY_ABORTED);
                }
            }
            break;
        case TRAJECTORY_ABORTED:
            if(_currentTrajectory.readNewest(currentTrajectory, false) == RTT::NewData)
            {
                std::cout << "Got new trajectory. Re-enabling Watchdog." << std::endl;
                state(WATCHING);
            }
            
            break;
        default:
            std::cout << "default state" << std::endl;
            break;
    }
    
    
    PoseWatchdogBase::updateHook();
}

bool PoseWatchdog::checkPose()
{
    return true;
}


void PoseWatchdog::errorHook()
{
    PoseWatchdogBase::errorHook();
}
void PoseWatchdog::stopHook()
{
    PoseWatchdogBase::stopHook();
}
void PoseWatchdog::cleanupHook()
{
    PoseWatchdogBase::cleanupHook();
}
