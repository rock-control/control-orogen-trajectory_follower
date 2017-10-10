/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseWatchdog.hpp"
#include <base-logging/Logging.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <ugv_nav4d/TravGenNode.hpp>

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
    
    //FIXME why is currentTrajectory a vector? Shouldn't it be just one SubTrajectory?
    
    switch(state())
    {
        case RUNNING:
            //first call to updateHook() after startHook was executed
            state(WAITING_FOR_DATA);
        case WAITING_FOR_DATA:
            gotTraj |= _currentTrajectory.readNewest(currentTrajectory, false) == RTT::NewData;
            gotPose |= _robot_pose.readNewest(pose, false) == RTT::NewData;
            gotMap |= _tr_map.readNewest(map, false) == RTT::NewData;
            
            if(currentTrajectory.empty())
                gotTraj = false;
            
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
               _tr_map.readNewest(map, false) == RTT::NewData ||
               _currentTrajectory.readNewest(currentTrajectory, false) == RTT::NewData)
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
                //FIXME renable state change
//                 state(WATCHING);
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
    //if we are not driving we cannot leave the map
    if(currentTrajectory.empty())
        return true;
    
    //FIXME this assumes that the kind is the same for all SubTrajectories in the vector.
    switch(currentTrajectory.front().kind)
    {
        //while rescuing from an unstable/unsafe position leaving the map is ok.
        case trajectory_follower::TRAJECTORY_KIND_RESCUE:
            return true;
        case trajectory_follower::TRAJECTORY_KIND_NORMAL:
        {
            const maps::grid::TraversabilityNodeBase* node = map.getData().getClosestNode(pose.position);
            if(node)
            {
                const double zDist = fabs(node->getHeight() - pose.position.z());
                if(zDist <= _stepHeight.value())
                {
                    return true;
                }
                std::cout << "checkPose: node too far away" << std::endl;
                return false;
            }
            std::cout << "checkPose: no node at position" << std::endl;
            return false;
        }
            break;
        default:
            throw std::runtime_error("unknown trajectory kind: " + currentTrajectory.front().kind);
    }
    std::cout << "checkPose failed" << std::endl;
    return false;
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
