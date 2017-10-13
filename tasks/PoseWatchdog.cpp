/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseWatchdog.hpp"
#include <base-logging/Logging.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <ugv_nav4d/TravGenNode.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>


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
    haltCommand.heading = base::Angle::fromRad(0);
    haltCommand.translation = 0;
    haltCommand.rotation = 0;
    
    nanCommand.heading = base::Angle::fromRad(std::nan(""));
    nanCommand.translation = std::nan("");
    nanCommand.rotation = std::nan("");
    
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
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(this);
    
    //FIXME why is currentTrajectory a vector? Shouldn't it be just one SubTrajectory?
    
    
    //the watchdog has to send either nan or an override command.
    //if it stops sending anything the safety switch will go into TIMEOUT state.
    
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

            _motion_command_override.write(nanCommand);//tell safety that we are still alive
            
            //check pose whenever we get a new pose or a new map
            //NOTE Do **not** use short circuit or operator here. We need to execute all reads on every loop!
            if((_robot_pose.readNewest(pose, false) == RTT::NewData) |
               (_tr_map.readNewest(map, false) == RTT::NewData) |
               (_currentTrajectory.readNewest(currentTrajectory, false)) == RTT::NewData)
            {
//                 std::cout << "checking..." << std::endl;
                if(!checkPose())
                {
                    DRAW_CYLINDER("watchdog_triggered", pose.position, base::Vector3d(0.03, 0.03, 0.4), vizkit3dDebugDrawings::Color::cyan);
                    //pose error, abort trajectory
                    std::cout << "Pose is leaving map. Stopping robot." << std::endl;
                    _motion_command_override.write(haltCommand);
                    state(TRAJECTORY_ABORTED);
                }
            }
            break;
        case TRAJECTORY_ABORTED:
                //keep overriding until someone resets the state
                _motion_command_override.write(haltCommand);
            
            break;
        case RESETTED:
            std::cout << "waiting for valid pose" << std::endl;
            //until we get a valid pose we simply belive that everything is fine... HACK
            _motion_command_override.write(nanCommand);//tell safety that we are still alive
            
            if((_robot_pose.readNewest(pose, false) == RTT::NewData) |
               (_tr_map.readNewest(map, false) == RTT::NewData) |
               (_currentTrajectory.readNewest(currentTrajectory, false)) == RTT::NewData)
            {
                if(checkPose())
                {
                    std::cout << "got valid pose, WATCHING" << std::endl;
                    state(WATCHING);
                }
            }
            
            break;
        default:
            std::cout << "default state" << std::endl;
            break;
    }
    
    
    _tr_map_out.write(map);
    
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
            std::cout << "ignoring rescue trajectory" << std::endl;
            return true;
        case trajectory_follower::TRAJECTORY_KIND_NORMAL:
        {
            maps::grid::TraversabilityNodeBase* node = map.getData().getClosestNode(pose.position);
            if(node)
            {
                //subtract distToGround to get from robot frame to map frame
                const double zDist = fabs(node->getHeight() - (pose.position.z() - _distToGround.get()));
                if(zDist > _stepHeight.value())
                {
                    std::cout << "checkPose: node too far away. Distance: " << zDist << ", allowed distance: " << _stepHeight.value()  << std::endl;
                    return false;
                }
                
                
                std::deque<maps::grid::TraversabilityNodeBase*> nodes;
                nodes.push_back(node);
                std::unordered_set<maps::grid::TraversabilityNodeBase*> visited;
                const double maxDist = 0.1;
                const double gridRes = map.getData().getResolution().x();
                while(nodes.size() > 0)
                {
                    maps::grid::TraversabilityNodeBase* currNode = nodes.front();
                    nodes.pop_front();
                    visited.insert(currNode);
                    
                    if(currNode->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                    {
                        for(maps::grid::TraversabilityNodeBase* neighbor : currNode->getConnections())
                        {
                            const double dist = (neighbor->getVec3(gridRes).topRows(2) - node->getVec3(gridRes).topRows(2)).norm();
                            if(dist <= maxDist && visited.find(neighbor) == visited.end())
                                nodes.push_back(neighbor);
                        }
                    }
                    else
                    {
DRAW_CYLINDER("growCheckFail", currNode->getVec3(gridRes), base::Vector3d(0.1, 0.1, 0.8), vizkit3dDebugDrawings::Color::yellow);
                        std::cout << "checkPose: node not traversable." << std::endl;
                        return false;
                    }
                    
                }
                return true;
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

void PoseWatchdog::reset()
{
    state(RESETTED);
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
