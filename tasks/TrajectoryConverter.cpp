#include "TrajectoryConverter.hpp"

using namespace trajectory_follower;

TrajectoryConverter::TrajectoryConverter(std::string const& name)
    : TrajectoryConverterBase(name)
{
}

TrajectoryConverter::~TrajectoryConverter()
{
}

void TrajectoryConverter::updateHook()
{
    TrajectoryConverterBase::updateHook();
    
    // Read list of poses
    std::vector<base::Pose> input;
    if(_poses.readNewest(input, false) != RTT::NewData)
        return;

    // Create the trajectory 
    std::vector<trajectory_follower::SubTrajectory> output;
    base::Position prev_position;
    base::Vector3d prev_tangent;
    bool first = true;

    for(base::Pose pose : input)
    {
        base::Vector3d tangent = pose.position + (pose.orientation * Eigen::Vector3d::UnitX());
        if(first)
        {
            first = false;
        }else
        {
            std::vector<base::Position> positions;
            std::vector<base::geometry::SplineBase::CoordinateType> types;
            positions.push_back(prev_position);
            positions.push_back(prev_tangent);
            types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
            types.push_back(base::geometry::SplineBase::TANGENT_POINT_FOR_PRIOR);

            positions.push_back(pose.position);
            types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
            positions.push_back(tangent);
            types.push_back(base::geometry::SplineBase::TANGENT_POINT_FOR_PRIOR);

            // Create SubTrajectory
            base::Trajectory trajectory;
            trajectory.spline.interpolate(positions, std::vector<double>(), types);
            trajectory_follower::SubTrajectory sub(trajectory);
            sub.driveMode = trajectory_follower::DriveMode::ModeAckermann;
            sub.speed = _velocity;
            output.push_back(sub);
        }

        prev_position = pose.position;
        prev_tangent = tangent;
    }
    _trajectory.write(output);
}

