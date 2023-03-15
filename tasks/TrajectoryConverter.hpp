#ifndef TRAJECTORY_FOLLOWER_TRAJECTORYCONVERTER_TASK_HPP
#define TRAJECTORY_FOLLOWER_TRAJECTORYCONVERTER_TASK_HPP

#include "trajectory_follower/TrajectoryConverterBase.hpp"

namespace trajectory_follower
{
    class TrajectoryConverter : public TrajectoryConverterBase
    {
    friend class TrajectoryConverterBase;
    protected:


    public:
        TrajectoryConverter(std::string const& name = "trajectory_follower::TrajectoryConverter");
        ~TrajectoryConverter();

        void updateHook();
    };
}

#endif

