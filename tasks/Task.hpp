#ifndef trajectory_follower_TASK_TASK_HPP
#define trajectory_follower_TASK_TASK_HPP

#include "trajectory_follower/TaskBase.hpp"
 
#include <base/geometry/spline.h>
#include "TrajectoryControllerNoOrientation.hpp" 
#include "TrajectoryControllerP.hpp" 
#include "TrajectoryControllerPI.hpp" 

#include <Eigen/Core>
#include <Eigen/Geometry> 

namespace RTT
{
    class NonPeriodicActivity;
}


namespace trajectory_follower {
    class Task : public TaskBase
    {
	friend class TaskBase;

	base::geometry::Spline<3> oCurve;
	bool bCurveGenerated;
	bool bFoundClosestPoint;    // check if the starting pose is added to the trajectory
	bool bInitStable; 

	double para;

    	TrajError error;
	CurvePoint curvePoint;
	RobotPose pose;

	trajectory_follower::noOrientation oTrajController_nO;
	trajectory_follower::chainedProportional oTrajController_P;
	trajectory_follower::chainedProportionalIntegral oTrajController_PI;

	Eigen::Vector2d motionCmd;

    protected:
    

    public:
        Task(std::string const& name = "trajectory_follower::Task");

        RTT::NonPeriodicActivity* getNonPeriodicActivity();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        // bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called. See README.txt for different
         * triggering options.
         *
         * The warning(), error() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeWarning, RunTimeError and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recovered()
         * allows you to go back into the Running state.  In the second case,
         * the errorHook() will be called instead of updateHook() and in the
         * third case the component is stopped and resetError() needs to be
         * called before starting it again.
         *
         * The \a updated_ports argument is the set of ports that have triggered
         * this call. If the trigger is caused by something different (for
         * instance, a periodic update), then this set is empty.
         */
         void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

