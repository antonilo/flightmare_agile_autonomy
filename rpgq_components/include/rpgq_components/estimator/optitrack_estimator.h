#pragma once

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// rpgq rpgq_common
#include <rpgq_common/time/base_timer.h>
#include <rpgq_common/types/state_6dof.h>
#include <rpgq_common/parameters/parameters.h>
#include <rpgq_common/quadrotor_id.h>

// rpgq messages
#include <rpgq_msgs/Command.h>
#include <rpgq_msgs/CommandSet.h>

// general
#include <deque>
#include <mutex>

namespace RPGQ
{
    class OptitrackEstimator
    {
    public:
        // parameters
        enum
        {
            tauPos = 0,         // [s]
            tauVel = 1,         // [s]
            tauQuat = 2,        // [s]
            tauOmega = 3,       // [s]
            tauThrustCmd = 4,   // [s]
            tauOmegaCmd = 5,    // [s]
            systemDelay = 6,    // [s]
            blackoutLength = 7, // [s]
            maxSimStep = 8,     // [s]
            NUM_PARAMS
        };

        // constructor & destructor
        OptitrackEstimator(QuadrotorID quadrotorID, Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("estimator/default"));
        ~OptitrackEstimator(void) {};

        // public estimator functions
        void MeasurementUpdate(const geometry_msgs::PoseStampedConstPtr &measurement);
        void FeedCommandQueue(const rpgq_msgs::CommandSet &commandSet);
        void Predict(double dt);
        void Predict(ros::Time t) {Predict((t - lastUpdate_).toSec());};
        void Predict(void) {Predict(params_.GetDouble(systemDelay));};

        // public get functions
        bool Initialized(void) const {return initialized_;};
        State6DoF GetEstimatedState(void) {return estimatedState_;};
        State6DoF GetPredictedState(void) {return predictedState_;};

    private:
        // general estimator variables
        const QuadrotorID quadrotorID_;
        bool initialized_;

        // estimator parameters, variables and functions
        Parameters<NUM_PARAMS> params_;
        State6DoF estimatedState_;
        State6DoF predictedState_;
        State6DoF measuredState_;
        State6DoF UpdatePrior(const double dt);
        State6DoF UpdatePosterior(const double dt, const geometry_msgs::PoseStampedConstPtr &measurement, bool initialize = false);
        void Reset(void);

        // command queue variables
        std::mutex commandQueueMutex_;
        std::deque<std::pair<ros::Time, rpgq_msgs::Command>> commandQueue_;

        // quadrotor dynamics
        State6DoF IntegrateSystemDynamics(const double dt, State6DoF &state, const rpgq_msgs::Command &command);
        State6DoF IntegrateSystemDynamicsSimple(const double dt, const State6DoF &state);

        // auxiliary variables & functions
        double thrust_;
        ros::Time lastMeasurementUpdate_, lastUpdate_;
        uint32_t lastSeq_;
        Eigen::Vector3d QuaternionDifferenceToOmega(const Eigen::Quaterniond &newQuat, const Eigen::Quaterniond &oldQuat, const double dt);
    };
} // namespace RPGQ