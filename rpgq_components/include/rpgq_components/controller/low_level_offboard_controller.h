#pragma once

// rpgq rpgq_common
#include <rpgq_common/msgs/command_msg_helper.h>
#include <rpgq_common/parameters/parameters.h>
#include <rpgq_common/time/ext_timer.h>
#include <rpgq_common/types/state_6dof.h>
#include <rpgq_common/quadrotor_id.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    class LowLevelOffboardController
    {
    public:
        // parameters
        enum
        {
            tauPosXY = 0,
            tauPosIntXY = 1,
            zetaPosXY = 2,
            tauPosZ = 3,
            tauPosIntZ = 4,
            zetaPosZ = 5,
            tauAttXY = 6,
            tauAttZ = 7,
            maxPosErrIntXY = 8,
            maxPosErrIntZ = 9,
            minThrust = 10,
            maxThrust = 11,
            maxOmegaXY = 12,
            maxOmegaZ = 13,
            maxIntStepSize = 14,
            NUM_PARAMS
        };

        // command level
        enum CommandLevel
        {
            POS_INT_CMD = 0,
            POS_CMD = 1,
            VEL_CMD = 2,
            ACC_CMD = 3,
            THRUST_ANG_VEL_CMD = 4,
            FEED_FORWARD = 5,
        };

        // constructor & destructor
        LowLevelOffboardController(QuadrotorID quadrotorID, std::shared_ptr<Timer> timer,
            Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("controller/default"));
        LowLevelOffboardController(std::string name, std::shared_ptr<Timer> timer,
            Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("controller/default")):
            LowLevelOffboardController(QuadrotorNameToID(name), timer, params)
        {};
        ~LowLevelOffboardController(void) {};

        // public run functions
        void Run(const State6DoF &state);

        // public get functions
        rpgq_msgs::Command GetCommand(void);
        double GetThrustCommand(void) {return thrustCmd_;};
        Eigen::Vector3d GetOmegaCommand(void) {return omegaCmd_;};

        // public set functions
        void SetCommandLevel(CommandLevel cmdLevel);
        void SetPosDes(Eigen::Vector3d posDes) {posDes_ = posDes;};
        void SetVelDes(Eigen::Vector3d velDes) {velDes_ = velDes;};
        void SetAccDes(Eigen::Vector3d accDes) {accDes_ = accDes;};
        void SetThrustDes(double thrustDes) {thrustDes_ = thrustDes;};
        void SetYawDes(double yawDes) {yawDes_ = yawDes;};
        void SetOmegaDes(Eigen::Vector3d omegaDes) {omegaDes_ = omegaDes;};

    private:
        // general variables
        const QuadrotorID quadrotorID_;

        // desired control variables
        CommandLevel cmdLevel_;
        Eigen::Vector3d posDes_;
        Eigen::Vector3d velDes_;
        Eigen::Vector3d accDes_;
        double thrustDes_;
        double yawDes_;
        Eigen::Vector3d omegaDes_;

        // low level offboard controller variables and functions
        Parameters<NUM_PARAMS> params_;
        Eigen::Vector3d posErrInt_;
        ExtTimer timerInt_;
        const Eigen::Vector3d I_eZ_I_;
        const Eigen::Vector3d I_g_;
        double thrustCmd_;
        Eigen::Vector3d omegaCmd_;
    };

} // namespace RPGQ