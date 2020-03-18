#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/onboard_code/base_onboard_code.h>

// rpgq rpgq_common
#include <rpgq_common/msgs/command_msg_helper.h>
#include <rpgq_common/quadrotor_id.h>

// rpgq rpgq_components
#include <rpgq_components/onboard/onboard_logic.h>

// standard librarry
#include <functional>

// others
#include <eigen3/Eigen/Dense>


namespace RPGQ
{
    namespace Simulator
    {        
        /*************************
         * Command Bridge Client *
         *************************/
        using CommandCallback_t = std::function<void (const rpgq_msgs::Command&)>;

        class CommandBridgeClient : public BaseOnboardCode
        {
        public:
            // constructor & destructor
            CommandBridgeClient(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~CommandBridgeClient(void) {};

            // public set functions
            void AddCommandCallback(CommandCallback_t callback);
            void SetCommandSet(const rpgq_msgs::CommandSet &commandSet);

        private:
            // general
            const QuadrotorID quadID_;

            // simulate onboard code
            void RunSimulation_(USecs dt);

            // command bridge client variables and functions
            bool newCommandSet_;
            rpgq_msgs::CommandSet commandSet_;
            rpgq_msgs::Command command_;
            ros::Publisher commandPub_;
            std::vector<CommandCallback_t> commandCallbacks_;
        };


        /*****************
         * Onboard Logic *
         *****************/
        class QuadrotorOnboardLogic : public BaseOnboardCode
        {
        public:
            // constructor & destructor
            QuadrotorOnboardLogic(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~QuadrotorOnboardLogic(void) {};

            // public callback functions
            void CommandCallback(const rpgq_msgs::Command &command);

            // (hack)
            void RunFlightCtrl(Eigen::Vector3d &omega) {onboardLogic_->RunFlightCtrl(omega);};
            Eigen::Vector4d GetRotOmegaCmd(void) {return onboardLogic_->rotOmegaCmd_;};
            
        private:
            // general
            const QuadrotorID quadID_;

            // simulate onboard code
            void RunSimulation_(USecs dt);

            // onboard logic variables and functions
            std::shared_ptr<Onboard::OnboardLogic> onboardLogic_;
        };


        /*********************
         * Flight Controller *
         *********************/
        class FlightController : public BaseOnboardCode
        {
        public:
            // constructor & destructor
            FlightController(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~FlightController(void) {};

            // public set functions
            void SetCommandSet(const rpgq_msgs::CommandSet &commandSet);

        private:
            // general
            const QuadrotorID quadID_;

            // simulate onboard code
            void RunSimulation_(USecs dt);

            // command bridge client variables and functions
            bool newCommandSet_;
            rpgq_msgs::CommandSet commandSet_;
            rpgq_msgs::Command command_;
            ros::Publisher commandPub_;
        };



        /*class Telemetry : public BaseOnboardCode
        {
        public:
            // constructor & destructor
            Telemetry(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~Telemetry(void) {};

        private:
            // general
            const QuadrotorID quadID_;

            // simulate onboard code
            void RunSimulation_(USecs dt);

            // telemetry logic variables and functions
        };*/


        /*class QuadrotorOnboardCode : public BaseOnboardCode
        {
        public:
            // constructor & destructor
            QuadrotorOnboardCode(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~QuadrotorOnboardCode() {};

            // public get functions
            Eigen::Vector4d GetRotorOmegaCmd(void) {return rotorOmegaCmd_;};

            // public set functions
            void SetCommandSet(const rpgq_msgs::CommandSet &cmdSet);

        private:
            // general
            const QuadrotorID quadID_;

            // simulate onboard code
            void RunSimulation_(USecs dt);

            // controller parameters
            Eigen::Matrix3d Kinv_tau_, K_zeta_;
            double k_p_xy_, k_p_z_; Eigen::Matrix3d K_D_;
            Eigen::Matrix3d Kinv_ang_vel_tau_;

            double mass_;
            Eigen::Matrix3d J_;

            // thrust / torque map parameters
            double cFInv_, fMin_, fMax_;
            Eigen::Matrix4d BInv_;

            // controller variables
            Eigen::Vector4d u_, rotorOmegaCmd_;
            Eigen::Vector3d posDes_, velDes_, accDes_; double normThrustDes_;
            double yawDes_; Eigen::Vector3d omegaDes_;
            Eigen::Matrix<double,13,1> state_;

            // auxiliary variables
            Eigen::Vector3d e_z_;
            Eigen::Vector3d g_;
        };*/

    } // namespace Simulator
} // namespace RGPQ