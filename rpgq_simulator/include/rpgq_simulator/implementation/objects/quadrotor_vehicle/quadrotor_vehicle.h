#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/directly_controlled_object.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_dynamics.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_onboard_code.h>

// rpgq rpgq_common
#include <rpgq_common/quadrotor_id.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        class QuadrotorVehicle : public DirectlyControlledObject
        {
        public:
            // constructor & destructor
            QuadrotorVehicle(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~QuadrotorVehicle() {};

            // public get functions
            Eigen::Vector3d GetPos(void) { return dynamicsPtr_->GetPos();};
            Eigen::Vector3d GetVel(void) {return dynamicsPtr_->GetVel();};
            Eigen::Vector3d GetAcc(void) {return dynamicsPtr_->GetAcc();};
            Eigen::Quaterniond GetQuat(void) {return dynamicsPtr_->GetQuat();};
            Eigen::Vector3d GetOmega(void) {return dynamicsPtr_->GetOmega();};
            Eigen::Vector3d GetPsi(void) {return dynamicsPtr_->GetPsi();};
            const std::shared_ptr<QuadrotorDynamics> GetDynamics(void) {return dynamicsPtr_;};
            // for unity only
            Eigen::Vector3d GetSize(void) { return size_; };

            // public set functions
            void SetPos(Eigen::Vector3d pos) {dynamicsPtr_->SetPos(pos);};
            void SetVel(Eigen::Vector3d vel) {dynamicsPtr_->SetVel(vel);};
            void SetQuat(Eigen::Quaterniond quat) {dynamicsPtr_->SetQuat(quat);};
            void SetOmega(Eigen::Vector3d omega) {dynamicsPtr_->SetOmega(omega);};
            void SetCommandSet(const rpgq_msgs::CommandSet &cmdSet) {commandBridgeClientPtr_->SetCommandSet(cmdSet);};
            // for unity only
            void SetSize(Eigen::Vector3d size) { size_ = size; };

        private:
            // quadrotor output
            void ComputeOutputOnce(void) {};
            void ComputeOutputContinuously(void) {};

            // mediators
            void OnboardCodeMediator(void);
            void ActuatorMediator(void);
            void DynamicsMediator(void);
            void SensorMediator(void) {};

            // auxiliary variables
            std::shared_ptr<QuadrotorDynamics> dynamicsPtr_;
            std::shared_ptr<CommandBridgeClient> commandBridgeClientPtr_;
            std::shared_ptr<QuadrotorOnboardLogic> onboardLogicPtr_;

            //
            Eigen::Vector3d size_{1.0, 1.0, 1.0};
        };

    } // namespace Simulator
} // namespace RPGQ