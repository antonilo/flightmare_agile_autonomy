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
            ~QuadrotorVehicle(){};

            // public get functions
            Eigen::Matrix<double, 13, 1> GetState(void) 
              {return dynamicsPtr_->GetState();}
            Eigen::Vector3d GetPos(void) 
              {return dynamicsPtr_->GetPos();};
            Eigen::Vector3d GetVel(void) 
              {return dynamicsPtr_->GetVel();};
            Eigen::Vector3d GetAcc(void) 
              {return dynamicsPtr_->GetAcc();};
            Eigen::Quaterniond GetQuat(void) 
              {return dynamicsPtr_->GetQuat();};
            Eigen::Vector3d GetOmega(void) 
              {return dynamicsPtr_->GetOmega();};
            Eigen::Vector3d GetPsi(void) 
              {return dynamicsPtr_->GetPsi();};
            const std::shared_ptr<QuadrotorDynamics> GetDynamics(void) 
              {return dynamicsPtr_;};
            // for unity only
            Eigen::Vector3d GetSize(void) 
              {return size_;};

            // public set functions
            void SetState(const Eigen::Matrix<double, 13, 1> & state) 
              {dynamicsPtr_->SetState(state);};
            void SetPos(const Eigen::Vector3d & pos) 
              {dynamicsPtr_->SetPos(pos);};
            void SetVel(const Eigen::Vector3d & vel) 
              {dynamicsPtr_->SetVel(vel);};
            void SetQuat(const Eigen::Quaterniond & quat) 
              {dynamicsPtr_->SetQuat(quat);};
            void SetOmega(const Eigen::Vector3d & omega) 
              {dynamicsPtr_->SetOmega(omega);};
            void SetCommandSet(const rpgq_msgs::CommandSet &cmdSet)
              {commandBridgeClientPtr_->SetCommandSet(cmdSet);};
            void SetBox(Eigen::Vector3d & box_size) 
              {dynamicsPtr_->SetBox(box_size);};

            // for unity only
            void SetSize(const Eigen::Vector3d & size) 
              {size_ = size; };
            void SetCollision(const bool & has_collision)
              {has_collision_ = has_collision; };

        private:
            // vehicle type
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
            bool has_collision_{false};

        };

    } // namespace Simulator
} // namespace RPGQ