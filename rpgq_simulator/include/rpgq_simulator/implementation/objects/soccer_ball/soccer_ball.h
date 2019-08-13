# pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/indirectly_controlled_object.h>
#include <rpgq_simulator/implementation/objects/soccer_ball/soccer_ball_dynamics.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        class SoccerBall : public IndirectlyControlledObject
        {
        public:
            // constructor & destructor
            SoccerBall(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~SoccerBall() {};

            // public get functions
            Eigen::Vector3d GetPos(void) {return dynamicsPtr_->GetPos();};
            Eigen::Vector3d GetVel(void) {return dynamicsPtr_->GetVel();};
            Eigen::Quaterniond GetQuat(void) {return dynamicsPtr_->GetQuat();};
            Eigen::Vector3d GetOmega(void) {return dynamicsPtr_->GetOmega();};
            const std::shared_ptr<SoccerBallDynamics> GetDynamics(void) {return dynamicsPtr_;};

            // public set functions
            void SetPos(Eigen::Vector3d pos) {dynamicsPtr_->SetPos(pos);};
            void SetVel(Eigen::Vector3d vel) {dynamicsPtr_->SetVel(vel);};
            void SetQuat(Eigen::Quaterniond quat) {dynamicsPtr_->SetQuat(quat);};
            void SetOmega(Eigen::Vector3d omega) {dynamicsPtr_->SetOmega(omega);};

        private:
            // quadrotor output
            void ComputeOutputOnce(void);
            void ComputeOutputContinuously(void) {};
            ros::Publisher posePub_;

            // mediators
            void DynamicsMediator(void) {};
            void SensorMediator(void) {};

            // auxiliary variables
            std::shared_ptr<SoccerBallDynamics> dynamicsPtr_;    
        };

    } // namespace Simulator
} // namespace RPGQ