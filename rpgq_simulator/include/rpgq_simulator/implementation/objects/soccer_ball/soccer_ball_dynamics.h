#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/dynamics/base_dynamics.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        class SoccerBallDynamics : public BaseDynamics
        {
        public:
            // constructor & destructor
            SoccerBallDynamics(DynamicsID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~SoccerBallDynamics() {};

            // simulate dynamics
            void RunSimulation(USecs dt);

            // public get functions
            Eigen::Matrix<double,13,1> GetState(void) {return state_;};
            Eigen::Vector3d GetPos(void) {return state_.segment(0,3);};
            Eigen::Vector3d GetVel(void) {return state_.segment(3,3);};
            Eigen::Quaterniond GetQuat(void) {return Eigen::Quaterniond(&state_(6));};
            Eigen::Vector3d GetOmega(void) {return state_.segment(10,3);};

            // public set functions
            void SetState(Eigen::Matrix<double,13,1> state) {state_ = state;};
            void SetPos(Eigen::Vector3d pos) {state_.segment(0,3) = pos;};
            void SetVel(Eigen::Vector3d vel) {state_.segment(3,3) = vel;};
            void SetQuat(Eigen::Quaterniond quat) {state_.segment(6,4) = quat.coeffs();};
            void SetOmega(Eigen::Vector3d omega) {state_.segment(10,3) = omega;};

        private:
            // system dynamics
            Eigen::Matrix<double,13,1> SystemDynamics(Eigen::Matrix<double,13,1> &state);
            Eigen::Matrix<double,13,1> state_; // position (3), velocity (3), attitude (4), angular velocity (3) 
        
            // dynamics parameters
            double r_;
            double mass_;
            Eigen::Matrix3d J_, Jinv_;
            Eigen::Vector3d g_;
            double cDrag_;
            double cor_; // coefficient of restitution
        };

    } // namespace Simulator
} // namespace RPGQ