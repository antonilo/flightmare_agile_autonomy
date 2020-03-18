#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/dynamics/base_dynamics.h>

// others
#include <eigen3/Eigen/Dense>

/***************************************************
 * 
 * Quadrotor setup:   
 *            
 *                 e_y^B
 *                   |
 * 
 * 
 *                  (2)
 *                   |
 *                   |
 *          (3)---(e_z^B)---(1)    --> e_x^B
 *                   |
 *                   |
 *                  (4)
 * 
 * rotors 1 and 3 spin CW, rotors 2 and 4 CCW.
 * 
 **************************************************/

namespace RPGQ
{
    namespace Simulator
    {
        class QuadrotorDynamics : public BaseDynamics
        {
        public:
            // constructor & destructor
            QuadrotorDynamics(DynamicsID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~QuadrotorDynamics() {};

            // simulate dynamics
            void RunSimulation(USecs dt);

            // public get functions
            Eigen::Matrix<double,13,1> GetState(void) {return state_;};
            Eigen::Vector3d GetPos(void) {return state_.segment(0,3);};
            Eigen::Vector3d GetVel(void) {return state_.segment(3,3);};
            Eigen::Vector3d GetAcc(void) {return acc_;};
            Eigen::Quaterniond GetQuat(void) {return Eigen::Quaterniond(&state_(6));};
            Eigen::Vector3d GetOmega(void) {return state_.segment(10,3);};
            Eigen::Vector3d GetPsi(void) {return psi_;};

            // public set functions
            void SetState(Eigen::Matrix<double,13,1> state) {state_ = state;};
            void SetPos(Eigen::Vector3d pos) {state_.segment(0,3) = pos;};
            void SetVel(Eigen::Vector3d vel) {state_.segment(3,3) = vel;};
            void SetQuat(Eigen::Quaterniond quat) {state_.segment(6,4) = quat.coeffs();};
            void SetOmega(Eigen::Vector3d omega) {state_.segment(10,3) = omega;};
            void SetRotorOmega(Eigen::Vector4d omega);
            void SetBox(Eigen::Vector3d box_size) { box_size_= box_size; };

        private:
            // system dynamics
            Eigen::Matrix<double,13,1> SystemDynamics(Eigen::Matrix<double,13,1> &state);
            Eigen::Matrix<double,13,1> state_; // position (3), velocity (3), attitude (4), angular velocity (3)
            Eigen::Vector3d acc_, psi_;
            Eigen::Vector3d f_, t_;    
        
            // dynamics parameters
            double mass_;
            Eigen::Matrix3d J_, Jinv_;
            Eigen::Vector3d g_;

            // thrust / torque map parameters
            double cF_;
            Eigen::Matrix4d B_;

            // World constraints (x, y, z -> in meters)
            Eigen::Vector3d box_size_{1e10, 1e10, 1e10};
        };

    } // namespace Simulator
} // namespace RPGQ