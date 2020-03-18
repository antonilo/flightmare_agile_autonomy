#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_dynamics.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        QuadrotorDynamics::QuadrotorDynamics(DynamicsID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            BaseDynamics(id, prevSimNode, maxSimUSecsInterval)
        {
            // system dynamics
            state_.setZero();
            state_(9) = 1.0; // eigen quaternion w = 1
            acc_.setZero();
            psi_.setZero();
            f_.setZero();
            t_.setZero();

            //
            box_size_ << 1e10, 1e10, 1e10;

            // dynamics parameters
            mass_ = 0.73;
            J_.setZero();
            J_(0,0) = 0.007; J_(1,1) = 0.007; J_(2,2) = 0.012;
            Jinv_ = J_.inverse();
            g_ = Eigen::Vector3d(0.0, 0.0, 9.81);

            // thrust / torque map parameters
            cF_ = 8.54858e-6;
            // arm length
            double l = 0.17;
            // rotor-torque coefficient
            double kappa = 0.016;

            B_ << 1, 1, 1, 1,
                0, l, 0, -l,
                -l, 0, l, 0,
                kappa, -kappa, kappa, -kappa;
        }

        // simulate dynamics
        void QuadrotorDynamics::RunSimulation(USecs dt)
        {
            Eigen::Matrix<double, 13, 1> old_state = state_;

            // Euler integration
            Eigen::Matrix<double,13,1> dstate = SystemDynamics(state_);
            state_ = state_ + dstate*USECS_TO_SECS(dt);

            // normalize quaternion
            state_.segment(6,4).normalize();
        
            // store current acceleration angular acceleration
            acc_ = dstate.segment(3,3);
            psi_ = dstate.segment(10,3);

            // satisfy box constraints
            if ( std::abs(state_(0)) - box_size_.x()/2  > 0.0 ){
                // if violating the position constraint in x axis
                // stop immediately in x direction
                state_(0) = old_state(0); 
                state_(3) = 0.0; 
            } 
            if (std::abs(state_(1)) - box_size_.y()/2 > 0.0)
            {
                // if violating the position constraint in y axis
                // stop immediately in y direction
                state_(1) = old_state(1);
                state_(4) = 0.0;
            } 
            if ( state_(2) - box_size_.z() > 0.0)
            {
                // if violating the position constraint in +z axis
                // stop immediately in +z direction
                state_(2) = old_state(2);
                state_(5) = 0.0;
            } 
            if (state_(2) < 0.0)
            {
                // if violating the position constraint in -z axis
                // stop immediately in xyz direction when touching the ground
                state_(2) = 0.0;
                // set velocity in all 3 axes equal to 0
                state_.segment(3, 3) = Eigen::Vector3d::Zero();
                // reset the rotation (hack)
                state_.segment(6, 4) = Eigen::Vector4d::Zero();
                state_(9) = 1.0;
                // reset the anguler velocity
                state_.segment(10, 3) = Eigen::Vector3d::Zero();

                // 
                acc_.setZero();
                psi_.setZero();
            }  
        };


        // public set functions
        void QuadrotorDynamics::SetRotorOmega(Eigen::Vector4d omega)
        {
            // compute rotor thrusts
            Eigen::Vector4d u = omega.array().square()*cF_;

            // compute virtual control inputs
            Eigen::Vector4d v = B_*u;

            // update total thrust and torque
            f_(2) = v(0);
            t_ = v.segment(1,3);
        };


        // system dynamics
        Eigen::Matrix<double,13,1> QuadrotorDynamics::SystemDynamics(Eigen::Matrix<double,13,1> &state)
        {
            Eigen::Matrix<double,13,1> dstate;

            // d/dt pos
            dstate.segment(0,3) = state.segment(3,3);

            // d/dt vel
            Eigen::Quaterniond quat(&state(6));
            dstate.segment(3,3) = quat*f_*1.0/mass_ - g_;

            // d/dt quat
            dstate.segment(6,4) = (quat*Eigen::Quaterniond(0.0,0.5*state(10),0.5*state(11),0.5*state(12))).coeffs();

            // d/dt omega
            Eigen::Vector3d omega(state.segment(10,3));
            dstate.segment(10,3) = Jinv_*(t_ - omega.cross(J_*omega));

            return dstate;
        };
        
    } // namespace Simulator
} // namespace RPGQ