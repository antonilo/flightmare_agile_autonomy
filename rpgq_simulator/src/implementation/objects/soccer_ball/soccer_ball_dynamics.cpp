#include <rpgq_simulator/implementation/objects/soccer_ball/soccer_ball_dynamics.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        SoccerBallDynamics::SoccerBallDynamics(DynamicsID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            BaseDynamics(id, prevSimNode, maxSimUSecsInterval)
        {
            // system dynamics
            state_.setZero();
            state_(9) = 1.0;

            // dynamics parameters
            r_ = 0.11;
            mass_ = 0.440;
            J_.setZero();
            J_(0,0) = 2.0/3.0*mass_*r_*r_;
            J_(1,1) = J_(0,0);
            J_(2,2) = J_(0,0);
            Jinv_ = J_.inverse();
            g_ = Eigen::Vector3d(0.0,0.0,9.81);
            cDrag_ = 0.0085;
            cor_ = 0.8;
        }


        // simulate dynamics
        void SoccerBallDynamics::RunSimulation(USecs dt)
        {
            // Euler integration
            Eigen::Matrix<double,13,1> dstate = SystemDynamics(state_);
            state_ = state_ + dstate*USECS_TO_SECS(dt);

            // normalize quaternion
            state_.segment(6,4).normalize();

            // satisfy ground constraints
            if (state_(2) < 0.0)
            {
                state_(2) = 0.0;
                state_(5) = cor_*std::fabs(state_(5));
            }
        };

        // system dynamics
        Eigen::Matrix<double,13,1> SoccerBallDynamics::SystemDynamics(Eigen::Matrix<double,13,1> &state)
        {
            Eigen::Matrix<double,13,1> dstate;

            // d/dt pos
            dstate.segment(0,3) = state.segment(3,3);

            // d/dt vel
            Eigen::Vector3d vel(state_.segment(3,3));
            Eigen::Quaterniond quat(&state(6));
            dstate.segment(3,3) = -cDrag_*vel.squaredNorm()*vel.normalized() - g_;

            // d/dt quat
            dstate.segment(6,4) = (quat*Eigen::Quaterniond(0.0,0.5*state(10),0.5*state(11),0.5*state(12))).coeffs(); // TODO: CHECK

            // d/dt omega
            Eigen::Vector3d omega(state.segment(10,3));
            dstate.segment(10,3).setZero(); //Jinv_*(t);

            return dstate;
        };
        
    } // namespace Simulator
} // namespace RPGQ