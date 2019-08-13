// rpgq rpgq_common
#include <rpgq_common/quadrotor_id.h>
#include <rpgq_common/time/base_timer.h>

// rpgq rpgq_components
#include <rpgq_components/onboard/state_machine.h>

namespace RPGQ
{
    namespace Onboard
    {
        class OnboardLogic
        {
        public:
            // constructor & destructor
            OnboardLogic(QuadrotorID quadID, USecs updateInterval);
            ~OnboardLogic(void) {};

            // public run functions
            void Run(void);

            // public callbacks (simulation only)
            void CommandCallback(const rpgq_msgs::Command &command);

            // public set functions
           

            // public test functions

            // public print functions

            // TODO (HACK)
            double thrust_;
            Eigen::Vector3d omega_;

            // controller parameters
            Eigen::Matrix3d Kinv_ang_vel_tau_;
            
            double mass_;
            Eigen::Matrix3d J_;

            Eigen::Vector4d rotThrustCmd_, rotOmegaCmd_;

            // thrust / torque map parameters
            double cFInv_, fMin_, fMax_;
            Eigen::Matrix4d BInv_;

            void RunFlightCtrl(Eigen::Vector3d &omega);

        private:
            // general onboard logic variables
            const QuadrotorID quadID_;

            // onboard logic parameters variables and functions

            // state machine
            StateMachine stateMachine_;

            // telemetry
            //void RunTelemetry(const unsigned numMissedTicks);

            // estimator variables and functions
            // none

            // controller variables and functions
            void RunActuatorIdle(void);
            void RunOffboardControlThrustAngularRate(void);
        };
    }
}