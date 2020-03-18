#include <rpgq_components/onboard/onboard_logic.h>

// rpgq rpgq_common
#include <rpgq_common/rpgq.h>

using namespace RPGQ::Onboard;

// constructor
OnboardLogic::OnboardLogic(QuadrotorID quadID, USecs updateInterval):
    quadID_(quadID)
{
    // TODO (Hack)
    double thrust_ = 0.0;
    omega_ = Eigen::Vector3d::Zero();

    // angular velocity controller
    Kinv_ang_vel_tau_.setZero();
    Kinv_ang_vel_tau_(0,0) = 1.0/0.01; // 100
    Kinv_ang_vel_tau_(1,1) = 1.0/0.01; // 100
    Kinv_ang_vel_tau_(2,2) = 1.0/0.1;  // 10

    // others
    mass_ = 0.73;
    J_.setZero();
    J_(0,0) = 0.007; J_(1,1) = 0.007; J_(2,2) = 0.012;
    double l = 0.17; // arm length
    double kappa = 0.016; // rotor drag coefficient

    // thrust / torque map parameters
    cFInv_ = 1.0/8.54858e-6;
    fMin_ = 0.15; // make sure motors can actually spin slow/fast enough
    fMax_ = 3.5;

    Eigen::Matrix4d B;
    B << 1, 1, 1, 1,
        0, l, 0, -l,
        -l, 0, l, 0,
        kappa, -kappa, kappa, -kappa;
    BInv_ = B.inverse();

    rotThrustCmd_.setZero();
    rotOmegaCmd_.setZero();
}


// public run functions 
void OnboardLogic::Run(void)
{
    // update counter

    // update sensors

    // store old state
    OnboardState oldState = stateMachine_.GetState();

    // update state machine (and warn about missed ticks)
    stateMachine_.Update();

    // process commands

    // run state machine
    switch (stateMachine_.GetState())
    {
    // 0-9: on ground
    case OnboardState::ARMED:
        // arm motors
        break;

    case OnboardState::ACTUATOR_IDLE:
        RunActuatorIdle();
        break;

    case OnboardState::SHUTDOWN:
        // TODO
        break;

    // 10 - 29: flying
    case OnboardState::THRUST_ANGULAR_RATE:
        RunOffboardControlThrustAngularRate();
        break;

    // 40 - 49: emergency
    case OnboardState::DISARMED:
    case OnboardState::COMMAND_LOST:
    case OnboardState::LOW_BATTERY:
    case OnboardState::INTERNAL_ERROR:
        // disable motors
        break;

    // unknown states
    default:
        stateMachine_.SetInternalError();
        // disable motors
        break;
    }

    // run telemetry
    //RunTelemetry();
}


// public callbacks (simulation only)
void OnboardLogic::CommandCallback(const rpgq_msgs::Command &command)
{
    stateMachine_.CommandCallback(command);
}


// HACK
void OnboardLogic::RunFlightCtrl(Eigen::Vector3d &omega)
{
    double f(0.0);
    Eigen::Vector3d t(0.0, 0.0, 0.0);
    if (stateMachine_.GetState() == OnboardState::THRUST_ANGULAR_RATE)
    {
        // thrust and angular velocity commands
        f = CONSTRAIN(mass_*thrust_, 4.0*fMin_, 4.0*fMax_);

        Eigen::Vector3d omegaErr = omega_ - omega;

        t = J_*Kinv_ang_vel_tau_*omegaErr + omega.cross(J_*omega);
    }

    // compute rotor thrusts
    rotThrustCmd_ = BInv_*Eigen::Vector4d(f,t(0),t(1),t(2));

    // compute rotor commands
    rotThrustCmd_(0)= CONSTRAIN(rotThrustCmd_(0), fMin_, fMax_);
    rotThrustCmd_(1)= CONSTRAIN(rotThrustCmd_(1), fMin_, fMax_);
    rotThrustCmd_(2)= CONSTRAIN(rotThrustCmd_(2), fMin_, fMax_);
    rotThrustCmd_(3)= CONSTRAIN(rotThrustCmd_(3), fMin_, fMax_);
    rotOmegaCmd_ = (rotThrustCmd_*cFInv_).array().sqrt();
}


// controller variables and functions
void OnboardLogic::RunActuatorIdle(void)
{

}

void OnboardLogic::RunOffboardControlThrustAngularRate(void)
{
    // copy command
    rpgq_msgs::Command command = stateMachine_.GetCurrentCommand();

    if (rpgq_msgs::GetThrustAngularVelocityFromCommand(command, thrust_, omega_))
    {
        // TODO (Hack)   
    }
    else
    {
        stateMachine_.SetInternalError();
    } 
}