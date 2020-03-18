#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_vehicle.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/implementation/actuators/motor.h>
#include <rpgq_simulator/implementation/sensors/imu.h>

// rpgq rpgq_common
#include <rpgq_common/quadrotor_id.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        QuadrotorVehicle::QuadrotorVehicle(ObjectID id, const Node* prevSimNode,
          USecs maxSimUSecsInterval) : DirectlyControlledObject(id, prevSimNode, maxSimUSecsInterval)
        {
            // onboard code
            // simulate every 1000us -> 1kHz / 5000 us -> 200Hz
            commandBridgeClientPtr_ = std::make_shared<CommandBridgeClient>(QuadrotorNameToID(id), ID::OnboardCode::CommandBridgeClient, &simNode_, 1000); 
            AddOnboardCode(commandBridgeClientPtr_);
            // simulate every 1000us -> 1kHz / 5000 us -> 200Hz
            onboardLogicPtr_ = std::make_shared<QuadrotorOnboardLogic>(QuadrotorNameToID(id), ID::OnboardCode::OnboardLogic, &simNode_, 1000);
            AddOnboardCode(onboardLogicPtr_);

            // set callbacks manually instead of using ROS publisher/subscriber for onboard code simulation
            commandBridgeClientPtr_->AddCommandCallback(std::bind(&QuadrotorOnboardLogic::CommandCallback, onboardLogicPtr_, std::placeholders::_1));

            // actuators
            AddActuator(std::make_shared<Motor>(ID::Actuator::Motor1, &simNode_, 10000)); // maximum simulation step size 10'000us -> 10ms
            AddActuator(std::make_shared<Motor>(ID::Actuator::Motor2, &simNode_, 10000));
            AddActuator(std::make_shared<Motor>(ID::Actuator::Motor3, &simNode_, 10000));
            AddActuator(std::make_shared<Motor>(ID::Actuator::Motor4, &simNode_, 10000));

            // dynamics
            dynamicsPtr_ = std::make_shared<QuadrotorDynamics>(ID::Dynamics::Quadrotor, &simNode_, 50000); // maximum simulation step size 50'000us -> 50ms
            AddDynamics(dynamicsPtr_); 

            // sensors
            std::shared_ptr<IMU> imu = std::make_shared<IMU>(ID::Sensor::IMU, &simNode_); // default 1kHz -> simulate every 1000us
            imu->SetCallbackFunctions(std::bind(&QuadrotorDynamics::GetAcc, dynamicsPtr_),
                                    std::bind(&QuadrotorDynamics::GetQuat, dynamicsPtr_),
                                    std::bind(&QuadrotorDynamics::GetOmega, dynamicsPtr_),
                                    std::bind(&QuadrotorDynamics::GetPsi, dynamicsPtr_));
            AddSensor(imu);
        }


        // mediators
        void QuadrotorVehicle::OnboardCodeMediator(void)
        {
        }

        void QuadrotorVehicle::ActuatorMediator(void)
        {
            /// (HACK)
            Eigen::Vector3d omega = dynamicsPtr_->GetOmega();
            onboardLogicPtr_->RunFlightCtrl(omega);

            // get rotor omega commands
            Eigen::Vector4d omegaDes = onboardLogicPtr_->GetRotOmegaCmd();
            
            // set rotor omega commands
            std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor1])->SetOmegaDes(omegaDes(0));
            std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor2])->SetOmegaDes(omegaDes(1));
            std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor3])->SetOmegaDes(omegaDes(2));
            std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor4])->SetOmegaDes(omegaDes(3));
        }

        void QuadrotorVehicle::DynamicsMediator(void)
        {
            // get rotor thrusts
            Eigen::Vector4d omega;
            omega(0) = std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor1])->GetOmega();
            omega(1) = std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor2])->GetOmega();
            omega(2) = std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor3])->GetOmega();
            omega(3) = std::static_pointer_cast<Motor>(actuators_[ID::Actuator::Motor4])->GetOmega();

            // set rotor thrusts
            dynamicsPtr_->SetRotorOmega(omega);
        }

    } // namespace Simulator
} // namespace RPGQ