#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_onboard_code.h>

// rpgq rpgq_common
#include <rpgq_common/rpgq.h>

// standard library
#include <math.h>
#include <iostream>

namespace RPGQ
{
    namespace Simulator
    {
        /*************************
         * Command Bridge Client *
         *************************/
        // constructor 
        CommandBridgeClient::CommandBridgeClient(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            BaseOnboardCode(id, prevSimNode, maxSimUSecsInterval),
            quadID_(quadID)
        {
            // command bridge client variables
            newCommandSet_ = false;
            command_ = rpgq_msgs::CreateIdleCommand(RPGQ::QUAD_INVALID);
            commandPub_ = pnh_.advertise<rpgq_msgs::Command>(GetSimTreePath(simNode_) + "/command", 1);
        };


        // public set functions
        void CommandBridgeClient::AddCommandCallback(CommandCallback_t callback)
        {
            commandCallbacks_.push_back(callback);
        };

        void CommandBridgeClient::SetCommandSet(const rpgq_msgs::CommandSet &commandSet)
        {
            newCommandSet_ = true;
            commandSet_ = commandSet;
        };


        // simulate onboard code
        void CommandBridgeClient::RunSimulation_(USecs dt)
        {
            // simulate command bridge client
            if (newCommandSet_)
            {
                newCommandSet_ = false;
                
                rpgq_msgs::GetCommandFromCommandSet(commandSet_, quadID_, command_);
                commandPub_.publish(command_);

                for (const auto callback : commandCallbacks_)
                {
                    callback(command_);
                }
            }
        };


        /*****************
         * Onboard Logic *
         *****************/
        // constructor
        QuadrotorOnboardLogic::QuadrotorOnboardLogic(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            BaseOnboardCode(id, prevSimNode, maxSimUSecsInterval),
            quadID_(quadID)
        {
            // onboard logic variables
            onboardLogic_ = std::make_shared<Onboard::OnboardLogic>(quadID, maxSimUSecsInterval);
        };


        // public callback functions
        void QuadrotorOnboardLogic::CommandCallback(const rpgq_msgs::Command &command)
        {
            onboardLogic_->CommandCallback(command);
        };


        // simulate onboard code
        void QuadrotorOnboardLogic::RunSimulation_(USecs dt)
        {
            // simulate onboard logic
            onboardLogic_->Run();
        };




        /*QuadrotorOnboardCode::QuadrotorOnboardCode(QuadrotorID quadID, OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            BaseOnboardCode(id, prevSimNode, maxSimUSecsInterval),
            quadID_(quadID)
        {

            // controller parameters
            // position controller
            Kinv_tau_.setZero();
            Kinv_tau_(0,0) = 1.0/0.35;
            Kinv_tau_(1,1) = 1.0/0.35;
            Kinv_tau_(2,2) = 1.0/0.25;

            K_zeta_.setZero();
            K_zeta_(0,0) = 0.95;
            K_zeta_(1,1) = 0.95;
            K_zeta_(2,2) = 0.8;

            // attitude controller
            k_p_xy_ = 3.286;
            k_p_z_ = 0.197;

            K_D_.setZero();
            K_D_(0,0) = 0.23;
            K_D_(1,1) = 0.23;
            K_D_(2,2) = 0.046;

            // angular velocity controller
            Kinv_ang_vel_tau_.setZero();
            Kinv_ang_vel_tau_(0,0) = 1.0/0.01;
            Kinv_ang_vel_tau_(1,1) = 1.0/0.01;
            Kinv_ang_vel_tau_(2,2) = 1.0/0.1;

            // others
            mass_ = 0.523;
            J_.setZero();
            J_(0,0) = 0.0023; J_(1,1) = 0.0023; J_(2,2) = 0.0046;
            double l = 0.17;
            double kappa = 0.016;

            // thrust / torque map parameters
            cFInv_ = 1.0/6.4127e-6;
            fMin_ = 0.15; // make sure motors can actually spin slow/fast enough
            fMax_ = 3.5;

            Eigen::Matrix4d B;
            B << 1, 1, 1, 1,
                0, l, 0, -l,
                -l, 0, l, 0,
                kappa, -kappa, kappa, -kappa;
            BInv_ = B.inverse();

            // controller variables
            u_.setZero();
            rotorOmegaCmd_.setZero();
            
            // reset desired state
            posDes_.setZero();
            velDes_.setZero();
            accDes_.setZero();
            normThrustDes_ = 0.0;

            yawDes_ = 0.0;
            omegaDes_.setZero();

            // reset state
            state_.setZero();
            state_(9) = 1.0;

            // auxiliary variables
            e_z_.setZero();
            e_z_(2) = 1.0;

            g_.setZero();
            g_(2) = 9.81;

        };



        // simulate onboard code
        void QuadrotorOnboardCode::RunSimulation_(USecs dt)
        {
            // simulate command bridge client app
            if (newCmdSetData_)
            {
                rpgq_msgs::Command cmd;
                if (rpgq_msgs::GetCommandFromCommandSet(cmdSet_, quadID_, cmd))
                {
                    cmdMsgPub_.publish(cmd);
                    ros::spinOnce();
                }

                newCmdSetData_ = false;
            }

            // simulate onboard logic app
            onboardLogic_.Run();




            // TODO


            /*if (cmdLevel_ < CommandLevel::THRUST_ANG_VEL_CMD)
            {
                // position controller
                Eigen::Vector3d pos_err = posDes_ - state_.segment(0,3);
                Eigen::Vector3d vel_err = velDes_ - state_.segment(3,3);
                Eigen::Vector3d acc(0.0,0.0,0.0);
                if (cmdLevel_ == CommandLevel::POS_CMD)
                {
                    acc += Kinv_tau_*Kinv_tau_*pos_err;
                }
                if (cmdLevel_ <= CommandLevel::VEL_CMD)
                {
                    acc += 2.0*K_zeta_*Kinv_tau_*vel_err;
                }
                if (cmdLevel_ <= CommandLevel::ACC_CMD)
                {
                    acc += accDes_;
                }

                // attitude controller
                Eigen::Quaterniond quat(&state_(6));

                Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(e_z_, acc + g_)*Eigen::Quaterniond(std::cos(0.5*yawDes_),0.0,0.0,std::sin(0.5*yawDes_));
                Eigen::Quaterniond quatErr = quat.inverse()*quatDes;

                double q0q3sqrt = std::sqrt(quatErr.w()*quatErr.w() + quatErr.z()*quatErr.z());
                Eigen::Quaterniond quatErrRed, quatErrYaw;

                if (q0q3sqrt < 1e-6)
                {
                    double omegaXYSqrt = std::sqrt(state_(10)*state_(10) + state_(11)*state_(11));
                    if (omegaXYSqrt < 1e-1)
                    {
                        quatErrRed = quatErr;
                    }
                    else
                    {
                        quatErrRed.w() = 0.0;
                        quatErrRed.x() = state_(10);
                        quatErrRed.y() = state_(11);
                        quatErrRed.z() = 0.0;
                        quatErrRed.normalize();
                    }

                    quatErrYaw = quatErr*quatErrRed.inverse();
                }
                else
                {
                    quatErrRed = Eigen::Quaterniond(quatErr.w()*quatErr.w() + quatErr.z()*quatErr.z(),
                                            quatErr.w()*quatErr.x() - quatErr.y()*quatErr.z(),
                                            quatErr.w()*quatErr.y() + quatErr.x()*quatErr.z(),
                                            0.0).coeffs()*1.0/q0q3sqrt;

                    quatErrYaw = Eigen::Quaterniond(quatErr.w(),
                                            0.0,
                                            0.0,
                                            quatErr.z()).coeffs()*1.0/q0q3sqrt;
                }

                Eigen::Vector3d omega = state_.segment(10,3);
                if (quatErr.w() < 0.0)
                {
                    quatErrYaw.coeffs() *= -1.0;
                }

                Eigen::Vector3d omegaErr = omegaDes_ - omega;

                Eigen::Vector3d t_ff = omega.cross(J_*omega); // TODO: add missing terms

                Eigen::Vector3d t = Eigen::Vector3d(quatErrRed.x()*k_p_xy_,
                                                    quatErrRed.y()*k_p_xy_,
                                                    quatErrYaw.z()*k_p_z_) + K_D_*omegaErr + t_ff;

                // compute rotor thrusts
                double f = CONSTRAIN(mass_*(acc(2) + g_(2))/quat.toRotationMatrix()(2,2), 4.0*fMin_, 4.0*fMax_);
                u_ = BInv_*Eigen::Vector4d(f,t(0),t(1),t(2));

            }
            else
            {
                // thrust and angular velocity commands
                double f = CONSTRAIN(mass_*normThrustDes_, 4.0*fMin_, 4.0*fMax_);

                Eigen::Vector3d omega = state_.segment(10,3);
                Eigen::Vector3d omegaErr = omegaDes_ - omega;

                Eigen::Vector3d t = J_*Kinv_ang_vel_tau_*omegaErr + omega.cross(J_*omega);

                // compute rotor thrusts
                u_ = BInv_*Eigen::Vector4d(f,t(0),t(1),t(2));
            }*/
/*
            // compute rotor commands
            u_(0)= CONSTRAIN(u_(0), fMin_, fMax_);
            u_(1)= CONSTRAIN(u_(1), fMin_, fMax_);
            u_(2)= CONSTRAIN(u_(2), fMin_, fMax_);
            u_(3)= CONSTRAIN(u_(3), fMin_, fMax_);
            rotorOmegaCmd_ = (u_*cFInv_).array().sqrt();

        }*/

    } // namespace Simulator
} // namespace RPGQ