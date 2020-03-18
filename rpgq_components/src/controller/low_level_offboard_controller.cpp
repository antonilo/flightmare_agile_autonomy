#include <rpgq_components/controller/low_level_offboard_controller.h>

// rpgq rpgq_common
#include <rpgq_common/rpgq.h>

namespace RPGQ
{
    // constructor 
    LowLevelOffboardController::LowLevelOffboardController(QuadrotorID quadrotorID,
      std::shared_ptr<Timer> timer, Parameters<NUM_PARAMS> params):
        quadrotorID_(quadrotorID),
        I_eZ_I_(0.0,0.0,1.0),
        I_g_(0.0,0.0,9.81)
    {
        // desired control variables
        cmdLevel_ = CommandLevel::POS_CMD;
        posDes_.setZero();
        velDes_.setZero();
        accDes_.setZero();
        thrustDes_ = 0.0;
        yawDes_ = 0.0;
        omegaDes_.setZero();

        // low level offboard controller variables
        params_ = params;
        posErrInt_.setZero();
        timerInt_ = ExtTimer(timer);
        thrustCmd_ = 0.0;
        omegaCmd_.setZero();
    }


    // public run functions
    void LowLevelOffboardController::Run(const State6DoF &state)
    {
        if (cmdLevel_ < CommandLevel::THRUST_ANG_VEL_CMD)
        {
            /**********************
            * position controller *
            **********************/
            Eigen::Vector3d posErr = posDes_ - state.pos;
            Eigen::Vector3d velErr = velDes_ - state.vel;

            Eigen::Vector3d accCmd(I_g_);
            if (cmdLevel_ == CommandLevel::POS_INT_CMD)
            {
                // update position error integral
                if (timerInt_.ElapsedSeconds() > params_.GetDouble(maxIntStepSize))
                {
                    timerInt_.Reset();
                }
                posErrInt_ += posErr*timerInt_.ElapsedSeconds();
                timerInt_.Reset();

                // constrain position error integral
                posErrInt_.x() = CONSTRAIN(posErrInt_.x(), -params_.GetDouble(maxPosErrIntXY), params_.GetDouble(maxPosErrIntXY));
                posErrInt_.y() = CONSTRAIN(posErrInt_.y(), -params_.GetDouble(maxPosErrIntXY), params_.GetDouble(maxPosErrIntXY));
                posErrInt_.z() = CONSTRAIN(posErrInt_.z(), -params_.GetDouble(maxPosErrIntZ), params_.GetDouble(maxPosErrIntZ));

                double gainIXY = 1.0/(params_.GetDouble(tauPosXY)*params_.GetDouble(tauPosXY)*params_.GetDouble(tauPosIntXY));
                double gainIZ = 1.0/(params_.GetDouble(tauPosZ)*params_.GetDouble(tauPosZ)*params_.GetDouble(tauPosIntZ));
               
                accCmd.x() = accCmd.x() + gainIXY*posErrInt_.x();
                accCmd.y() = accCmd.y() + gainIXY*posErrInt_.y();
                accCmd.z() = accCmd.z() + gainIZ*posErrInt_.z();
            }
            if (cmdLevel_ <= CommandLevel::POS_CMD)
            {
                double gainPXY = 1.0/(params_.GetDouble(tauPosXY)*params_.GetDouble(tauPosXY));
                double gainPZ = 1.0/(params_.GetDouble(tauPosZ)*params_.GetDouble(tauPosZ));
                if (cmdLevel_ == CommandLevel::POS_INT_CMD)
                {
                    gainPXY += 2.0*params_.GetDouble(zetaPosXY)/(params_.GetDouble(tauPosXY)*params_.GetDouble(tauPosIntXY));
                    gainPZ += 2.0*params_.GetDouble(zetaPosZ)/(params_.GetDouble(tauPosZ)*params_.GetDouble(tauPosIntZ));
                }

                accCmd.x() = accCmd.x() + gainPXY*posErr.x();
                accCmd.y() = accCmd.y() + gainPXY*posErr.y();
                accCmd.z() = accCmd.z() + gainPZ*posErr.z();
            }
            if (cmdLevel_ <= CommandLevel::VEL_CMD)
            {
                double gainDXY = 2.0*params_.GetDouble(zetaPosXY)/params_.GetDouble(tauPosXY);
                double gainDZ = 2.0*params_.GetDouble(zetaPosZ)/params_.GetDouble(tauPosZ);
                if (cmdLevel_ == CommandLevel::POS_INT_CMD)
                {
                    gainDXY += 1.0/params_.GetDouble(tauPosIntXY);
                    gainDZ += 1.0/params_.GetDouble(tauPosIntZ);
                }

                accCmd.x() = accCmd.x() + gainDXY*velErr.x();
                accCmd.y() = accCmd.y() + gainDXY*velErr.y();
                accCmd.z() = accCmd.z() + gainDZ*velErr.z();
            }
            if (cmdLevel_ <= CommandLevel::ACC_CMD)
            {
                accCmd += accDes_;
            }

            // compute thrust command
            thrustCmd_ = accCmd.z()/state.quat.toRotationMatrix()(2,2);

            /**********************
            * attitude controller *
            **********************/
            Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(I_eZ_I_, accCmd)*Eigen::Quaterniond(std::cos(0.5*yawDes_),0.0,0.0,std::sin(0.5*yawDes_));
            Eigen::Quaterniond quatErr = state.quat.inverse()*quatDes;
            if (quatErr.w() < 0.0)
            {
                quatErr.coeffs() *= -1.0;
            }

            // compute reduced attitude and yaw error
            double q0q3sqrt = std::sqrt(quatErr.w()*quatErr.w() + quatErr.z()*quatErr.z());
            Eigen::Quaterniond quatErrRed, quatErrYaw;
            if (q0q3sqrt < 1e-6)
            {
                double omegaXYSqrt = std::sqrt(state.omega.x()*state.omega.x() + state.omega.y()*state.omega.y());
                if (omegaXYSqrt < 1e-1)
                {
                    quatErrRed = quatErr;
                }
                else
                {
                    quatErrRed.w() = 0.0;
                    quatErrRed.x() = state.omega.x();
                    quatErrRed.y() = state.omega.y();
                    quatErrRed.z() = 0.0;
                    quatErrRed.normalize();
                }

                quatErrYaw = quatErrRed.inverse()*quatErr;
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

            // ensure error is smaller than pi
            if (quatErr.w() < 0.0)
            {
                quatErrYaw.coeffs() *= -1.0;
            }

            // nonliner attitude control law
            Eigen::Vector3d omegaDesTilde = omegaDes_; // TODO
            omegaCmd_.x() = 2.0/params_.GetDouble(tauAttXY)*quatErrRed.x() + omegaDesTilde.x();
            omegaCmd_.y() = 2.0/params_.GetDouble(tauAttXY)*quatErrRed.y() + omegaDesTilde.y();
            omegaCmd_.z() = 2.0/params_.GetDouble(tauAttZ)*quatErrYaw.z() + omegaDesTilde.z();

        } else
        {
            // only feed-forward
            thrustCmd_ = thrustDes_;
            omegaCmd_ = omegaDes_;
        }

        // constrain commands
        thrustCmd_ = CONSTRAIN(thrustCmd_, params_.GetDouble(minThrust), params_.GetDouble(maxThrust));
        omegaCmd_.x() = CONSTRAIN(omegaCmd_.x(), -params_.GetDouble(maxOmegaXY), params_.GetDouble(maxOmegaXY));
        omegaCmd_.y() = CONSTRAIN(omegaCmd_.y(), -params_.GetDouble(maxOmegaXY), params_.GetDouble(maxOmegaXY));
        omegaCmd_.z() = CONSTRAIN(omegaCmd_.z(), -params_.GetDouble(maxOmegaZ), params_.GetDouble(maxOmegaZ));
    }


    // public get functions
    rpgq_msgs::Command LowLevelOffboardController::GetCommand(void)
    {
        return rpgq_msgs::CreateThrustAngularVelocityCommand(quadrotorID_, thrustCmd_, omegaCmd_);
    }


    // public set functions
    void LowLevelOffboardController::SetCommandLevel(CommandLevel cmdLevel)
    {
        if (cmdLevel_ == cmdLevel) return;
        //
        cmdLevel_ = cmdLevel;
        switch (cmdLevel)
        {
            case CommandLevel::POS_INT_CMD:
            {
                // resetting position error integral
                posErrInt_.setZero();
                break;
            }

            case CommandLevel::POS_CMD:
            {
                // nothing
                break;
            }

            case CommandLevel::VEL_CMD:
            {
                // nothing
                break;
            }

            case CommandLevel::ACC_CMD:
            {
                // nothing
                break;
            }

            case CommandLevel::THRUST_ANG_VEL_CMD:
            {
                // nothing
                break;
            }

            case CommandLevel::FEED_FORWARD:
            {
                // nothing
                break;   
            }
            default:
            std::printf("[Controller ERROR] Ignoring invalid command level!\n");
        }
    }

} // namespace RPGQ