#include <rpgq_components/estimator/optitrack_estimator.h>

// rpgq rpgq_common
#include <rpgq_common/rpgq.h>
#include <rpgq_common/msgs/command_msg_helper.h>

// standard library
#include <limits>

namespace RPGQ
{
    // constructor
    OptitrackEstimator::OptitrackEstimator(QuadrotorID quadrotorID, RPGQ::Parameters<NUM_PARAMS> params):
        quadrotorID_(quadrotorID)
    {
        // general estimator variables
        initialized_ = false;

        // estimator parameters and variables
        params_ = params;
        Reset();

        // auxiliary variables
        // already done when calling Reset()
    }


    // public estimator functions
    void OptitrackEstimator::MeasurementUpdate(const geometry_msgs::PoseStampedConstPtr &measurement)
    {
        // verify optitrack data
        if ((measurement->header.stamp - lastUpdate_) > ros::Duration(params_.GetDouble(blackoutLength)))
        {
            // long blackout
            std::printf("[Estimator INFO] Long optitrack blackout! Resetting estimator...\n\n");
            initialized_ = false;
        }
        else if (measurement->header.seq < lastSeq_)
        {
            // wrong order
            const uint32_t wrapDeltaSeq = (std::numeric_limits<uint32_t>::max() - lastSeq_) + measurement->header.seq;
            const uint32_t nonWrapDeltaSeq = lastSeq_ - measurement->header.seq;

            if (wrapDeltaSeq < nonWrapDeltaSeq)
            {
                // appears to be wrap around, reset
                std::printf("[Estimator INFO] Estimator time wrap! Resetting estimator...\n\n");
                initialized_ = false;
            }
            else
            {
                // appears to be out of order, ignore it
                return;
            }
        }

        // update prior
        const double dtPrior = (measurement->header.stamp - lastUpdate_).toSec();
        estimatedState_ = UpdatePrior(dtPrior);

        // initialize estimator if necessary
        if (initialized_)
        {
            // update posterior
            const double dtPosterior = (measurement->header.stamp - lastMeasurementUpdate_).toSec();
            estimatedState_ = UpdatePosterior(dtPosterior, measurement);
        }
        else
        {
            // initialize estimator
            Reset();
            initialized_ = true;

            // update posterior
            estimatedState_ = UpdatePosterior(0.0, measurement, true);
        }

        lastUpdate_ = measurement->header.stamp;
        lastMeasurementUpdate_ = measurement->header.stamp;

        // clean command queue from all commands
        commandQueueMutex_.lock();
        while (commandQueue_.size() > 1)
        {
            // delete all commands up to the last one before the current frame
            if (commandQueue_.at(1).first < lastUpdate_)
            {
                commandQueue_.pop_front();
            }
            else
            {
                break;
            }
        }
        commandQueueMutex_.unlock();
    }

    void OptitrackEstimator::FeedCommandQueue(const rpgq_msgs::CommandSet &commandSet)
    {
        rpgq_msgs::Command command;
        if (!rpgq_msgs::GetCommandFromCommandSet(commandSet, quadrotorID_, command)) return;

        if (command.type == rpgq_msgs::CommandType::THRUST_ANGULAR_VELOCITY)
        {
            commandQueueMutex_.lock();
            commandQueue_.push_back(std::pair<ros::Time, rpgq_msgs::Command>(ros::Time::now() + ros::Duration(params_.GetDouble(systemDelay)), command));
            commandQueueMutex_.unlock();
        }
    }

    void OptitrackEstimator::Predict(double dt)
    {
        predictedState_ = UpdatePrior(dt);
    }


    // estimator functions
    State6DoF OptitrackEstimator::UpdatePrior(const double dt)
    {
        ros::Time startTime = lastUpdate_;
        double remainingTimeToSimulate = dt;

        State6DoF priorState = estimatedState_;
        if (!commandQueue_.empty())
        {
            commandQueueMutex_.lock();

            // simulate until first command
            double updateStep = CONSTRAIN((commandQueue_.front().first - startTime).toSec(), 0.0, remainingTimeToSimulate);

            // no commands in queue, simulate quadrotor assuming constant linear and angular velocity
            while (updateStep > 1e-6)
            {
                // divide update step into smaller steps for increased accuracy
                const double smallUpdateStep = std::min(updateStep, params_.GetDouble(maxSimStep));

                priorState = IntegrateSystemDynamicsSimple(smallUpdateStep, priorState);
                updateStep -= smallUpdateStep;
            }

            // use commands in queue to simulate quadrotor
            for (std::deque<std::pair<ros::Time, rpgq_msgs::Command>>::iterator it = commandQueue_.begin(); it != commandQueue_.end(); it++)
            {
                if (it + 1 == commandQueue_.end())
                {
                    // reached last command in queue, simulate with this command until end
                    updateStep = remainingTimeToSimulate;
                }
                else
                {
                    // simulate until next command
                    updateStep = CONSTRAIN(((it + 1)->first - startTime).toSec(), 0.0, remainingTimeToSimulate);
                }

                // push startTime and remainingTimeToSimulate forward
                startTime += ros::Duration(updateStep);
                remainingTimeToSimulate -= updateStep;

                // simulate quadrotor
                while (updateStep > 1e-6)
                {
                    // divide update step into smaller steps for increased accuracy
                    const double smallUpdateStep = std::min(updateStep, params_.GetDouble(maxSimStep));

                    priorState = IntegrateSystemDynamics(smallUpdateStep, priorState, it->second);
                    updateStep -= smallUpdateStep;
                }
            }

            commandQueueMutex_.unlock();
        }
        else
        {
            // no commands in queue, simulate quadrotor assuming constant linear and angular velocity
            
            double updateStep = remainingTimeToSimulate;
            while (updateStep > 1e-6)
            {
                // divide update step into smaller steps for increased accuracy
                const double smallUpdateStep = std::min(updateStep, params_.GetDouble(maxSimStep));

                priorState = IntegrateSystemDynamicsSimple(smallUpdateStep, priorState);
                updateStep -= smallUpdateStep;
            }
        }

        return priorState;
    }

    State6DoF OptitrackEstimator::UpdatePosterior(const double dt,
        const geometry_msgs::PoseStampedConstPtr &measurement, bool initialize)
    {
        // compute state from measurement
        State6DoF measuredState;
        measuredState.pos = Eigen::Vector3d(measurement->pose.position.x,
                                            measurement->pose.position.y,
                                            measurement->pose.position.z);
        measuredState.quat = Eigen::Quaterniond(measurement->pose.orientation.w,
                                                measurement->pose.orientation.x,
                                                measurement->pose.orientation.y,
                                                measurement->pose.orientation.z);
        if (!initialize)
        {
            measuredState.vel = (measuredState.pos - measuredState_.pos)/dt;
            measuredState.omega = QuaternionDifferenceToOmega(measuredState.quat, measuredState_.quat, dt);
        }
        else
        {
            measuredState.vel.setZero();
            measuredState.omega.setZero();
        }

        // store measured state
        measuredState_ = measuredState;

        // compute filter time constant
        double c_pos = std::exp(-dt/params_.GetDouble(tauPos));
        double c_vel = std::exp(-dt/params_.GetDouble(tauVel));
        double c_quat = std::exp(-dt/params_.GetDouble(tauQuat));
        double c_omega = std::exp(-dt/params_.GetDouble(tauOmega));

        // compute innovation
        State6DoF innovation;
        innovation.pos = measuredState.pos - estimatedState_.pos;
        innovation.vel = measuredState.vel - estimatedState_.vel;
        innovation.quat = estimatedState_.quat.inverse()*measuredState.quat;
        if (innovation.quat.w() < 0)
        {   
            // ensure rotation is smaller than pi
            innovation.quat.coeffs() = -innovation.quat.coeffs();
        }
        innovation.omega = measuredState.omega - estimatedState_.omega;

        // compute posterior state
        State6DoF posteriorState;
        if (!initialize)
        {
            posteriorState.pos = estimatedState_.pos + (1.0 - c_pos)*innovation.pos;
            posteriorState.vel = estimatedState_.vel + (1.0 - c_vel)*innovation.vel;
            Eigen::AngleAxisd angleAxis(innovation.quat);
            angleAxis.angle() = (1.0 - c_quat)*angleAxis.angle();
            posteriorState.quat = estimatedState_.quat*Eigen::Quaterniond(angleAxis);
            posteriorState.omega = estimatedState_.omega + (1.0 - c_omega)*innovation.omega;
        }
        else
        {
            // set posterior state to measured state when initializing
            posteriorState = measuredState;
        }

        return posteriorState;
    }

    void OptitrackEstimator::Reset(void)
    {
        // need to initialized estimator again
        initialized_ = false;

        // reset all states
        estimatedState_.pos.setZero();
        estimatedState_.vel.setZero();
        estimatedState_.quat.setIdentity();
        estimatedState_.omega.setZero();

        predictedState_ = estimatedState_;
        measuredState_ = estimatedState_;

        // reset auxiliary variables
        thrust_ = 0.0;
        lastMeasurementUpdate_ = ros::Time::now();
        lastUpdate_ = ros::Time::now();
        lastSeq_ = 0;
    }


    // quadrotor dynamics
    State6DoF OptitrackEstimator::IntegrateSystemDynamics(const double dt, State6DoF &state, const rpgq_msgs::Command &command)
    {
        // first-order dynamics on thrust and angular velocity
        double c_thrust = std::exp(-dt/params_.GetDouble(tauThrustCmd));
        double c_omega = std::exp(-dt/params_.GetDouble(tauOmegaCmd));

        double thrustCmd;
        Eigen::Vector3d omegaCmd;
        if (!rpgq_msgs::GetThrustAngularVelocityFromCommand(command, thrustCmd, omegaCmd))
        {
            std::printf("[Optitrack Estimator] Error: Wrong command type in IntegrateSystemDynamics()!\n");
        }

        State6DoF newState;

        newState.omega = c_omega*state.omega + (1.0 - c_omega)*omegaCmd;
        newState.quat = state.quat;
        newState.quat.coeffs() += (state.quat*Eigen::Quaterniond(0.0,0.5*newState.omega.x()*dt,0.5*newState.omega.y()*dt,0.5*newState.omega.z()*dt)).coeffs();
        newState.quat.normalize();

        thrust_ = c_thrust*thrust_ + (1.0 - c_thrust)*thrustCmd;
        Eigen::Vector3d acc = state.quat*Eigen::Vector3d(0.0,0.0,thrust_) + Eigen::Vector3d(0.0,0.0,-9.81);
        newState.vel = state.vel + acc*dt;
        newState.pos = state.pos + state.vel*dt + 0.5*acc*dt*dt;

        return newState;
    }


    State6DoF OptitrackEstimator::IntegrateSystemDynamicsSimple(const double dt, const State6DoF &state)
    {
        State6DoF newState;

        newState.omega = state.omega;
        newState.quat = state.quat;
        newState.quat.coeffs() += (state.quat*Eigen::Quaterniond(0.0,0.5*state.omega.x()*dt,0.5*state.omega.y()*dt,0.5*state.omega.z()*dt)).coeffs();
        newState.quat.normalize();

        newState.vel = state.vel;
        newState.pos = state.pos + dt*state.vel;

        return newState;
    }


    // auxiliary functions
    Eigen::Vector3d OptitrackEstimator::QuaternionDifferenceToOmega(const Eigen::Quaterniond &newQuat, const Eigen::Quaterniond &oldQuat, const double dt)
    {
        // newQuat = oldQuat + 0.5*oldQuat*omegaQuat*dt
        Eigen::Quaterniond temp;
        temp.coeffs() = (newQuat.coeffs() - oldQuat.coeffs())*(2.0/dt);
        Eigen::Quaterniond omegaQuat = oldQuat.inverse()*temp;

        return Eigen::Vector3d(temp.x(), temp.y(), temp.z());
    }

} // namespace RPGQ