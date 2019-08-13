#include <rpgq_simulator/implementation/sensors/imu.h>

// ros
#include <sensor_msgs/Imu.h>

// standard library
#include <cmath>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        IMU::IMU(SensorID id, const Node* prevSimNode, Parameters<NUM_PARAMS> params):
            BaseSensor(id, prevSimNode, SECS_TO_USECS(1.0/params.GetDouble(IMU::updateFrequency)))
        {
            // set up publisher
            imuPub_ = pnh_.advertise<sensor_msgs::Imu>(GetSimTreePath(simNode_), 1);

            // general IMU variables
            accFilterState_.setZero();
            gyroFilterState_.setZero();

            // IMU parameters (for MPU 9250)
            B_r_BS_ = Eigen::Vector3d(0.0,0.0,0.0);
            R_SB_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()).inverse().toRotationMatrix();

            double accFilterBandwidth = params.GetDouble(IMU::accFilterBW);
            double gyroFilterBandwidth = params.GetDouble(IMU::gyroFilterBW);
            accFilterGain_ = std::exp(-USECS_TO_SECS(maxSimUSecsInterval_)*accFilterBandwidth);
            gyroFilterGain_ = std::exp(-USECS_TO_SECS(maxSimUSecsInterval_)*gyroFilterBandwidth);

            double accNoisePowerSpectralDensity = params.GetDouble(IMU::accNoisePowerSpectralDensity);
            double gyroNoisePowerSpectralDensity = params.GetDouble(IMU::gyroNoisePowerSpectralDensity);
            accNoise_ = std::normal_distribution<double>(0.0, accNoisePowerSpectralDensity/std::sqrt(USECS_TO_SECS(maxSimUSecsInterval_)));
            gyroNoise_ = std::normal_distribution<double>(0.0, gyroNoisePowerSpectralDensity/std::sqrt(USECS_TO_SECS(maxSimUSecsInterval_)));
            accCov_ = accNoisePowerSpectralDensity*accNoisePowerSpectralDensity/USECS_TO_SECS(maxSimUSecsInterval_);
            gyroCov_ = gyroNoisePowerSpectralDensity*gyroNoisePowerSpectralDensity/USECS_TO_SECS(maxSimUSecsInterval_);

            // auxiliary variables
            g_.setZero();
            g_(2) = 9.81;
        };


        // simulate sensor
        void IMU::RunSimulation_(void)
        {
            // W - world frame
            // B - vehicle frame
            // S - sensor frame

            // compute acceleration of IMU
            Eigen::Matrix3d R_BW = GetQuat_().inverse().toRotationMatrix();
            Eigen::Vector3d B_omega_WB = GetOmega_();
            Eigen::Vector3d B_psi_WB = GetPsi_();

            Eigen::Vector3d B_acc_S = R_BW*(GetAcc_() + g_) + B_omega_WB.cross(B_omega_WB.cross(B_r_BS_)) + B_psi_WB.cross(B_r_BS_); // TODO, check
            Eigen::Vector3d S_acc_S = R_SB_*B_acc_S;

            // compute angular velocity of IMU
            Eigen::Vector3d S_omega_WB = R_SB_*GetOmega_();

            // add noise
            S_acc_S(0) += accNoise_(gen_);
            S_acc_S(1) += accNoise_(gen_);
            S_acc_S(2) += accNoise_(gen_);

            S_omega_WB(0) += gyroNoise_(gen_);
            S_omega_WB(1) += gyroNoise_(gen_);
            S_omega_WB(2) += gyroNoise_(gen_);

            // apply low pass filter
            accFilterState_ = (1.0 - accFilterGain_) * S_acc_S + accFilterGain_*accFilterState_;
            gyroFilterState_ = (1.0 - gyroFilterGain_) * S_omega_WB + gyroFilterGain_*gyroFilterState_;

            // publish IMU data
            sensor_msgs::Imu imu_msg;

            imu_msg.header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());
            imu_msg.header.frame_id = "map";

            // orientation
            imu_msg.orientation.w = 1.0;
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;

            imu_msg.orientation_covariance[0] = -1.0;

            // angular velocity
            imu_msg.angular_velocity.x = gyroFilterState_(0);
            imu_msg.angular_velocity.y = gyroFilterState_(1);
            imu_msg.angular_velocity.z = gyroFilterState_(2);

            imu_msg.angular_velocity_covariance[0] = 0.0;
            imu_msg.angular_velocity_covariance[3] = 0.0;
            imu_msg.angular_velocity_covariance[6] = 0.0;

            // linear acceleration
            imu_msg.linear_acceleration.x = accFilterState_(0);
            imu_msg.linear_acceleration.y = accFilterState_(1);
            imu_msg.linear_acceleration.z = accFilterState_(2);

            imu_msg.linear_acceleration_covariance[0] = 0.0;
            imu_msg.linear_acceleration_covariance[3] = 0.0;
            imu_msg.linear_acceleration_covariance[6] = 0.0;

            imuPub_.publish(imu_msg);
        };

    } // namespace Simulator
} // namespace RPGQ