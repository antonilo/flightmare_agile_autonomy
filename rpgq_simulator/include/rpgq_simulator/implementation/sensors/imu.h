#pragma once

// rpgq rpgq_common
#include <rpgq_common/parameters/parameters.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/sensors/base_sensor.h>

// standard library
#include <random>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        namespace IMUTypes
        {
            typedef std::function<Eigen::Vector3d()> GetAcc_t;
            typedef std::function<Eigen::Quaterniond()> GetQuat_t;
            typedef std::function<Eigen::Vector3d()> GetOmega_t;
            typedef std::function<Eigen::Vector3d()> GetPsi_t;
        }

        class IMU : public BaseSensor
        {
        public:
            // parameters
            enum
            {
                updateFrequency = 0, // [Hz]
                accFilterBW = 1, // [Hz]
                gyroFilterBW = 2, // [Hz]
                accNoisePowerSpectralDensity = 3, // [(m/s²) / sqrt(Hz)]
                gyroNoisePowerSpectralDensity = 4, // [(rad/s) / sqrt(Hz)]
                NUM_PARAMS
            };

            // constructor & destructor
            IMU(SensorID id, const Node* prevSimNode,
              Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("simulator/sensors/imu"));
            ~IMU() {};

            // public set functions
            void SetCallbackFunctions(IMUTypes::GetAcc_t GetAcc,
                                    IMUTypes::GetQuat_t GetQuat,
                                    IMUTypes::GetOmega_t GetOmega,
                                    IMUTypes::GetPsi_t GetPsi)
            {
                GetAcc_ = GetAcc;
                GetQuat_ = GetQuat;
                GetOmega_ = GetOmega;
                GetPsi_ = GetPsi;
            };
            void SetRelPose(Eigen::Vector3d &B_r_BS, Eigen::Matrix3d &R_SB)
            {
                B_r_BS_ = B_r_BS;
                R_SB_ = R_SB;
            }


        private:
            // simulate sensor
            void RunSimulation_(void);
            USecs UpdateSamplingInterval(void) {};

            // publishers
            ros::Publisher imuPub_;
            
            // general IMU variables
            std::default_random_engine gen_;
            Eigen::Vector3d accFilterState_, gyroFilterState_;

            // IMU parameters
            Eigen::Vector3d B_r_BS_;
            Eigen::Matrix3d R_SB_;
            double accFilterGain_, gyroFilterGain_;
            std::normal_distribution<double> accNoise_, gyroNoise_;
            double accCov_, gyroCov_;

            // IMU "callback" functions
            IMUTypes::GetAcc_t GetAcc_;
            IMUTypes::GetQuat_t GetQuat_;
            IMUTypes::GetOmega_t GetOmega_;
            IMUTypes::GetPsi_t GetPsi_;

            // auxiliary variables
            Eigen::Vector3d g_;
        };

    } // namespace Simulator
} // namespace RPGQ