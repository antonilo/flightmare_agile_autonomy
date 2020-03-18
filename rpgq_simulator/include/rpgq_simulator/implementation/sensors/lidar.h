#pragma once

// rpgq rpgq_common
#include <rpgq_common/parameters/parameters.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/sensors/base_sensor.h>

// standard library
#include <random>
#include <deque>
#include <mutex>
#
#include <rpgq_msgs/Lidar.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
  namespace Simulator
  {
    namespace LidarTypes
    {
      typedef Eigen::Matrix4d Mat4_t;
      typedef Eigen::Vector3d Vec3_t;

      typedef std::vector<double> Lidars;

      struct Lidar_t
      {
        Lidars lidars;
        USecs elapsed_useconds;
      };
    }

    class Lidar : public BaseSensor
    {
     public:
      // parameters
      enum
      {
        updateFrequency = 0, // [Hz]
        maxDistance = 1, // [m]
        numBeams = 2,
        startScanAngle = 3,
        endScanAngle = 4,
        NUM_PARAMS
      };

      // constructor & destructor
      Lidar(SensorID id, const Node* prevSimNode,
          Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("simulator/sensors/lidar"));
      ~Lidar() {};

      void SetRelPose(Eigen::Vector3d &B_r_BS, Eigen::Matrix3d &R_SB)
      {
        B_r_BS_ = B_r_BS;
        T_BS_.block<3,3>(0,0) = R_SB;
        T_BS_.block<3,1>(0,3) = B_r_BS_;
        T_BS_.row(3) << 0.0, 0.0, 0.0, 1.0;
      }

      // feed lidar data
      void FeedLidarQueue(const ros::Time & lidar_timestamp,
        const LidarTypes:: Lidars & lidars);

      // public get functions
      LidarTypes::Mat4_t GetRelPose(void) {return T_BS_ ;};
      int GetBeams(void) {return num_beams_; };
      double GetMaxDistance(void) {return max_distance_; };
      double GetNumBeams(void) {return num_beams_; };
      double GetStartScanAngle(void) { return start_scan_angle_; };
      double GetEndScanAngle(void) {return end_scan_angle_; };

     private:
      // simulate sensor
      void RunSimulation_(void);
      USecs UpdateSamplingInterval(void) {};

      // publishers
      ros::Publisher lidarPub_;

      // Lidar parameters
      LidarTypes::Vec3_t B_r_BS_;
      LidarTypes::Mat4_t T_BS_;

      //
      double max_distance_;
      int num_beams_;
      double start_scan_angle_;
      double end_scan_angle_;

      // lidar data buffer
      std::mutex queue_mutex_;
      std::deque<LidarTypes::Lidar_t> lidar_queue_;
    };

  } // namespace Simulator
} // namespace RPGQ