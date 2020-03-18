#include <rpgq_simulator/implementation/sensors/lidar.h>


namespace RPGQ
{
  namespace Simulator
  {
    // constructor
    Lidar::Lidar(SensorID id, const Node* prevSimNode, Parameters<NUM_PARAMS> params):
      BaseSensor(id, prevSimNode, SECS_TO_USECS(1.0/params.GetDouble(Lidar::updateFrequency)))
    {
      // set up publisher
      lidarPub_ = pnh_.advertise<rpgq_msgs::Lidar>(GetSimTreePath(simNode_), 1);

      // Lidar parameters
      B_r_BS_ = Eigen::Vector3d(0.0,0.0,0.0);
      Eigen::Matrix3d R_BS;
      R_BS << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
      T_BS_.block<3,3>(0,0) = R_BS;
      T_BS_.block<3,1>(0,3) = B_r_BS_;
      T_BS_.row(3) << 0.0, 0.0, 0.0, 1.0;

      // general lidar variables
      max_distance_ = params.GetDouble(Lidar::maxDistance);
      num_beams_ = params.GetInt(Lidar::numBeams);
      start_scan_angle_ = params.GetDouble(Lidar::startScanAngle);
      end_scan_angle_ = params.GetDouble(Lidar::endScanAngle);
    };

    // simulate sensor
    void Lidar::RunSimulation_(void)
    {
      if(!lidar_queue_.empty())
      {
        LidarTypes::Lidar_t lidar_t = lidar_queue_.front();
        rpgq_msgs::Lidar lidar_msg;
        lidar_msg.header.stamp.fromNSec(1000*lidar_t.elapsed_useconds);
        for (int i=0; i < lidar_t.lidars.size(); i++) {
          lidar_msg.ranges.push_back(lidar_t.lidars[i]);
        }
        //
        lidarPub_.publish(lidar_msg);
        lidar_queue_.pop_front();
      }
    }

    void Lidar::FeedLidarQueue(const ros::Time & lidar_timestamp,
      const LidarTypes::Lidars & lidars)
    {
      queue_mutex_.lock();
      LidarTypes::Lidar_t lidar_t;
      lidar_t.elapsed_useconds = ROSTIME_TO_USECS(lidar_timestamp);

      for (auto lidar_i : lidars) {
        lidar_t.lidars.push_back(lidar_i);
      }
      //
      lidar_queue_.push_back(lidar_t);
      queue_mutex_.unlock();
    }

  } // namespace Simulator
} // namespace RPGQ