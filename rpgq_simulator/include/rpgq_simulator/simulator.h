// ros
#include <ros/ros.h>

// images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// rpgq rpgq_common
#include <rpgq_common/parameters/parameters.h>
#include <rpgq_common/time/ext_timer.h>
#include <rpgq_common/msgs/command_msg_helper.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>
#include <rpgq_simulator/visualization/visualizer.h>
#include <rpgq_simulator/visualization/flightmare_bridge.hpp>

// standard library
#include <memory>
#include <queue>
#include <random>
#include <unordered_map>

namespace RPGQ {
namespace Simulator {

  // forward declaration
  class BaseObject;
  class UnityObject;
  class DirectlyControlledObject;
  class CompositeObject;
  class Optitrack;

  class Simulator
  {
  public:
    // constructor & destructor
    Simulator(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
      USecs maxSimUSecsInterval = 1000000);
    Simulator(void):
        Simulator(ros::NodeHandle(""), ros::NodeHandle(ID::Object::Root))
    {};
    ~Simulator(void) {};

    void Reset() {
      // simulation timing variables
      timer_.reset(new ExtTimer(false, false));
      minUSecsNextSim_ = 1;   
      while (!commandSetQueue_.empty()) {
        commandSetQueue_.pop();
      }
    }

    // simulation functions
    void Run(double t);
    void ConnectFlightmare(const size_t scene_id);
    void DisconnectFlightmare();

    // add objects to rpgq_simulator
    void AddObject(std::shared_ptr<BaseObject> object);
    void AddObjectToOptitrack(std::shared_ptr<BaseObject> object);

    // add objects to Unity for visulization or simulation
    void AddObjectToUnity(std::shared_ptr<UnityObject> object);
    void AddObjectToUnity(std::shared_ptr<QuadrotorVehicle> vehicle);
    void AddObjectToUnity(std::shared_ptr<QuadLidar> lidar_vehicle);
    void AddObjectToUnity(std::shared_ptr<QuadRGBCamera> rgb_vehicle);
    void AddObjectToUnity(std::shared_ptr<QuadStereoRGBCamera>
      stereo_rgb_vehicle);

    // public set functions
    void SetCommandSet(const rpgq_msgs::CommandSet &cmdSet);
    void SetFlightmare(const  bool & on);
    // void SetScene(const std::string & scene_name);

    bool FlightmareIsReady(void) { return flightmareReady_; };

    // public get functions
    double ElapsedSeconds(void) {return timer_->ElapsedSeconds();};
    std::shared_ptr<ExtTimer> GetSimTimer(void) {return timer_;};

  private:
    // general rpgq_simulator variables
    std::unordered_map<ObjectID, std::shared_ptr<BaseObject>> objects_;

    ros::NodeHandle nh_, pnh_;

    // simulation timing variables
    std::shared_ptr<ExtTimer> timer_;
    ros::Publisher timer_pub_;
    const USecs maxSimUSecsInterval_;
    USecs minUSecsNextSim_;

    // flightmare
    bool flightmareReady_{false};
    RenderMessage_t unity_output_;
    std::shared_ptr<FlightmareBridge> flightmareBridge_;

    // optitrack
    std::shared_ptr<CompositeObject> optitrackComposite_;
    std::shared_ptr<Optitrack> optitrack_;
    void AddObjectToOptitrackRecursively(std::shared_ptr<BaseObject> object);

    // laird
    double lairdDropRate_;
    USecs lairdDelay_;
    std::queue<rpgq_msgs::CommandSet> commandSetQueue_;

    // random numbers
    std::default_random_engine randomEngineGenerator_;
    std::uniform_real_distribution<double> uniformDistribution_;

    // auxiliary variables and functions
    std::unordered_map<ObjectID, std::shared_ptr<DirectlyControlledObject>>
      directlyControlledObjects_;
    void AddObjectToDirectlyControlledObjectsRecursively(
      std::shared_ptr<BaseObject> object);
  };
} // namespace Simulator
} // namespace RPGQ