#include <rpgq_simulator/simulator.h>

// ros
#include <rosgraph_msgs/Clock.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/base_object.h>
#include <rpgq_simulator/framework/unity/unity_object.h>
#include <rpgq_simulator/framework/objects/directly_controlled_object.h>
#include <rpgq_simulator/framework/objects/composite_object.h>
#include <rpgq_simulator/implementation/sensors/optitrack.h>

// standard library
#include <algorithm>

namespace RPGQ {
namespace Simulator {
// constructor & destructor
Simulator::Simulator(const ros::NodeHandle &nh, 
 const ros::NodeHandle &pnh, USecs maxSimUSecsInterval):
  nh_(nh),
  pnh_(pnh),
  maxSimUSecsInterval_(maxSimUSecsInterval) {
  // simulation timing variables
  timer_.reset(new ExtTimer(false, false));
  minUSecsNextSim_ = 1;
  timer_pub_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 1);

  // optitrack
  optitrackComposite_ = nullptr;
  optitrack_ = nullptr;

  // laird
  lairdDropRate_ = 0.0; // default (0.1)
  lairdDelay_ = 1; // default (20'000us -> 20ms)

  // random numbers
  uniformDistribution_ = std::uniform_real_distribution<double>(0.0, 1.0);
}

// simulation functions
void Simulator::Run(double t) {
  USecs remainingUSecsToSimulate = SECS_TO_USECS(t);

  while (remainingUSecsToSimulate > 0) {
    // publish time
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time(timer_->ElapsedSeconds());
    timer_pub_.publish(clock_msg);
    ros::spinOnce();
    
    // compute simulation step size
    USecs dt = std::min(remainingUSecsToSimulate, minUSecsNextSim_);
    minUSecsNextSim_ = maxSimUSecsInterval_;

    // simulate laird
    if (!commandSetQueue_.empty()) {

      while ( !(ROSTIME_TO_USECS(commandSetQueue_.front().header.stamp) > 
        timer_->ElapsedUSeconds() - lairdDelay_) ) {
        // send it to all objects
        for (const auto & object : directlyControlledObjects_) {
          object.second->SetCommandSet(commandSetQueue_.front());
        }
        // remove command set from queue
        commandSetQueue_.pop();

        if (commandSetQueue_.empty()) {
            break;
        }
      }

      if (!commandSetQueue_.empty()) {
        USecs usecsNextLaird = ROSTIME_TO_USECS(commandSetQueue_.front().header.stamp) 
          + lairdDelay_ - timer_->ElapsedUSeconds();
        minUSecsNextSim_ = std::min(usecsNextLaird, minUSecsNextSim_);
      }
    }

    // simulate objects
    for (const auto & object : objects_) {
      object.second->RunSimulation(dt);
    }

    if (flightmareReady_) {
      // send message to unity and handle output 
      flightmareBridge_->getRender(1000*timer_->ElapsedUSeconds());
      flightmareBridge_->handleOutput(unity_output_);
    }

    // push timer forward
    timer_->AdvanceUSeconds(dt);
    remainingUSecsToSimulate -= dt;
  }
}

// add object
void Simulator::AddObject(std::shared_ptr<BaseObject> object) {
    // add object if object ID is unique
    objects_.emplace(object->GetID(), object);

    // add object to list of directly controlled objects
    AddObjectToDirectlyControlledObjectsRecursively(object);
}

void Simulator::AddObjectToOptitrack(std::shared_ptr<BaseObject> object) {
  if (optitrackComposite_ == nullptr) {
    // create composite object containing optitrack sensor
    optitrackComposite_ = std::make_shared<CompositeObject>(
      ID::Object::Optitrack, nullptr, SECS_TO_USECS(1.0));
    AddObject(optitrackComposite_);

    // (optionally, set nullptr below to optitrackComposite_->GetSimNode())
    optitrack_ = std::make_shared<Optitrack>(ID::Sensor::Optitrack, nullptr);
    optitrackComposite_->AddSensor(optitrack_);
  }

  // add to composite
  optitrackComposite_->AddChild(object);

  // add recursively to sensor
  AddObjectToOptitrackRecursively(object);

  // add object to list of directly controlled objects
  AddObjectToDirectlyControlledObjectsRecursively(object);
}

void Simulator::AddObjectToUnity(std::shared_ptr<UnityObject> unity_object) {
  if (flightmareBridge_ != nullptr) {
    flightmareBridge_->addObject(unity_object);
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
      pnh_.getNamespace().c_str());
  }
}

void Simulator::AddObjectToUnity(std::shared_ptr<QuadrotorVehicle> vehicle) {
  if (flightmareBridge_ != nullptr) {
    flightmareBridge_->addQuad(vehicle);
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
      pnh_.getNamespace().c_str());
  }
}

void Simulator::AddObjectToUnity(std::shared_ptr<QuadRGBCamera> rgb_vehicle) {
  if (flightmareBridge_ != nullptr) {
    flightmareBridge_->addQuadRGB(rgb_vehicle);
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
      pnh_.getNamespace().c_str());
  }
}

void Simulator::AddObjectToUnity(std::shared_ptr<QuadLidar> lidar_vehicle) {
  if (flightmareBridge_ != nullptr) {
    flightmareBridge_->addQuadLidar(lidar_vehicle);
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
      pnh_.getNamespace().c_str());
  }
}

void Simulator::AddObjectToUnity(std::shared_ptr<QuadStereoRGBCamera> stereo_rgb_vehicle) {
  if (flightmareBridge_ != nullptr) {
    flightmareBridge_->addQuadStereoRGB(stereo_rgb_vehicle);
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
      pnh_.getNamespace().c_str());
  }
}

void Simulator::ConnectFlightmare(const size_t scene_id) {
  if(flightmareBridge_ != nullptr) {
      flightmareBridge_->setScene(scene_id);
      flightmareReady_ = flightmareBridge_->connectUnity();
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
      pnh_.getNamespace().c_str());
  }
}

void Simulator::DisconnectFlightmare() {
  if(flightmareBridge_ != nullptr) {
      flightmareBridge_->disconnectUnity();
      flightmareReady_ = false;
  } else {
    ROS_ERROR("[%s] Flightmare Bridge is nullptr, did you forget to initialize it.", 
    pnh_.getNamespace().c_str());
  }
}

// public set functions
void Simulator::SetCommandSet(const rpgq_msgs::CommandSet &cmdSet) {
  if (uniformDistribution_(randomEngineGenerator_) > lairdDropRate_) {
    // make sure we don't simulate too far
    if (commandSetQueue_.empty()) {
        USecs usecsNextLaird = ROSTIME_TO_USECS(cmdSet.header.stamp) + 
          lairdDelay_ - timer_->ElapsedUSeconds();
        minUSecsNextSim_ = std::min(usecsNextLaird, minUSecsNextSim_);
    }
    // put command set in queue
    commandSetQueue_.push(cmdSet);
  }
}

void Simulator::SetFlightmare(const bool & on) {
  if (on) {
    flightmareBridge_ = FlightmareBridge::getInstance();
    flightmareBridge_->initializeConnections();
  } 
}

// optitrack
void Simulator::AddObjectToOptitrackRecursively(std::shared_ptr<BaseObject> object) {
  if (object->GetObjectType() == ObjectType::COMPOSITE) {
    for (const auto& child : 
      std::static_pointer_cast<CompositeObject>(object)->GetChildren()) {
        AddObjectToOptitrackRecursively(child.second);
    }
  } else if (object->GetObjectType() == ObjectType::DIRECTLY_CONTROLLED || 
    object->GetObjectType() == ObjectType::INDIRECTLY_CONTROLLED) {
      optitrack_->AddObject(std::static_pointer_cast<PhysicalObject>(object));
  }
}

// auxiliary functions
void Simulator::AddObjectToDirectlyControlledObjectsRecursively(
  std::shared_ptr<BaseObject> object) {
  if (object->GetObjectType() == ObjectType::COMPOSITE) {
    for (const auto& child : std::static_pointer_cast<CompositeObject>(
      object)->GetChildren()) {
      AddObjectToDirectlyControlledObjectsRecursively(child.second);
    }
  } else if (object->GetObjectType() == ObjectType::DIRECTLY_CONTROLLED) {
    // add object if object ID is unique
    directlyControlledObjects_.emplace(object->GetID(), 
      std::static_pointer_cast<DirectlyControlledObject>(object));
  }
}

} // namespace Simulator
} // namespace RPGQ
