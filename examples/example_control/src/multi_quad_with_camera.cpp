// rpqg simulator
#include <rpgq_simulator/simulator.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h>
#include <rpgq_simulator/implementation/unity/unity_gate.h>

// rpgq components
#include <rpgq_components/controller/low_level_offboard_controller.h>
#include <rpgq_components/estimator/optitrack_estimator.h>

// standard library
#include <cmath>

// others
#include <trajectory/fig8_trajectory.h>

#define CONTROL_UPDATE_RATE 50.0

using namespace RPGQ;

int main(int argc, char * argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  // define the scene name
  FlightmareTypes::SceneID scene_id = FlightmareTypes::SCENE_WAREHOUSE;

  // create simulator
  std::shared_ptr<Simulator::Simulator> sim = std::make_shared<Simulator::Simulator>();
  //
  sim->SetFlightmare(true);

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim->GetSimTimer()));

  // define center position of multiple agents
  int n_vehicle_x = 3;
  int n_vehicle_y = 3;
  Eigen::ArrayXd vehicle_center_x = Eigen::ArrayXd::LinSpaced(n_vehicle_x, -3, 3);
  Eigen::ArrayXd vehicle_center_y = Eigen::ArrayXd::LinSpaced(n_vehicle_y, -3, 3);

  double center_z = 10.0;

  std::vector<std::string> quad_agents;
  std::vector<OptitrackEstimator*> estimators;
  std::vector<ros::Subscriber> pose_subs;
  std::vector<LowLevelOffboardController> controllers;
  std::vector<Trajectory> trajectories;

  uint16_t quadID = 0;
  for (int i=0; i<n_vehicle_x; i++)
  {
    for (int j=0; j<n_vehicle_y; j++)
    {
      std::string quadName = QuadrotorName(quadID);

      std::shared_ptr<Simulator::QuadRGBCamera> quadRGB =
        std::make_shared<Simulator::QuadRGBCamera>(quadName, nullptr, 1000000);
      //
      std::shared_ptr<Simulator::QuadrotorVehicle> quad = quadRGB->GetQuad();
      quad->SetPos(Eigen::Vector3d(vehicle_center_x(i), vehicle_center_y(j), center_z));
      quad->SetQuat(Eigen::Quaterniond(std::cos(0.5 * M_PI_2), 0.0, 0.0, std::sin(0.5 * M_PI_2)));
      quad->SetSize(Eigen::Vector3d(0.5, 0.5, 0.5));
      //
      std::shared_ptr<Simulator::RGBCamera> rgb = quadRGB->GetRGBCamera();
//      rgb->EnableDepth(true);
//      rgb->EnableObjectSegment(true);
//      rgb->EnableCategorySegment(true);
      // reset the image size
//      rgb->SetWidth(320);
//      rgb->SetHeight(240);

      sim->AddObjectToOptitrack(quad);

      // add objects to unity for simulation and visulization.
      sim->AddObjectToUnity(quadRGB);

      // set up estimator
      OptitrackEstimator *est = new OptitrackEstimator(quadID);

      // set up pose subscriber
      ros::Subscriber poseSub;
      poseSub = nh.subscribe("/rpgq_simulator/optitrack/" + quadName, 10,
        &OptitrackEstimator::MeasurementUpdate, est);

      // set up controller
      LowLevelOffboardController ctrl(quadID, timer);
      ctrl.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
      ctrl.SetPosDes(Eigen::Vector3d(vehicle_center_x(i), vehicle_center_y(j), center_z));
      ctrl.SetYawDes(0.0);

      // Figure8 trajectories
      Trajectory traj(sim->GetSimTimer());
      traj.SetStartPose(Eigen::Vector3d(vehicle_center_x(i),
        vehicle_center_y(j), center_z), 0.0);

      quad_agents.push_back(quadName);
      estimators.push_back(est);
      pose_subs.push_back(poseSub);
      controllers.push_back(ctrl);
      trajectories.push_back(traj);
      quadID++;
    }
  }

  // add gate to the simulator
  std::string gate_prefab_name = "rpg_gate";
  // define center position of multiple agents
  int n_gate_x = 3;
  int n_gate_y = 3;
  Eigen::ArrayXd gate_center_x = Eigen::ArrayXd::LinSpaced(n_gate_x, -20, 20);
  Eigen::ArrayXd gate_center_y = Eigen::ArrayXd::LinSpaced(n_gate_y, -30, 30);
  for (size_t i=0; i< n_gate_x; i++)
  {
    for (size_t j=0; j<n_gate_y; j++)
    {
      std::string gate_name = "gate_"+std::to_string(i)+"_"+std::to_string(j);
      std::shared_ptr<Simulator::UnityGate> gate_i =
        std::make_shared<Simulator::UnityGate>(gate_name, gate_prefab_name);
      gate_i->SetPosition(Eigen::Vector3d(gate_center_x(i), gate_center_y(j), 2.4));
      // add objects to unity for simulation and visulization.
      sim->AddObjectToUnity(gate_i);
    }
  }

  // set up figure8 trajectory
  bool fig8Started = false;

  // main loop
  Timer loopTimer;

  while (ros::ok()) {
    // enable visualization and add objects
    if (!sim->FlightmareIsReady()) {
      sim->ConnectFlightmare(scene_id);
    }
    // start measuring time
    loopTimer.Reset();

    // run ros callbacks
    ros::spinOnce();

    for (auto est : estimators) {
      est->Predict();
    }

    if (sim->ElapsedSeconds() > 2.0) {
      if (!fig8Started) {
        for (int quad_i=0; quad_i< quad_agents.size(); quad_i++) {
          fig8Started = trajectories[quad_i].StartMotion(Trajectory::Motion::Figure8);
        }
      }
      if (fig8Started)
      {
        for (int quad_i=0; quad_i< quad_agents.size(); quad_i++) {
          TrajectoryState trajStateDes = trajectories[quad_i].GetCurrentState();

          controllers[quad_i].SetPosDes(trajStateDes.pos);
          controllers[quad_i].SetVelDes(trajStateDes.vel);
          controllers[quad_i].SetAccDes(trajStateDes.acc);
          controllers[quad_i].SetYawDes(trajStateDes.yaw);
          controllers[quad_i].SetOmegaDes(trajStateDes.omega);
        }
      }
    }

    // run controller
    rpgq_msgs::CommandSet cmdSet;
    for (int quad_i=0; quad_i<quad_agents.size(); quad_i++) {
      if (timer->ElapsedSeconds() < 1.0)
      {
        rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quad_i), cmdSet);
      } else {
        controllers[quad_i].Run(estimators[quad_i]->GetPredictedState());
        rpgq_msgs::AddCommandToCommandSet(controllers[quad_i].GetCommand(), cmdSet);
      }
    }

    for (int quad_i=0; quad_i<quad_agents.size(); quad_i++) {
      estimators[quad_i]->FeedCommandQueue(cmdSet);
    }

    sim->SetCommandSet(cmdSet);
    sim->Run(1.0/CONTROL_UPDATE_RATE);

    // stop measuring time, sleep accordingly
    double secsToSleep = 1.0/CONTROL_UPDATE_RATE - loopTimer.ElapsedSeconds();
    if (secsToSleep > 0.0)
    {
      usleep(secsToSleep*1e6);
    }
  }
  return 0;
}

