// rpqg simulator
#include <rpgq_simulator/simulator.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_stereo_rgb_camera.h>
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

  // quad ID can be any real number between
  // 0 ~ 25, each ID corresponding to a unique
  // quad name
  QuadrotorID quadID = 0;
  std::string quadName = QuadrotorName(quadID);

  // create simulator
  std::shared_ptr<Simulator::Simulator> sim = std::make_shared<Simulator::Simulator>();
  //
  sim->SetFlightmare(true);

  // create a quadrotor with a RGB Camera on it.
  std::shared_ptr<Simulator::QuadStereoRGBCamera> quadStereoRGB =
    std::make_shared<Simulator::QuadStereoRGBCamera>(quadName, nullptr, 1000000);
  //
  Eigen::Vector3d init_position{0.0, 0.0, 5.0};
  std::shared_ptr<Simulator::QuadrotorVehicle> quad = quadStereoRGB->GetQuad();
  quad->SetPos(init_position);
  quad->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  quad->SetSize(Eigen::Vector3d(0.5, 0.5, 0.5));
  //
  std::shared_ptr<Simulator::RGBCamera> left_rgb = quadStereoRGB->GetLeftRGBCamera();
  left_rgb->EnableOpticalFlow(false);// TODO: the color encoding of optical flow is not right
  left_rgb->EnableDepth(false);
  left_rgb->EnableObjectSegment(false);
  left_rgb->EnableCategorySegment(false);
//  left_rgb->SetWidth(320);
//  left_rgb->SetHeight(240);
  std::shared_ptr<Simulator::RGBCamera> right_rgb = quadStereoRGB->GetRightRGBCamera();
  right_rgb->EnableOpticalFlow(false);// TODO: the color encoding of optical flow is not right
  right_rgb->EnableDepth(false);
  right_rgb->EnableObjectSegment(false);
  right_rgb->EnableCategorySegment(false);
//  right_rgb->SetWidth(320);
//  right_rgb->SetHeight(240);
  //
  sim->AddObjectToOptitrack(quad);
  sim->AddObjectToUnity(quadStereoRGB);

  // add gate to the simulator
  std::string gate_prefab_name = "rpg_gate";
  // define center position of multiple agents
  int n_x = 10;
  int n_y = 10;
  Eigen::ArrayXd center_x = Eigen::ArrayXd::LinSpaced(n_x, -20, 20);
  Eigen::ArrayXd center_y = Eigen::ArrayXd::LinSpaced(n_y, -30, 30);
  for (size_t i=0; i< n_x; i++)
  {
    for (size_t j=0; j< n_y; j++)
    {
      std::string gate_name = "gate_"+std::to_string(i)+"_"+std::to_string(j);
      std::shared_ptr<Simulator::UnityGate> gate_i =
        std::make_shared<Simulator::UnityGate>(gate_name, gate_prefab_name);
      gate_i->SetPosition(Eigen::Vector3d(center_x(i), center_x(j), 2.4));
      // add objects to unity for simulation and visulization.
      sim->AddObjectToUnity(gate_i);
    }
  }

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim->GetSimTimer()));

  // set up estimator
  OptitrackEstimator est(quadID);
  //
  ros::Subscriber poseSub;
  //
  poseSub = nh.subscribe("/rpgq_simulator/optitrack/" + quadName, 10, &OptitrackEstimator::MeasurementUpdate, &est);

  // set up controller
  LowLevelOffboardController ctrl(quadID, timer);
  ctrl.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl.SetPosDes(init_position);
  ctrl.SetYawDes(0.0);

  bool fig8Started = false;
  Trajectory trajectory(sim->GetSimTimer());
  trajectory.SetStartPose(init_position, 0.0);

  // main loop
  Timer loopTimer;

  while (ros::ok())
  {

    // enable visualization and add objects
    if (!sim->FlightmareIsReady())
    {
      sim->ConnectFlightmare(scene_id);
    }
    // start measuring time
    loopTimer.Reset();

    // run ros callbacks
    ros::spinOnce();

    est.Predict();//ros::Time::now()); // TODO

    if (sim->ElapsedSeconds() > 2.0)
    {
      if (!fig8Started)
      {
        fig8Started = trajectory.StartMotion(Trajectory::Motion::Figure8);
      }

      if (fig8Started)
      {
        TrajectoryState trajStateDes = trajectory.GetCurrentState();
        ctrl.SetPosDes(trajStateDes.pos);
        ctrl.SetVelDes(trajStateDes.vel);
        ctrl.SetAccDes(trajStateDes.acc);
        ctrl.SetYawDes(trajStateDes.yaw);
        ctrl.SetOmegaDes(trajStateDes.omega);
      }
    }

    // run controller
    rpgq_msgs::CommandSet cmdSet;

    if (timer->ElapsedSeconds() < 1.0)
    {
      rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quadID), cmdSet);
    }
    else
    {
      ctrl.Run(est.GetPredictedState());
      rpgq_msgs::AddCommandToCommandSet(ctrl.GetCommand(), cmdSet);
    }

    // feed estimator with commands
    est.FeedCommandQueue(cmdSet);
    sim->SetCommandSet(cmdSet);

    // run simulation
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

