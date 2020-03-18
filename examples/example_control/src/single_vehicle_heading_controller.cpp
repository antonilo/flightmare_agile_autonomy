#include <rpgq_simulator/simulator.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h>
#include <heading_controller/heading_controller.h>
#include <rpgq_simulator/implementation/unity/unity_gate.h>

//
#include <rpgq_components/controller/low_level_offboard_controller.h>

// standard library
#include <cmath>

#define controllerUpdateRate 0.01   // controller runs at 100Hz

using namespace RPGQ;

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "simulator");

  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  // define the scene name
  FlightmareTypes::SceneID scene_id = FlightmareTypes::SCENE_WAREHOUSE;

  // quad ID can be any real number between
  // 0 ~ 25, each ID corresponding to a unique
  // quad name
  QuadrotorID quadID = 1;
  std::string quadName = QuadrotorName(quadID);

  // set up simulator
  Simulator::Simulator sim(ros::NodeHandle(), ros::NodeHandle("~"));

  // enable flightmare for visualization
  sim.SetFlightmare(true);

  // add quadrotor to simulator
  std::shared_ptr<Simulator::QuadRGBCamera> quadRGB =
    std::make_shared<Simulator::QuadRGBCamera>(quadName, nullptr, 1000000);
  //
  std::shared_ptr<Simulator::QuadrotorVehicle> quad = quadRGB->GetQuad();
  sim.AddObject(quad);

  // add quadrotor to Unity standalone
  // for visulization
  sim.AddObjectToUnity(quadRGB);

  // user variables
  HeadingController headingController;

  Eigen::Vector3d initial_position{10.0, 10.0, 2.0};
  quad->SetPos(initial_position);
  quad->SetVel(Eigen::Vector3d(0.0,-1.0,0.0));
  Eigen::Matrix3d rot_mat =  Eigen::AngleAxisd(-0.5 * M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  quad->SetQuat(Eigen::Quaterniond(rot_mat));

  std::vector<Eigen::Vector3d> targetPoints;
  targetPoints.resize(6);
  targetPoints[0] = Eigen::Vector3d(3.0, 3.0, 3.0); // -45
  targetPoints[1] = Eigen::Vector3d(10.0, 10.0, 5.0); // -45
  targetPoints[2] = Eigen::Vector3d(15.0, 15.0, 6.0); // -45
  targetPoints[3] = Eigen::Vector3d(20, 2.0, 7.0); // -45
  targetPoints[4] = Eigen::Vector3d(22, 7.0, 4.0); // -90
  targetPoints[5] = Eigen::Vector3d(42.0, 18.0, 5.0); // 45

  Eigen::Quaterniond gateRotation[3];
  gateRotation[0] = Eigen::AngleAxis<double>(0.0*M_PI, Eigen::Vector3d::UnitZ());
  gateRotation[1] = Eigen::AngleAxis<double>(0.0*M_PI, Eigen::Vector3d::UnitZ());
  gateRotation[2] = Eigen::AngleAxis<double>(0.0*M_PI, Eigen::Vector3d::UnitZ());

  //   add gate to the simulator
  std::string gate_prefab_name = "rpg_gate";
  for (size_t i=0; i < targetPoints.size(); i++)
  {
    std::string gate_name = "gate_"+std::to_string(i);
    std::shared_ptr<Simulator::UnityGate> gate_i =
      std::make_shared<Simulator::UnityGate>(gate_name, gate_prefab_name);
    gate_i->SetPosition(targetPoints[i] + Eigen::Vector3d(0.0, 3.0, 2.4));
    gate_i->SetRotation(gateRotation[i]);
    // add objects to unity for simulation and visulization.
    sim.AddObjectToUnity(gate_i);
  }

  // main loop
  int targetCounter = 0;
  {
    State6DoF state;
    state.vel = quad->GetVel();
    headingController.Reset(state);
    headingController.SetTargetPosition(targetPoints[targetCounter]);
    headingController.SetSpeed(4.0);
  }

  // main loop
  Timer loopTimer;
  while (ros::ok())
  {
    // enable visualization and add objects
    if (!sim.FlightmareIsReady())
    {
      sim.ConnectFlightmare(scene_id);
      // sleep for 1 second, dont send request to fast if flightmare is not ready.
      usleep(0.2*1e5);
      continue;
    }

    // start measuring time
    loopTimer.Reset();

    State6DoF state;

    // run user code
    if ((quad->GetPos() - targetPoints[targetCounter]).norm() < 1.0) {

      targetCounter++;
      if (targetCounter > 3) {
        targetCounter = 0;
      }
      headingController.SetTargetPosition(targetPoints[targetCounter]);

      state.pos = quad->GetPos();
      double dot = state.pos(0)*targetPoints[targetCounter](0)
        + state.pos(1) * targetPoints[targetCounter](1);

      double length1 = sqrt(state.pos(0) * state.pos(0) + state.pos(1) * state.pos(1));
      double length2 = sqrt(targetPoints[targetCounter][0] * targetPoints[targetCounter][0]
                              + targetPoints[targetCounter][1] * targetPoints[targetCounter][1]);
      float a = dot / (length1 * length2);

      double angle;
      if (a >= 1.0) {
        angle=0.0;
      } else if (a <= -1.0) {
        angle=M_PI;
      } else {
        angle = acos(a);
      }
      std::cout << angle << std::endl;
      Eigen::Matrix3d rot_mat =  Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      quad->SetQuat(Eigen::Quaterniond(rot_mat));
    }

    state.pos = quad->GetPos();
    Eigen::Vector3d velDes = headingController.Run(state, controllerUpdateRate);
    quad->SetVel(velDes);

    // simulate
    sim.Run(controllerUpdateRate);

    // stop measuring time, sleep accordingly
    double secsToSleep = controllerUpdateRate - loopTimer.ElapsedSeconds();
    if (secsToSleep > 0.0)
    {
      usleep(secsToSleep*1e6);
    }
  }

  return 0;
}