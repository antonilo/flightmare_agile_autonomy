// Include ZMQ bindings for comms with Unity.
#include <iostream>
#include <string>
#include <unistd.h>
#include <zmqpp/zmqpp.hpp>

#include <rpgq_simulator/visualization/flightmare_message_types.hpp>
#include "rpgq_simulator/tools/json.hpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    // ZMQ variables and functions
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);
    usleep(1000000);
    std::cout << "hello"<<std::endl;
    zmqpp::message msg;
    msg << "PCD";
    RPGQ::Simulator::PointCloudMessage_t pcd_msg;
    pcd_msg.scene_id=0;
    pcd_msg.bounding_box_scale = std::vector<double>{20.0, 20.0, 20.0};
    pcd_msg.file_name = "sendViaZMQ";
    json json_msg = pcd_msg;
    msg << json_msg.dump();
    pub_.send(msg, true);

}