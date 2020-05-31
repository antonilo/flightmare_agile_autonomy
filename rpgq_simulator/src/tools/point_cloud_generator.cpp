#include <iostream>
#include <string>
#include <unistd.h>
#include <zmqpp/zmqpp.hpp>

#include <rpgq_simulator/visualization/flightmare_message_types.hpp>
#include "rpgq_simulator/tools/json.hpp"
#include "rpgq_simulator/tools/point_cloud_generator.h"

using json = nlohmann::json;

void RPGQ::Simulator::pointCloudGenerator(RPGQ::Simulator::PointCloudMessage_t &pcd_msg){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "PCD";
    json json_msg = pcd_msg;
    msg << json_msg.dump();
    pub_.send(msg, true);

    pub_.close();
}