#include "cpp_epuck/types.hpp"
#include <rclcpp/rclcpp.hpp>

#define ROS2

#include <cpp_epuck/UDPComms.hpp>

int main(int argc, char **argv) // NOLINT(bugprone-exception-escape)
{
    rclcpp::init(argc, argv);
    auto node                          = std::make_shared<rclcpp::Node>("robot_comms_model");
    auto robot_id                      = node->declare_parameter("robot_id", 0);
    auto manager_host                  = node->declare_parameter("manager_host", "127.0.0.1");
    auto manager_port                  = node->declare_parameter("manager_port", 50000);
    auto robot_comms_host              = node->declare_parameter("robot_comms_host", "127.0.0.1");
    auto robot_comms_request_port      = node->declare_parameter("robot_comms_request_port", 50001);
    auto robot_knowledge_host          = node->declare_parameter("robot_knowledge_host", "127.0.0.1");
    auto robot_knowledge_exchange_port = node->declare_parameter("robot_knowledge_exchange_port", 50002);
    auto network_factory               = std::make_shared<NetworkFactory>();

    auto robot_model = std::make_shared<RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>>(
        robot_id, host_size_string(manager_host), manager_port, host_size_string(robot_comms_host),
        robot_comms_request_port, host_size_string(robot_knowledge_host), robot_knowledge_exchange_port,
        network_factory);

    robot_model->start();
    rclcpp::spin(node);
    robot_model->stop();
}
