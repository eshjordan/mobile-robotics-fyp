#include "cpp_epuck/types.hpp"
#include <rclcpp/rclcpp.hpp>

#define ROS2

#include <cpp_epuck/UDPComms.hpp>

std::shared_ptr<rclcpp::Node> LOGGING_NODE;

int main(int argc, char **argv) // NOLINT(bugprone-exception-escape)
{
    rclcpp::init(argc, argv);
    auto node                          = std::make_shared<rclcpp::Node>("robot_comms_model");
    LOGGING_NODE                       = node;
    auto robot_id                      = node->declare_parameter("robot_id", 0);
    auto manager_server_host           = node->declare_parameter("manager_server_host", "127.0.0.1");
    auto manager_server_port           = node->declare_parameter("manager_server_port", 50000);
    auto robot_command_host            = node->declare_parameter("robot_command_host", "127.0.0.1");
    auto robot_command_port            = node->declare_parameter("robot_command_port", 50001);
    auto robot_knowledge_host          = node->declare_parameter("robot_knowledge_host", "127.0.0.1");
    auto robot_knowledge_exchange_port = node->declare_parameter("robot_knowledge_exchange_port", 50002);
    auto network_factory               = std::make_shared<NetworkFactory>();

    RCLCPP_ERROR(LOGGING_NODE->get_logger(), "%s", "##__VA_ARGS__");

    auto robot_model = std::make_shared<RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>>(
        robot_id, host_size_string(manager_server_host), manager_server_port, host_size_string(robot_command_host),
        robot_command_port, host_size_string(robot_knowledge_host), robot_knowledge_exchange_port, network_factory);

    robot_model->start();
    rclcpp::spin(node);
    robot_model->stop();
}
