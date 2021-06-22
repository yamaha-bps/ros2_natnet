// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSEt 2021 Cyberteam

#include "natnet_ros2/natnet_component.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec{};
  auto options = rclcpp::NodeOptions{};
  auto teleop_comp = std::make_shared<NatnetComponent>(options);

  exec.add_node(teleop_comp);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
