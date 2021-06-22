// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSEt 2021 Cyberteam

#ifndef NATNET_ROS2__NATNET_COMPONENT_HPP_
#define NATNET_ROS2__NATNET_COMPONENT_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <memory>

// forward-declarations
struct sFrameOfMocapData;
void NatnetDataCallback(sFrameOfMocapData * data, void * userData);


enum class State {OFF, UPDATING, STREAMING};


class NatnetComponent : public rclcpp::Node
{
public:
  explicit NatnetComponent(const rclcpp::NodeOptions & opts);

  ~NatnetComponent();

private:
  State state_;

  bool connect();
  bool get_data_description();

  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_trans_;
  rclcpp::TimerBase::SharedPtr timer_;

  struct Impl;
  std::unique_ptr<Impl> pImpl;

  friend void NatnetDataCallback(sFrameOfMocapData *, void *);
};

#endif  // NATNET_ROS2__NATNET_COMPONENT_HPP_
