// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_drivers/blob/master/LICENSEt 2021 Cyberteam

#ifndef NATNET__NATNET_NODE_HPP_
#define NATNET__NATNET_NODE_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <memory>


// forward-declarations
struct sFrameOfMocapData;
void NatnetDataCallback(sFrameOfMocapData * data, void * userData);

namespace cbr
{

enum class State {OFF, UPDATING, STREAMING};

class NatnetNode : public rclcpp::Node
{
public:
  explicit NatnetNode(const rclcpp::NodeOptions & opts);

  ~NatnetNode();

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

}  // namespace cbr

#endif  // NATNET__NATNET_NODE_HPP_
