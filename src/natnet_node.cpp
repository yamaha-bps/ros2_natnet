// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/ros2_natnet/blob/master/LICENSE

#include "natnet/natnet_node.hpp"

#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetTypes.h>

#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cbr
{

struct NatnetNode::Impl
{
  NatNetClient client;

  uint64_t server_clockfreq;

  std::atomic<bool> need_update;

  std::mutex desc_mtx;
  std::unordered_map<int32_t, std::pair<std::string, Eigen::Isometry3d>> rigid_bodies;
  std::unordered_map<std::string, std::vector<std::string>> markers;

  std::unordered_map<std::string, Eigen::Isometry3d> name_to_offset;
};

void NatnetDataCallback(sFrameOfMocapData * data, void * userData)
{
  NatnetNode * node = static_cast<NatnetNode *>(userData);
  auto stamp = node->now();

  std::lock_guard lock(node->pImpl->desc_mtx);

  if (data->CameraMidExposureTimestamp != 0) {
    auto latency = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(
        node->pImpl->client.SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp)));
    stamp -= latency;
  }

  // RIGID BODIES
  for (auto i = 0u; i != data->nRigidBodies; ++i) {
    if (data->RigidBodies[i].params & 0x01) {
      auto it = node->pImpl->rigid_bodies.find(data->RigidBodies[i].ID);

      if (it == node->pImpl->rigid_bodies.end()) {
        node->pImpl->need_update = true;
        continue;
      }

      const auto & [name, offset] = it->second;

      Eigen::Isometry3d P_W_RB(
        Eigen::Translation3d(
          data->RigidBodies[i].x,
          data->RigidBodies[i].y,
          data->RigidBodies[i].z
        ) *
        Eigen::Quaterniond(
          data->RigidBodies[i].qw,
          data->RigidBodies[i].qx,
          data->RigidBodies[i].qy,
          data->RigidBodies[i].qz
        )
      );

      P_W_RB = P_W_RB * offset;

      auto msg = std::make_unique<geometry_msgs::msg::TransformStamped>();

      msg->header.stamp = stamp;
      msg->header.frame_id = "optitrack";
      msg->child_frame_id = name;
      msg->transform.translation.x = P_W_RB.translation().x();
      msg->transform.translation.y = P_W_RB.translation().y();
      msg->transform.translation.z = P_W_RB.translation().z();
      const Eigen::Quaterniond quat(P_W_RB.linear());
      msg->transform.rotation.w = quat.w();
      msg->transform.rotation.x = quat.x();
      msg->transform.rotation.y = quat.y();
      msg->transform.rotation.z = quat.z();

      node->pub_trans_->publish(std::move(msg));
    }
  }

  // MARKERS IN MARKER SETS (published as TransformStamped)
  for (auto i = 0u; i != data->nMarkerSets; ++i) {
    std::string set_name(data->MocapData[i].szName);
    if (set_name == "all") {
      continue;  // skip all set
    }

    auto it = node->pImpl->markers.find(set_name);

    if (it == node->pImpl->markers.end()) {
      node->pImpl->need_update = true;
      continue;
    }

    const auto & marker_names = it->second;

    if (marker_names.size() != data->MocapData[i].nMarkers) {
      RCLCPP_WARN(
        node->get_logger(), "Size discrepancy set %s: %d != %d",
        set_name.c_str(), marker_names.size(), data->MocapData[i].nMarkers);
      node->pImpl->need_update = true;
      continue;
    }

    for (auto j = 0u; j != data->MocapData[i].nMarkers; ++j) {
      auto msg = std::make_unique<geometry_msgs::msg::TransformStamped>();

      msg->header.stamp = stamp;
      msg->header.frame_id = "optitrack";
      msg->child_frame_id = set_name + "_" + marker_names[j];

      msg->transform.translation.x = data->MocapData[i].Markers[j][0];
      msg->transform.translation.y = data->MocapData[i].Markers[j][1];
      msg->transform.translation.z = data->MocapData[i].Markers[j][2];

      node->pub_trans_->publish(std::move(msg));
    }
  }

  // UNLABELED MARKERS (published as TransformStamped)
  for (auto i = 0u; i != data->nLabeledMarkers; ++i) {
    if (data->LabeledMarkers[i].params & 0x10) {  // unlabeled
      auto msg = std::make_unique<geometry_msgs::msg::TransformStamped>();

      msg->header.stamp = stamp;
      msg->header.frame_id = "optitrack";
      msg->child_frame_id = std::string("unlabeled_") + std::to_string(data->LabeledMarkers[i].ID);

      msg->transform.translation.x = data->LabeledMarkers[i].x;
      msg->transform.translation.y = data->LabeledMarkers[i].y;
      msg->transform.translation.z = data->LabeledMarkers[i].z;

      node->pub_trans_->publish(std::move(msg));
    }
  }


  // NOTE: ADD OTHER DATA TYPES WHEN/IF NEEDED
}

NatnetNode::NatnetNode(const rclcpp::NodeOptions & opts)
: rclcpp::Node("natnet", opts), pImpl(std::make_unique<Impl>())
{
  declare_parameter<std::string>("local_address", "");
  declare_parameter<std::string>("server_address", "192.168.0.1");
  declare_parameter<int>("server_port", 0);

  declare_parameter<std::string>("offset_file", "");

  pub_trans_ =
    create_publisher<geometry_msgs::msg::TransformStamped>("optitrack", rclcpp::SensorDataQoS());

  auto pose_file = get_parameter("offset_file").as_string();

  if (std::filesystem::exists(pose_file)) {
    auto pose_yaml = YAML::LoadFile(pose_file);
    for (const auto & item : pose_yaml) {
      auto name = item["name"].as<std::string>();
      Eigen::Isometry3d pose(
        Eigen::Translation3d(
          item["x"].as<double>(),
          item["y"].as<double>(),
          item["z"].as<double>()
        ) *
        Eigen::Quaterniond(
          item["qw"].as<double>(),
          item["qx"].as<double>(),
          item["qy"].as<double>(),
          item["qz"].as<double>()
        )
      );
      pImpl->name_to_offset[name] = pose;
    }
  } else if (!pose_file.empty()) {
    RCLCPP_ERROR(get_logger(), "Offset file %s not found", pose_file.c_str());
  }

  // set up natnet
  std::array<uint8_t, 4> ver;
  NatNet_GetVersion(ver.data());

  RCLCPP_INFO(get_logger(), "Using NatNet SDK v%d.%d.%d.%d", ver[0], ver[1], ver[2], ver[3]);

  // set callback
  pImpl->client.SetFrameReceivedCallback(&NatnetDataCallback, static_cast<void *>(this));

  state_ = State::OFF;
  if (connect()) {
    state_ = State::UPDATING;
    if (get_data_description()) {
      state_ = State::STREAMING;
    }
  }

  // timer to monitor status
  timer_ = create_wall_timer(
    std::chrono::seconds(3), [this] {
      switch (state_) {
        case State::OFF: {
          if (connect()) {
            state_ = State::UPDATING;
          }
          break;
        }
        case State::UPDATING: {
          if (get_data_description()) {
            state_ = State::STREAMING;
          }
          break;
        }
        case State::STREAMING: {
          if (pImpl->need_update) {
            state_ = State::UPDATING;
          }
          break;
        }
      }
    });

  RCLCPP_INFO(get_logger(), "Started node");
}

NatnetNode::~NatnetNode() {pImpl->client.Disconnect();}

bool NatnetNode::connect()
{
  auto local_address = get_parameter("local_address").as_string();
  auto server_address = get_parameter("server_address").as_string();
  auto server_port = get_parameter("server_port").as_int();

  // connect to server
  sNatNetClientConnectParams prm;
  prm.connectionType = ConnectionType_Multicast;
  prm.localAddress = local_address.data();
  prm.serverAddress = server_address.data();
  prm.serverDataPort = server_port;

  RCLCPP_INFO(
    get_logger(), "Connecting to %s:%d from local address %s", prm.serverAddress,
    prm.serverDataPort, prm.localAddress);

  auto rc = pImpl->client.Connect(prm);

  if (rc != ErrorCode_OK) {
    RCLCPP_ERROR(get_logger(), "Connection failed with code %d", rc);
    return false;
  }

  // read server information
  sServerDescription desc;
  rc = pImpl->client.GetServerDescription(&desc);

  RCLCPP_INFO(
    get_logger(), "Connected to server %s running %s v.%d.%d.%d.%d", desc.szHostComputerName,
    desc.szHostApp, desc.HostAppVersion[0], desc.HostAppVersion[1], desc.HostAppVersion[2],
    desc.HostAppVersion[3]);

  pImpl->server_clockfreq = desc.HighResClockFrequency;
  RCLCPP_INFO(get_logger(), "Server clock frequency %d", pImpl->server_clockfreq);

  void * result;
  int nBytes = 0;

  if (pImpl->client.SendMessageAndWait("FrameRate", &result, &nBytes) == ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "Frame rate %f", *static_cast<float *>(result));
  } else {
    RCLCPP_ERROR(get_logger(), "Frame rate request failed");
    return false;
  }

  if (
    pImpl->client.SendMessageAndWait("AnalogSamplesPerMocapFrame", &result, &nBytes) ==
    ErrorCode_OK)
  {
    RCLCPP_INFO(get_logger(), "Analog samples per frame rate %d", *static_cast<int *>(result));
  } else {
    RCLCPP_ERROR(get_logger(), "Analog frame rate request failed");
    return false;
  }

  return true;
}

bool NatnetNode::get_data_description()
{
  RCLCPP_INFO(get_logger(), "Requesting data descriptions");

  sDataDescriptions * dataDesc = NULL;
  auto rc = pImpl->client.GetDataDescriptionList(&dataDesc);
  if (rc != ErrorCode_OK) {
    RCLCPP_ERROR(get_logger(), "Data description request failed with code %d", rc);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Received %d data descriptions", dataDesc->nDataDescriptions);

  std::lock_guard lock(pImpl->desc_mtx);

  // RIGID BODIES
  pImpl->rigid_bodies.clear();
  for (auto i = 0u; i != dataDesc->nDataDescriptions; ++i) {
    if (dataDesc->arrDataDescriptions[i].type == Descriptor_RigidBody) {
      sRigidBodyDescription * p = dataDesc->arrDataDescriptions[i].Data.RigidBodyDescription;

      std::string name(p->szName);
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();

      auto rb_it = pImpl->name_to_offset.find(name);
      if (rb_it != pImpl->name_to_offset.end()) {
        offset = rb_it->second;
      }

      const Eigen::Quaterniond quat(offset.linear());
      RCLCPP_INFO(get_logger(), "Adding rigid body %s", name.c_str());
      RCLCPP_INFO(
        get_logger(), "Rigid body %s has associated offset %f %f %f %f %f %f %f ",
        name.c_str(),
        name.c_str(), offset.translation().x(),
        name.c_str(), offset.translation().y(),
        name.c_str(), offset.translation().z(),
        name.c_str(), quat.w(),
        name.c_str(), quat.x(),
        name.c_str(), quat.y(),
        name.c_str(), quat.z()
      );

      pImpl->rigid_bodies[p->ID] = {name, offset};
    }

    if (dataDesc->arrDataDescriptions[i].type == Descriptor_MarkerSet) {
      sMarkerSetDescription * p = dataDesc->arrDataDescriptions[i].Data.MarkerSetDescription;

      std::string set_name(p->szName);
      std::vector<std::string> marker_names;

      RCLCPP_INFO(
        get_logger(), "Adding markerset %s with %d markers",
        set_name.c_str(), p->nMarkers
      );

      for (auto j = 0u; j != p->nMarkers; ++j) {
        std::string name(p->szMarkerNames[j]);
        RCLCPP_INFO(get_logger(), "  member %d: %s", j, name.c_str());
        marker_names.push_back(std::move(name));
      }

      pImpl->markers[set_name] = std::move(marker_names);
    }
  }

  // NOTE: ADD OTHER DATA TYPES WHEN/IF NEEDED

  if (dataDesc) {
    NatNet_FreeDescriptions(dataDesc);
  }

  pImpl->need_update = false;
  return true;
}

}  // namespace cbr

RCLCPP_COMPONENTS_REGISTER_NODE(cbr::NatnetNode)
