// Copyright 2026 signalbotics
// Licensed under the Apache License, Version 2.0.
#ifndef GENESIS_ROS2_CONTROL__GENESIS_SYSTEM_HPP_
#define GENESIS_ROS2_CONTROL__GENESIS_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "genesis_ros2_control/shm_protocol.h"

namespace genesis_ros2_control
{

/**
 * @brief SystemInterface backed by a shared-memory region written to by
 * the Python genesis_bridge. Maps /dev/shm/genesis_ros2_control.<robot>
 * in on_init() and services read() / write() by copying between the
 * shm buffers and the exported hardware_interface handles.
 *
 * URDF parameters (on the <hardware> element):
 *   robot_name            shm basename (required, e.g. "franka")
 *   wait_for_bridge_sec   seconds to wait for the Python bridge to
 *                         write the handshake (default 5.0)
 */
class GenesisSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GenesisSystem)

  GenesisSystem();
  ~GenesisSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint mirror of exported interfaces. The hardware_interface
  // StateInterface / CommandInterface constructors take raw pointers to
  // these, so their addresses must be stable for the lifetime of the
  // plugin; store them as value types in a std::vector sized once.
  struct Joint
  {
    std::string name;
    std::size_t shm_index;              // index into GenesisShm.state arrays
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double position_cmd{std::numeric_limits<double>::quiet_NaN()};
    double velocity_cmd{std::numeric_limits<double>::quiet_NaN()};
    double effort_cmd{std::numeric_limits<double>::quiet_NaN()};
    bool has_position_cmd{false};
    bool has_velocity_cmd{false};
    bool has_effort_cmd{false};
  };

  // Open + mmap the shm region; wait up to wait_for_bridge_sec for the
  // handshake magic/version to land, then validate the joint-name list
  // matches what the URDF promised.
  hardware_interface::CallbackReturn open_shm(
    const std::string & robot_name, double wait_for_bridge_sec);
  void close_shm() noexcept;

  // Seqlock-read the state region into the Joint mirror.
  void read_state_locked() noexcept;
  // Seqlock-write the exported commands into the cmd region.
  void write_command_locked() noexcept;

  std::vector<Joint> joints_;
  int shm_fd_{-1};
  void * shm_ptr_{nullptr};
  std::size_t shm_size_{0};
  GenesisShm * shm_{nullptr};
  std::string shm_path_;
};

}  // namespace genesis_ros2_control

#endif  // GENESIS_ROS2_CONTROL__GENESIS_SYSTEM_HPP_
