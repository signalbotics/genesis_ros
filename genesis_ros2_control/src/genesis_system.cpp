// Copyright 2026 signalbotics
// Licensed under the Apache License, Version 2.0.
#include "genesis_ros2_control/genesis_system.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace genesis_ros2_control
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

GenesisSystem::GenesisSystem() = default;

GenesisSystem::~GenesisSystem() { close_shm(); }

// ---------------------------------------------------------------- on_init
CallbackReturn GenesisSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto logger = rclcpp::get_logger("GenesisSystem");

  const auto robot_it = info_.hardware_parameters.find("robot_name");
  if (robot_it == info_.hardware_parameters.end() || robot_it->second.empty()) {
    RCLCPP_ERROR(
      logger,
      "GenesisSystem requires the 'robot_name' hardware parameter "
      "(the shm basename written by the Python bridge).");
    return CallbackReturn::ERROR;
  }
  const std::string robot_name = robot_it->second;

  double wait_sec = 5.0;
  const auto wait_it = info_.hardware_parameters.find("wait_for_bridge_sec");
  if (wait_it != info_.hardware_parameters.end()) {
    try {
      wait_sec = std::stod(wait_it->second);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        logger, "Invalid wait_for_bridge_sec=%s (%s); defaulting to %.1f",
        wait_it->second.c_str(), e.what(), wait_sec);
    }
  }

  if (info_.joints.size() > GENESIS_SHM_MAX_JOINTS) {
    RCLCPP_ERROR(
      logger, "URDF has %zu joints but the shm layout caps at %u.",
      info_.joints.size(),
      static_cast<unsigned>(GENESIS_SHM_MAX_JOINTS));
    return CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    Joint j;
    j.name = info_.joints[i].name;
    j.shm_index = i;  // filled in properly once we see the handshake
    // Determine which command interfaces this joint declared.
    for (const auto & ci : info_.joints[i].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) j.has_position_cmd = true;
      else if (ci.name == hardware_interface::HW_IF_VELOCITY) j.has_velocity_cmd = true;
      else if (ci.name == hardware_interface::HW_IF_EFFORT) j.has_effort_cmd = true;
    }
    joints_.push_back(std::move(j));
  }

  const auto ret = open_shm(robot_name, wait_sec);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  // Validate URDF joint names against the Python bridge's handshake
  // list and record each joint's shm_index.
  const std::uint32_t n_shm = shm_->handshake.n_joints;
  for (auto & j : joints_) {
    bool found = false;
    for (std::uint32_t k = 0; k < n_shm; ++k) {
      const char * nk = shm_->handshake.joint_names[k];
      if (std::strncmp(nk, j.name.c_str(), GENESIS_SHM_JOINT_NAME_LEN) == 0) {
        j.shm_index = k;
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(
        logger,
        "Joint '%s' from the URDF was not registered by the Python "
        "bridge (shm has %u joints). Check that register_shm_bridge() "
        "was called with matching joint names.",
        j.name.c_str(), n_shm);
      close_shm();
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    logger, "GenesisSystem '%s' ready (%zu joints, shm=%s, %zu bytes).",
    robot_name.c_str(), joints_.size(), shm_path_.c_str(), shm_size_);
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- shm mgmt
CallbackReturn GenesisSystem::open_shm(
  const std::string & robot_name, double wait_for_bridge_sec)
{
  auto logger = rclcpp::get_logger("GenesisSystem");
  shm_path_ = "/dev/shm/" GENESIS_SHM_PATH_PREFIX + robot_name;
  shm_size_ = sizeof(GenesisShm);

  // Open RW -- do not create. The Python bridge creates and sizes
  // the region; if it's not there yet, wait for it.
  const auto deadline = std::chrono::steady_clock::now()
    + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(0.0, wait_for_bridge_sec)));
  while (true) {
    shm_fd_ = ::open(shm_path_.c_str(), O_RDWR);
    if (shm_fd_ >= 0) {
      struct stat st{};
      if (::fstat(shm_fd_, &st) == 0 &&
          static_cast<std::size_t>(st.st_size) >= shm_size_)
      {
        break;
      }
      ::close(shm_fd_);
      shm_fd_ = -1;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        logger,
        "Timed out waiting for genesis_bridge shm at %s "
        "(expected size >= %zu). Is the Python bridge running?",
        shm_path_.c_str(), shm_size_);
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  shm_ptr_ = ::mmap(
    nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (shm_ptr_ == MAP_FAILED) {
    RCLCPP_ERROR(
      logger, "mmap(%s) failed: %s", shm_path_.c_str(), std::strerror(errno));
    ::close(shm_fd_);
    shm_fd_ = -1;
    shm_ptr_ = nullptr;
    return CallbackReturn::ERROR;
  }
  shm_ = static_cast<GenesisShm *>(shm_ptr_);

  // Wait for the Python side to have written the handshake (it zeros
  // and then stamps magic/version). Without this we race on startup.
  while (true) {
    if (shm_->handshake.magic == GENESIS_SHM_MAGIC &&
        shm_->handshake.version == GENESIS_SHM_VERSION)
    {
      break;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        logger,
        "genesis_bridge shm at %s has magic=0x%08x version=%u; "
        "expected magic=0x%08x version=%u. ABI mismatch or bridge not up.",
        shm_path_.c_str(), shm_->handshake.magic,
        shm_->handshake.version,
        static_cast<unsigned>(GENESIS_SHM_MAGIC),
        static_cast<unsigned>(GENESIS_SHM_VERSION));
      close_shm();
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return CallbackReturn::SUCCESS;
}

void GenesisSystem::close_shm() noexcept
{
  if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
    ::munmap(shm_ptr_, shm_size_);
  }
  if (shm_fd_ >= 0) {
    ::close(shm_fd_);
  }
  shm_ = nullptr;
  shm_ptr_ = nullptr;
  shm_fd_ = -1;
}

// ------------------------------------------------------ interface export
std::vector<hardware_interface::StateInterface> GenesisSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 3);
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.position);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.velocity);
    out.emplace_back(j.name, hardware_interface::HW_IF_EFFORT,   &j.effort);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> GenesisSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size() * 3);
  for (auto & j : joints_) {
    if (j.has_position_cmd) {
      out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.position_cmd);
    }
    if (j.has_velocity_cmd) {
      out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.velocity_cmd);
    }
    if (j.has_effort_cmd) {
      out.emplace_back(j.name, hardware_interface::HW_IF_EFFORT, &j.effort_cmd);
    }
  }
  return out;
}

// ------------------------------------------------------ lifecycle hooks
CallbackReturn GenesisSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Seed command mirrors from current state so controllers that read
  // state into their command on startup (e.g. JointTrajectoryController)
  // don't instantly jerk the arm.
  read_state_locked();
  for (auto & j : joints_) {
    if (j.has_position_cmd) j.position_cmd = j.position;
    if (j.has_velocity_cmd) j.velocity_cmd = 0.0;
    if (j.has_effort_cmd)   j.effort_cmd   = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GenesisSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Zero all command masks so the bridge stops applying stale commands
  // after deactivation. Use the seqlock write protocol.
  if (shm_ == nullptr) return CallbackReturn::SUCCESS;
  auto & cmd = shm_->command;
  auto * seq = reinterpret_cast<std::atomic<std::uint64_t> *>(&cmd.seq);
  seq->fetch_add(1, std::memory_order_acq_rel);
  std::memset(cmd.cmd_mask, 0, sizeof(cmd.cmd_mask));
  seq->fetch_add(1, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- I/O
return_type GenesisSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (shm_ == nullptr) return return_type::ERROR;
  read_state_locked();
  return return_type::OK;
}

return_type GenesisSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (shm_ == nullptr) return return_type::ERROR;
  write_command_locked();
  return return_type::OK;
}

void GenesisSystem::read_state_locked() noexcept
{
  auto & st = shm_->state;
  auto * seq = reinterpret_cast<std::atomic<std::uint64_t> *>(&st.seq);
  for (int tries = 0; tries < 8; ++tries) {
    const std::uint64_t s1 = seq->load(std::memory_order_acquire);
    if ((s1 & 1u) != 0u) continue;  // writer in progress
    for (auto & j : joints_) {
      const auto k = j.shm_index;
      j.position = st.position[k];
      j.velocity = st.velocity[k];
      j.effort   = st.effort[k];
    }
    const std::uint64_t s2 = seq->load(std::memory_order_acquire);
    if (s1 == s2) return;
  }
  // Fell through: torn read persisted. Leave mirrors at their previous
  // values -- next read() will retry.
}

void GenesisSystem::write_command_locked() noexcept
{
  auto & cmd = shm_->command;
  auto * seq = reinterpret_cast<std::atomic<std::uint64_t> *>(&cmd.seq);

  // Seqlock write: odd during payload write.
  seq->fetch_add(1, std::memory_order_acq_rel);
  for (auto & j : joints_) {
    const auto k = j.shm_index;
    std::uint32_t mask = 0;
    if (j.has_position_cmd && !std::isnan(j.position_cmd)) {
      cmd.position[k] = j.position_cmd;
      mask |= GENESIS_CMD_POSITION;
    }
    if (j.has_velocity_cmd && !std::isnan(j.velocity_cmd)) {
      cmd.velocity[k] = j.velocity_cmd;
      mask |= GENESIS_CMD_VELOCITY;
    }
    if (j.has_effort_cmd && !std::isnan(j.effort_cmd)) {
      cmd.effort[k] = j.effort_cmd;
      mask |= GENESIS_CMD_EFFORT;
    }
    cmd.cmd_mask[k] = mask;
  }
  seq->fetch_add(1, std::memory_order_release);
}

}  // namespace genesis_ros2_control

PLUGINLIB_EXPORT_CLASS(
  genesis_ros2_control::GenesisSystem,
  hardware_interface::SystemInterface)
