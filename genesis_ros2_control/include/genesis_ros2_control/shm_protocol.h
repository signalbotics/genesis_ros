/**
 * Shared-memory protocol between the Python ``genesis_bridge`` and the
 * native ``genesis_ros2_control::GenesisSystem`` hardware-interface plugin.
 *
 * Design goals
 * ------------
 * - Fixed layout, C ABI stable. Both sides mmap the same /dev/shm file
 *   and cast to these structs.
 * - Single writer per region (Python writes state; C++ writes commands)
 *   with a seqlock so the reader can detect torn reads without locks.
 * - Cache-line padded so the state and command seq counters live on
 *   different lines (avoids false sharing at 1 kHz+).
 * - Hardware-interface native units: position in rad/m, velocity in
 *   rad/s / m/s, effort in N·m / N.
 *
 * ABI version bump rule: any change to the struct layout -> bump
 * GENESIS_SHM_VERSION. Both sides verify magic + version on handshake
 * and refuse to proceed on mismatch.
 */
#ifndef GENESIS_ROS2_CONTROL_SHM_PROTOCOL_H_
#define GENESIS_ROS2_CONTROL_SHM_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
#include <atomic>
using atomic_uint64 = std::atomic<uint64_t>;
extern "C" {
#else
#include <stdatomic.h>
typedef _Atomic uint64_t atomic_uint64;
#endif

#define GENESIS_SHM_MAGIC    0x474E5253u   /* 'GNRS' */
#define GENESIS_SHM_VERSION  1u

#define GENESIS_SHM_MAX_JOINTS      64
#define GENESIS_SHM_JOINT_NAME_LEN  64
#define GENESIS_SHM_CACHE_LINE      64

/* Per-joint command interface encoding. Bit set => the controller wrote
 * to that interface this cycle. Plugin OR's these into cmd_mask; Python
 * dispatches accordingly and clears the relevant bit after consumption. */
#define GENESIS_CMD_POSITION  (1u << 0)
#define GENESIS_CMD_VELOCITY  (1u << 1)
#define GENESIS_CMD_EFFORT    (1u << 2)

/* Double-buffered region using a seqlock: writer does
 *   seq <- seq + 1  (odd, write-in-progress)
 *   <write payload>
 *   seq <- seq + 1  (even, stable)
 * Reader does:
 *   s1 = seq
 *   if (s1 & 1) retry
 *   <copy payload>
 *   s2 = seq
 *   if (s1 != s2) retry
 * No locks, no syscalls. Safe for one writer + many readers at any rate.
 */

typedef struct GenesisShmState {
  atomic_uint64 seq;                                  /* seqlock counter */
  uint64_t      stamp_ns;                             /* sim time ns     */
  double        position[GENESIS_SHM_MAX_JOINTS];
  double        velocity[GENESIS_SHM_MAX_JOINTS];
  double        effort  [GENESIS_SHM_MAX_JOINTS];
  uint8_t       _pad[GENESIS_SHM_CACHE_LINE];         /* avoid false sharing */
} GenesisShmState;

typedef struct GenesisShmCommand {
  atomic_uint64 seq;
  uint32_t      cmd_mask [GENESIS_SHM_MAX_JOINTS];    /* per-joint GENESIS_CMD_* bits */
  double        position [GENESIS_SHM_MAX_JOINTS];
  double        velocity [GENESIS_SHM_MAX_JOINTS];
  double        effort   [GENESIS_SHM_MAX_JOINTS];
  uint8_t       _pad[GENESIS_SHM_CACHE_LINE];
} GenesisShmCommand;

typedef struct GenesisShmHandshake {
  uint32_t magic;                                     /* GENESIS_SHM_MAGIC */
  uint32_t version;                                   /* GENESIS_SHM_VERSION */
  uint32_t n_joints;                                  /* <= GENESIS_SHM_MAX_JOINTS */
  uint32_t _pad0;
  /* Joint names the Python side wrote. C++ plugin verifies these match
   * the URDF joint list it was given by controller_manager. */
  char     joint_names[GENESIS_SHM_MAX_JOINTS][GENESIS_SHM_JOINT_NAME_LEN];
  uint64_t boot_stamp_ns;                             /* helps diagnose restarts */
  uint8_t  _pad1[GENESIS_SHM_CACHE_LINE];
} GenesisShmHandshake;

/* Full shared region layout. The shm file is sized to sizeof(GenesisShm)
 * and mmap'd by both sides. */
typedef struct GenesisShm {
  GenesisShmHandshake handshake;
  GenesisShmState     state;
  GenesisShmCommand   command;
} GenesisShm;

/* Default shm basename: actual path is /dev/shm/<prefix><robot_name>. */
#define GENESIS_SHM_PATH_PREFIX "genesis_ros2_control."

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* GENESIS_ROS2_CONTROL_SHM_PROTOCOL_H_ */
