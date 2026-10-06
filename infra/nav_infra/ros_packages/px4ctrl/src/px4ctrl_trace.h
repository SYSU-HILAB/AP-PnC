#ifndef _PX4CTRL_TRACE_H_
#define _PX4CTRL_TRACE_H_

#include <atomic>
#include <chrono>
#include <cstdio>

/**
 * Zero-dependency structured trace emitter for px4ctrl (l2 decision trace).
 * One JSON line per FSM event on stdout, collected alongside the legacy text
 * logs for post-flight segmentation of autonomy windows.
 *
 * Vocabulary (layer "fsm"):
 *   state_transition {from, to, cause_branch}  — every FSM edge, including
 *     the pilot/system terminal branches that close an autonomy trace:
 *       CMD_CTRL->AUTO_HOVER   rc_command_mode_off | cmd_timeout
 *       CMD_CTRL->MANUAL_CTRL  not_armed | rc_hover_mode_off | odom_timeout
 *   takeoff_completed  — AUTO_TAKEOFF reached target height (opens a window)
 *   land_completed     — AUTO_LAND finished on the ground (closes completed)
 *   trigger_sent       — planner start trigger sent, user command allowed
 *   takeoff_retry_ready / land_ground_wait — one-shot readiness notes
 *   planner_reset_failed — /planner/reset unavailable (5 s-throttled with the
 *     legacy WARN; the success path stays legacy-only: routine while
 *     MANUAL_CTRL-locked, and its message text needs escaping)
 *
 * Kept as legacy ROS logging on purpose: the per-tick ROS_ERROR reject guards
 * (emitting them as events would flood the seq space) and the 5 s battery
 * status line. Field values here are fixed identifiers only — no JSON
 * escaping is required or performed.
 */
namespace px4ctrl_trace
{

  /**
   * Wall-clock timestamp in nanoseconds (matches std::chrono wall time; all
   * processes on one device share the kernel clock).
   * @return Current wall time [ns]
   */
  inline long long now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  /**
   * Monotonic per-process event counter (function-local static: C++11-safe).
   * @return Next sequence number, starting at 1
   */
  inline unsigned long next_seq()
  {
    static std::atomic<unsigned long> counter{0};
    return counter.fetch_add(1) + 1;
  }

  /**
   * Emit one trace event with the shared envelope.
   * @param[in] event Event name (lowercase snake_case)
   */
  inline void emit(const char *event)
  {
    std::printf(
        "{\"seq\":%lu,\"ts_ns\":%lld,\"level\":\"info\",\"event\":\"%s\","
        "\"node\":\"px4ctrl\",\"layer\":\"fsm\"}\n",
        next_seq(), now_ns(), event);
    std::fflush(stdout);
  }

  /**
   * Emit one FSM state transition event.
   * @param[in] from         Source state name (e.g. "CMD_CTRL")
   * @param[in] to           Target state name (e.g. "AUTO_HOVER")
   * @param[in] cause_branch Which branch of the source state's guard chain
   * fired
   */
  inline void transition(const char *from, const char *to,
                         const char *cause_branch)
  {
    std::printf(
        "{\"seq\":%lu,\"ts_ns\":%lld,\"level\":\"info\",\"event\":\"state_"
        "transition\",\"node\":\"px4ctrl\",\"layer\":\"fsm\",\"from\":\"%s\","
        "\"to\":\"%s\",\"cause_branch\":\"%s\"}\n",
        next_seq(), now_ns(), from, to, cause_branch);
    std::fflush(stdout);
  }

}  // namespace px4ctrl_trace

#endif  // _PX4CTRL_TRACE_H_
