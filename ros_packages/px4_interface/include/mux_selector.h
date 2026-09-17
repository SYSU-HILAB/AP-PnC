/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Sun Yat-sen University.
 * All rights reserved.
 *
 * Authors:
 * Hanamy: rongerch@outlook.com
 *
 * Handover selection logic between the px4ctrl stream (hover/FSM control)
 * and the NMPC rates bridge stream, shared with the gtest suite.
 */

#pragma once

#include <algorithm>

/**
 * State machine selecting which setpoint stream drives mavros.
 *
 * Policy: the /mode trigger latches NMPC intent; the NMPC stream is only
 * selected while it stays fresh. Any staleness falls back to the px4ctrl
 * stream so offboard setpoint continuity is preserved.
 */
class MuxSelector
{
 public:
  enum class Source
  {
    kPx4ctrl,
    kNmpc,
  };

  /**
   * Advance the selection state.
   *
   * @param[in] mode_trigger True latches NMPC intent [-]
   * @param[in] nmpc_stream_age Age of the newest NMPC setpoint [s]
   * @param[in] now Current time [s]
   * @return The stream to forward this cycle [-]
   */
  Source update(bool mode_trigger, double nmpc_stream_age, double now)
  {
    if (mode_trigger)
    {
      nmpc_latched_ = true;
    }
    if (nmpc_latched_ && nmpc_stream_age < stream_timeout_)
    {
      source_ = Source::kNmpc;
    }
    else
    {
      source_ = Source::kPx4ctrl;
    }
    last_update_time_ = now;
    return source_;
  }

  /** @return Currently selected stream [-] */
  Source source() const { return source_; }

  /** @return True while NMPC is selected [-] */
  bool isNmpcActive() const { return source_ == Source::kNmpc; }

  /**
   * Drop the NMPC latch (operator reset or trajectory end handling).
   *
   * @param[in] now Current time [s]
   */
  void reset(double now)
  {
    nmpc_latched_     = false;
    source_           = Source::kPx4ctrl;
    last_update_time_ = now;
  }

  /** @param[in] timeout NMPC stream staleness threshold [s] */
  void setStreamTimeout(double timeout)
  {
    stream_timeout_ = std::max(timeout, 0.01);
  }

 private:
  double stream_timeout_   = 0.2;
  bool   nmpc_latched_     = false;
  Source source_           = Source::kPx4ctrl;
  double last_update_time_ = 0.0;
};
