#pragma once

#include <cstddef>
#include <cstdint>
#include <climits>

#include "mpcore/mpcore.h"

namespace mpcore {

// One vehicle's telemetry, decoded from its messages the way Mission Planner's CurrentState does
// (Parent_OnPacketReceived and the property setters), without I/O: its numeric telemetry, as
// CurrentState's stored values, before its display-unit multipliers and altitude offsets. Frames
// from other vehicles are ignored, except RADIO/RADIO_STATUS (from the radio, for everyone on the
// link), as CurrentState takes them.
// Left in C# for now: everything that reads settings, parameters or the firmware's mode names
// (the mode string, messageHigh, speech, low-airspeed warnings), what needs the home altitude
// (HIGH_LATENCY, HIGH_LATENCY2, POSITION_TARGET_GLOBAL_INT), NAMED_VALUE_FLOAT's named custom
// fields, GIMBAL_DEVICE_ATTITUDE_STATUS (C#'s quaternion), the transponder status, the HIL
// channels, and what CurrentState derives from the wall clock (climb rate without VFR_HUD, mAh from
// SYS_STATUS current, time in the air, log playback speed-up), though the mAh integration is done
// with the caller's clock.
class VehicleState {
public:
    VehicleState(uint32_t sysid, uint8_t compid);

    // Applies a frame; returns whether it changed the state (a message this decodes, for this
    // vehicle). now_us: the caller's clock, for the mAh integration.
    bool on_frame(const uint8_t *frame, std::size_t len, int64_t now_us);

    const mp_vehicle_state &state() const { return s_; }

private:
    void set_current(int index, double value, int64_t now_us);

    uint32_t sysid_;
    uint8_t compid_;
    mp_vehicle_state s_{};
    int64_t last_current_us_[2] = {-1, -1};
    // the downward DISTANCE_SENSOR that gives sonarrange (CurrentState._rangefinderalt_index)
    int rangefinder_index_ = INT32_MAX;
};

}  // namespace mpcore
