#pragma once

#include <cstddef>
#include <cstdint>

#include "mpcore/mpcore.h"

namespace mpcore {

// One vehicle's telemetry, decoded from its messages the way Mission Planner's CurrentState does
// (Parent_OnPacketReceived and the property setters), without I/O. Covers the messages the HUD,
// map and status screens are built from; values are CurrentState's stored values, before its
// display-unit multipliers and altitude offsets. Frames from other vehicles are ignored, except
// RADIO/RADIO_STATUS (from the radio, for everyone on the link) and NAMED_VALUE_FLOAT from another
// component of the same vehicle, as CurrentState takes them.
// Left in C# for now: everything that reads settings, parameters or the firmware's mode names
// (the mode string, messageHigh, speech, low-airspeed warnings), and the values CurrentState derives
// from the wall clock (climb rate without VFR_HUD, mAh integrated from SYS_STATUS current, time in
// the air), though battery mAh integration is done with the caller's clock.
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
};

}  // namespace mpcore
