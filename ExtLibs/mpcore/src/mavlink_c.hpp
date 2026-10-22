#pragma once

// The only header that pulls in the generated MAVLink C code; include it from .cpp files only.
// mpcore uses caller-owned parser and status buffers, never the per-channel globals, so any
// number of links can run side by side. The helpers are static inline, so their global C names
// stay local to the including file. (MAVLINK_USE_CXX_NAMESPACE does not compile: protocol.h
// uses the types outside the namespace.)

// Signatures are verified by mpcore against a key ring, as Mission Planner does, not by the
// generated parser (which would need a signing struct per channel).
#define MAVLINK_NO_SIGNATURE_CHECK

#include MPCORE_MAVLINK_DIALECT_HEADER
