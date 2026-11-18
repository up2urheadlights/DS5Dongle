#pragma once

#include <cstddef>
#include <cstdint>

namespace port {

// The config record has a 4 KB flash sector of its own, separate from
// BTstack's bond store (see board_haute.h for the map). Dropping or wiping
// link keys therefore cannot touch settings, and a config write cannot touch
// a bond.
constexpr size_t kConfigStorageSize = 4096;
constexpr size_t kConfigStoragePageSize = 256;

} // namespace port
