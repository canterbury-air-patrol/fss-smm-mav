#pragma once

#include "../fmu-core-types.hpp"
#include <cstdint>

class IFSS
{
  public:
    IFSS () = default;
    IFSS (const IFSS &) = delete;
    IFSS (IFSS &&) = delete;
    auto operator= (const IFSS &) -> IFSS & = delete;
    auto operator= (IFSS &&) -> IFSS & = delete;
    virtual ~IFSS () = default;

    virtual auto getGoto () -> Point = 0;
    virtual auto getAltitude () -> uint16_t = 0;
};
