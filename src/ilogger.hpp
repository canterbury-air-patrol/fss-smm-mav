#pragma once

#include "fmu-core-types.hpp"
#include <string_view>

/* Thin seam over Logger (src/logger.hpp) so subsystems (MAV, SMM,
 * known_aircraft) can route their runtime diagnostics to the persistent,
 * rotating fmu.log instead of std::cout/std::cerr (todo/59), without their
 * unit tests having to construct a real file-backed Logger. */
class ILogger
{
  public:
    ILogger () = default;
    ILogger (const ILogger &) = delete;
    ILogger (ILogger &&) = delete;
    auto operator= (const ILogger &) -> ILogger & = delete;
    auto operator= (ILogger &&) -> ILogger & = delete;
    virtual ~ILogger () = default;

    /* Log at an explicit level; emitted only if it passes the logger's
     * configured verbosity. */
    virtual void log (LogLevel level, std::string_view msg) = 0;
    /* Log at info level. */
    void
    log (std::string_view msg)
    {
        log (LogLevel::info, msg);
    }
};
