#pragma once
#include <cstddef>

/* Pure rotation-accounting helper for Logger::writeLine(), factored out so the
 * "did this actually reach the disk" decision is unit-testable without a real
 * failing filesystem. Mirrors altitude-cap.hpp / search-altitude.hpp's style.
 *
 * Rotation is destructive: it shifts fmu.log.1..5 along and discards the
 * oldest. Driving it from bytes that were never written means a log device that
 * has stopped accepting writes (full disk, disconnected mount) erases the
 * existing log history while producing nothing to replace it --- the failure
 * mode that makes the logs matter is the one that would delete them. So a
 * failed write must leave the byte count exactly where it was. */

struct RotationState
{
    /* Byte count to carry forward for the current file. */
    std::size_t bytes_written = 0;
    /* Whether the caller should now rotate and reopen. */
    bool rotate = false;
};

/* Account for one attempted write of `line_size` bytes against a file that
 * already holds `bytes_written`, rotating at `max_log_bytes`. A `write_ok` of
 * false leaves the count untouched and never rotates. */
inline auto
next_rotation_state (bool write_ok, std::size_t bytes_written, std::size_t line_size, std::size_t max_log_bytes)
    -> RotationState
{
    if (!write_ok)
    {
        return RotationState{ bytes_written, false };
    }
    std::size_t total = bytes_written + line_size;
    return RotationState{ total, total >= max_log_bytes };
}
