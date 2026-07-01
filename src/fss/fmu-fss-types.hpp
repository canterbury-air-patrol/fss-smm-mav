#pragma once

#include "../fmu-core-types.hpp"
#include <cstdint>

enum FSSCommand
{
    fss_cmd_unknown,
    fss_cmd_manual,
    fss_cmd_rtl,
    fss_cmd_hold,
    fss_cmd_altitude,
    fss_cmd_goto,
    fss_cmd_continue,
    fss_cmd_disarm,
    fss_cmd_terminate,
};

/* The target a goto/altitude command carries. Delivered together with its
 * FSSCommand (in the FSSCommandEvent and through FSSNewCommand) so a command and
 * its target travel as a single unit, rather than the target arriving via a
 * separate FSS side-channel that the state machine re-reads at action time
 * (todo/53). Fields stay default for commands that carry no target; only the one
 * matching the command is meaningful (position for goto, altitude for altitude). */
struct FSSCommandTarget
{
    Point position{};
    /* Feet, as delivered on the FSS wire (fss_message_asset_command::getAltitude
     * is uint32_t). Kept full-width here so the value is narrowed only by the
     * regulatory [floor, cap] clamp in clamp_command_altitude(), never by a lossy
     * cast that could wrap a large altitude down to a low one before the clamp
     * ever sees it (todo/55). */
    uint32_t altitude{ 0 };
};

enum FSSCommsStatus
{
    fss_comms_unknown,
    fss_comms_okay,
    fss_comms_failure,
};