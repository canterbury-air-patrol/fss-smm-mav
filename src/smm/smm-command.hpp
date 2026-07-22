#pragma once

/* Only the two operator commands the Tier-3 suite exercises (todo/90) get a
 * value here: smm_asset_command's SMM_COMMAND_RTL/GOTO/CIRCLE/CONTINUE/UNKNOWN
 * have no FMU-side meaning distinct from "no SMM command in effect" and are
 * folded into smm_cmd_none rather than modelled 1:1. */
enum SMMCommand
{
    smm_cmd_none,
    smm_cmd_abandon_search,
    smm_cmd_mission_complete,
};
