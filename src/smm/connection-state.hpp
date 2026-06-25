#pragma once

extern "C"
{
#include <smm-asset.h>
};

inline auto
smm_connection_is_connected (smm_connection conn) -> bool
{
    return conn != nullptr && smm_asset_connection_get_state (conn) == SMM_CONNECTION_CONNECTED;
}
