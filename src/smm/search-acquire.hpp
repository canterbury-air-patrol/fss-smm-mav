#pragma once
#include <cstdint>

/* Pure decision for the pre-fetch step of SMM::tryAcquireSearch(), factored out
 * so the "never call into the SMM C library with a null asset" rule can be unit
 * tested without a live SMM connection or the smm-asset library.
 *
 * tryAcquireSearch() can be reached from reportPosition() whenever a search is
 * still active but not yet acquired, including after SMM disconnects or asset
 * discovery fails. smm_asset_get_search() relies on undefined behaviour when
 * handed a null asset, so the fetch must be gated on the asset being present. */
enum class SearchAcquireAction
{
    /* The retry timer has not elapsed yet; do nothing this tick. */
    backoff,
    /* No asset (SMM disconnected / discovery failed): command a safe RTL and
     * back off. The caller keeps the search active so it is retried once the
     * asset is rediscovered after reconnect. */
    disconnected_rtl,
    /* Asset present and not backing off: it is safe to fetch a search. */
    fetch,
};

inline auto
search_acquire_action (bool asset_present, uint64_t retry_ts, uint64_t now) -> SearchAcquireAction
{
    if (retry_ts > now)
    {
        return SearchAcquireAction::backoff;
    }
    if (!asset_present)
    {
        return SearchAcquireAction::disconnected_rtl;
    }
    return SearchAcquireAction::fetch;
}
