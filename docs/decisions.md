# Settled decisions

Findings that have been investigated and deliberately **rejected**, recorded so
they are not re-proposed. Each entry states the finding as it was raised, why it
is not a defect, and what not to change.

This is the durable home for that reasoning. Items here were previously tracked
as `todo/` entries; those files were removed once closed, since `todo/` holds
open work.

## The repository name is `fss-smm-mav`; the product is `cap-fmu`

**Do not "fix" the clone URL, the `cd` target, or `debian/copyright`'s
`Source:`.**

The GitHub repository is still named `fss-smm-mav` even though the product and
installed binary are `cap-fmu`. So all of these are correct as written:

- `README.md`'s build instructions —
  `git clone https://github.com/canterbury-air-patrol/fss-smm-mav.git`
  followed by `cd fss-smm-mav`.
- `debian/copyright`'s
  `Source: https://github.com/canterbury-air-patrol/fss-smm-mav`, alongside
  `Upstream-Name: cap-fmu`.

This reads like stale documentation left behind by a rename, and has been filed
as such. It is not: the repository was never renamed. A review that flags the
mismatch is working from that false premise. If the repository is renamed
later, these change together — and this entry stops applying.

## CI: `develop` is the only trigger branch

**Do not add `master` or tag triggers to `.github/workflows/build.yml`.**

Build and Test triggers only on push/PR to `develop`. That is deliberate, not an
oversight: `master` only ever fast-forwards from a green `develop`, so release
commits are already gated by the run that passed on `develop`. Adding `master`
and tag triggers would re-run the same verified commits without proving anything
new.

Note this covers *branch triggers only*. Whether Docker **publication** is
mechanically tied to a tested SHA was a separate question, and it is now
answered: `docker-build.yml` calls the same reusable `verify.yml` and gates its
publish job on it with `needs:`, so release branches and version tags do run
the full gate set on the exact commit being published (todo/87, closed
2026-08-02). That was always about artifact gating, and was correctly *not*
resolved by adding redundant `master` triggers here — this entry still stands
as written.

## CI: the duplicated apt-setup block stays duplicated

**Do not factor it into a composite action.**

All four verification jobs (`build`, `distcheck`, `sanitizers`,
`static-analysis`) repeat the same block to add the CAP apt repository and its
signing key. The duplication is intentional; a composite-action refactor is not
planned.

The related half of that finding *was* actioned: every one of those jobs fetches
the repository and key over `https://` (946079b), not plain HTTP.

(Those jobs moved from `build.yml` into the reusable `verify.yml` when Docker
publication was gated on them, todo/87. The duplication, and this decision about
it, are unchanged — only the file they live in.)

## Pure helper headers declare at global scope, not in a namespace

**Do not wrap `log-rotation.hpp`, `altitude-cap.hpp`, `search-altitude.hpp` and
friends in a namespace.**

Raised on PR 242 against `log-rotation.hpp`'s `RotationState` /
`next_rotation_state`: a dedicated (or `detail`) namespace would avoid symbol
collisions and clarify intended scope.

Declined, because it would make one header inconsistent with every other one
like it. The tree has ten of these small pure-logic headers — `altitude-cap`,
`search-altitude`, `search-acquire`, `mission-plan`, `mode-resolve`,
`mav-comms`, `velocity`, `battery-voltage`, `latlon-encoding`, `aircraft` —
and **none** of them uses a namespace. They export types (`SearchAcquireAction`,
`MissionItemKind`, `MissionPlanMode`, `MissionItem`) and functions
(`over_altitude_cap`, `clamp_search_altitude`, `next_synthetic_icao`) at global
scope. `log-rotation.hpp` was written to match them deliberately; adopting a
namespace for it alone buys nothing and costs the consistency that makes the
pattern recognisable.

This is a single-binary project with no public API and no third-party consumers
linking against it, so the collision risk the suggestion guards against does not
arise. If the convention is ever revisited it should be revisited for all ten
headers in one change, not one at a time as each new one is reviewed.
