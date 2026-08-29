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

## Docker: a rebuild of one SHA may still differ; the manifest says how

**Do not pin `libfss-client-ssl` or `libsmm-asset-dev` to exact versions, and do
not treat two builds of one SHA as interchangeable without comparing their
manifests.**

The question todo/113 required an answer to: *is a rebuild from an unchanged git
SHA permitted to produce a different artifact?* The answer is **yes, within a
bounded window, and the image records which one it is.**

What is now fixed by the source:

- The base is a digest, not a tag (`ARG BASE_IMAGE` in `docker/Dockerfile`), and
  `.github/workflows/verify.yml` runs every job on that same digest.
  `tools/check-image-pins.sh` fails `check-code.sh` if the two stop agreeing.
- Neither the Dockerfile nor `docker/setup.sh` runs `apt upgrade -y` any more.
  It was what made a base pin ineffective — pin the base, then upgrade every
  package in it, and the pin describes nothing. Base security updates now arrive
  by bumping the digest, which is a reviewable commit.
- The Canterbury Air Patrol libraries are installed with `apt-get satisfy` under
  the bounds `configure.ac` states, derived from it by
  `docker/dep-constraints.sh` rather than copied. A release outside the window
  fails the build instead of being installed.

What is *not* fixed, and deliberately:

`apt.canterburyairpatrol.org` is a rolling repository that publishes exactly one
version of each package. Pinning `libfss-client-ssl=1.3.0` would make every
image build fail the day 1.3.1 is published, because 1.3.0 is no longer *in* the
repository to install — an exact pin against a rolling repository is not a pin,
it is an expiry date. So the install is bounded, not pinned, and a rebuild after
an upstream release resolves to the new version if it is inside the window.

That is why `/etc/cap-fmu-manifest` exists (`docker/manifest.sh`). It records
the base digest, the source revision, and every installed package with its
version, inside the image, so:

```
docker run --rm --entrypoint cat canterburyairpatrol/cap-fmu:<tag> \
  /etc/cap-fmu-manifest
```

answers "what was flying" with no rebuild and no access to the repository's
history — which a rebuild could not answer anyway, since it would resolve to
whatever is current now. CI extracts the same file as a build artifact next to
the traceability report. Everything under the manifest's `# packages` heading is
the comparable part; the header above it carries a build timestamp that differs
on every build by construction.

The stronger option — building a `.deb` and installing that, so dependency
solving is recorded in package metadata — was not taken. It moves the same
unpinned resolution into `debian/control`'s `Depends` and adds a packaging step
to the image build, for a record the manifest already provides. Revisit it if
the image ever needs to be installable rather than merely reproducible.

### The two stages must agree, and are compared rather than pinned

todo/114 split the image into a builder and a runtime stage, which gives the
rolling repository a second bite: the builder resolves `libfss-client-ssl` for
the compile, the runtime stage resolves it again minutes later for the shipped
filesystem, and a release published in between satisfies the same window at both
ends. Nothing would report it. The image would fly a library the binary was
never linked against, and the manifest — correctly recording the runtime
stage — would say so without anything noticing.

Pinning the runtime stage to the builder's exact versions is the obvious fix and
is the expiry date again, one stage further in. So the runtime stage installs
under the same derived window and then **compares**:
`docker/dep-constraints.sh --versions` writes what the builder linked against,
`docker/runtime-setup.sh` diffs the runtime stage's resolution against it and
fails the build on any difference. A mid-build release is then a build failure
with an explanation, and rebuilding takes the new version in both stages.

The comparison is keyed by pkg-config module, not by Debian package, because the
two stages deliberately install different packages for `smm-asset`
(`libsmm-asset-dev` to compile against, `libsmmasset0` to run against). They are
published from one source at one version, which is what makes the two version
strings comparable; if that ever stops being true this check is where it
surfaces.
