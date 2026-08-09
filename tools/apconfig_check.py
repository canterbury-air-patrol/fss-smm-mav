#!/usr/bin/env python3
"""
Verify an autopilot's configuration against what cap-fmu's safety design
depends on.

cap-fmu relies on the autopilot's own GCS/telemetry failsafe as the backstop
for a companion-computer failure (HZ-16): if the FMU stops heartbeating, the
aircraft must return to launch on its own. That backstop is off by default,
watches the wrong system id by default, and on Plane its default action is to
carry on flying -- so it has to be configured deliberately, per airframe, and
verified. This tool does the verifying.

Run it on the ground against a new or reconfigured aircraft, e.g. through a
MAVProxy output:

    tools/apconfig_check.py --device udp:127.0.0.1:14550
    tools/apconfig_check.py --device /dev/ttyUSB0 --baud 57600 \\
        --terminate-action terminate --json evidence.json

Exit status: 0 all checks passed, 1 one or more failed, 2 could not talk to
the aircraft.

The in-flight counterpart is src/mav/failsafe-params.hpp, which warns about
the same configuration on the first heartbeat but never blocks flight. It
checks every expectation below except the failsafe timeouts and SYSID_ENFORCE;
keep the two tables in step. This tool is where a misconfiguration is meant to
be caught.
"""
import argparse
import datetime
import json
import sys
import time

# MAVLink MAV_TYPE values, declared here rather than imported so the
# expectation logic below stays free of pymavlink and can be unit tested
# without it (tools/test_apconfig_check.py). These are protocol constants
# from common.xml and do not change.
MAV_TYPE_FIXED_WING = 1
MAV_TYPE_QUADROTOR = 2
MAV_TYPE_COAXIAL = 3
MAV_TYPE_HELICOPTER = 4
MAV_TYPE_GROUND_ROVER = 10
MAV_TYPE_HEXAROTOR = 13
MAV_TYPE_OCTOROTOR = 14
MAV_TYPE_TRICOPTER = 15

# Vehicle families share a parameter set. The mapping mirrors
# resolve_vehicle_family() in src/mav/failsafe-params.hpp, and the checks
# below mirror expected_failsafe_params(); keep the two in step. The Plane and Copter names were read back by name from
# ArduPilot 4.6 (SITL); the Rover names come from Rover/Parameters.cpp and
# have never been requested from a running vehicle. See this repo's README.
FAMILY_BY_MAV_TYPE = {
    MAV_TYPE_FIXED_WING: 'plane',
    MAV_TYPE_QUADROTOR: 'copter',
    MAV_TYPE_COAXIAL: 'copter',
    MAV_TYPE_HELICOPTER: 'copter',
    MAV_TYPE_HEXAROTOR: 'copter',
    MAV_TYPE_OCTOROTOR: 'copter',
    MAV_TYPE_TRICOPTER: 'copter',
    MAV_TYPE_GROUND_ROVER: 'rover',
}

# cap-fmu's own MAVLink system id (SYS_ID in src/mav/mavlink.cpp). The
# autopilot must watch *this* heartbeat, not the ground station's: under the
# deployed topology MAVProxy runs on the same companion computer and
# heartbeats as 255, so leaving SYSID_MYGCS at its default makes an FMU
# process death invisible to the failsafe.
FMU_SYSTEM_ID = 200

# Longest acceptable failsafe detection time, seconds. The plan's RTL budget
# is measured from this expiring, so a fleet value above it makes TC-FS-001
# unmeetable regardless of how well everything else behaves.
MAX_FAILSAFE_TIMEOUT_S = 5


class Check:
    """
    One parameter expectation: how to judge the value, and why it matters.

    `accept` takes the parameter's value and returns True when the aircraft
    is configured acceptably; `expected` is the same rule in words, for the
    report.
    """
    # pylint: disable=too-few-public-methods
    def __init__(self, param, expected, accept, why):
        self.param = param
        self.expected = expected
        self.accept = accept
        self.why = why


class Result:
    """The outcome of one Check against one aircraft."""
    # pylint: disable=too-few-public-methods
    def __init__(self, check, value):
        self.param = check.param
        self.expected = check.expected
        self.why = check.why
        self.value = value
        # A parameter the aircraft never answered for is a failure, not a
        # skip: "not checked" must never read as "all clear".
        self.passed = value is not None and check.accept(value)

    def as_dict(self):
        """Serialisable form, for the --json evidence record."""
        return {
            'parameter': self.param,
            'value': self.value,
            'expected': self.expected,
            'passed': self.passed,
            'why': self.why,
        }


def _bit_clear(bit):
    return lambda value: not int(value) & (1 << bit)


COMMON_CHECKS = [
    Check('SYSID_MYGCS', f'== {FMU_SYSTEM_ID}',
          lambda value: int(value) == FMU_SYSTEM_ID,
          'cap-fmu heartbeats as this system id; the GCS failsafe must watch it, '
          'not the ground station, or an FMU failure is invisible to the autopilot'),
    Check('SYSID_ENFORCE', '== 0',
          lambda value: int(value) == 0,
          'enforcing would reject every packet from any other system id, '
          "including the pilot's own GCS"),
]

FAMILY_CHECKS = {
    'plane': [
        Check('FS_GCS_ENABL', '1 or 2',
              lambda value: int(value) in (1, 2),
              'GCS failsafe enable; 3 only fails safe in AUTO, and cap-fmu also '
              'flies GUIDED and LOITER'),
        Check('FS_LONG_ACTN', '== 1 (RTL)',
              lambda value: int(value) == 1,
              'the default (0, Continue) means the failsafe fires and the aircraft '
              'carries on flying the mission in AUTO/GUIDED'),
        Check('FS_LONG_TIMEOUT', f'<= {MAX_FAILSAFE_TIMEOUT_S}',
              lambda value: value <= MAX_FAILSAFE_TIMEOUT_S,
              'how long the FMU can be silent before the aircraft returns to launch'),
    ],
    'copter': [
        Check('FS_GCS_ENABLE', '1 or 3',
              lambda value: int(value) in (1, 3),
              'GCS failsafe action: RTL, or SmartRTL falling back to RTL'),
        Check('FS_GCS_TIMEOUT', f'<= {MAX_FAILSAFE_TIMEOUT_S}',
              lambda value: value <= MAX_FAILSAFE_TIMEOUT_S,
              'how long the FMU can be silent before the aircraft returns to launch'),
        Check('FS_OPTIONS', 'bit 1 clear',
              _bit_clear(1),
              'bit 1 continues the mission in AUTO on a GCS failsafe, which is the '
              'same trap as FS_LONG_ACTN=0 on Plane'),
    ],
    'rover': [
        Check('FS_GCS_ENABLE', '== 1',
              lambda value: int(value) == 1,
              '2 continues the mission in Auto instead of failing safe'),
        Check('FS_ACTION', '1 or 3',
              lambda value: int(value) in (1, 3),
              'the default is Hold; RTL (or SmartRTL falling back to RTL) is what '
              'the safety case assumes'),
        Check('FS_TIMEOUT', f'<= {MAX_FAILSAFE_TIMEOUT_S}',
              lambda value: value <= MAX_FAILSAFE_TIMEOUT_S,
              'how long the FMU can be silent before the vehicle fails safe'),
    ],
}

# Only relevant when the FMU is run with --terminate-action=terminate; the
# same pair the in-flight advisory check warns about.
TERMINATE_CHECKS = [
    Check('AFS_ENABLE', '== 1',
          lambda value: int(value) == 1,
          'MAV_CMD_DO_FLIGHTTERMINATION has no effect unless AFS is enabled'),
    Check('AFS_TERM_ACTION', '!= 0',
          lambda value: int(value) != 0,
          'the termination action itself must be configured'),
]


def checks_for(family, terminate_action):
    """
    Every check that applies to this vehicle family and FMU configuration.
    """
    if family not in FAMILY_CHECKS:
        raise ValueError(f'no expectations defined for vehicle family {family!r}')
    checks = list(COMMON_CHECKS) + list(FAMILY_CHECKS[family])
    if terminate_action == 'terminate':
        checks += TERMINATE_CHECKS
    return checks


def evaluate(family, params, terminate_action=None):
    """
    Judge fetched parameter values against the expectations for this family.

    params maps parameter name to value; a name absent from it (or mapped to
    None) is treated as unanswered, and fails.
    """
    return [Result(check, params.get(check.param)) for check in checks_for(family, terminate_action)]


def format_report(results, header=None):
    """
    Render results as a fixed-width table, failures explained underneath.
    """
    lines = []
    if header:
        lines.append(header)
        lines.append('')
    width = max(len(result.param) for result in results)
    for result in results:
        value = 'no reply' if result.value is None else f'{result.value:g}'
        lines.append(f'{"PASS" if result.passed else "FAIL"}  {result.param:<{width}}  '
                     f'{value:>10}  (expected {result.expected})')
    failures = [result for result in results if not result.passed]
    if failures:
        lines.append('')
        for result in failures:
            lines.append(f'{result.param}: {result.why}')
    return '\n'.join(lines)


def fetch_params(conn, names, timeout=10.0):
    """
    Read each named parameter via PARAM_REQUEST_READ.

    Requesting by name (rather than downloading the full parameter set) keeps
    this quick and works through a MAVProxy relay. Returns a name -> value
    mapping; a parameter that never answers is simply absent, which evaluate()
    treats as a failure.
    """
    values = {}
    for name in names:
        deadline = time.monotonic() + timeout
        # Re-request on each pass: over a lossy link the request itself is as
        # likely to go missing as the reply.
        while time.monotonic() < deadline:
            conn.mav.param_request_read_send(
                conn.target_system, conn.target_component, name.encode(), -1)
            msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode(errors='replace')
            param_id = param_id.rstrip('\x00')
            if param_id == name:
                values[name] = msg.param_value
                break
            # A reply for something else: another GCS is walking the parameter
            # list at the same time. Keep waiting for ours.
    return values


def connect(device, baud, source_system, timeout):
    """
    Open the MAVLink connection and wait for the autopilot's heartbeat.

    Returns (connection, heartbeat). Raises RuntimeError if pymavlink is
    missing or if nothing answers.
    """
    try:
        from pymavlink import mavutil  # pylint: disable=import-outside-toplevel
    except ImportError as exc:
        raise RuntimeError(
            'pymavlink is not installed, so this tool cannot talk to an aircraft '
            '(pip install pymavlink). Only the MAVLink I/O needs it; the expectation '
            'logic does not, which is why tools/test_apconfig_check.py runs without '
            'it.') from exc

    conn = mavutil.mavlink_connection(device, baud=baud, source_system=source_system)
    heartbeat = conn.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise RuntimeError(
            f'no heartbeat from {device} within {timeout}s. If this aircraft already has '
            f'SYSID_ENFORCE=1 set, it will not answer a system id it is not expecting '
            f'(this tool used {source_system}); connect through the configured GCS instead.')
    return conn, heartbeat


def main(argv=None):
    """Entry point; returns the process exit status."""
    parser = argparse.ArgumentParser(
        description='Check an autopilot against the configuration cap-fmu depends on.')
    parser.add_argument('--device', required=True,
                        help='MAVLink endpoint, e.g. udp:127.0.0.1:14550, tcp:host:5760, /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=57600, help='serial baud rate (serial devices only)')
    parser.add_argument('--source-system', type=int, default=254,
                        help='system id to speak as; keep it clear of the FMU (200) and the GCS (255)')
    parser.add_argument('--terminate-action', choices=('none', 'disarm', 'terminate'),
                        help='the FMU --terminate-action this airframe is flown with; '
                             '"terminate" adds the AFS checks')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for the heartbeat, and for each parameter')
    parser.add_argument('--json', metavar='PATH',
                        help='write a machine-readable record for the aircraft change log')
    args = parser.parse_args(argv)

    try:
        conn, heartbeat = connect(args.device, args.baud, args.source_system, args.timeout)
    except (RuntimeError, OSError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    family = FAMILY_BY_MAV_TYPE.get(heartbeat.type)
    if family is None:
        print(f'error: no expectations defined for MAV_TYPE {heartbeat.type}; '
              f'this tool covers fixed wing, multirotor and ground rover',
              file=sys.stderr)
        return 2

    params = fetch_params(conn, [check.param for check in checks_for(family, args.terminate_action)],
                          timeout=args.timeout)
    results = evaluate(family, params, args.terminate_action)

    header = (f'{args.device}: {family}, system {conn.target_system}'
              + (f', --terminate-action={args.terminate_action}' if args.terminate_action else ''))
    print(format_report(results, header))

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump({
                'device': args.device,
                'checked': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'family': family,
                'mav_type': heartbeat.type,
                'system_id': conn.target_system,
                'terminate_action': args.terminate_action,
                'passed': all(result.passed for result in results),
                'checks': [result.as_dict() for result in results],
            }, handle, indent=2)
            handle.write('\n')

    return 0 if all(result.passed for result in results) else 1


if __name__ == '__main__':
    sys.exit(main())
