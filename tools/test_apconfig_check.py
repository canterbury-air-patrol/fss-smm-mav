#!/usr/bin/env python3
"""
Unit tests for tools/apconfig_check.py (todo/103).

Covers the expectation logic only -- deliberately no pymavlink and no
aircraft, so this runs anywhere python3 does. The point of the tool is that
a misconfigured airframe fails, so most of these assert the *failing*
direction: a check that cannot fail is worse than no check, because it
reads as evidence.

Run: python3 -m unittest discover -s tools
"""
import unittest

import apconfig_check as apconfig


def passing_params(family):
    """A parameter set that satisfies every check for this family."""
    return {
        'plane': {
            'SYSID_MYGCS': 200, 'SYSID_ENFORCE': 0,
            'FS_GCS_ENABL': 1, 'FS_LONG_ACTN': 1, 'FS_LONG_TIMEOUT': 5,
        },
        'copter': {
            'SYSID_MYGCS': 200, 'SYSID_ENFORCE': 0,
            'FS_GCS_ENABLE': 1, 'FS_GCS_TIMEOUT': 5, 'FS_OPTIONS': 16,
        },
        'rover': {
            'SYSID_MYGCS': 200, 'SYSID_ENFORCE': 0,
            'FS_GCS_ENABLE': 1, 'FS_ACTION': 1, 'FS_TIMEOUT': 5,
        },
    }[family]


def failed(results):
    """Names of the parameters whose checks failed."""
    return {result.param for result in results if not result.passed}


class ConfiguredFleetAircraftPasses(unittest.TestCase):
    """The decided fleet configuration passes for every supported family."""

    def test_every_family_has_a_passing_configuration(self):
        for family in ('plane', 'copter', 'rover'):
            with self.subTest(family=family):
                results = apconfig.evaluate(family, passing_params(family))
                self.assertEqual(failed(results), set())
                self.assertTrue(results)


class StockAutopilotFails(unittest.TestCase):
    """
    An out-of-the-box airframe must fail, and fail on the parameters that
    actually leave it without a backstop -- these are the firmware defaults
    read from ArduPilot 4.6.
    """

    def test_plane_defaults_fail_on_the_failsafe_and_the_sysid(self):
        results = apconfig.evaluate('plane', {
            'SYSID_MYGCS': 255, 'SYSID_ENFORCE': 0,
            'FS_GCS_ENABL': 0, 'FS_LONG_ACTN': 0, 'FS_LONG_TIMEOUT': 5,
        })
        self.assertEqual(failed(results), {'SYSID_MYGCS', 'FS_GCS_ENABL', 'FS_LONG_ACTN'})

    def test_copter_defaults_fail(self):
        results = apconfig.evaluate('copter', {
            'SYSID_MYGCS': 255, 'SYSID_ENFORCE': 0,
            'FS_GCS_ENABLE': 0, 'FS_GCS_TIMEOUT': 5, 'FS_OPTIONS': 16,
        })
        self.assertEqual(failed(results), {'SYSID_MYGCS', 'FS_GCS_ENABLE'})

    def test_rover_defaults_fail(self):
        results = apconfig.evaluate('rover', {
            'SYSID_MYGCS': 255, 'SYSID_ENFORCE': 0,
            'FS_GCS_ENABLE': 0, 'FS_ACTION': 2, 'FS_TIMEOUT': 1.5,
        })
        self.assertEqual(failed(results), {'SYSID_MYGCS', 'FS_GCS_ENABLE', 'FS_ACTION'})


class HalfConfiguredAircraftFails(unittest.TestCase):
    """
    The cases this tool exists for: the failsafe looks enabled but does
    nothing. Each of these passes the in-flight advisory check today
    (todo/104).
    """

    def test_plane_enabled_failsafe_that_continues_the_mission(self):
        params = passing_params('plane') | {'FS_LONG_ACTN': 0}
        self.assertEqual(failed(apconfig.evaluate('plane', params)), {'FS_LONG_ACTN'})

    def test_plane_failsafe_that_only_acts_in_auto(self):
        # FS_GCS_ENABL=3 is "Heartbeat, but only in AUTO"; cap-fmu also flies
        # GUIDED and LOITER, where this would do nothing.
        params = passing_params('plane') | {'FS_GCS_ENABL': 3}
        self.assertEqual(failed(apconfig.evaluate('plane', params)), {'FS_GCS_ENABL'})

    def test_copter_continue_in_auto_option_bit(self):
        # FS_OPTIONS bit 1: continue the mission in AUTO on GCS failsafe.
        params = passing_params('copter') | {'FS_OPTIONS': 2}
        self.assertEqual(failed(apconfig.evaluate('copter', params)), {'FS_OPTIONS'})

    def test_copter_other_option_bits_are_not_confused_with_bit_one(self):
        params = passing_params('copter') | {'FS_OPTIONS': 1 | 4 | 8 | 16 | 32}
        self.assertEqual(failed(apconfig.evaluate('copter', params)), set())

    def test_failsafe_watching_the_ground_station_instead_of_the_fmu(self):
        params = passing_params('plane') | {'SYSID_MYGCS': 255}
        self.assertEqual(failed(apconfig.evaluate('plane', params)), {'SYSID_MYGCS'})

    def test_enforced_sysid_locks_out_the_pilots_gcs(self):
        params = passing_params('plane') | {'SYSID_ENFORCE': 1}
        self.assertEqual(failed(apconfig.evaluate('plane', params)), {'SYSID_ENFORCE'})

    def test_detection_timeout_beyond_the_budget(self):
        params = passing_params('plane') | {'FS_LONG_TIMEOUT': 30}
        self.assertEqual(failed(apconfig.evaluate('plane', params)), {'FS_LONG_TIMEOUT'})
        params = passing_params('copter') | {'FS_GCS_TIMEOUT': 10}
        self.assertEqual(failed(apconfig.evaluate('copter', params)), {'FS_GCS_TIMEOUT'})


class UnansweredParametersFail(unittest.TestCase):
    """"Not checked" must never render as "all clear"."""

    def test_missing_parameter_fails(self):
        params = passing_params('plane')
        del params['FS_LONG_ACTN']
        results = apconfig.evaluate('plane', params)
        self.assertEqual(failed(results), {'FS_LONG_ACTN'})

    def test_empty_reply_set_fails_everything(self):
        results = apconfig.evaluate('plane', {})
        self.assertEqual(len(failed(results)), len(results))

    def test_missing_value_is_reported_as_no_reply(self):
        report = apconfig.format_report(apconfig.evaluate('plane', {}))
        self.assertIn('no reply', report)


class TerminateActionChecks(unittest.TestCase):
    """The AFS pair applies only when the FMU is flown with terminate."""

    def test_afs_not_checked_for_other_actions(self):
        for action in (None, 'none', 'disarm'):
            with self.subTest(action=action):
                params = [check.param for check in apconfig.checks_for('plane', action)]
                self.assertNotIn('AFS_ENABLE', params)

    def test_afs_required_for_terminate(self):
        results = apconfig.evaluate('plane', passing_params('plane'), 'terminate')
        self.assertEqual(failed(results), {'AFS_ENABLE', 'AFS_TERM_ACTION'})

    def test_afs_configured_passes(self):
        params = passing_params('plane') | {'AFS_ENABLE': 1, 'AFS_TERM_ACTION': 42}
        self.assertEqual(failed(apconfig.evaluate('plane', params, 'terminate')), set())


class FamilyResolution(unittest.TestCase):
    """Vehicle family comes from the heartbeat's MAV_TYPE."""

    def test_known_types_map_to_families(self):
        self.assertEqual(apconfig.FAMILY_BY_MAV_TYPE[apconfig.MAV_TYPE_FIXED_WING], 'plane')
        self.assertEqual(apconfig.FAMILY_BY_MAV_TYPE[apconfig.MAV_TYPE_QUADROTOR], 'copter')
        self.assertEqual(apconfig.FAMILY_BY_MAV_TYPE[apconfig.MAV_TYPE_HEXAROTOR], 'copter')
        self.assertEqual(apconfig.FAMILY_BY_MAV_TYPE[apconfig.MAV_TYPE_GROUND_ROVER], 'rover')

    def test_unknown_family_raises_rather_than_passing_vacuously(self):
        with self.assertRaises(ValueError):
            apconfig.checks_for('submarine', None)


class ReportFormatting(unittest.TestCase):
    """The report has to make the failure and its reason obvious."""

    def test_failures_are_explained(self):
        params = passing_params('plane') | {'FS_LONG_ACTN': 0}
        report = apconfig.format_report(apconfig.evaluate('plane', params), header='test aircraft')
        self.assertIn('test aircraft', report)
        self.assertIn('FAIL  FS_LONG_ACTN', report)
        self.assertIn('carries on flying', report)

    def test_passing_report_has_no_explanations(self):
        report = apconfig.format_report(apconfig.evaluate('plane', passing_params('plane')))
        self.assertNotIn('FAIL', report)
        self.assertEqual(report.count('PASS'), 5)

    def test_result_serialises_for_the_evidence_record(self):
        result = apconfig.evaluate('plane', passing_params('plane'))[0].as_dict()
        self.assertEqual(result['parameter'], 'SYSID_MYGCS')
        self.assertEqual(result['value'], 200)
        self.assertTrue(result['passed'])


if __name__ == '__main__':
    unittest.main()
