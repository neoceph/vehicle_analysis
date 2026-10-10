"""Focused steering-input and telemetry-contract checks (Chrono environment)."""

import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd

from analyze_lane_change import _validate_telemetry
from double_lane_change import DoubleLaneChangeSimulation
from lane_change import SteeringProfile
from single_lane_change import SingleLaneChangeSimulation


class SteeringProfileTests(unittest.TestCase):
    def test_single_pair_signs_and_zero_steering_outside_input(self):
        profile = SteeringProfile(math.radians(5.0), 1.0, 3.0)
        self.assertAlmostEqual(profile.angle(1.75), math.radians(5.0))
        self.assertAlmostEqual(profile.angle(3.25), math.radians(-5.0))
        for time in (0.0, 1.0, 2.5, 4.0, 7.0):
            self.assertAlmostEqual(profile.angle(time), 0.0)
        self.assertEqual(profile.phase(0.5), "run_in")
        self.assertEqual(profile.phase(1.0), "outbound")
        self.assertEqual(profile.phase(4.0), "run_out")

    def test_double_pair_returns_with_opposite_steering_and_zero_dwell(self):
        profile = SteeringProfile(math.radians(5.0), 1.0, 3.0, double=True, dwell_s=0.5)
        for elapsed in np.linspace(0.0, 3.0, 101):
            self.assertAlmostEqual(profile.angle(1.0 + elapsed), -profile.angle(4.5 + elapsed))
        self.assertAlmostEqual(profile.angle(4.25), 0.0)
        self.assertEqual(profile.phase(4.25), "dwell")
        self.assertEqual(profile.phase(4.5), "return")
        self.assertEqual(profile.phase(7.5), "run_out")

    def test_negative_amplitude_mirrors_the_full_maneuver(self):
        left = SteeringProfile(0.1, 1.0, 3.0, double=True, dwell_s=0.0)
        right = SteeringProfile(-0.1, 1.0, 3.0, double=True, dwell_s=0.0)
        for time in np.linspace(0.0, 8.0, 201):
            self.assertAlmostEqual(left.angle(time), -right.angle(time))

    def test_zero_angle_rate_at_all_lobe_boundaries(self):
        profile = SteeringProfile(0.1, 1.0, 3.0, double=True, dwell_s=0.5)
        h = 1e-6
        for time in (1.0, 2.5, 4.0, 4.5, 6.0, 7.5):
            with self.subTest(time=time):
                self.assertAlmostEqual(profile.angle(time), 0.0)
                self.assertLess(abs((profile.angle(time + h) - profile.angle(time)) / h), 1e-5)
                self.assertLess(abs((profile.angle(time) - profile.angle(time - h)) / h), 1e-5)

    def test_invalid_profile_parameters(self):
        for values in ((0.1, -1.0, 3.0), (0.1, 1.0, 0.0), (math.nan, 1.0, 3.0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                SteeringProfile(*values)
        with self.assertRaises(ValueError):
            SteeringProfile(0.1, 1.0, 3.0, double=True, dwell_s=-0.1)


class SimulationConfigTests(unittest.TestCase):
    @patch("cornering.ThreeWheelerModel")
    def test_invalid_overrides_fail_before_building_the_vehicle(self, model):
        for simulation in (SingleLaneChangeSimulation, DoubleLaneChangeSimulation):
            for overrides in (
                {"duration_s": 0.0}, {"start_s": -1.0}, {"max_time_s": 1.0},
                {"steering_angle_deg": 46.0}, {"steering_angle_deg": math.inf},
                {"target_speed_kph": 0.0}, {"target_speed_kph": math.nan},
            ):
                with self.subTest(simulation=simulation.__name__, overrides=overrides):
                    with self.assertRaises(ValueError):
                        simulation(**overrides)
        model.assert_not_called()


class TelemetryValidationTests(unittest.TestCase):
    def setUp(self):
        self.path = Path("test_telemetry.csv")
        columns = [
            "steer_cmd_deg", "steer_angle_deg", "speed_kph", "v_cog_mps",
            "drive_torque_Nm", "beta_deg", "yaw_rate_dps", "ay_g", "roll_deg",
            "RI", "RI_model", "CoG_x", "CoG_y",
        ]
        for wheel in ("Wheel_F", "Wheel_RL", "Wheel_RR"):
            columns += [f"{wheel}_{suffix}" for suffix in ("contact_Fz_N", "Fy_wheel_N", "slip_angle_deg")]
        self.df = pd.DataFrame({column: [0.0, 0.0, 0.0] for column in columns})
        self.df["time"] = [0.0, 0.005, 0.01]

    def test_missing_fields_and_nonmonotonic_time_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing telemetry columns"):
            _validate_telemetry(self.df.drop(columns="steer_cmd_deg"), self.path)
        self.df.loc[2, "time"] = 0.005
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            _validate_telemetry(self.df, self.path)

    def test_only_load_based_ri_may_be_undefined_during_loss_of_contact(self):
        self.df.loc[1, "RI"] = math.nan
        _validate_telemetry(self.df, self.path)
        self.df.loc[1, "roll_deg"] = math.nan
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            _validate_telemetry(self.df, self.path)


if __name__ == "__main__":
    unittest.main()
