"""Double lane change with an outbound pair and its reversed return pair."""

from lane_change import LaneChangeSimulation, run_cli


class DoubleLaneChangeSimulation(LaneChangeSimulation):
    double = True
    config_file = "DoubleLaneChange_Config.json"
    default_output = "results/double_lane_change_telemetry.csv"
    test_name = "double_lane_change"
    run_description = "double lane-change"


CONFIG_FILE = DoubleLaneChangeSimulation.config_file
DEFAULT_OUTPUT = DoubleLaneChangeSimulation.default_output


if __name__ == "__main__":
    run_cli(DoubleLaneChangeSimulation)
