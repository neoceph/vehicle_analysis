"""Single lane change with a prescribed front-wheel steer/countersteer pair."""

from lane_change import LaneChangeSimulation, run_cli


class SingleLaneChangeSimulation(LaneChangeSimulation):
    pass


CONFIG_FILE = SingleLaneChangeSimulation.config_file
DEFAULT_OUTPUT = SingleLaneChangeSimulation.default_output


if __name__ == "__main__":
    run_cli(SingleLaneChangeSimulation)
