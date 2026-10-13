from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict


ROOT = Path(__file__).resolve().parent

# Tire parameter file; set THREEWHEELER_TIRE=ThreeWheeler_Bicycle_Tire.json to swap it.
TIRE_FILE = os.environ.get("THREEWHEELER_TIRE", "ThreeWheeler_Tire.json")


def load_json(filename: str) -> Dict[str, Any]:
    path = ROOT / filename
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_all() -> Dict[str, Dict[str, Any]]:
    return {
        "chassis": load_json("ThreeWheeler_Chassis.json"),
        "rear_suspension": load_json("ThreeWheeler_RearSuspension.json"),
        "front_wheel": load_json("ThreeWheeler_FrontWheel.json"),
        "rear_wheel": load_json("ThreeWheeler_RearWheel.json"),
        "tire": load_json(TIRE_FILE),
        "braking": load_json("Braking_Config.json"),
    }
