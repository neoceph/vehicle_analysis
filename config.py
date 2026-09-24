from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict


ROOT = Path(__file__).resolve().parent


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
        "tire": load_json("ThreeWheeler_Tire.json"),
        "braking": load_json("Braking_Config.json"),
    }
