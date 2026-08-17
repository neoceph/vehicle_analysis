import pychrono as chrono
import pychrono.vehicle as veh
import pandas as pd
from pathlib import Path

class BrakingSimulation:
    def __init__(self, target_speed_kph=100.0, road_friction=0.9, step_size=2e-3):
        self.target_speed = target_speed_kph / 3.6
        self.road_friction = road_friction
        self.step_size = step_size
        self.max_time = 60.0
        self.telemetry_data = []
        
        self._setup_model()

    def _setup_model(self):
        """Initializes the PyChrono Sedan and terrain."""
        veh.SetVehicleDataPath(chrono.GetChronoDataPath() + 'vehicle/')
        
        self.sedan = veh.Sedan()
        self.sedan.SetContactMethod(chrono.ChContactMethod_NSC)
        self.sedan.SetChassisFixed(False)
        # Offset Y by -2.0 so we spawn in the right lane
        self.sedan.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -2.0, 0.5), chrono.QUNIT))
        self.sedan.SetTireType(veh.TireModelType_TMEASY)
        self.sedan.Initialize()

        # NOTE: Visualization types are intentionally left default to avoid the PyChrono 10.0 NONE bug.
        # Since we don't attach an Irrlicht window, it runs headless anyway!

        self.terrain = veh.RigidTerrain(self.sedan.GetSystem())
        mat = chrono.ChContactMaterialNSC()
        mat.SetFriction(self.road_friction)
        self.terrain.AddPatch(mat, chrono.ChCoordsysd(chrono.VNULL, chrono.QUNIT), 1000.0, 100.0)
        self.terrain.Initialize()

        cls = getattr(veh, "DriverInputs", None) or getattr(veh, "ChDriverInputs")
        self.inputs = cls()
        
        # Grab pointers to wheels for Blender export
        self.spindles = {
            "Wheel_FL": self.sedan.GetVehicle().GetWheel(0, veh.LEFT).GetSpindle(),
            "Wheel_FR": self.sedan.GetVehicle().GetWheel(0, veh.RIGHT).GetSpindle(),
            "Wheel_RL": self.sedan.GetVehicle().GetWheel(1, veh.LEFT).GetSpindle(),
            "Wheel_RR": self.sedan.GetVehicle().GetWheel(1, veh.RIGHT).GetSpindle(),
        }

    def run(self):
        """Executes the maneuver and logs data at 200 Hz."""
        chassis = self.sedan.GetVehicle().GetChassisBody()
        phase = "accelerate"
        last_sample_time = -1e9
        output_hz = 200.0
        
        print("Starting headless physics simulation...")
        while self.sedan.GetSystem().GetChTime() < self.max_time:
            time = self.sedan.GetSystem().GetChTime()
            v_abs = chassis.GetPosDt()
            speed = chassis.TransformDirectionParentToLocal(v_abs).x

            if phase == "accelerate":
                self.inputs.m_throttle, self.inputs.m_braking = 1.0, 0.0
                if speed >= self.target_speed:
                    phase = "brake"
            elif phase == "brake":
                self.inputs.m_throttle, self.inputs.m_braking = 0.0, 1.0
                if speed <= 0.1:
                    break

            # Log Telemetry at desired Hz
            if time - last_sample_time >= (1.0 / output_hz) - (0.5 * self.step_size):
                pos, rot = chassis.GetPos(), chassis.GetRot()
                
                # Base chassis data
                frame_data = {
                    "time": time, "speed_mps": speed, "brake_input": self.inputs.m_braking,
                    "Chassis_x": pos.x, "Chassis_y": pos.y, "Chassis_z": pos.z,
                    "Chassis_qw": rot.e0, "Chassis_qx": rot.e1, "Chassis_qy": rot.e2, "Chassis_qz": rot.e3
                }
                
                # Add wheel data
                for name, spindle in self.spindles.items():
                    w_pos, w_rot = spindle.GetPos(), spindle.GetRot()
                    frame_data.update({
                        f"{name}_x": w_pos.x, f"{name}_y": w_pos.y, f"{name}_z": w_pos.z,
                        f"{name}_qw": w_rot.e0, f"{name}_qx": w_rot.e1, f"{name}_qy": w_rot.e2, f"{name}_qz": w_rot.e3
                    })
                    
                self.telemetry_data.append(frame_data)
                last_sample_time = time

            self.terrain.Synchronize(time)
            self.sedan.Synchronize(time, self.inputs, self.terrain)
            self.terrain.Advance(self.step_size)
            self.sedan.Advance(self.step_size)

    def export_results(self, output_path="results/simulation_telemetry.csv"):
            # Convert the string path to a Path object
            out_file = Path(output_path)
            
            # Ensure the parent directory ('results/') exists before saving
            out_file.parent.mkdir(parents=True, exist_ok=True)
            
            # Convert data to DataFrame and save to the new path
            df = pd.DataFrame(self.telemetry_data)
            df.to_csv(out_file, index=False)
            
            print(f"Simulation complete. Telemetry saved to {out_file.resolve()}")

if __name__ == "__main__":
    sim = BrakingSimulation()
    sim.run()
    sim.export_results()