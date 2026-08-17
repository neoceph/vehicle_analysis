import bpy
import csv

class BlenderTelemetryImporter:
    def __init__(self, csv_filepath):
        self.csv_filepath = csv_filepath
        self.fps = 60 # Set your desired Blender frame rate
        bpy.context.scene.render.fps = self.fps
        
        # Map Chrono objects to exact names of your imported Blender objects
        self.object_names = ["Chassis", "Wheel_FL", "Wheel_FR", "Wheel_RL", "Wheel_RR"]

    def apply_keyframes(self):
        print(f"Loading telemetry from: {self.csv_filepath}")
        
        # Ensure all objects use Quaternion rotation, not Euler
        for name in self.object_names:
            if name in bpy.data.objects:
                bpy.data.objects[name].rotation_mode = 'QUATERNION'

        with open(self.csv_filepath, 'r') as file:
            reader = csv.DictReader(file)
            
            for row in reader:
                time = float(row["time"])
                frame = int(time * self.fps)
                
                # Apply transforms for the Chassis and 4 Wheels
                for name in self.object_names:
                    obj = bpy.data.objects.get(name)
                    if not obj:
                        continue # Skip if you haven't imported the mesh yet
                    
                    # Read position
                    obj.location.x = float(row[f"{name}_x"])
                    obj.location.y = float(row[f"{name}_y"])
                    obj.location.z = float(row[f"{name}_z"])
                    
                    # Read rotation (Blender is W,X,Y,Z. Chrono is e0(W), e1(X), e2(Y), e3(Z))
                    obj.rotation_quaternion.w = float(row[f"{name}_qw"])
                    obj.rotation_quaternion.x = float(row[f"{name}_qx"])
                    obj.rotation_quaternion.y = float(row[f"{name}_qy"])
                    obj.rotation_quaternion.z = float(row[f"{name}_qz"])
                    
                    # Insert keyframes
                    obj.keyframe_insert(data_path="location", frame=frame)
                    obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                    
        print("Keyframes successfully imported!")

# --- Execution ---
# CHANGE THIS PATH TO YOUR ACTUAL CSV LOCATION
CSV_FILE_PATH = r"C:\path\to\your\simulation_telemetry.csv" 

importer = BlenderTelemetryImporter(CSV_FILE_PATH)
importer.apply_keyframes()