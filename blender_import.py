import bpy
import csv
import os
import pychrono as chrono

class BlenderSceneBuilder:
    def __init__(self, csv_filepath, output_blend, assets):
        self.csv_filepath = os.path.abspath(csv_filepath)
        self.output_blend = os.path.abspath(output_blend)
        self.assets = assets  # Dictionary containing paths to our OBJs and Textures
        self.fps = 60
        self.object_names = ["Chassis", "Wheel_FL", "Wheel_FR", "Wheel_RL", "Wheel_RR"]

    def setup_base_scene(self):
        """Clears the scene, builds the textured road, and imports the Chrono meshes."""
        print("Initializing clean Blender scene...")
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.context.scene.render.fps = self.fps

        self._build_road()
        self._import_vehicles()

    def _build_road(self):
        """Creates a 1000m road and applies the asphalt texture with proper UV scaling."""
        print("Building realistic road...")
        # Create a 1000m x 100m plane. Offset X by 400 so the car has room to drive forward.
        bpy.ops.mesh.primitive_plane_add(size=1.0, location=(400, 0, 0))
        road = bpy.context.active_object
        road.name = "Road"
        road.scale = (1000.0, 100.0, 1.0)
        
        # Create Material
        mat = bpy.data.materials.new(name="Asphalt_Material")
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        
        bsdf = nodes.get("Principled BSDF")
        bsdf.inputs['Roughness'].default_value = 0.85 # Make road look matte, not shiny

        # Check if texture exists, then apply it
        tex_path = os.path.abspath(self.assets["road_texture"])
        if os.path.exists(tex_path):
            tex_image = nodes.new('ShaderNodeTexImage')
            tex_image.image = bpy.data.images.load(tex_path)
            
            # Map the UVs so the texture tiles seamlessly (matching the 4.0 scale from PyChrono)
            # Plane is 1000x100. If 1 tile = 4m, we need 250 tiles on X, and 25 tiles on Y.
            mapping = nodes.new('ShaderNodeMapping')
            mapping.inputs['Scale'].default_value = (250.0, 25.0, 1.0)
            
            tex_coord = nodes.new('ShaderNodeTexCoord')
            
            links.new(tex_coord.outputs['UV'], mapping.inputs['Vector'])
            links.new(mapping.outputs['Vector'], tex_image.inputs['Vector'])
            links.new(tex_image.outputs['Color'], bsdf.inputs['Base Color'])
        else:
            print(f"  -> WARNING: {tex_path} not found. Road will be solid gray.")
            bsdf.inputs['Base Color'].default_value = (0.2, 0.2, 0.2, 1)

        road.data.materials.append(mat)

    def _import_vehicles(self):
        """Imports the actual Chrono OBJs, or falls back to blocks if missing."""
        print("Importing vehicle meshes...")
        import math
        
        # --- CHASSIS ---
        chassis_path = os.path.abspath(self.assets["chassis_obj"])
        if os.path.exists(chassis_path):
            # 1. Deselect everything in the scene so we don't accidentally join the road!
            bpy.ops.object.select_all(action='DESELECT')
            
            # 2. Import the OBJ (which spawns multiple loose parts like glass, body, etc.)
            bpy.ops.wm.obj_import(filepath=chassis_path)
            
            # 3. Select all newly imported pieces and join them into one solid object
            imported_objects = bpy.context.selected_objects
            if imported_objects:
                # Make the first piece the "active" one, then merge the rest into it
                bpy.context.view_layer.objects.active = imported_objects[0]
                bpy.ops.object.join()
                
                chassis = bpy.context.view_layer.objects.active
                chassis.name = "Chassis"
                
                # 4. Correct the Blender/Chrono axis mismatch on the unified car
                chassis.rotation_euler = (math.radians(0), 0, 0)
                # Shift the visual mesh down to counteract the physics Center of Mass height.
                # (You can tweak this -0.5 value slightly if it needs to go higher or lower)
                chassis.location = (0, 0, -0.25)
                # Bake BOTH the rotation and the location offset permanently into the vertices
                bpy.ops.object.transform_apply(location=True, rotation=True, scale=False)
        else:
            print("  -> WARNING: Chassis OBJ not found. Using placeholder.")
            bpy.ops.mesh.primitive_cube_add(size=1.0)
            chassis = bpy.context.active_object
            chassis.name = "Chassis"
            chassis.scale = (4.0, 1.8, 1.0)
            
        chassis.rotation_mode = 'QUATERNION'

        # --- WHEELS ---
        wheel_path = os.path.abspath(self.assets["wheel_obj"])
        for name in self.object_names[1:]:
            if os.path.exists(wheel_path):
                bpy.ops.object.select_all(action='DESELECT')
                bpy.ops.wm.obj_import(filepath=wheel_path)
                
                # Join wheel parts just in case the tire and rim are separated
                imported_wheels = bpy.context.selected_objects
                if imported_wheels:
                    bpy.context.view_layer.objects.active = imported_wheels[0]
                    bpy.ops.object.join()
                    wheel = bpy.context.view_layer.objects.active
            else:
                if name == "Wheel_FL": print("  -> WARNING: Wheel OBJ not found. Using cylinders.")
                bpy.ops.mesh.primitive_cylinder_add(radius=0.35, depth=0.25)
                wheel = bpy.context.active_object
                wheel.rotation_euler[0] = 1.5708
                bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
                
            wheel.name = name
            wheel.rotation_mode = 'QUATERNION'

    def apply_kinematics(self):
        """Reads the CSV and applies position/rotation keyframes."""
        if not os.path.exists(self.csv_filepath):
            print(f"ERROR: Cannot find telemetry CSV at {self.csv_filepath}")
            return

        print(f"Applying physics keyframes from: {self.csv_filepath}")
        with open(self.csv_filepath, 'r') as file:
            reader = csv.DictReader(file)
            
            for row in reader:
                time = float(row["time"])
                frame = int(time * self.fps)
                
                for name in self.object_names:
                    obj = bpy.data.objects.get(name)
                    if not obj: continue
                    
                    obj.location.x = float(row[f"{name}_x"])
                    obj.location.y = float(row[f"{name}_y"])
                    obj.location.z = float(row[f"{name}_z"])
                    
                    obj.rotation_quaternion.w = float(row[f"{name}_qw"])
                    obj.rotation_quaternion.x = float(row[f"{name}_qx"])
                    obj.rotation_quaternion.y = float(row[f"{name}_qy"])
                    obj.rotation_quaternion.z = float(row[f"{name}_qz"])
                    
                    obj.keyframe_insert(data_path="location", frame=frame)
                    obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

        bpy.context.scene.frame_end = frame

    def save_file(self):
        """Saves the .blend file."""
        os.makedirs(os.path.dirname(self.output_blend), exist_ok=True)
        # Pack external data (like the road texture) directly into the .blend file
        bpy.ops.file.pack_all()
        bpy.ops.wm.save_as_mainfile(filepath=self.output_blend)
        print(f"\nSUCCESS! Blender file generated at: {self.output_blend}")


if __name__ == "__main__":
    # Programmatically ask PyChrono where it installed its data files!
    chrono_data_dir = chrono.GetChronoDataPath()
    sedan_dir = os.path.join(chrono_data_dir, "vehicle", "sedan")

    # Build exact paths to the meshes inside the Conda environment
    assets = {
        "chassis_obj": os.path.join(sedan_dir, "sedan_chassis_vis.obj"),
        "wheel_obj": os.path.join(sedan_dir, "sedan_tire.obj"),
        "road_texture": "realistic_road.jpg"  # Still local to your project folder
    }
    
    input_csv = "results/simulation_telemetry.csv"
    output_blend = "results/animated_sedan.blend"

    builder = BlenderSceneBuilder(input_csv, output_blend, assets)
    builder.setup_base_scene()
    builder.apply_kinematics()
    builder.save_file()