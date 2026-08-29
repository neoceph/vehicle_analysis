import bpy
import csv
import os
import pychrono as chrono
import math

class BlenderSceneBuilder:
    def __init__(self, csv_filepath, output_blend, assets):
        self.csv_filepath = os.path.abspath(csv_filepath)
        self.output_blend = os.path.abspath(output_blend)
        self.assets = assets  # Dictionary containing paths to our OBJs and Textures
        self.fps = 60
        self.object_names = ["Chassis", "Wheel_FL", "Wheel_FR", "Wheel_RL", "Wheel_RR"]

    def setup_base_scene(self):
        """Clears the scene, builds the textured road, imports meshes, and sets up the tracking camera."""
        print("Initializing clean Blender scene...")
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.context.scene.render.fps = self.fps

        self._build_road()
        self._import_vehicles()
        self._setup_camera()
        self._setup_lighting()
        self._setup_sky()

    def _setup_camera(self):
        """Creates a camera, positions it behind/above the car, and parents it to the chassis."""
        print("Setting up tracking camera...")
        
        # 1. Create a new camera object
        bpy.ops.object.camera_add(location=(0, 0, 0))
        camera = bpy.context.active_object
        camera.name = "Chasing_Camera"
        
        # 2. Set the camera's local offset relative to the car 
        # (e.g., 8 meters behind, 2.5 meters up for a nice chase-cam view)
        camera.location = (-12.0, 0.0, 2.5)
        
        # 3. Rotate the camera slightly downward to look at the car
        camera.rotation_euler = (math.radians(80), 0, math.radians(-90))
        
        # 4. Find the Chassis object we imported earlier
        chassis = bpy.data.objects.get("Chassis")
        if chassis:
            # Parent the camera to the chassis
            camera.parent = chassis
            print("Camera successfully parented to the Chassis!")
        else:
            print("  -> WARNING: Chassis not found for camera parenting.")

    def _setup_lighting(self):
        """Adds a softened key sun and overhead fill sun to achieve an even daylight look with minimal harsh shadows."""
        print("Setting up multi-point daylight sun sources...")
        import math
        
        # 1. Main Key Sun (Direct sunlight with a wider angle for soft, diffused shadows)
        bpy.ops.object.light_add(type='SUN', location=(400, 0, 50))
        key_light = bpy.context.active_object
        key_light.name = "Daylight_Key_Sun"
        key_light.data.energy = 3.0
        key_light.data.angle = math.radians(15.0) # Softens shadow edges
        key_light.rotation_euler = (math.radians(50), math.radians(20), math.radians(0))
        
        # 2. Overhead Fill Sun (Washes out deep dark shadows from the opposite angle)
        bpy.ops.object.light_add(type='SUN', location=(400, 0, 50))
        fill_light = bpy.context.active_object
        fill_light.name = "Daylight_Fill_Sun"
        fill_light.data.energy = 2.0
        fill_light.rotation_euler = (math.radians(-20), math.radians(-50), math.radians(180))

    def _setup_sky(self):
        """Sets up a realistic Hosek/Wilkie sky background compatible with Eevee."""
        print("Setting up procedural sky background...")
        
        world = bpy.context.scene.world
        if not world:
            world = bpy.data.worlds.new("World")
            bpy.context.scene.world = world
            
        world.use_nodes = True
        nodes = world.node_tree.nodes
        links = world.node_tree.links
        nodes.clear()
        
        bg_node = nodes.new(type='ShaderNodeBackground')
        bg_node.inputs['Strength'].default_value = 1.2  # Ambient fill strength
        
        sky_node = nodes.new(type='ShaderNodeTexSky')
        sky_node.sky_type = 'HOSEK_WILKIE'  # Compatible with Eevee & standalone bpy
        
        output_node = nodes.new(type='ShaderNodeOutputWorld')
        links.new(sky_node.outputs['Color'], bg_node.inputs['Color'])
        links.new(bg_node.outputs['Background'], output_node.inputs['Surface'])

    def _build_road(self):
        """Creates a 1000m road and applies the asphalt texture once without tiling or scaling."""
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
            
            # Set extension to 'CLIP' so it doesn't repeat infinitely like a grid
            tex_image.extension = 'CLIP'
            
            # Connect Texture Coordinates directly to the Image Texture (No Mapping/Tiling node)
            tex_coord = nodes.new('ShaderNodeTexCoord')
            
            links.new(tex_coord.outputs['UV'], tex_image.inputs['Vector'])
            links.new(tex_image.outputs['Color'], bsdf.inputs['Base Color'])
        else:
            print(f"  -> WARNING: {tex_path} not found. Road will be solid gray.")
            bsdf.inputs['Base Color'].default_value = (0.2, 0.2, 0.2, 1)

        road.data.materials.append(mat)

    def _import_vehicles(self):
        """Imports the actual Chrono OBJs, or falls back to blocks if missing."""
        print("Importing vehicle meshes...")
        
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
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sedan_dir = os.path.join(script_dir, "data", "vehicle", "sedan")

    # Build exact paths to the meshes inside the Conda environment
    # Build paths to the local meshes and textures
    assets = {
        "chassis_obj": os.path.join(sedan_dir, "sedan_chassis_vis.obj"),
        "wheel_obj": os.path.join(sedan_dir, "sedan_tire.obj"),
        "road_texture": "realistic_road.jpg"  # Still local to your project folder
        "road_texture": os.path.join(script_dir, "realistic_road.jpg")
    }
    
    input_csv = "results/simulation_telemetry.csv"
    output_blend = "results/animated_sedan.blend"
    input_csv = os.path.join(script_dir, "results", "simulation_telemetry.csv")
    output_blend = os.path.join(script_dir, "results", "animated_sedan.blend")

    builder = BlenderSceneBuilder(input_csv, output_blend, assets)
    builder.setup_base_scene()
    builder.apply_kinematics()
    builder.save_file()