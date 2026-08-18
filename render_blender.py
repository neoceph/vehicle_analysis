import bpy
import os
import tempfile
import subprocess
import shutil
import time

def render_and_compile_video(blend_filepath, output_video_path):
    start_total_time = time.time()  # Start global timer
    
    abs_blend_path = os.path.abspath(blend_filepath)
    abs_output_path = os.path.abspath(output_video_path)
    
    os.makedirs(os.path.dirname(abs_output_path), exist_ok=True)
    
    if not os.path.exists(abs_blend_path):
        print(f"ERROR: Cannot find Blender file at {abs_blend_path}")
        return

    print(f"Loading Blender file: {abs_blend_path}")
    bpy.ops.wm.open_mainfile(filepath=abs_blend_path)
    
    scene = bpy.context.scene
    
    # Ensure the chasing camera is active (pre-configured in the blend file)
    camera = bpy.data.objects.get("Chasing_Camera")
    if camera:
        scene.camera = camera
        print("-> Chasing_Camera successfully set as active camera.")
    else:
        print("-> WARNING: 'Chasing_Camera' not found!")
        
    # Render settings (PNG frame output)
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.render.resolution_percentage = 100 
    
    # Create temp directory in the script's root directory
    script_root = os.path.dirname(os.path.abspath(__file__))
    temp_dir = tempfile.mkdtemp(dir=script_root, prefix="temp_render_frames_")
    print(f"Created temporary frame directory: {temp_dir}")
    
    try:
        scene.render.filepath = os.path.join(temp_dir, "frame_")
        
        # Track time specifically for the frame rendering process
        print("Rendering animation frames...")
        render_start_time = time.time()
        
        bpy.ops.render.render(animation=True)
        
        render_duration = time.time() - render_start_time
        print(f"Frames rendered successfully in {render_duration:.2f} seconds.")
        
        # Compile frames into MP4 using FFmpeg and track compile time
        print(f"Compiling video into {abs_output_path}...")
        compile_start_time = time.time()
        
        ffmpeg_cmd = [
            "ffmpeg", "-y",
            "-framerate", str(scene.render.fps),
            "-i", os.path.join(temp_dir, "frame_%04d.png"),
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            abs_output_path
        ]
        
        result = subprocess.run(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        compile_duration = time.time() - compile_start_time
        
        if result.returncode != 0:
            print("ERROR: FFmpeg failed to compile the video.")
            print(result.stderr)
        else:
            total_duration = time.time() - start_total_time
            print(f"\n==========================================")
            print(f"SUCCESS! Video saved to: {abs_output_path}")
            print(f"Render Time:   {render_duration:.2f} seconds")
            print(f"Compile Time:  {compile_duration:.2f} seconds")
            print(f"Total Elapsed: {total_duration:.2f} seconds")
            print(f"==========================================")
            
    finally:
        print(f"Cleaning up temporary directory: {temp_dir}")
        shutil.rmtree(temp_dir, ignore_errors=True)
        print("Cleanup complete.")

if __name__ == "__main__":
    blend_file = "results/animated_sedan.blend"
    output_video = "results/sedan_braking_animation.mp4"
    
    render_and_compile_video(blend_file, output_video)