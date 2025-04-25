import sys
import os

custom_module_path = os.path.join(
    os.getenv("APPDATA"), "Blender Foundation", "Blender", "4.3", "scripts", "module"
)
if custom_module_path not in sys.path:
    sys.path.append(custom_module_path)
import bpy
from math import radians
import threading
from flask import Flask, request, jsonify

# Setup
armature_name = "Armature"
arm = bpy.data.objects[armature_name]
bpy.context.view_layer.objects.active = arm
bpy.ops.object.mode_set(mode="POSE")

scene = bpy.context.scene
fps = scene.render.fps

# Initialize
if not hasattr(scene, "barista_current_frame"):
    scene["barista_current_frame"] = 1
if not hasattr(scene, "barista_bone_rotations"):
    scene["barista_bone_rotations"] = {}


def move(bone_names, delta_rotations, durations):
    frame_start = int(scene["barista_current_frame"])
    max_duration = max(durations)
    frame_end = frame_start + int(max_duration * fps)

    for name, (dx, dy, dz), dur in zip(bone_names, delta_rotations, durations):
        frame_target = frame_start + int(dur * fps)
        bone = arm.pose.bones[name]
        bone.rotation_mode = "XYZ"
        last_rot = scene["barista_bone_rotations"].get(name)
        if last_rot is None:
            last_rot = tuple(radians(a) for a in bone.rotation_euler[:])
        new_rot = (
            last_rot[0] + radians(dx),
            last_rot[1] + radians(dz),
            last_rot[2] + radians(dy),
        )
        bone.rotation_euler = last_rot
        bone.keyframe_insert(data_path="rotation_euler", frame=frame_start)
        bone.rotation_euler = new_rot
        bone.keyframe_insert(data_path="rotation_euler", frame=frame_target)
        scene["barista_bone_rotations"][name] = new_rot

    scene["barista_current_frame"] = frame_end
    scene.frame_start = 1
    scene.frame_end = frame_end
    scene.frame_set(1)


def set_visibility(object_name, visible, frame):
    obj = bpy.data.objects[object_name]
    obj.hide_viewport = not visible
    obj.hide_render = not visible
    obj.keyframe_insert(data_path="hide_viewport", frame=frame)
    obj.keyframe_insert(data_path="hide_render", frame=frame)


def pick(frame):  # show cup & cap
    set_visibility("Cup", True, frame)
    set_visibility("Cap", True, frame)


def serve(frame):  # hide cup & cap
    set_visibility("Cup", False, frame)
    set_visibility("Cap", False, frame)


def clear_animations():
    # Remove all keyframes from bones
    for bone in arm.pose.bones:
        if bone.animation_data:
            bone.animation_data_clear()

    # Remove all keyframes from objects (including Cup, Cap, etc.)
    for obj in bpy.data.objects:
        if obj.animation_data:
            obj.animation_data_clear()

    # Clear the custom stored data (bone rotations, current frame)
    scene["barista_bone_rotations"].clear()
    scene["barista_current_frame"] = 1

    # Reset the scene's start and end frames
    scene.frame_start = 1
    scene.frame_end = 1
    scene.frame_set(1)


def play_and_reset():
    start = scene.frame_start
    end = scene.frame_end
    fps = scene.render.fps
    frame_duration = 1.0 / fps

    def advance():
        current = scene.frame_current
        if current >= end:
            bpy.ops.screen.animation_cancel(restore_frame=False)
            scene.frame_set(1)  # Reset the scene frame to 1
            scene["barista_current_frame"] = 1
            scene[
                "barista_bone_rotations"
            ].clear()  # Clear the bone rotations after reset
            return None
        else:
            scene.frame_set(current + 1)
            return frame_duration

    scene.frame_set(start)
    bpy.app.timers.register(
        lambda: bpy.ops.screen.animation_play() or None
    )  # Start playback
    bpy.app.timers.register(advance)  # Auto-reset logic


# Flask Server
app = Flask(_name_)


@app.route("/move", methods=["POST"])
def handle_move():
    data = request.get_json()
    bpy.app.timers.register(
        lambda: move(data["bone_names"], data["delta_rotations"], data["durations"])
        or None
    )
    return jsonify({"status": "movement scheduled"})


@app.route("/pick", methods=["POST"])
def handle_pick():
    bpy.app.timers.register(lambda: pick(scene["barista_current_frame"]) or None)
    return jsonify({"status": "pick triggered"})


@app.route("/serve", methods=["POST"])
def handle_serve():
    bpy.app.timers.register(lambda: serve(scene["barista_current_frame"]) or None)
    return jsonify({"status": "serve triggered"})


@app.route("/clear", methods=["POST"])
def handle_clear():
    bpy.app.timers.register(lambda: clear_animations() or None)
    return jsonify({"status": "animations cleared, ready to play again"})


@app.route("/play", methods=["POST"])
def handle_play():
    bpy.app.timers.register(lambda: play_and_reset() or None)
    return jsonify({"status": "playback started"})


# Start server
def start_server():
    print("🛰 Flask server starting on port 5000")
    app.run(host="0.0.0.0", port=5000)


threading.Thread(target=start_server, daemon=True).start()
