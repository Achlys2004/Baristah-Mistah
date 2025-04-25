import bpy
import math
import sys
import os
from flask import Flask, jsonify
from threading import Thread

# 1) No UI operators in threads — use data API only.
def run_animation():
    try:
        body = bpy.data.objects["Plane"]
        lid  = bpy.data.objects["Cylinder"]

        start, end = 1, 100
        bpy.context.scene.frame_start = start
        bpy.context.scene.frame_end   = end

        body.animation_data_clear()
        lid .animation_data_clear()

        # Parent safely via data API
        lid.parent = body
        print("✅ Parent set")

        # Bake the float animation
        for f in range(start, end+1):
            bpy.context.scene.frame_set(f)
            z = 1 + math.sin((f / (end - start)) * 2*math.pi)
            body.location.z = z
            body.keyframe_insert("location", index=2)
            lid .keyframe_insert("location", index=2)

        # Linear interpolation
        for obj in (body, lid):
            if obj.animation_data and obj.animation_data.action:
                for fc in obj.animation_data.action.fcurves:
                    for kp in fc.keyframe_points:
                        kp.interpolation = 'LINEAR'

        # If you want playback in GUI mode, uncomment:
        # bpy.context.scene.frame_set(start)
        # bpy.ops.screen.animation_play()

        print("✅ Animation baked")
    except Exception as e:
        print("❌ run_animation error:", e)

# 2) Flask server to listen for triggers
app = Flask(__name__)

@app.route("/start-animation", methods=["POST"])
def start_animation():
    Thread(target=run_animation).start()
    return jsonify({"message": "Animation started!"})

def run_server():
    # Bind to all interfaces so your VM can reach it
    app.run(host="0.0.0.0", port=5000, debug=False)

# 3) When this script is run via `--python`, start the server in a thread
Thread(target=run_server).start()
print("🛰️ Flask server running inside Blender, listening on port 5000")
