# 🤖 Blender Animation Control via ROS2 & Flask

This project integrates a **ROS 2 Humble node**, a **Flask HTTP server**, and a **Blender 4.3 animation system** to remotely control bone movements and visibility in a Blender scene using RESTful commands.

---

## 📌 Project Overview

- **Goal:** Trigger and orchestrate complex Blender animations via ROS2 messages sent over HTTP.
- **Setup:**
  - ROS 2 runs on a **Ubuntu VM (guest)**.
  - Blender with Flask server runs on the **Windows host**.
- **Communication:** ROS2 client sends animation commands to a Flask server embedded inside Blender.

---

## 🧠 System Architecture

```
+----------------------+        HTTP POST        +----------------------------+
|  ROS2 Node (Ubuntu)  |  -------------------->  | Flask Server inside Blender |
|  blender_anim_commander.py |                  |  Controls bone animation     |
+----------------------+                        +----------------------------+
```

---

## 🚀 Features

- **Remote Bone Rotation** via keyframe insertion
- **Visibility Control** of Blender objects (e.g., show/hide cup)
- **Sequenced Animations** with custom duration and frame management
- **Play & Auto Reset** animation logic
- **Clear Functionality** to reset the timeline

---

## 📂 Folder Structure

```
project-root/
├── ClientSide/
│   ├── src/
│   │   └── ros2_flask_comm/
│   │       ├── ros2_flask_comm/
│   │       │   └── blender_client.py       # ROS2 client node
│   │       └── setup.py
└── ServerSide/
    └── flask_blender_server.py                      # Runs inside Blender
```

---

## 🛠 Requirements

### Host (Windows)

- Blender 4.3+
- Flask (`pip install flask` inside Blender's Python)
- Custom bone-rigged Blender scene
- `flask_blender_server.py` run from Blender Text Editor

### Guest (Ubuntu VM)

- ROS 2 Humble
- Python 3.10+
- Internet/network access to host machine

---

## ⚙️ Usage

### 1. Setup Blender

- Place `flask_blender_server.py` inside Blender’s Text Editor.
- Ensure the rig (armature) and objects (`Cup`, `Cap`) are named correctly.
- Run the script to start Flask on port 5000.

### 2. Verify Network

Make sure your guest (VM) can reach host IP:

```bash
curl http://<host-ip>:5000
```

### 3. Run ROS 2 Node

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run ros2_flask_comm blender_client
```

### 4. Watch It Happen

Blender will:

- Accept move/visibility commands
- Play animation
- Reset scene

---

## 📜 Sample REST Endpoints

| Endpoint | Method | Description             |
| -------- | ------ | ----------------------- |
| `/move`  | POST   | Apply rotation to bones |
| `/pick`  | POST   | Show `Cup` and `Cap`    |
| `/serve` | POST   | Hide `Cup` and `Cap`    |
| `/play`  | POST   | Play animation timeline |
| `/clear` | POST   | Clear all keyframes     |

---

## 🧩 Sample Payload

```json
{
  "bone_names": ["ArmShort", "ArmShortest"],
  "delta_rotations": [
    [60, 0, 0],
    [-60, 0, 0]
  ],
  "durations": [1, 1]
}
```

---

## 🔧 Debugging Tips

- ❗ **If Flask doesn't start**, make sure the path to custom Python modules is added correctly.
- 🛑 **If bones don’t move**, verify their names match those used in the payload.
- 🌐 **If Flask isn’t reachable**, double-check host IP and network settings (use Bridged/NAT with port forwarding).
- 🔌 **To check Connection**, There are 2 files provided: `ClientSide\src\ros2_flask_comm\ros2_flask_comm\flask_client_node.py` and `ServerSide\flask_connection.py` to check the connection. (Make sure to change the setup.py `blender_client = ros2_flask_comm.blender_client:main` -> `blender_client = ros2_flask_comm.flask_client_node:main` and run using `ros2 run ros2_flask_comm flask_client_node` )

---

## ✨ Credits

- Built by **Accuracy.exe** and **Achlys**
- Uses Blender, Flask, and ROS2 in a tightly integrated animation pipeline.
