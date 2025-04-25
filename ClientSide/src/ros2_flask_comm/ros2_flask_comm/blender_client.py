import rclpy
from rclpy.node import Node
import requests
import time

class BlenderAnimationCommander(Node):
    def __init__(self):
        super().__init__('blender_anim_commander')
        self.url = 'http://192.168.56.1:5000'  # Your host IP where Flask is running

        self.sequence = [
            ("serve", None),
            ("move", {"bone_names": ["ArmShort", "ArmShortest"], "delta_rotations": [[60, 0, 0], [-60, 0, 0]], "durations": [1, 1]}),
            ("move", {"bone_names": ["Base"], "delta_rotations": [[0, 0, -30]], "durations": [1]}),
            ("move", {"bone_names": ["ArmShort", "ArmShortest"], "delta_rotations": [[-70, 0, 0], [-20, 0, 0]], "durations": [1, 1]}),
            ("pick", None),
            ("move", {"bone_names": ["ArmShort", "ArmShortest"], "delta_rotations": [[20, 0, 0], [-19, 0, 0]], "durations": [1, 1]}),
            ("move", {"bone_names": ["Base", "ArmShortest", "ArmShort", "ArmLong"], "delta_rotations": [[0, 0, 60], [190, 0, 0], [5, 0, 0], [0, 0, 15]], "durations": [1, 0.5, 1, 1]}),
            ("move", {"bone_names": ["ArmLong", "ArmShort", "ArmShortest"], "delta_rotations": [[0, 0, -10], [-20, 0, 0], [10, 0, 0]], "durations": [1, 1, 1]}),
            ("move", {"bone_names": ["ArmShort"], "delta_rotations": [[0, 0, 0]], "durations": [2]}),
            ("move", {"bone_names": ["ArmLong", "ArmShort", "ArmShortest"], "delta_rotations": [[0, 0, 15], [35, 0, 0], [-20, 0, 0]], "durations": [1, 1, 1]}),
            ("move", {"bone_names": ["Base", "ArmLong", "ArmShortest"], "delta_rotations": [[0, 0, -30], [0, 0, 0], [0, 0, 0]], "durations": [1, 1, 1]}),
            ("move", {"bone_names": ["ArmLong", "ArmShort", "ArmShortest"], "delta_rotations": [[0, 0, -20], [-50, 0, 0], [30, 0, 0]], "durations": [1, 1, 1]}),
            ("move", {"bone_names": ["ArmShort"], "delta_rotations": [[0, 0, 0]], "durations": [3]}),
            ("serve", None),
            ("move", {"bone_names": ["ArmShort"], "delta_rotations": [[0, 0, 0]], "durations": [3]}),
            ("move", {"bone_names": ["ArmShort", "ArmShortest"], "delta_rotations": [[90, 0, 0], [-170, 0, 0]], "durations": [1, 1]}),
            ("move", {"bone_names": ["ArmShort"], "delta_rotations": [[0, 0, 0]], "durations": [3]}),
        ]

        self.send_sequence()

    def send_sequence(self):
        for endpoint, payload in self.sequence:
            url = f"{self.url}/{endpoint}"
            try:
                if payload:
                    response = requests.post(url, json=payload)
                else:
                    response = requests.post(url)
                self.get_logger().info(f"✅ {endpoint.upper()} | {response.status_code} | {response.json()}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to call {endpoint}: {e}")
            time.sleep(2)

        self.start_animation()

    def start_animation(self):
        try:
            url = f"{self.url}/play"
            response = requests.post(url)
            self.get_logger().info(f"✅ ANIMATION STARTED | {response.status_code} | {response.json()}")
            time.sleep(5)
            self.clear_animation()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start animation: {e}")

    def clear_animation(self):
        try:
            url = f"{self.url}/clear"
            response = requests.post(url)
            self.get_logger().info(f"🧹 CLEARED | {response.status_code} | {response.json()}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to clear animation: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BlenderAnimationCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

