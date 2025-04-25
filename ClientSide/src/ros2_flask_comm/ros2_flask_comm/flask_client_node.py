import rclpy
from rclpy.node import Node
import requests

class BlenderTriggerNode(Node):
    def __init__(self):
        super().__init__('blender_trigger_node')

        # Flask server URL (change this if necessary)
        self.flask_url = 'http://192.168.56.1:5000/start-animation'  # Ensure this IP matches your Flask server

        # Create a timer to trigger the animation every 5 seconds
        self.timer = self.create_timer(5.0, self.send_start_command)
        self.get_logger().info("🚀 BlenderTriggerNode initialized and ready to send animation trigger!")

    def send_start_command(self):
        try:
            # Send a POST request to the Flask server
            response = requests.post(self.flask_url)
            if response.status_code == 200:
                self.get_logger().info(f"✅ Animation trigger successful: {response.json()}")
            else:
                self.get_logger().error(f"⚠️ Unexpected status from server: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to contact Flask server: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BlenderTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

