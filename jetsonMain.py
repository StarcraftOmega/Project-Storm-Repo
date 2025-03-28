import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

# USE ROS_DOMAIN_ID:48 befor launch

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        # Subscription to receive inputs from pcmain
        self.remote_sub = self.create_subscription(
            String,
            'remote_key_input',
            self.remote_input_callback,
            10
        )
        
        # Publisher to share inputs with other nodes
        self.command_pub = self.create_publisher(
            String, 
            'key_input', 
            10
        )
        
        self.active_processes = []
        self.current_group = None
        self.get_logger().info("Main system ready - bridging inputs")

    def remote_input_callback(self, msg):
        # Handle system commands internally
        if msg.data in ['1', '2', '-']:
            self.handle_system_command(msg.data)
        # Publish all inputs to other nodes
        self.command_pub.publish(msg)
        self.get_logger().debug(f"Relayed input: {msg.data}")

    def handle_system_command(self, command):
        self.get_logger().info(f"Processing system command: {command}")
        if command == '1':
            self.launch_group(1, [
                ('joystick_cotrol/src', 'publisher_joystick'),
                ('package2', 'node2_executable')
            ])
        elif command == '2':
            self.launch_group(2, [
                ('package3', 'node3_executable'),
                ('package4', 'node4_executable')
            ])
        elif command == '-':
            self.terminate_current_group()

    def launch_group(self, group_id, nodes):
        self.terminate_current_group()
        self.current_group = group_id
        for package, executable in nodes:
            process = subprocess.Popen(
                ['ros2', 'run', package, executable],
                env={'ROS_DOMAIN_ID': '42'}  # Match domain ID
            )
            self.active_processes.append(process)
        self.get_logger().info(f'Node group {group_id} activated')

    def terminate_current_group(self):
        if self.active_processes:
            for process in self.active_processes:
                process.terminate()
                process.wait()
            self.active_processes = []
            self.get_logger().info(f'Node group {self.current_group} terminated')
            self.current_group = None

    def destroy_node(self):
        self.terminate_current_group()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
