import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

# USE ROS_DOMAIN_ID:48 befor launch

class RemoteKeyboardNode(Node):
    def __init__(self):
        super().__init__('remote_keyboard_node')
        self.publisher = self.create_publisher(String, 'remote_key_input', 10)
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.get_logger().info("Remote keyboard node started. Press keys to send inputs.")
        
        # Set to same domain ID as main node
        self.declare_parameter('domain_id', 42)
        self._domain_id = self.get_parameter('domain_id').value
        self.get_logger().info(f"Using ROS_DOMAIN_ID: {self._domain_id}")

    def on_key_press(self, key):
        try:
            msg = String()
            msg.data = key.char
            self.publisher.publish(msg)
            self.get_logger().debug(f"Sent key: {msg.data}")
        except AttributeError:
            special_keys = {
                keyboard.Key.enter: 'ENTER',
                keyboard.Key.space: 'SPACE',
                keyboard.Key.backspace: 'BACKSPACE',
                keyboard.Key.delete: 'DELETE'
            }
            if key in special_keys:
                msg = String()
                msg.data = special_keys[key]
                self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RemoteKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()