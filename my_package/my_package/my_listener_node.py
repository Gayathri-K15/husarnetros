# Import the necessary ROS 2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyListener(Node):
    def __init__(self):
        # Initialize the node with the name 'my_listener_node'
        super().__init__('my_listener_node')
        
        # Create a subscriber.
        # It listens for messages of type String on the 'chatter' topic.
        # When a message is received, the 'listener_callback' function is called.
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10) # QoS profile depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message to the console
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our listener node
    my_listener = MyListener()
    
    # "Spin" the node, which keeps it running and allows it to process callbacks
    rclpy.spin(my_listener)
    
    # Clean up and destroy the node when done
    my_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()