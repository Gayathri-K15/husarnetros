# Import the necessary ROS 2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # We will publish a String message

class MyPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'my_publisher_node'
        super().__init__('my_publisher_node')
        
        # Create a publisher. 
        # It publishes messages of type 'String' on the topic 'chatter'
        # The queue size of 10 is for QoS (Quality of Service)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create a timer that calls the timer_callback function every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # A counter for our messages
        self.i = 0

    def timer_callback(self):
        # Create a new String message
        msg = String()
        msg.data = f'Hello from my_package: {self.i}'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment the counter
        self.i += 1

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our publisher node
    my_publisher = MyPublisher()
    
    # "Spin" the node, which keeps it running and allows it to process callbacks
    rclpy.spin(my_publisher)
    
    # Clean up and destroy the node when done (e.g., on Ctrl+C)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()