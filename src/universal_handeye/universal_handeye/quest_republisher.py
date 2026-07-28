import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalRepublisher(Node):
    """
    A simple node that subscribes to a PoseStamped topic and republishes
    it to another topic.
    """
    def __init__(self):
        super().__init__('goal_republisher')

        # Declare parameters for topic names
        self.declare_parameter('input_pose_topic', '/goal_pose')
        self.declare_parameter('output_pose_topic', '/quest/right/pose')

        input_topic = self.get_parameter('input_pose_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_pose_topic').get_parameter_value().string_value

        # Create the publisher
        self.publisher_ = self.create_publisher(PoseStamped, output_topic, 10)

        # Create the subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            input_topic,
            self.listener_callback,
            10)
        
        self.get_logger().info(f"Republishing from '{input_topic}' to '{output_topic}'")

    def listener_callback(self, msg):
        """
        Callback function for the subscriber. It receives a message and
        publishes it to the new topic.
        """
        self.get_logger().debug(f"Received pose on '{self.subscription.topic}', republishing...")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    goal_republisher = GoalRepublisher()
    try:
        rclpy.spin(goal_republisher)
    except KeyboardInterrupt:
        pass
    goal_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

