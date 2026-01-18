import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('dont_hit_the_wall')

        # Create a publisher that can send control commands to the simulated robot
        self.publisher_ = self.create_publisher(Twist, '/eduard/blue/cmd_vel', 10)
        
        # Create a timer that executes the timer callback method every 0.1 seconds
        timer_period = 0.1  # 0.1 seconds => 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create subscriber that receives the messages from one range sensor
        self.subscription = self.create_subscription(
            Range,
            '/eduard/blue/range/front/left/range',
            self.front_left_range_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=2)
            )
        self.subscription  # prevent unused variable warning
        self.fl_range = 0.0

    # This method is executed every time a new range measurement is received from the robot
    def front_left_range_callback(self, msg):
        self.fl_range = msg.range
        #self.get_logger().info('Got range value: ' + str(self.fl_range))

    # This method is executed every time the timer interval has passed (0.1 seconds in this example)
    def timer_callback(self):
        vel_msg = Twist()
        if self.fl_range < 0.5:
            vel_msg.linear.x  = 0.0
        else:
            vel_msg.linear.x  = 0.2

        vel_msg.linear.y  = 0.0
        vel_msg.angular.z = 0.0

        self.publisher_.publish(vel_msg)
        self.get_logger().info('Publishing cmd_vel. Range: ' + str(self.fl_range))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()