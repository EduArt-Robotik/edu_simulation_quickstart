import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('dont_hit_the_wall_lidar')

        # Create a publisher that can send control commands to the simulated robot
        self.publisher_ = self.create_publisher(Twist, '/eduard/blue/cmd_vel', 10)
        
        # Create a timer that executes the timer callback method every 0.1 seconds
        timer_period = 0.1  # 0.1 seconds => 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create subscriber that receives the messages from the lidar sensor
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/eduard/blue/scan',
            self.lidar_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=2)
            )
        self.lidar_sub  # prevent unused variable warning

        # Create subscriber that receives the messages from the range sensor
        # We cant get the distance in the back of the robot from the lidar,
        # so we still need the range measurement
        self.rr_range_sub = self.create_subscription(
            Range,
            '/eduard/blue/range/rear/right/range',
            self.rear_right_range_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=2)
            )
        self.rr_range_sub  # prevent unused variable warning

        self.range_front = 0.0
        self.range_left  = 0.0
        self.range_right = 0.0
        self.range_back  = 0.0


    # This method is executed every time a new range measurement is received from the robot
    def rear_right_range_callback(self, msg):
        self.range_back = msg.range if msg.range < 2.0 else float('inf')
        #self.get_logger().info('Got range value: ' + str(self.fl_range))


    # This method is executed every time a new lidar measurement is received from the robot
    def lidar_callback(self, msg):
        
        # Calculate which indices of the LaserScan we need for the front, left and right directions
        # You can also add more lidar measurements in different directions here if you need
        idx_right = int(((-3.1415 / 2.0) - msg.angle_min) / msg.angle_increment) # -90°
        idx_front = int((0.0 - msg.angle_min) / msg.angle_increment) # 0°
        idx_left  = int(((+3.1415 / 2.0) - msg.angle_min) / msg.angle_increment) # +90°

        # Save the front, left and right distances
        self.range_right = msg.ranges[idx_right]
        self.range_front = msg.ranges[idx_front]
        self.range_left  = msg.ranges[idx_left]

        #self.get_logger().info(f'Front: {idx_front:5.2f}, Left: {idx_left:5.2f}, Right: {idx_right:5.2f}')
        #self.get_logger().info(f'Front: {self.range_front:5.2f}, Left: {self.range_left:5.2f}, Right: {self.range_right:5.2f}')


    # This method is executed every time the timer interval has passed (0.1 seconds in this example)
    def timer_callback(self):

        # Here we now got the distances in all 4 directions => We can actually do cool stuff here now

        # Task for you: Drive to the other side of the maze
        # You could just follow one wall until you reached the goal

        vel_msg = Twist()
        if self.range_front < 0.5:
            vel_msg.linear.x  = 0.0
        else:
            vel_msg.linear.x  = 0.2

        vel_msg.linear.y  = 0.0
        vel_msg.angular.z = 0.0

        self.publisher_.publish(vel_msg)
        self.get_logger().info(f'Publishing cmd_vel. Range Front: {self.range_front:5.2f}, Left: {self.range_left:5.2f}, Right: {self.range_right:5.2f}, Back: {self.range_back:5.2f}')


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