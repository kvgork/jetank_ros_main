#!/usr/bin/env python3
"""
Differential Drive Test Script for JeTank Simulation

Tests basic movement commands to verify diff_drive_controller is working correctly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')

        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry feedback
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_pose = None
        self.get_logger().info('Drive Test Node initialized')

    def odom_callback(self, msg):
        """Store current odometry for position tracking"""
        self.current_pose = msg.pose.pose

    def publish_velocity(self, linear_x, angular_z, duration):
        """Publish velocity command for specified duration"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz

        while (time.time() - start_time) < duration:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()

    def stop(self):
        """Stop the robot"""
        msg = Twist()
        self.publisher.publish(msg)
        time.sleep(0.5)

    def wait_for_odom(self, timeout=5.0):
        """Wait for odometry messages to start"""
        self.get_logger().info('Waiting for odometry...')
        start_time = time.time()

        while self.current_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_pose is None:
            self.get_logger().error('Odometry timeout! Is the simulation running?')
            return False

        self.get_logger().info('Odometry received!')
        return True

    def test_forward(self):
        """Test forward motion"""
        self.get_logger().info('=== Test 1: Forward Motion ===')
        self.get_logger().info('Driving forward at 0.2 m/s for 2 seconds...')

        initial_pose = self.current_pose
        self.publish_velocity(0.2, 0.0, 2.0)
        self.stop()

        if self.current_pose:
            dx = self.current_pose.position.x - initial_pose.position.x
            dy = self.current_pose.position.y - initial_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f'Distance traveled: {distance:.3f} m (expected ~0.4 m)')
            self.get_logger().info('✓ Forward motion test complete\n')

    def test_backward(self):
        """Test backward motion"""
        self.get_logger().info('=== Test 2: Backward Motion ===')
        self.get_logger().info('Driving backward at -0.2 m/s for 2 seconds...')

        initial_pose = self.current_pose
        self.publish_velocity(-0.2, 0.0, 2.0)
        self.stop()

        if self.current_pose:
            dx = self.current_pose.position.x - initial_pose.position.x
            dy = self.current_pose.position.y - initial_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f'Distance traveled: {distance:.3f} m (expected ~0.4 m)')
            self.get_logger().info('✓ Backward motion test complete\n')

    def test_rotation_left(self):
        """Test left rotation"""
        self.get_logger().info('=== Test 3: Left Rotation ===')
        self.get_logger().info('Rotating left at 0.5 rad/s for 3.14 seconds (90 degrees)...')

        self.publish_velocity(0.0, 0.5, 3.14)
        self.stop()

        self.get_logger().info('✓ Left rotation test complete\n')

    def test_rotation_right(self):
        """Test right rotation"""
        self.get_logger().info('=== Test 4: Right Rotation ===')
        self.get_logger().info('Rotating right at -0.5 rad/s for 3.14 seconds (90 degrees)...')

        self.publish_velocity(0.0, -0.5, 3.14)
        self.stop()

        self.get_logger().info('✓ Right rotation test complete\n')

    def test_arc(self):
        """Test curved motion (arc)"""
        self.get_logger().info('=== Test 5: Arc Motion ===')
        self.get_logger().info('Driving in arc (linear + angular) for 3 seconds...')

        self.publish_velocity(0.2, 0.3, 3.0)
        self.stop()

        self.get_logger().info('✓ Arc motion test complete\n')

    def run_all_tests(self):
        """Run complete test suite"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('JeTank Differential Drive Test Suite')
        self.get_logger().info('='*50 + '\n')

        if not self.wait_for_odom():
            self.get_logger().error('Cannot proceed without odometry!')
            return False

        time.sleep(1)

        try:
            self.test_forward()
            time.sleep(1)

            self.test_backward()
            time.sleep(1)

            self.test_rotation_left()
            time.sleep(1)

            self.test_rotation_right()
            time.sleep(1)

            self.test_arc()

            self.get_logger().info('='*50)
            self.get_logger().info('All tests completed successfully! ✓')
            self.get_logger().info('='*50)

            return True

        except Exception as e:
            self.get_logger().error(f'Test failed with error: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)

    tester = DriveTest()

    try:
        success = tester.run_all_tests()

        if success:
            tester.get_logger().info('\n✓ Differential drive controller is working correctly!')
        else:
            tester.get_logger().error('\n✗ Tests failed. Check simulation and controller status.')

    except KeyboardInterrupt:
        tester.get_logger().info('\nTest interrupted by user')
    finally:
        tester.stop()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
