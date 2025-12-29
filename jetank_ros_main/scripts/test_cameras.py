#!/usr/bin/env python3
"""
Stereo Camera Validation Script for JeTank Simulation

Validates that stereo cameras are publishing correctly with proper synchronization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import time


class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')

        # Image subscribers
        self.left_image_sub = self.create_subscription(
            Image,
            '/stereo_camera/left/image_raw',
            self.left_image_callback,
            10
        )
        self.right_image_sub = self.create_subscription(
            Image,
            '/stereo_camera/right/image_raw',
            self.right_image_callback,
            10
        )

        # Camera info subscribers
        self.left_info_sub = self.create_subscription(
            CameraInfo,
            '/stereo_camera/left/camera_info',
            self.left_info_callback,
            10
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo,
            '/stereo_camera/right/camera_info',
            self.right_info_callback,
            10
        )

        # Storage for received messages
        self.left_image_count = 0
        self.right_image_count = 0
        self.left_info = None
        self.right_info = None
        self.left_timestamps = []
        self.right_timestamps = []

        self.get_logger().info('Camera Test Node initialized')

    def left_image_callback(self, msg):
        """Count left camera images and store timestamps"""
        self.left_image_count += 1
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.left_timestamps.append(timestamp)

    def right_image_callback(self, msg):
        """Count right camera images and store timestamps"""
        self.right_image_count += 1
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.right_timestamps.append(timestamp)

    def left_info_callback(self, msg):
        """Store left camera info"""
        if self.left_info is None:
            self.left_info = msg

    def right_info_callback(self, msg):
        """Store right camera info"""
        if self.right_info is None:
            self.right_info = msg

    def wait_for_messages(self, duration=5.0):
        """Wait for camera messages"""
        self.get_logger().info(f'Waiting {duration} seconds for camera messages...')

        start_time = time.time()
        rate = self.create_rate(10)

        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()

    def check_image_reception(self):
        """Check if images are being received"""
        self.get_logger().info('\n=== Image Reception Test ===')

        if self.left_image_count == 0:
            self.get_logger().error('✗ No images received from left camera!')
            return False

        if self.right_image_count == 0:
            self.get_logger().error('✗ No images received from right camera!')
            return False

        self.get_logger().info(f'✓ Left camera: {self.left_image_count} images received')
        self.get_logger().info(f'✓ Right camera: {self.right_image_count} images received')

        # Check image rate (should be ~30 Hz)
        if len(self.left_timestamps) > 1:
            intervals = [self.left_timestamps[i+1] - self.left_timestamps[i]
                        for i in range(len(self.left_timestamps)-1)]
            avg_interval = sum(intervals) / len(intervals)
            fps = 1.0 / avg_interval if avg_interval > 0 else 0
            self.get_logger().info(f'Left camera rate: {fps:.1f} Hz (expected ~30 Hz)')

        if len(self.right_timestamps) > 1:
            intervals = [self.right_timestamps[i+1] - self.right_timestamps[i]
                        for i in range(len(self.right_timestamps)-1)]
            avg_interval = sum(intervals) / len(intervals)
            fps = 1.0 / avg_interval if avg_interval > 0 else 0
            self.get_logger().info(f'Right camera rate: {fps:.1f} Hz (expected ~30 Hz)')

        return True

    def check_camera_info(self):
        """Validate camera info parameters"""
        self.get_logger().info('\n=== Camera Info Validation ===')

        if self.left_info is None:
            self.get_logger().error('✗ No camera_info received from left camera!')
            return False

        if self.right_info is None:
            self.get_logger().error('✗ No camera_info received from right camera!')
            return False

        self.get_logger().info('✓ Camera info received from both cameras')

        # Check resolution
        self.get_logger().info(f'Left camera: {self.left_info.width}x{self.left_info.height}')
        self.get_logger().info(f'Right camera: {self.right_info.width}x{self.right_info.height}')

        if self.left_info.width != 640 or self.left_info.height != 360:
            self.get_logger().warn(f'⚠ Left camera resolution is not 640x360!')

        if self.right_info.width != 640 or self.right_info.height != 360:
            self.get_logger().warn(f'⚠ Right camera resolution is not 640x360!')

        # Check intrinsics
        fx_left = self.left_info.k[0]
        fy_left = self.left_info.k[4]
        fx_right = self.right_info.k[0]
        fy_right = self.right_info.k[4]

        self.get_logger().info(f'Left camera focal length: fx={fx_left:.1f}, fy={fy_left:.1f}')
        self.get_logger().info(f'Right camera focal length: fx={fx_right:.1f}, fy={fy_right:.1f}')

        return True

    def check_synchronization(self):
        """Check timestamp synchronization between cameras"""
        self.get_logger().info('\n=== Synchronization Test ===')

        if len(self.left_timestamps) == 0 or len(self.right_timestamps) == 0:
            self.get_logger().error('✗ Not enough data for synchronization test!')
            return False

        # Compare first timestamps
        time_diff = abs(self.left_timestamps[0] - self.right_timestamps[0])
        self.get_logger().info(f'Initial timestamp difference: {time_diff*1000:.1f} ms')

        if time_diff > 0.1:  # 100ms tolerance
            self.get_logger().warn('⚠ Cameras may not be well synchronized!')
        else:
            self.get_logger().info('✓ Cameras appear to be synchronized')

        return True

    def run_all_tests(self):
        """Run complete camera validation suite"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('JeTank Stereo Camera Validation Suite')
        self.get_logger().info('='*50)

        # Collect data
        self.wait_for_messages(5.0)

        # Run tests
        reception_ok = self.check_image_reception()
        info_ok = self.check_camera_info()
        sync_ok = self.check_synchronization()

        self.get_logger().info('\n' + '='*50)
        if reception_ok and info_ok and sync_ok:
            self.get_logger().info('All camera tests passed! ✓')
            self.get_logger().info('='*50)
            return True
        else:
            self.get_logger().error('Some camera tests failed! ✗')
            self.get_logger().info('='*50)
            return False


def main(args=None):
    rclpy.init(args=args)

    tester = CameraTest()

    try:
        success = tester.run_all_tests()

        if success:
            tester.get_logger().info('\n✓ Stereo cameras are working correctly!')
            tester.get_logger().info('You can view images with:')
            tester.get_logger().info('  ros2 run rqt_image_view rqt_image_view')
        else:
            tester.get_logger().error('\n✗ Camera validation failed.')
            tester.get_logger().info('Check that Gazebo simulation is running.')

    except KeyboardInterrupt:
        tester.get_logger().info('\nTest interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
