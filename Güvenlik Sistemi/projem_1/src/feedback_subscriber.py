#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from projem_1.action import Patrol

class FeedbackSubscriber(Node):

    def __init__(self):
        super().__init__('feedback_subscriber')
        # '/patrol_feedback' topic'ine abone ol
        self.subscription = self.create_subscription(
            Patrol.Feedback,
            '/patrol_feedback',
            self.feedback_callback,
            10)
        self.subscription  # Abonelik tutucusu

    def feedback_callback(self, msg):
        self.get_logger().info(f"Gelen feedback: Sensör Değeri: {msg.sensor_value}, Alarm: {msg.alarm_triggered}")

def main(args=None):
    rclpy.init(args=args)
    feedback_subscriber = FeedbackSubscriber()
    rclpy.spin(feedback_subscriber)
    feedback_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
