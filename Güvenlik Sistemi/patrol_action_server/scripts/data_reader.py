import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PatrolFeedback  # custom_interfaces, kullanılan mesaj türüne göre değişebilir

class DataReaderNode(Node):
    def __init__(self):
        super().__init__('data_reader_node')
        # Burada '/patrol_feedback' topic'ini dinleyeceğiz
        self.subscription = self.create_subscription(
            PatrolFeedback,
            '/patrol_feedback',  # Arduino'dan gelen veri bu topic üzerinden gelebilir
            self.listener_callback,
            10  # QoS ayarı (en fazla 10 mesaj bekle)
        )
    
    def listener_callback(self, msg):
        # Arduino'dan gelen mesajı terminalde yazdırıyoruz
        self.get_logger().info(f"Sensor Value: {msg.sensor_value}, Alarm Triggered: {msg.alarm_triggered}")

def main(args=None):
    # ROS 2'yi başlatıyoruz
    rclpy.init(args=args)
    # Node'umuzu oluşturuyoruz
    node = DataReaderNode()
    # Node'u çalıştırıyoruz
    rclpy.spin(node)
    # Node kapandığında
    rclpy.shutdown()

if __name__ == '__main__':
    main()
