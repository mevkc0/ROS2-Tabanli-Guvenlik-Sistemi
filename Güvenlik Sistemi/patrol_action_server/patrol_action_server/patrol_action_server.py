#!/usr/bin/env python3

import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse, CancelResponse
from projem_1.action import Patrol
from geometry_msgs.msg import Twist

class PatrolActionServer(Node):

    def __init__(self):
        super().__init__('patrol_action_server')
        
        # Seri port başlatılıyor
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Arduino'nun bağlı olduğu portu belirt
        
        # Action server başlatılıyor
        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        # Feedback için bir topic publisher oluştur
        self.feedback_publisher = self.create_publisher(Patrol.Feedback, '/patrol_feedback', 10)
        
        self.get_logger().info('Patrol action server başlatıldı.')

    def goal_callback(self, goal_request):
        self.get_logger().info('Hedef isteği alındı.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('İptal isteği alındı.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Hedef işleniyor...')
        self.get_logger().info(f"Alınan eşik değeri: {goal_handle.request.threshold}")

        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()

        while rclpy.ok():
            # Arduino'dan veri oku
            if self.ser.in_waiting > 0:
                arduino_verisi = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Arduino'dan gelen veri: {arduino_verisi}")

                # Sensör değerini ayrıştır ve alarm durumunu kontrol et
                try:
                    sensor_degeri, alarm_durumu = self.parse_arduino_data(arduino_verisi)
                    feedback_msg.sensor_value = sensor_degeri
                    feedback_msg.alarm_triggered = alarm_durumu

                    # Feedback mesajını client'a gönder
                    goal_handle.publish_feedback(feedback_msg)

                    # Eşik değeri kontrol et
                    if sensor_degeri > goal_handle.request.threshold:
                        self.get_logger().warn(
                            f"Eşik değer aşıldı! Sensör değeri: {sensor_degeri}, Eşik: {goal_handle.request.threshold}"
                        )
                    else:
                        self.get_logger().info(
                            f"Sensör değeri: {sensor_degeri}, eşik değerinin altında: {goal_handle.request.threshold}"
                        )
                    
                    # Alarm tetiklenmişse
                    if alarm_durumu:
                        self.get_logger().error(f"Acil durum! Alarm tetiklendi. Sensör değeri: {sensor_degeri}")
                        self.get_logger().info("Su motoru aktif hale geldi.")
                        result.success = True
                        goal_handle.succeed()
                        self.get_logger().info(f'Ortam güvenliği başarıyla sağlandı : {result.success}')
                        return result

                except ValueError:
                    self.get_logger().error("Arduino'dan geçersiz veri alındı.")

            # 0.3 saniye bekle, asenkron çalışma için `rclpy.spin_once()` kullanıyoruz
            rclpy.spin_once(self, timeout_sec=0.3)

        return result

    def parse_arduino_data(self, data):
        # Arduino'dan gelen veriyi ayrıştır (örnek: "300 - Eşik değer aşıldı!")
        parts = data.split(' - ')
        if len(parts) != 2:
            raise ValueError("Geçersiz veri formatı.")
        
        sensor_value = int(parts[0])
        alarm_triggered = 'aşıldı!' in parts[1]  # Alarm durumu, mesajda "aşıldı!" varsa True
        
        return sensor_value, alarm_triggered

def main(args=None):
    rclpy.init(args=args)
    patrol_action_server = PatrolActionServer()
    rclpy.spin(patrol_action_server)
    patrol_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

