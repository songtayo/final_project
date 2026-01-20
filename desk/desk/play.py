import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time

class MotorSignalNode(Node):
    def __init__(self):
        super().__init__('motor_signal_node')
        
        # 상태 제어 변수 (더 엄격하게 관리)
        self.state = "IDLE"  # IDLE -> MOVING_FORWARD -> WAITING_FOR_BACKWARD -> IDLE
        
        self.motor_pub = self.create_publisher(Int32, '/motor_command', 10)
        self.done_pub = self.create_publisher(Int32, '/done', 10)
        
        self.arrival_sub = self.create_subscription(String, '/arrival_status', self.arrival_callback, 10)
        self.feedback_sub = self.create_subscription(String, '/motor_result', self.feedback_callback, 10)
            
        self.last_arrival_time = 0.0
        self.get_logger().info('=== 모터 중복 방지 강화 버전 가동 ===')

    def arrival_callback(self, msg):
        """도착 신호를 받아도 1초 이내에 다시 들어오면 무시함"""
        current_time = self.get_clock().now().nanoseconds / 1e9  # ROS 시간 사용
        
        # 마지막으로 신호를 처리한 지 1초가 안 지났으면 중복으로 간주하고 버림
        if (current_time - self.last_arrival_time) < 1.0:
            return

        if msg.data == "arrived" and self.state == "IDLE":
            self.last_arrival_time = current_time # 처리 시간 기록
            self.state = "MOVING_FORWARD"
            self.get_logger().info('>>> [Step 1] 전진 명령(1) 전송.')
            
            cmd = Int32()
            cmd.data = 1
            self.motor_pub.publish(cmd)

    def feedback_callback(self, msg):
        received = msg.data.strip()
        
        # 1. 전진 완료 시 -> 후진 명령
        if received == "Forward" and self.state == "MOVING_FORWARD":
            self.state = "MOVING_BACKWARD" # 상태 선제적 변경
            self.get_logger().info('>>> [Step 2] Forward 확인. 1초 대기 후 후진(2) 전송.')
            
            time.sleep(1.0) 
            cmd = Int32()
            cmd.data = 2
            self.motor_pub.publish(cmd)
            
        # 2. 후진 완료 시 -> 완료 보고
        elif received == "Backward" and self.state == "MOVING_BACKWARD":
            self.state = "IDLE" # 모든 시퀀스 종료 및 초기화
            self.get_logger().info('>>> [Final] Backward 확인. /done 전송.')
            
            done_msg = Int32()
            done_msg.data = 1
            self.done_pub.publish(done_msg)
            self.get_logger().info('--- 시퀀스 완료 및 대기 상태로 복귀 ---')

def main(args=None):
    rclpy.init(args=args)
    node = MotorSignalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 종료되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
