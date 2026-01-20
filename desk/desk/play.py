import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time

class MotorSignalNode(Node):
    def __init__(self):
        super().__init__('motor_signal_node')
        
        # 1. 아두이노 노드로 명령(1 또는 2)을 보낼 퍼블리셔
        self.motor_pub = self.create_publisher(Int32, '/motor_command', 10)
        
        # 2. 모든 동작 완료 시 결과를 보낼 퍼블리셔 (타입: Int32)
        self.done_pub = self.create_publisher(Int32, '/done', 10)
        
        # 3. 터틀봇(Nav2)의 도착 신호를 받을 서브스크라이버
        self.arrival_sub = self.create_subscription(
            String,
            '/arrival_status',
            self.arrival_callback,
            10)
            
        # 4. 아두이노 노드로부터 동작 완료 피드백을 받을 서브스크라이버
        self.feedback_sub = self.create_subscription(
            String,
            '/motor_result',
            self.feedback_callback,
            10)
            
        self.get_logger().info('=== 모터 결과 알림(/done) 노드 가동 ===')

    def arrival_callback(self, msg):
        """터틀봇 도착 시 전진 명령(1) 전송"""
        if msg.data == "arrived":
            self.get_logger().info('>>> [Step 1] 로봇 도착 확인. 전진 명령(1) 전송.')
            cmd = Int32()
            cmd.data = 1
            self.motor_pub.publish(cmd)

    def feedback_callback(self, msg):
        # 양끝 공백 제거 및 소문자로 변환하여 비교 (실수 방지)
        received_feedback = msg.data.strip()
        
        if received_feedback == "Forward":
            self.get_logger().info(f'>>> [Step 2] {received_feedback} 수신.')
            
            # 1초 대기 후 전송해야 한다면 타이머를 쓰는 게 정석이지만, 
            # 간단한 테스트용이라면 아래처럼 진행
            time.sleep(1.0) 
            
            cmd = Int32()
            cmd.data = 2
            self.motor_pub.publish(cmd)
            self.get_logger().info('>>> 후진 명령(2) 전송 완료.')
            
        elif received_feedback == "Backward":
            self.get_logger().info(f'>>> [Final] {received_feedback} 수신.')
            
            done_msg = Int32()
            done_msg.data = 1
            self.done_pub.publish(done_msg)
            
            self.get_logger().info('--- 모든 시퀀스 완료 (done: 1) ---')

def main(args=None):
    rclpy.init(args=args)
    node = MotorSignalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 종료되었습니다.')
    finally:
        node.destroy_node()
        # rclpy.shutdown() 호출 시 예외 발생을 방지하기 위해 체크
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
