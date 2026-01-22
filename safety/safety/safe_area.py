import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from action_msgs.msg import GoalStatusArray # Nav2 상태 확인을 위해 필요

class PersonDistanceMonitor(Node):
    def __init__(self):
        super().__init__('person_distance_monitor')
        
        # 1. 구독자 및 발행자 설정
        # Nav2의 속도 명령 구독
        self.nav_sub = self.create_subscription(Twist, 'cmd_vel_nav', self.nav_callback, 10)
        # 카메라/센서의 사람 감지 신호 구독
        self.dist_sub = self.create_subscription(Int32, '/blue_box_signal', self.dist_callback, 10)
        # [추가] Nav2 액션 상태 구독 (도착 여부를 판단하기 위함)
        self.status_sub = self.create_subscription(
            GoalStatusArray, 
            '/navigate_to_pose/_action/status', 
            self.status_callback, 
            10)
        
        # 최종 필터링된 속도 발행
        self.publisher = self.create_publisher(Twist, 'cmd_vel_filtered', 10)

        # 2. 상태 변수 초기화
        self.current_person_dist = 0    
        self.latest_nav_msg = Twist()   
        self.last_nav_time = self.get_clock().now()
        self.nav2_status = 0 # 현재 Nav2의 액션 상태 저장용

        # 3. 제어 루프 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Person Distance Monitor with Action Watchdog Started')

    def dist_callback(self, msg):
        self.current_person_dist = msg.data

    def nav_callback(self, msg):
        self.latest_nav_msg = msg
        self.last_nav_time = self.get_clock().now()

    def status_callback(self, msg):
        """
        Nav2 액션 상태를 업데이트합니다.
        상태 코드 정의:
        1: 처리중(ACTIVE), 3: 성공(SUCCEEDED), 4: 취소됨(ABORTED)
        """
        if msg.status_list:
            # 가장 최근 액션의 상태를 가져옴
            self.nav2_status = msg.status_list[-1].status

    def control_loop(self):
        new_msg = Twist()
        now = self.get_clock().now()
        
        # 마지막 Nav2 신호로부터 경과 시간 계산
        time_diff = (now - self.last_nav_time).nanoseconds / 1e9

        # [단계 1] Nav2 제어권 판단 (Watchdog)
        # 1. Nav2가 토픽을 안 보낸지 0.3초가 넘었거나
        # 2. Nav2 액션 상태가 SUCCEEDED(3) 또는 CANCELED(4) 등 종료 상태라면
        # 무조건 속도를 0으로 초기화하여 '유령 값' 제거
        stop_statuses = [3, 4, 5, 6] # 성공, 취소, 거절 등 종료 상태들
        
        if time_diff > 0.3 or self.nav2_status in stop_statuses:
            raw_x = 0.0
            raw_z = 0.0
        else:
            # 주행 중일 때만 값 가져오기
            raw_x = self.latest_nav_msg.linear.x
            raw_z = self.latest_nav_msg.angular.z

        # [단계 2] 데드존(Deadzone) 필터 적용 (미세 진동 방지)
        if abs(raw_x) < 0.01: raw_x = 0.0
        if abs(raw_z) < 0.01: raw_z = 0.0

        # [단계 3] 최종 발행 로직
        # 사람이 감지(1)되었고, Nav2 명령이 유효할 때만 전달
        if self.current_person_dist == 1 and (raw_x != 0.0 or raw_z != 0.0):
            new_msg.linear.x = raw_x
            new_msg.angular.z = raw_z
            # 로깅 (필요 시 주석 해제)
            self.get_logger().info(f'Moving: x={raw_x:.2f}, z={raw_z:.2f}')
        else:
            # 그 외 모든 상황에서는 무조건 정지
            new_msg.linear.x = 0.0
            new_msg.angular.z = 0.0

        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDistanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 안전하게 멈춤
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()