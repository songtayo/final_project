import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
import time

class TurtleBotNavigator(Node):
    def __init__(self):
        super().__init__('turtlebot_waypoint_navigator')
        self.publisher_ = self.create_publisher(String, 'arrival_status', 10)
        
        self.done_signal_received = False
        self.subscription = self.create_subscription(Int32, 'done', self.done_callback, 10)
        
        self.navigator = BasicNavigator()
        
        # 경로 및 이름 설정
        self.waypoints = []
        self.waypoint_names = ["목표 지점 1", "원점(복귀)", "목표 지점 2", "원점(최종 복귀)"]
        
        self.setup_navigation()
        self.run_mission()

    def done_callback(self, msg):
        if msg.data == 1:
            # 현재 실제로 대기 중일 때만 로그를 찍고 상태를 변경합니다.
            if hasattr(self, 'waiting_for_done') and self.waiting_for_done:
                self.get_logger().info('계속 진행 신호(done: 1)를 수신했습니다.')
                self.done_signal_received = True
                self.waiting_for_done = False # 신호를 받았으므로 대기 상태 해제

    def create_pose(self, x, y, z_rot, w_rot):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z_rot
        pose.pose.orientation.w = w_rot
        return pose

    def setup_navigation(self):
        initial_pose = self.create_pose(-0.2, 0.0, -0.707, 0.707)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('Nav2 활성화를 기다리는 중...')
        self.navigator.waitUntilNav2Active()
        
        origin = self.create_pose(-0.2, 0.0, -0.707, 0.707)
        goal1 = self.create_pose(-0.3, -2.0, -0.707, 0.707)
        goal2 = self.create_pose(-1.8, -1.9, 1.0, 0.0)
        self.waypoints = [goal1, origin, goal2, origin]

    def run_mission(self):
        self.get_logger().info('왕복 주행 미션을 시작합니다.')

        for index, (goal, name) in enumerate(zip(self.waypoints, self.waypoint_names)):
            self.get_logger().info(f'[{name}] (으)로 이동을 시작합니다.')
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'✅ {name} 도착 성공!')
                
                # 원점이 아닐 때만 도착 신호 발행
                if "원점" not in name:
                    # 이 지점에서 이미 보냈는지 확인하는 로컬 플래그 (중요)
                    arrival_msg = String()
                    arrival_msg.data = 'arrived'
                    self.publisher_.publish(arrival_msg)
                    self.get_logger().info(f'신호 발행: "{name}" 도착 알림 전송')
                    
                    # 신호를 보낸 후, 즉시 다음 'done'을 기다리는 루프로 진입하여 
                    # 다시 위로 올라가서 publish를 실행하지 못하게 막음
                    if index < len(self.waypoints) - 1:
                        self.get_logger().info(f"--- [대기] 'done' 수신 대기 ---")
                        self.done_signal_received = False
                        self.waiting_for_done = True
                        while not self.done_signal_received:
                            rclpy.spin_once(self, timeout_sec=0.1)
                    # --------------------------------------------------------

                # 2. 원점일 경우 자동으로 2초 대기
                else:
                    self.get_logger().info(f'"{name}"은 원점이므로 2초 대기 후 자동으로 이동합니다.')
                    time.sleep(2.0)
                
                time.sleep(1.0)
            else:
                self.get_logger().error(f'❌ {name} 주행 실패.')
                break

def main(args=None):
    rclpy.init(args=args)
    navigator_node = TurtleBotNavigator()
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()