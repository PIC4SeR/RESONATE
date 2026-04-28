import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time
import subprocess
import time
import yaml
import subprocess
import argparse
from pathlib import Path
import time
import argparse
import yaml
import os


def normalize_namespace(namespace):
    return namespace.strip().strip('/')


class ObjectivePublishingNode(Node):
    def __init__(self, case, goals, namespace=''):
        super().__init__('publishing_node', namespace=normalize_namespace(namespace))
        self.case = case
        self.objective = self.load_goal_from_yaml(goals=goals,case=case)
        self.pose_published = False
        self.timer = self.create_timer(1.0, self.objective_publisher)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        ) 

        self.goal_pub = self.create_publisher(PoseStamped, 'objective', qos_profile)
    def load_goal_from_yaml(self, goals, case):
        # Load the goal information from the YAML file
        goals_path = Path(goals)
        goals_loaded = yaml.load(goals_path.read_text(), Loader=yaml.FullLoader)
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = goals_loaded[case]["robot_goal"]["x"]
        goal.pose.position.y = goals_loaded[case]["robot_goal"]["y"]
        goal.pose.position.z = 0.
        goal.pose.orientation.x = 0.
        goal.pose.orientation.y = 0.
        goal.pose.orientation.z = 0.
        goal.pose.orientation.w = 1.
        print(f"goal:{goal.pose.position.x},{goal.pose.position.y}")
        #raise "coso"
        return goal
    def objective_publisher(self):
        self.objective.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(self.objective)
        #self.get_logger().info(f'Published goal: x={self.goal_pose.pose.position.x}, y={self.goal_pose.pose.position.y}')
class NavigationGoalPublisher(Node):
    def __init__(self, case, scenario, algo, goals, namespace=''):
        super().__init__('navigation_goal_publisher', namespace=normalize_namespace(namespace))
        #self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Load goal information from the YAML file
        self.goal_pose = self.load_goal_from_yaml(goals=goals,case=case)
        self.scenario = scenario
        self.algo = algo
        self.case = case
        self.namespace = normalize_namespace(namespace)
        self.ns_prefix = f'/{self.namespace}' if self.namespace else ''
        
        # Set up the bag recording command
        self.bag_name = f"bag_for_video/{self.case}/{self.algo}/scenario_{self.scenario}"
        self.goal_pub = self.create_publisher(PoseStamped, 'objective', 1)

        # Start the bag recording and goal publishing
        #self.start_recording_and_navigation()
    def objective_publisher_first(self):
        self.objective_point = PoseStamped()
        self.objective_point = self.goal_pose
        #self.load_goal_from_yaml(goals=goals,case=case)
        self.goal_pub.publish(self.objective_point)

    def load_goal_from_yaml(self, goals, case):
        # Load the goal information from the YAML file
        goals_path = Path(goals)
        goals_loaded = yaml.load(goals_path.read_text(), Loader=yaml.FullLoader)
       
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = goals_loaded[case]["robot_goal"]["x"]
        goal.pose.position.y = goals_loaded[case]["robot_goal"]["y"]
        goal.pose.position.z = 0.
        goal.pose.orientation.x = 0.
        goal.pose.orientation.y = 0.
        goal.pose.orientation.z = 0.
        goal.pose.orientation.w = 1.
        print(f"goal:{goal.pose.position.x},{goal.pose.position.y}")
        #raise "coso"
        return goal

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self.action_client.wait_for_server()
        self.get_logger().info(
            f'Sending goal on namespace {self.ns_prefix or "/"} -> {self.ns_prefix}/navigate_to_pose'
        )
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached')
        self.stop_recording()
        rclpy.shutdown()

    def start_recording(self):
        #self.bag_name = f"metric_drop/{self.case}/{self.algo}/scenario_{self.scenario}"
        cmd = ['ros2', 'bag', 'record', '-o', self.bag_name, 'vicon/odom', 'vicon/people', 'odom','odometry/filtered',
               'jackal_velocity_controller/cmd_vel_unstamped', '/objective','scan','tf','tf_static']
        if self.algo == 'adascore':
            cmd.append('cost_weights')
        self.bag_process = subprocess.Popen(cmd)
        self.get_logger().info(f'Started recording bag: {self.bag_name}')

    def stop_recording(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.get_logger().info('Stopped recording bag')

def main(args=None):
    parser = argparse.ArgumentParser(description="Start recording a ROS 2 bag and publish a navigation goal.")
    parser.add_argument("--goals", type=str, required=True, help="Path to the YAML file containing goal information.")
    parser.add_argument("--case", type=str, required=True,help = "case to be studied")
    parser.add_argument("--scenario", type=str, required=True, help="Integer to modify the recorded bag name.")
    parser.add_argument("--algo", type=str, required=True, help="String to modify the recorded bag name.")
    parser.add_argument("--namespace", type=str, default="jackal", help="ROS namespace (e.g. jackal).")
    args = parser.parse_args()
    print(f'goals {args.goals}')
    print(f'case {args.case}')
    print(f'scenario {args.scenario}')
    print(f'algo {args.algo}')
    print(f'namespace {args.namespace}')

    # Initialize the ROS 2 Python client library
    # rclpy.init(args)
    rclpy.init()

    # Start the navigation goal publisher node
    navigation_goal_publisher = NavigationGoalPublisher(
        case=args.case,
        scenario=args.scenario,
        algo=args.algo,
        goals=args.goals,
        namespace=args.namespace,
    )
    goal_publisher = ObjectivePublishingNode(
        case=args.case,
        goals=args.goals,
        namespace=args.namespace,
    )
    navigation_goal_publisher.objective_publisher_first()
    navigation_goal_publisher.start_recording()
    time.sleep(2)
    navigation_goal_publisher.send_goal()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(navigation_goal_publisher)
    executor.add_node(goal_publisher)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        navigation_goal_publisher.destroy_node()
        goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()