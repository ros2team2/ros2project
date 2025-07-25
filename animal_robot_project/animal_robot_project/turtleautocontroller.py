import rclpy
import threading
import time
import math
import random
import json
import socket
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def get_valid_min(ranges):
    return min([r for r in ranges if not math.isnan(r) and not math.isinf(r) and r > 0.05 and r < 10.0],
               default=float('inf'))

class TurtleEmotionController(Node):
    def __init__(self):
        super().__init__('turtle_emotion_controller')
        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback,
            qos_profile=qos_profile)

        # 랜덤 동작 관련 변수
        self.scan_ranges = []
        self.has_scan_received = False
        self.avoid_mode = False
        self.action_thread = None
        self.action_mutex = threading.Lock()
        self.stop_action = False
        self.avoid_threshold = 0.3
        self.avoid_turn_angle = math.radians(20)
        self.special_behavior_interval = random.uniform(5.0, 10.0)
        self.last_special_behavior_time = time.time()

        # 감정 및 대기 상태 관련 변수
        self.min_distance_front = 0.5
        self.min_distance_rear = 0.3
        self.obstacle_detected_front = False
        self.obstacle_detected_rear = False
        self.collision_avoidance_active = False
        self.rear_avoidance_active = False
        self.emotion_received = False
        self.waiting_for_emotion = False
        self.current_emotion = None

        # TCP 서버 설정
        self.server_host = "0.0.0.0"
        self.server_port = 12345
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_host, self.server_port))
        self.server_socket.listen(1)
        self.get_logger().info(f"TCP server started at {self.server_host}:{self.server_port}")

        # 스레드 시작
        self.running = True
        self.timer = self.create_timer(0.05, self.obstacle_check_timer)
        self.behavior_thread = threading.Thread(target=self.behavior_loop)
        self.behavior_thread.daemon = True
        self.behavior_thread.start()
        self.tcp_thread = threading.Thread(target=self.run_tcp_server)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()

    def laser_callback(self, msg: LaserScan):
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        angle_range = 15
        delta = int(math.radians(angle_range) / angle_increment)
        total_len = len(ranges)
        rear_ranges = list(ranges[0:delta]) + list(ranges[total_len - delta:total_len])
        self.scan_ranges = rear_ranges
        self.has_scan_received = True

        front_min_angle_rad = -math.radians(20)
        front_max_angle_rad = math.radians(20)
        self.obstacle_detected_front = self._check_obstacle_in_range(
            msg, front_min_angle_rad, front_max_angle_rad, self.min_distance_front
        )
        rear_min_angle_rad = math.radians(160)
        rear_max_angle_rad = math.radians(200)
        self.obstacle_detected_rear = self._check_obstacle_in_range(
            msg, rear_min_angle_rad, rear_max_angle_rad, self.min_distance_rear
        )

    def _check_obstacle_in_range(self, msg, min_angle_rad, max_angle_rad, threshold_distance):
        start_index = math.floor((min_angle_rad - msg.angle_min) / msg.angle_increment)
        end_index = math.ceil((max_angle_rad - msg.angle_min) / msg.angle_increment)
        start_index = max(0, min(start_index, len(msg.ranges) - 1))
        end_index = max(0, min(end_index, len(msg.ranges) - 1))
        ranges_in_view = msg.ranges[start_index:end_index + 1] if start_index <= end_index else msg.ranges[start_index:] + msg.ranges[:end_index + 1]
        closest_distance = float('inf')
        for r in ranges_in_view:
            if msg.range_min < r < closest_distance:
                closest_distance = r
        return closest_distance < threshold_distance

    def obstacle_check_timer(self):
        if not self.has_scan_received or self.avoid_mode:
            return
        min_rear = get_valid_min(self.scan_ranges)
        if min_rear <= self.avoid_threshold:
            self.get_logger().info("🚧 후방 장애물 감지")
            self.avoid_mode = True
            self.stop_action = True
            if self.action_thread is not None:
                self.action_thread.join()
            self.action_thread = threading.Thread(target=self.avoid_loop)
            self.action_thread.start()

    def avoid_loop(self):
        with self.action_mutex:
            self.get_logger().info("⚠️ 회피 동작: 왼쪽으로 20도 회전")
            twist = Twist()
            twist.angular.z = 0.6
            duration = abs(self.avoid_turn_angle) / twist.angular.z
            start_time = time.time()
            while time.time() - start_time < duration:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)
            self.get_logger().info("✅ 회피 완료. 일반 행동 재개")
            self.avoid_mode = False
            self.stop_action = False

    def behavior_loop(self):
        while rclpy.ok() and self.running:
            if self.avoid_mode or self.emotion_received or self.waiting_for_emotion:
                self.stop_robot()
                time.sleep(0.1)
                continue
            now = time.time()
            if now - self.last_special_behavior_time >= self.special_behavior_interval:
                behavior = random.choice(['circle', 'zigzag', 'figure_eight'])
                self.get_logger().info(f"🌟 특수 행동: {behavior}")
                self.stop_action = False
                self.action_thread = threading.Thread(target=self.special_behavior, args=(behavior,))
                self.action_thread.start()
                self.action_thread.join()
                self.last_special_behavior_time = time.time()
                self.special_behavior_interval = random.uniform(5.0, 10.0)
                continue
            direction = random.choice([-1, 1])
            angle = random.uniform(math.radians(30), math.radians(90)) * direction
            self.get_logger().info(
                f"🌀 랜덤 회전: {'왼쪽' if direction == -1 else '오른쪽'} {math.degrees(abs(angle)):.1f}도")
            self.stop_action = False
            self.action_thread = threading.Thread(target=self.random_move, args=(direction, angle))
            self.action_thread.start()
            self.action_thread.join()

    def random_move(self, direction, angle):
        with self.action_mutex:
            if self.stop_action or self.avoid_mode or self.emotion_received or self.waiting_for_emotion:
                return
            twist = Twist()
            twist.angular.z = direction * 0.6
            duration = abs(angle) / abs(twist.angular.z)
            start_time = time.time()
            while time.time() - start_time < duration:
                if self.stop_action or self.avoid_mode or self.emotion_received or self.waiting_for_emotion:
                    self.cmd_vel_pub.publish(Twist())
                    return
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)
            forward_duration = random.uniform(1.0, 5)
            self.get_logger().info(f"🚗 전진 {forward_duration:.2f}초")
            twist = Twist()
            twist.linear.x = 0.1
            start_time = time.time()
            while time.time() - start_time < forward_duration:
                if self.stop_action or self.avoid_mode or self.emotion_received or self.waiting_for_emotion:
                    self.cmd_vel_pub.publish(Twist())
                    return
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)

    def special_behavior(self, behavior):
        with self.action_mutex:
            if self.stop_action or self.avoid_mode or self.emotion_received or self.waiting_for_emotion:
                return
            twist = Twist()
            start_time = time.time()
            duration = 0.0
            if behavior == 'circle':
                twist.linear.x = 0.2
                twist.angular.z = 0.8
                duration = 3.5
            elif behavior == 'zigzag':
                duration = 1.5
                half = duration / 2
                twist.linear.x = 0.2
            elif behavior == 'figure_eight':
                duration = 3.5
                half = duration / 2
                twist.linear.x = 0.2
            while time.time() - start_time < duration:
                if self.stop_action or self.avoid_mode or self.emotion_received or self.waiting_for_emotion:
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info("🛑 특수 행동 중단")
                    return
                t = time.time() - start_time
                if behavior == 'zigzag':
                    twist.angular.z = 1.0 if t < half else -1.0
                elif behavior == 'figure_eight':
                    twist.angular.z = 1.0 if t < half else -1.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.1)

    def avoid_obstacle_front(self):
        self.stop_robot()
        self.get_logger().info("Avoiding front obstacle: Moving back a bit.")
        self.move_robot(-0.1, 0.2)
        self.stop_logger_spam()
        self.get_logger().info("Avoiding front obstacle: Rotating to find clear path.")
        self.rotate_robot(math.radians(90), 0.5)
        self.stop_logger_spam()
        self.get_logger().info("Front obstacle avoidance routine finished.")

    def avoid_obstacle_rear(self):
        self.stop_robot()
        self.get_logger().warn("Rear obstacle detected during backward movement! Initiating rear avoidance.")
        self.move_robot(0.15, 0.2)
        self.stop_logger_spam()
        self.get_logger().warn("Rear obstacle avoided: Rotating to clear path.")
        self.rotate_robot(math.radians(90), 0.5)
        self.stop_logger_spam()
        self.get_logger().warn("Rear avoidance routine finished.")
        self.rear_avoidance_active = False

    def perform_dori_dori(self, angle_rad, angular_speed, repetitions):
        self.get_logger().info(f"Starting Dori-Dori motion: {repetitions} repetitions, {math.degrees(angle_rad):.1f} degrees each side.")
        for _ in range(repetitions):
            self.get_logger().info("Dori-Dori: Rotating right.")
            self.rotate_robot(-angle_rad, angular_speed)
            self.stop_robot()
            time.sleep(0.5)
            self.get_logger().info("Dori-Dori: Rotating left.")
            self.rotate_robot(angle_rad * 2, angular_speed)
            self.stop_robot()
            time.sleep(0.5)
            self.get_logger().info("Dori-Dori: Returning to center.")
            self.rotate_robot(-angle_rad, angular_speed)
            self.stop_robot()
            time.sleep(0.5)
        self.get_logger().info("Dori-Dori motion completed.")

    def process_emotion(self, emotion):
        self.get_logger().info(f"Processing emotion: '{emotion}'")
        self.emotion_received = True
        self.stop_action = True
        if self.action_thread is not None:
            self.action_thread.join()
        self.stop_robot()
        if emotion == 'Neutral':
            self.get_logger().info("Emotion received: 'neutral' - Performing Dori-Dori.")
            self.perform_dori_dori(math.radians(45), 0.5, 2)
        elif emotion == 'Happy':
            self.get_logger().info("Emotion received: 'happy' - Rotating 360 degrees.")
            self.rotate_robot(math.pi * 2, 0.5)
        elif emotion == 'Sad':
            self.get_logger().info("Emotion received: 'sad' - Moving backward.")
            self.move_robot(-0.5, 0.2)
        else:
            self.get_logger().warn(f"Unknown emotion: '{emotion}'. Ignoring.")
        self.emotion_received = False
        self.get_logger().info("Emotion action completed.")

    def rotate_robot(self, angle_rad, angular_speed):
        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed
        start_time = self.get_clock().now()
        duration_s = abs(angle_rad / angular_speed)
        self.get_logger().info(f"Rotating for {duration_s:.2f} seconds with angular speed {angular_speed}...")
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_s:
            if self.obstacle_detected_front:
                self.get_logger().warn("Front obstacle detected during rotation, initiating front avoidance!")
                self.avoid_obstacle_front()
                return
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()
        self.get_logger().info("Rotation finished.")

    def move_robot(self, distance_m, linear_speed):
        twist = Twist()
        twist.linear.x = linear_speed if distance_m > 0 else -linear_speed
        start_time = self.get_clock().now()
        duration_s = abs(distance_m / linear_speed)
        self.get_logger().info(f"Moving for {duration_s:.2f} seconds with linear speed {linear_speed}...")
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_s:
            if distance_m > 0 and self.obstacle_detected_front:
                self.get_logger().warn("Front obstacle detected during forward movement, initiating front avoidance!")
                self.avoid_obstacle_front()
                return
            elif distance_m < 0 and self.obstacle_detected_rear and not self.rear_avoidance_active:
                self.get_logger().warn("Rear obstacle detected during backward movement! Initiating rear avoidance.")
                self.rear_avoidance_active = True
                self.avoid_obstacle_rear()
                return
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()
        self.get_logger().info("Movement finished.")

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def stop_logger_spam(self):
        time.sleep(0.1)

    def run_tcp_server(self):
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Client connected: {addr}")
                while self.running:
                    data = client_socket.recv(1024).decode('utf-8')
                    if not data:
                        break
                    self.get_logger().info(f"Received data: {data}")
                    parsed_json = json.loads(data)
                    if "command" in parsed_json:
                        command = parsed_json["command"]
                        if command == "start_recognition":
                            self.get_logger().info("Received start_recognition signal. Stopping and waiting for emotion.")
                            self.waiting_for_emotion = True
                            self.stop_action = True
                            if self.action_thread is not None:
                                self.action_thread.join()
                            self.stop_robot()
                        elif command == "end_recognition":
                            self.get_logger().info("Received end_recognition signal. Resuming random behavior.")
                            self.waiting_for_emotion = False
                            self.stop_action = False
                    elif "emotion" in parsed_json:
                        emotion = parsed_json["emotion"]
                        self.process_emotion(emotion)
                client_socket.close()
                self.get_logger().info(f"Client disconnected: {addr}")
            except Exception as e:
                self.get_logger().error(f"TCP server error: {e}")

    def destroy_node(self):
        self.running = False
        self.stop_action = True
        if self.action_thread is not None:
            self.action_thread.join()
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleEmotionController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료됨 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()