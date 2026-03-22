import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from venom_serial_driver.msg import RobotStatus, GameStatus
import time
from .serial_interface import SerialInterface
from . import serial_protocol


class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 声明参数
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('loop_rate', 50)

        # 获取参数
        port = self.get_parameter('port_name').value
        baudrate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        rate = self.get_parameter('loop_rate').value

        self.get_logger().info(f'Initializing serial: {port} @ {baudrate}')

        # 初始化串口
        self.serial = SerialInterface(port, baudrate, timeout)
        if not self.serial.connect():
            self.get_logger().error('Failed to connect to serial port')
            return

        self.get_logger().info('Serial port connected')

        # 接收缓冲区
        self.rx_buffer = bytearray()
        self.last_valid_time = time.time()

        # 发布者和订阅者
        self.robot_status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.game_status_pub = self.create_publisher(GameStatus, '/game_status', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 定时器
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        """定期读取串口数据并解析"""
        if not self.serial.is_connected():
            return

        # 读取数据到缓冲区
        data = self.serial.read_bytes(128)
        if data:
            self.rx_buffer.extend(data)

        # 滑动窗口解析
        while len(self.rx_buffer) >= 6:
            # 查找帧头
            if self.rx_buffer[0] != serial_protocol.SOF_RX:
                self.rx_buffer.pop(0)
                continue

            # 解析帧
            success, state = serial_protocol.unpack_state_frame(bytes(self.rx_buffer))

            if success and state:
                # 发布机器人状态
                robot_status = RobotStatus()
                robot_status.velocity.linear.x = float(state.linear_x)
                robot_status.velocity.linear.y = float(state.linear_y)
                robot_status.velocity.linear.z = float(state.linear_z)
                robot_status.velocity.angular.x = float(state.gyro_wz)
                robot_status.velocity.angular.y = float(state.angular_y)
                robot_status.velocity.angular.z = float(state.angular_z)
                robot_status.angular_speed.angular.y = float(state.angular_y_speed)
                robot_status.angular_speed.angular.z = float(state.angular_z_speed)
                self.robot_status_pub.publish(robot_status)

                # 发布比赛状态
                game_status = GameStatus()
                game_status.timestamp_us = state.timestamp_us
                game_status.game_progress = state.game_progress
                game_status.stage_remain_time = state.stage_remain_time
                game_status.center_outpost_occupancy = state.center_outpost_occupancy
                game_status.hp_percentage = float(state.current_HP) / float(state.maximum_HP) if state.maximum_HP > 0 else 0.0
                game_status.shooter_barrel_heat_limit = state.shooter_barrel_heat_limit
                game_status.power_management = state.power_management
                game_status.shooter_17mm_barrel_heat = state.shooter_17mm_barrel_heat
                game_status.shooter_42mm_barrel_heat = state.shooter_42mm_barrel_heat
                game_status.armor_id = state.armor_id
                game_status.hp_deduction_reason = state.HP_deduction_reason
                game_status.launching_frequency = float(state.launching_frequency)
                game_status.initial_speed = float(state.initial_speed)
                game_status.projectile_allowance_17mm = state.projectile_allowance_17mm
                game_status.projectile_allowance_42mm = state.projectile_allowance_42mm
                game_status.rfid_status = state.rfid_status
                game_status.distance = float(state.distance)
                self.game_status_pub.publish(game_status)

                self.last_valid_time = time.time()

                # 移除已解析的帧
                data_len = int.from_bytes(self.rx_buffer[1:3], 'little')
                frame_len = 4 + data_len + 2
                self.rx_buffer = self.rx_buffer[frame_len:]
            else:
                self.rx_buffer.pop(0)

    def cmd_vel_callback(self, msg):
        """接收cmd_vel控制指令并发送到串口"""
        if not self.serial.is_connected():
            return

        try:
            ctrl = serial_protocol.RobotCtrlData()
            ctrl.flags = 0x07  # bit0=检测到, bit1=追踪中, bit2=允许开火
            ctrl.lx = float(msg.linear.x)
            ctrl.ly = float(msg.linear.y)
            ctrl.lz = float(msg.linear.z)
            ctrl.ax = float(msg.angular.x)
            ctrl.ay = float(msg.angular.y)
            ctrl.az = float(msg.angular.z)
            ctrl.dist = 0.0
            ctrl.frame_x = 640
            ctrl.frame_y = 360

            frame = serial_protocol.pack_ctrl_frame(ctrl)
            self.serial.write_bytes(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to send ctrl: {e}')

    def is_online(self):
        """检查是否在线（100ms超时）"""
        return (time.time() - self.last_valid_time) < 0.1

    def destroy_node(self):
        """节点销毁时断开串口"""
        self.serial.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


