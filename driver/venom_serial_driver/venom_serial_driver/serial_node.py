import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from .serial_interface import SerialInterface
from . import serial_protocol


class SerialDriverNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 声明参数
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
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
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        self.ctrl_sub = self.create_subscription(
            String, 'vision_ctrl', self.ctrl_callback, 10)

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
                # 发布状态
                msg = String()
                msg.data = json.dumps({
                    'timestamp_us': state.timestamp_us,
                    'pitch': state.angular_y,
                    'yaw': state.angular_z,
                    'pitch_speed': state.angular_y_speed,
                    'yaw_speed': state.angular_z_speed,
                    'current_HP': state.current_HP,
                    'game_progress': state.game_progress
                })
                self.state_pub.publish(msg)
                self.last_valid_time = time.time()

                # 移除已解析的帧
                data_len = int.from_bytes(self.rx_buffer[1:3], 'little')
                frame_len = 4 + data_len + 2
                self.rx_buffer = self.rx_buffer[frame_len:]
            else:
                self.rx_buffer.pop(0)

    def ctrl_callback(self, msg):
        """接收视觉控制指令并发送到串口"""
        if not self.serial.is_connected():
            return

        try:
            data = json.loads(msg.data)
            ctrl = serial_protocol.VisionCtrlData()
            ctrl.tracking_state = data.get('tracking_state', 0)
            ctrl.target_pitch = data.get('target_pitch', 0.0)
            ctrl.target_yaw = data.get('target_yaw', 0.0)
            ctrl.target_pitch_v = data.get('target_pitch_v', 0.0)
            ctrl.target_yaw_v = data.get('target_yaw_v', 0.0)

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


