#!/usr/bin/env python3
import sys
import time
import math
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from venom_serial_driver.serial_interface import SerialInterface
from venom_serial_driver import serial_protocol


class HardwareTest:
    def __init__(self, port, baudrate=921600):
        self.serial = SerialInterface(port, baudrate, 0.1)
        self.rx_buffer = bytearray()
        self.running = False
        self.rx_thread = None
        self.last_state = None
        self.frame_count = 0

    def start_receiver(self):
        """启动接收线程"""
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()

    def stop_receiver(self):
        """停止接收线程"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)

    def _receive_loop(self):
        """接收循环"""
        while self.running:
            data = self.serial.read_bytes(128)
            if data:
                self.rx_buffer.extend(data)
                self._parse_frames()
            time.sleep(0.001)

    def _parse_frames(self):
        """解析接收到的帧"""
        while len(self.rx_buffer) >= 6:
            if self.rx_buffer[0] != serial_protocol.SOF_RX:
                self.rx_buffer.pop(0)
                continue

            success, state = serial_protocol.unpack_state_frame(bytes(self.rx_buffer))

            if success and state:
                self.last_state = state
                self.frame_count += 1

                data_len = int.from_bytes(self.rx_buffer[1:3], 'little')
                frame_len = 4 + data_len + 2
                self.rx_buffer = self.rx_buffer[frame_len:]
            else:
                self.rx_buffer.pop(0)

    def send_control(self, lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
        """发送控制指令"""
        ctrl = serial_protocol.RobotCtrlData()
        ctrl.flags = 0x07
        ctrl.lx = float(lx)
        ctrl.ly = float(ly)
        ctrl.lz = float(lz)
        ctrl.ax = float(ax)
        ctrl.ay = float(ay)
        ctrl.az = float(az)
        ctrl.dist = 0.0
        ctrl.frame_x = 640
        ctrl.frame_y = 360

        frame = serial_protocol.pack_ctrl_frame(ctrl)
        self.serial.write_bytes(frame)

    def test_yaw_rotation(self, duration=10.0):
        """Yaw 轴旋转测试"""
        print(f"\n=== Yaw 旋转测试 (持续 {duration}s) ===")
        print("参数: pitch=0°, yaw=±30°, 角速度=10°/s, 频率=200Hz\n")

        if not self.serial.connect():
            print("❌ 串口连接失败")
            return

        self.start_receiver()

        yaw_range = math.radians(30)
        angular_speed = math.radians(10)
        freq = 200
        dt = 1.0 / freq

        start_time = time.time()
        send_count = 0

        try:
            while time.time() - start_time < duration:
                t = time.time() - start_time
                yaw = yaw_range * math.sin(2 * math.pi * angular_speed * t / (2 * yaw_range))

                self.send_control(ay=0.0, az=yaw)
                send_count += 1

                if send_count % 200 == 0:
                    print(f"[{t:.1f}s] 发送: {send_count} 帧, 接收: {self.frame_count} 帧, Yaw: {math.degrees(yaw):.1f}°")

                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n测试中断")
        finally:
            self.send_control()
            self.stop_receiver()
            self.serial.disconnect()

            elapsed = time.time() - start_time
            print(f"\n测试完成:")
            print(f"  运行时间: {elapsed:.2f}s")
            print(f"  发送帧数: {send_count} ({send_count/elapsed:.1f} Hz)")
            print(f"  接收帧数: {self.frame_count} ({self.frame_count/elapsed:.1f} Hz)")

    def test_chassis_motion(self):
        """底盘运动测试 - 前后左右"""
        print(f"\n=== 底盘运动测试 ===")
        print("序列: 前进2s → 后退2s → 左移2s → 右移2s")
        print("参数: 速度=0.2m/s, 频率=200Hz\n")

        if not self.serial.connect():
            print("❌ 串口连接失败")
            return

        self.start_receiver()

        speed = 0.2
        freq = 200
        dt = 1.0 / freq

        sequences = [
            ('前进', 2.0, speed, 0, 0),
            ('后退', 2.0, -speed, 0, 0),
            ('左移', 2.0, 0, speed, 0),
            ('右移', 2.0, 0, -speed, 0)
        ]

        total_send = 0

        try:
            for name, duration, lx, ly, lz in sequences:
                print(f"开始 {name} ({duration}s)...")
                start_time = time.time()
                send_count = 0

                while time.time() - start_time < duration:
                    self.send_control(lx=lx, ly=ly, lz=lz)
                    send_count += 1
                    time.sleep(dt)

                total_send += send_count
                print(f"  完成 {name}: 发送 {send_count} 帧")

        except KeyboardInterrupt:
            print("\n测试中断")
        finally:
            self.send_control()
            self.stop_receiver()
            self.serial.disconnect()

            print(f"\n测试完成:")
            print(f"  总发送帧数: {total_send}")
            print(f"  总接收帧数: {self.frame_count}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='硬件测试程序')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='串口设备')
    parser.add_argument('--test', choices=['yaw', 'chassis', 'all'], required=True, help='测试类型')
    parser.add_argument('--duration', type=float, default=10.0, help='Yaw测试持续时间(秒)')
    args = parser.parse_args()

    tester = HardwareTest(args.port)

    if args.test == 'yaw':
        tester.test_yaw_rotation(args.duration)
    elif args.test == 'chassis':
        tester.test_chassis_motion()
    elif args.test == 'all':
        tester.test_yaw_rotation(args.duration)
        time.sleep(1)
        tester.test_chassis_motion()


if __name__ == '__main__':
    main()



