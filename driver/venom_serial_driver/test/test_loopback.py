#!/usr/bin/env python3
import sys
import time
from pathlib import Path

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from venom_serial_driver.serial_interface import SerialInterface
from venom_serial_driver import serial_protocol
from venom_serial_driver.crc_utils import crc16


def test_loopback(port='/dev/ttyUSB0'):
    """测试串口回环 (TX-RX 短接)"""
    print(f"\n=== 测试串口回环: {port} ===")

    serial = SerialInterface(port, 921600, 0.5)
    if not serial.connect():
        print("❌ 串口连接失败")
        return False

    print("✓ 串口连接成功")

    # 测试数据
    test_data = b'Hello Serial!'
    serial.write_bytes(test_data)
    time.sleep(0.1)

    rx_data = serial.read_bytes(len(test_data))
    serial.disconnect()

    if rx_data == test_data:
        print(f"✓ 回环测试通过: {test_data}")
        return True
    else:
        print(f"❌ 回环测试失败: 发送 {test_data}, 接收 {rx_data}")
        return False


def test_ctrl_frame():
    """测试控制帧打包和 CRC16 校验"""
    print("\n=== 测试控制帧 (NUC→C板) ===")

    ctrl = serial_protocol.RobotCtrlData()
    ctrl.flags = 0x07
    ctrl.lx = 0.0
    ctrl.ly = 0.0
    ctrl.lz = 0.0
    ctrl.ax = 0.0
    ctrl.ay = 15.5
    ctrl.az = -20.3
    ctrl.dist = 2.5
    ctrl.frame_x = 640
    ctrl.frame_y = 360

    frame = serial_protocol.pack_ctrl_frame(ctrl)
    print(f"✓ 帧长度: {len(frame)} 字节")
    print(f"✓ 帧头: 0x{frame[0]:02X} (期望 0xA5)")

    # 校验 CRC16
    crc_calc = crc16(frame[:-2])
    crc_frame = int.from_bytes(frame[-2:], 'little')

    if crc_calc == crc_frame:
        print(f"✓ CRC16 校验通过: 0x{crc_calc:04X}")
        return True
    else:
        print(f"❌ CRC16 校验失败: 计算 0x{crc_calc:04X}, 帧内 0x{crc_frame:04X}")
        return False


def test_state_frame():
    """测试状态帧解析和 CRC16 校验"""
    print("\n=== 测试状态帧 (C板→NUC) ===")

    # 构造完整的 72 字节测试数据
    import struct
    data = struct.pack('<I9fBHB3HB2HB2f2HI',
                       123456789,      # timestamp_us
                       1.0, 0.5, 0.0,  # linear_x, y, z
                       0.2,            # gyro_wz
                       10.5, -15.3,    # angular_y, z
                       2.1, -3.5,      # angular_y_speed, z_speed
                       1.5,            # distance
                       3,              # game_progress
                       180,            # stage_remain_time
                       1,              # center_outpost_occupancy
                       400, 600, 200,  # current_HP, maximum_HP, shooter_barrel_heat_limit
                       1,              # power_management
                       50, 0,          # shooter_17mm/42mm_barrel_heat
                       33,             # armor_id | HP_deduction_reason
                       10.0, 28.0,     # launching_frequency, initial_speed
                       100, 0,         # projectile_allowance_17mm/42mm
                       0x1F)           # rfid_status

    data_len = len(data)
    header = struct.pack('<BHB', serial_protocol.SOF_RX, data_len, serial_protocol.CMD_ID_STATE)
    frame_no_crc = header + data
    crc = crc16(frame_no_crc)
    frame = frame_no_crc + struct.pack('<H', crc)

    print(f"✓ 构造帧长度: {len(frame)} 字节 (数据 {data_len} 字节)")

    # 解析
    success, parsed = serial_protocol.unpack_state_frame(frame)

    if success and parsed:
        print(f"✓ 解析成功")
        print(f"  timestamp: {parsed.timestamp_us}")
        print(f"  pitch: {parsed.angular_y:.2f}, yaw: {parsed.angular_z:.2f}")
        print(f"  HP: {parsed.current_HP}/{parsed.maximum_HP}")
        return True
    else:
        print(f"❌ 解析失败")
        return False


def test_txrx_loopback(port='/dev/ttyUSB0'):
    """测试完整协议收发 (TX-RX 短接)"""
    print(f"\n=== 测试协议收发回环: {port} ===")

    serial = SerialInterface(port, 115200, 0.5)
    if not serial.connect():
        print("❌ 串口连接失败")
        return False

    # 发送控制帧
    ctrl = serial_protocol.RobotCtrlData()
    ctrl.flags = 0x07
    ctrl.ay = 12.3
    ctrl.az = -45.6

    frame = serial_protocol.pack_ctrl_frame(ctrl)
    serial.write_bytes(frame)
    time.sleep(0.1)

    # 接收并验证
    rx_data = serial.read_bytes(len(frame))
    serial.disconnect()

    if rx_data == frame:
        print(f"✓ 协议帧收发一致，长度 {len(frame)} 字节")
        return True
    else:
        print(f"❌ 收发不一致")
        return False


if __name__ == '__main__':
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

    results = []
    results.append(('串口回环', test_loopback(port)))
    results.append(('控制帧CRC', test_ctrl_frame()))
    results.append(('状态帧CRC', test_state_frame()))
    results.append(('协议收发', test_txrx_loopback(port)))

    print("\n" + "="*50)
    print("测试结果汇总:")
    for name, result in results:
        status = "✓ 通过" if result else "❌ 失败"
        print(f"  {name}: {status}")

    total = len(results)
    passed = sum(1 for _, result in results if result)
    print(f"\n总计: {passed}/{total} 通过")

