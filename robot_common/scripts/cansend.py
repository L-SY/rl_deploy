import can
import argparse

def send_can_frame(interface, can_id, frame_type):
    # 配置CAN接口
    bus = can.interface.Bus(channel=interface, bustype='socketcan')

    # 定义不同类型的CAN帧数据
    frame_data = {
        'close': [0xFF] * 7 + [0xFD],
        'start': [0xFF] * 7 + [0xFC],
        'setZero': [0xFF] * 7 + [0xFE]
    }

    if frame_type not in frame_data:
        raise ValueError(f"Invalid frame type: {frame_type}. Valid types are 'close', 'start', 'setZero'.")

    can_data = frame_data[frame_type]

    # 创建CAN消息
    msg = can.Message(arbitration_id=can_id,
                      data=can_data,
                      is_extended_id=False)

    # 发送CAN消息
    bus.send(msg)
    print(f"Sent {frame_type}")
    print(f"message: {msg}")

if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Send a CAN frame via SocketCAN.')
    parser.add_argument('interface', type=str, help='CAN interface (e.g., can0)')
    parser.add_argument('can_id', type=lambda x: int(x, 16), help='CAN ID in hex (e.g., 0x001)')
    parser.add_argument('frame_type', type=str, choices=['close', 'start', 'setZero'], help='Type of frame to send (close, start, setZero)')

    args = parser.parse_args()

    # 发送CAN帧
    send_can_frame(args.interface, args.can_id, args.frame_type)
