#!/usr/bin/env python3
import socket
import pickle
import struct
import time
import rospy
from std_msgs.msg import Float64MultiArray

def handle_client(client_socket):
    """
    处理客户端连接
    """
    last_heartbeat_time = time.time()
    while True:
        try:
            # 接收数据包头（类型 + 长度）
            header = client_socket.recv(5)
            if not header:
                break

            # 解析数据包头
            packet_type = header[0]
            data_length = struct.unpack('>I', header[1:5])[0]

            # 接收数据部分
            data = b''
            while len(data) < data_length:
                packet = client_socket.recv(data_length - len(data))
                if not packet:
                    break
                data += packet

            # 处理数据包
            if packet_type == 0:  # 数据包
                received_data = pickle.loads(data)
                print("接收到的数据:", received_data)

                real_marker_position_msg= Float64MultiArray()
                real_marker_position_msg.data=received_data
                real_marker_position_pub.publish(real_marker_position_msg)
            elif packet_type == 1:  # 心跳包
                print("接收到心跳包")
                last_heartbeat_time = time.time()
            else:
                print("未知数据包类型")

            # 检查心跳超时
            if time.time() - last_heartbeat_time > 10:  # 10 秒未收到心跳包
                print("心跳超时，关闭连接")
                break

        except Exception as e:
            print(f"错误: {e}")
            break

    # 关闭连接
    client_socket.close()

def start_server():
    """
    启动服务器
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 12345))  # 绑定地址和端口
    server_socket.listen(1)  # 监听连接
    print("服务器已启动，等待客户端连接...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"客户端已连接: {addr}")
        handle_client(client_socket)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("leica_receive_server_node", anonymous=True)
    # 用于接收leica marker实际位置，并发布topic
    real_marker_position_pub = rospy.Publisher("/real_marker_position", Float64MultiArray, queue_size=10)   # 发布消息给leica，让他运动
    start_server()