#!/usr/bin/env python3
import socket
import struct
import time
import threading
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import pickle

def clear_socket_buffer(sock):
    """
    读取并清空 socket 缓冲区，防止残留数据影响下一次读取
    """
    sock.setblocking(False)  # 设置为非阻塞模式
    try:
        while True:
            extra_data = sock.recv(1024)  # 读取所有剩余数据
            if not extra_data:
                break
    except BlockingIOError:
        pass  # 缓冲区已清空
    finally:
        sock.setblocking(True)  # 恢复为阻塞模式

def send_heartbeat(client_socket):
    """
    发送心跳包
    """
    while True:
        try:
            # 发送心跳包
            packet_type = 1  # 心跳包类型
            data_length = 0  # 数据长度为 0
            header = struct.pack('>BI', packet_type, data_length)
            client_socket.send(header)
            print("发送心跳包")
            time.sleep(5)  # 每 5 秒发送一次心跳包
        except Exception as e:
            print(f"心跳包发送失败: {e}")
            break

def send_data(client_socket, data):
    """
    发送数据包
    """
    try:
        # 序列化数据
        data_bytes = pickle.dumps(data)
        data_length = len(data_bytes)

        # 发送数据包头（类型 + 长度）
        packet_type = 0  # 数据包类型
        header = struct.pack('>BI', packet_type, data_length)
        client_socket.send(header)

        # 发送数据部分
        client_socket.send(data_bytes)
        print("数据已发送")
    except Exception as e:
        print(f"数据发送失败: {e}")

def start_client():
    """
    启动客户端
    """
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))  # 连接到服务器

    # 启动心跳包线程
    heartbeat_thread = threading.Thread(target=send_heartbeat, args=(client_socket,))
    heartbeat_thread.daemon = True
    heartbeat_thread.start()

    # # 发送多种类型的数据
    # data_types = [
    #     True,  # 布尔值
    #     42,    # 整型
    #     3.14,  # 浮点型
    #     [1.0, 2.0, 3.0, 4.0]  # 数组
    # ]

    # for data in data_types:
    #     send_data(client_socket, data)
    #     time.sleep(1)  # 间隔 1 秒发送一次数据
'''
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
                real_marker_position_msg = Float64MultiArray()
                real_marker_position_msg.data=received_data
                real_marker_position_pub.publish(real_marker_position_msg)


            elif packet_type == 1:  # 心跳包
                print("接收到心跳包")
                last_heartbeat_time = time.time()
            else:
                print("未知数据包类型")


        except Exception as e:
            print(f"错误: {e}")
            break

    # 关闭连接
    # client_socket.close()
'''
def recv_all(sock, length):
    """
    确保接收完整的数据
    """
    data = b''
    while len(data) < length:
        packet = sock.recv(length - len(data))
        if not packet:
            return None  # 连接断开
        data += packet
    return data

def handle_client(client_socket):
    """
    处理客户端连接
    """
    last_heartbeat_time = time.time()

    while True:
        try:
            # # 检查心跳超时
            # if time.time() - last_heartbeat_time > HEARTBEAT_TIMEOUT:
            #     print("心跳超时，关闭连接")
            #     break

            # 接收数据包头（类型 + 长度）
            header = recv_all(client_socket, 5)
            if not header:
                print("连接已关闭")
                break

            # 解析数据包头
            packet_type = header[0]
            data_length = struct.unpack('>I', header[1:5])[0]

            # 接收数据部分
            data = recv_all(client_socket, data_length)
            if not data:
                print("数据接收失败")
                break

            # 处理数据包
            if packet_type == 0:  # 数据包
                try:
                    received_data = pickle.loads(data)
                    print("接收到的数据:", received_data)
                    
                    real_marker_position_msg = Float64MultiArray()
                    real_marker_position_msg.data = received_data
                    real_marker_position_pub.publish(real_marker_position_msg)
                    break
                
                except Exception as e:
                    print(f"数据解析错误: {e}")
                    break

            elif packet_type == 1:  # 心跳包
                print("接收到心跳包")
                last_heartbeat_time = time.time()
                break
            else:
                print("未知数据包类型")
                break

        except socket.error as e:
            print(f"Socket 错误: {e}")
            break
        except Exception as e:
            print(f"处理数据时发生错误: {e}")
            break

    print("I am out of callback")
    #print("关闭客户端连接")
    #client_socket.close()

def callback_marker_position_cmd(msg):
    marker_position_cmd=msg.data
    global client_socket

    # 发送数组
    array = np.array(marker_position_cmd, dtype=np.float64)
    send_data(client_socket, array)
    handle_client(client_socket)



# 服务器配置
HOST = '169.254.5.11'    # 服务器的IP地址,'localhost' 或者'192.168.1.120'
PORT = 12345            # 服务器的端口

client_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
success=False
while not success:
    try:
        ss=client_socket.connect((HOST, PORT))
        if ss==None:
            success=True
    except Exception as e:
        print(f"服务器尚未启动，连接失败: {e}")
    time.sleep(1)
print(f"Connected to {HOST}:{PORT}")
# 启动心跳包线程
# heartbeat_thread = threading.Thread(target=send_heartbeat, args=(client_socket,))
# heartbeat_thread.daemon = True
# heartbeat_thread.start()

if __name__ == "__main__":

    # Initialize the node
    rospy.init_node("leica_cmd_client_node", anonymous=True)
    # Create a subscriber to subscribe path points for the real robot motion
    rospy.Subscriber('/marker_position_cmd', Float64MultiArray, callback_marker_position_cmd)
    real_marker_position_pub = rospy.Publisher("/real_marker_position", Float64MultiArray, queue_size=10)   # 发布消息给leica，让他运动

    rospy.spin()