#!/usr/bin/env python3
import socket
import struct
import time
import threading

# 服务器配置
HOST = '192.168.1.120'  # 服务器的IP地址
PORT = 54321            # 服务器的端口

# 数据类型标识
DATA_TYPE_STRING = 0
DATA_TYPE_BOOL = 1
DATA_TYPE_INT = 2
DATA_TYPE_FLOAT = 3
DATA_TYPE_DOUBLE= 4

# 发送数据的函数
def send_data(sock, data_type, data):
    if data_type == DATA_TYPE_STRING:
        data = data.encode('utf-8')
        data_length = len(data)
    elif data_type == DATA_TYPE_BOOL:
        data = struct.pack('!?', data)
        data_length = 1
    elif data_type == DATA_TYPE_INT:
        data = struct.pack('!i', data) #'i' format requires -2147483648 <= number <= 2147483647
        data_length = 4
    elif data_type == DATA_TYPE_FLOAT:
        data = struct.pack('!f', data)
        data_length = 4
    elif data_type == DATA_TYPE_DOUBLE:
        data = struct.pack('!d', data)
        data_length = 8
    
    # 发送数据头和数据体
    header = struct.pack('!HH', data_type, data_length)
    sock.sendall(header + data)

# 发送心跳包的函数
def send_heartbeat(sock):
    while True:
        send_data(sock, DATA_TYPE_STRING, 'HEARTBEAT')
        time.sleep(5)

# 启动客户端
def start_client():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")
        
        # 启动心跳包线程
        heartbeat_thread = threading.Thread(target=send_heartbeat, args=(s,))
        heartbeat_thread.daemon = True
        heartbeat_thread.start()
        
        # 发送各种数据类型
        send_data(s, DATA_TYPE_STRING, "Hello, Server, I am a great person!")
        send_data(s, DATA_TYPE_BOOL, True)
        send_data(s, DATA_TYPE_INT, -214748364)
        send_data(s, DATA_TYPE_FLOAT, -3.14159265358979)
        send_data(s, DATA_TYPE_DOUBLE, -3.14159265358979)
        
        # 保持连接
        while True:
            time.sleep(1)

if __name__ == "__main__":
    start_client()