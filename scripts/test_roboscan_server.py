#!/usr/bin/env python3
import socket
import struct
import time
import threading
import numpy as np

# 服务器配置
HOST = '0.0.0.0'  # 监听所有网络接口
PORT = 12345     # 监听的端口 65432

# 数据类型标识
DATA_TYPE_STRING = 0
DATA_TYPE_BOOL = 1
DATA_TYPE_INT = 2
DATA_TYPE_FLOAT = 3
DATA_TYPE_DOUBLE= 4

# 处理客户端连接的函数
def handle_client(conn, addr):
    print(f"Connected by {addr}")
    last_heartbeat = time.time()
    
    with conn:
        while True:
            # 接收数据头
            header = conn.recv(4)
            if not header:
                break
            
            # 解析数据头
            data_type, data_length = struct.unpack('!HH', header)
            
            # 接收数据体
            data = conn.recv(data_length)
            if not data:
                break
            
            # 处理心跳包
            if data_type == DATA_TYPE_STRING and data.decode('utf-8') == 'HEARTBEAT':
                last_heartbeat = time.time()
                print(f"Heartbeat from {addr}")
                continue
            
            # 处理其他数据类型
            if data_type == DATA_TYPE_STRING:
                print(f"Received string: {data.decode('utf-8')}")
            elif data_type == DATA_TYPE_BOOL:
                value = struct.unpack('!?', data)[0]
                print(f"Received bool: {value}")
            elif data_type == DATA_TYPE_INT:
                value = struct.unpack('!i', data)[0]
                print(f"Received int: {value}")
            elif data_type == DATA_TYPE_FLOAT:
                value = struct.unpack('!f', data)[0]
                print(f"Received float: {value}")
            elif data_type == DATA_TYPE_DOUBLE:
                value = struct.unpack('!d', data)[0]
                print(f"Received double: {value}")
            
            # 检查心跳包超时
            if time.time() - last_heartbeat > 10:
                print(f"Heartbeat timeout from {addr}")
                break

            # 接收数组数据
            # data = conn.recv(1024)
            # array = np.frombuffer(data, dtype=np.float32)  # 将字节数据转换为数组
            # print("接收到的数组:", array)

# 启动服务器
def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        
        while True:
            conn, addr = s.accept()
            thread = threading.Thread(target=handle_client, args=(conn, addr))
            thread.start()

if __name__ == "__main__":
    start_server()