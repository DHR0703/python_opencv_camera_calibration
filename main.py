# -*- coding: utf-8 -*-
"""
@Time ： 2024/3/1 14:26
@Auth ： 邓浩然
@File ：main.py
@IDE ：PyCharm
@Description：主程序,主要包含了拍照部分的功能
"""
import os
import numpy as np
import socket
import struct
import keyboard
import camera_D435i
import camera_calibration
import robotcontrol

"""
标定安装设置:X:0  Y:0   Z:0
抓取安装设置X:0    Y:0  Z:200
"""

# UR-5机器人的IP和端口,IP可以在机器人的网络设置处看到,端口号建议看手册
HOST = "192.168.31.32"
PORT = 30003

# 数据txt位置
XYZ_PATH = 'data/xyz.txt'
RXRYRZ_PATH = 'data/RxRyRz.txt'

# 照片存储位置:
PHOTO_PATH = 'photo/'
POINT_PHOTO_PATH = 'point_photo/'


# 注意安装位置都为0
# 获得当前位姿,前三个是XYZ单位是mm 后三个是RX RY RZ旋转向量
def get_current_tcp():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    data = tcp_socket.recv(1108)
    position = struct.unpack('!6d', data[444:492])
    tcp_socket.close()
    pos_list = np.asarray(position)
    # 单位转化,机械臂传过来的单位是米,我们需要毫米
    pos_list[0:3] = pos_list[0:3] * 1000

    return pos_list


# 删除上次的拍照照片
def delete_photo():
    for files in os.listdir(PHOTO_PATH):
        if files.endswith(".png"):
            os.remove(os.path.join(PHOTO_PATH, files))
    for files in os.listdir(POINT_PHOTO_PATH):
        if files.endswith(".png"):
            os.remove(os.path.join(POINT_PHOTO_PATH, files))


# 拍照的方法,按s拍照,按q退出
def get_photo(photos_num):
    # 创建相机线程
    camera = camera_D435i.RealSenseCamera()
    # 线程开始运行
    camera.start_display_and_capture()
    # 递增，拍照次数
    num = 0
    # 用于存储位姿信息
    pose = np.zeros((photos_num, 6))
    # 文件名,使用ASCII编码
    name_num = 97
    # 删除上次的拍照照片
    delete_photo()
    # 循环读取每一帧
    while num < photos_num:
        keyboard.wait('S')
        # 拍照操作
        camera.save_screenshot(f"photo/{chr(name_num)}.png")
        # 获得并输出位姿
        pose_tcp = get_current_tcp()
        print('机械臂位姿: ', np.around(pose_tcp, decimals=3))
        # 把位姿写入数组
        pose[num] = pose_tcp
        print('第', (num + 1), '张照片拍摄完成')
        print("-------------------------")
        # 循环标志+1
        num += 1
        # 文件名ASCII码+1
        name_num += 1

    # 写入xyz数据到xyz.txt
    np.savetxt(XYZ_PATH, pose[:, 0:3], delimiter=',')
    # 写入rxryrz数据到rxryrz.txt
    np.savetxt(RXRYRZ_PATH, pose[:, 3:6], delimiter=',')
    camera.stop()



if __name__ == '__main__':
    ro = robotcontrol.RobotControl()
    # ro.reset()
    # ro.moveJ_Angle([-60, -128, -60, -15, 90, -95, ])
    # 自由移动模式
    ro.rtde_c.teachMode()
    # 拍几张照片
    get_photo(8)
    camera_calibration.camera_calibration()
