# -*- coding: utf-8 -*-
"""
@Time ： 2024/3/1 14:26
@Auth ： 邓浩然
@File ：main.py
@IDE ：PyCharm
@Description：主程序,主要包含了拍照部分的功能
"""
import os
import cv2 as cv
import numpy as np
import socket
import struct
import camera_calibration
import robotcontrol

"""
标定安装设置:X:0  Y:0   Z:0
抓取安装设置X:0    Y:0  Z:208
"""

# UR-5机器人的IP和端口,IP可以在机器人的网络设置处看到,端口号建议看手册
HOST = "192.168.31.32"
PORT = 30003

# 数据txt位置
XYZ_PATH = 'data/xyz.txt'
RXRYRZ_PATH = 'data/RxRyRz.txt'

# 照片存储位置:
PHOTO_PATH = 'D:/project/Python/camera_calibration/photo/'
POINT_PHOTO_PATH = 'D:/project/Python/camera_calibration/point_photo/'


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
def get_photo(photos_num, camera_num):
    # 创建一个 VideoCapture 对象，2为外接的USB摄像头(USB3.0)
    cap = cv.VideoCapture(camera_num, cv.CAP_DSHOW)
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
        # 返回两个参数，第一个是bool是否正常打开，第二个是照片数组，如果只设置一个则变成一个tuple包含bool和图片
        ret_flag, image = cap.read()
        # 窗口显示，显示名为 s-get Photo   q-finish
        cv.imshow("s-take Photo   q-finish", image)
        # 每帧数据延时 1ms，延时不能为 0，否则读取的结果会是静态帧
        k = cv.waitKey(1) & 0xFF
        # 若检测到按键 ‘s’，打印信息
        if k == ord('s') or k == ord('S'):
            # 保存当前照片
            cv.imwrite(PHOTO_PATH + chr(name_num) + ".png", image)
            # 输出照片信息
            print(f'照片长: {cap.get(3)}')
            print(f'照片宽: {cap.get(4)}')
            print(f"照片名: {chr(name_num)}.png")
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
        elif k == ord('q') == ord('Q'):  # 若检测到按键 ‘q’，退出
            break

    # 写入xyz数据到xyz.txt
    np.savetxt(XYZ_PATH, pose[:, 0:3], delimiter=',')

    # 写入rxryrz数据到rxryrz.txt
    np.savetxt(RXRYRZ_PATH, pose[:, 3:6], delimiter=',')

    # 释放摄像头
    cap.release()
    # 删除建立的全部窗口
    cv.destroyAllWindows()


if __name__ == '__main__':
    ro = robotcontrol.RobotControl()
    # ro.reset()
    # ro.moveJ_Angle([-60, -128, -60, -15, 90, -95, ])
    # 自由移动模式
    ro.rtde_c.teachMode()
    get_photo(8, 2)
    camera_calibration.camera_calibration()
