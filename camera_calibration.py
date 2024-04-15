# -*- coding: utf-8 -*-
"""
@Time ： 2022/8/20 14:26
@Auth ： 邓浩然
@File ：camera_calibration.py
@IDE ：PyCharm
@Description：标定部分的代码,需要先完成拍照操作
"""

import cv2 as cv
import numpy as np
import glob

# 不使用e来表示小数,让控制台输出更加人性化
np.set_printoptions(suppress=True)
# 数据txt位置
xyz_path = 'data/xyz.txt'
rxryrz_path = 'data/RxRyRz.txt'
rvecsfilePath = 'data/rvecs.txt'
tvecsfilePath = 'data/tvecs.txt'
# 照片存储位置:
photo_path = 'photo/'
# 点阵图片位置
point_photo_path = 'point_photo/'

RESULT_PATH = 'data/result.txt'

# 棋盘标定板参数
w = 11
h = 8
checker_size = 15


# w = 8
# h = 5
# checker_size = 21.5


# 把矩阵写入到文件里,按照指定格式
def write_vec3d(fpath, mat):
    # 打开文件
    file = open(fpath, 'w')
    # 按照指定格式写入文件
    for num in mat:
        file.write('Vec3d(' + str(num[0]) + ',' + str(num[1]) + ',' + str(num[2]) + '),\n')


# 标定并把数据写入文件
def camera_calibration():
    # 找棋盘格角点
    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # 阈值
    # 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
    objp = np.zeros((w * h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
    # checker_size是棋盘格子的大小,单位是mm
    objp = objp * checker_size
    # 储存棋盘格角点的世界坐标和图像坐标对
    # 在世界坐标系中的三维点
    objpoints = []
    # 在图像平面的二维点
    imgpoints = []
    # 加载photo文件夹下所有的jpg图像,要注意读取的顺序必须和拍摄顺序一致
    images = glob.glob(photo_path + '*.png')
    # 文件名,使用ASCII编码
    name_num = 97
    i = 0
    for fname in images:

        img = cv.imread(fname)
        # 获取画面中心点
        # 获取图像的长宽

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        u, v = img.shape[:2]
        # 找到棋盘格角点,SB函数的鲁棒性更好,噪点忍受能力高
        # ret, corners = cv.findChessboardCorners(gray, (w, h), None)
        ret, corners = cv.findChessboardCornersSB(gray, (w, h), None)
        # 如果找到足够点对，将其存储起来
        if ret:
            print(f"第{i + 1}张图片生成点阵")
            i = i + 1
            # 在原角点的基础上寻找亚像素角点
            cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # 追加进入世界三维点和平面二维点中
            objpoints.append(objp)
            imgpoints.append(corners)
            # 将角点在图像上显示
            cv.drawChessboardCorners(img, (w, h), corners, ret)
            cv.namedWindow('findCorners', cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO | cv.WINDOW_GUI_EXPANDED)
            cv.resizeWindow('findCorners', 640, 480)
            cv.imshow('findCorners', img)
            cv.imwrite(point_photo_path + chr(name_num) + ".png", img)
            name_num += 1
            cv.waitKey(500)
    cv.destroyAllWindows()
    # 标定
    print('正在计算')
    # 标定
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None,
                                                                       None)

    print("retval:", ret)
    # 内参数矩阵
    print("cameraMatrix内参矩阵:\n", camera_matrix)
    # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
    print("distCoeffs畸变值:\n", np.array(dist_coeffs))
    # 旋转向量  # 外参数
    print("rvecs旋转向量外参:\n", np.array(rvecs))
    # 平移向量  # 外参数
    print("tvecs平移向量外参:\n", np.array(tvecs))
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (u, v), 0, (u, v))
    print('newcameramtx内参', newcameramtx)

    # N*3*1旋转矢量
    rvec = np.zeros((len(rvecs), 3, 1), np.double)
    # N*3*1平移XYZ
    tvec = np.zeros((len(tvecs), 3, 1), np.double)
    # 使用solvePnP获得新的标定板在相机坐标系下的位姿信息
    for num_photo in range(len(images)):
        retval, rv, tv = cv.solvePnP(np.array(objpoints[num_photo]),
                                     np.array(imgpoints[num_photo]),
                                     camera_matrix, dist_coeffs)
        rvec[num_photo] = rv
        tvec[num_photo] = tv
    # 写入旋转向量,把N*3*1转成N*3写入TXT
    np.savetxt(rvecsfilePath, np.squeeze(rvec), fmt='%.8f', delimiter=',')
    # 写入平移向量,把N*3*1转成N*3写入TXT
    np.savetxt(tvecsfilePath, np.squeeze(tvec), fmt='%.8f', delimiter=',')

    # 读取xyz
    xyz = np.loadtxt(xyz_path, dtype=np.double, delimiter=',')
    # 读取RxRyRz
    rxryrz = np.loadtxt(rxryrz_path, dtype=np.double, delimiter=',')

    R_base2gripper = []
    t_base2gripper = []
    R_target2cam = []
    t_target2cam = []

    # 开始为各个参数赋值
    for i in range(len(xyz)):
        # 处理R_base2gripper和t_base2gripper
        # 创建新的4*4的矩阵用来存放R和T拼接矩阵
        rtarray = np.zeros((4, 4), np.double)
        # 旋转向量转旋转矩阵,dst是是旋转矩阵,jacobian是雅可比矩阵
        dst, jacobian = cv.Rodrigues(rxryrz[i])
        # 存入旋转矩阵
        rtarray[:3, :3] = dst
        # 传入平移向量
        rtarray[:3, 3] = xyz[i]
        rtarray[3, 3] = 1
        # 求逆矩阵
        rb2e = np.linalg.inv(rtarray)
        # 放入用来传给calibrateHandEye的参数中
        R_base2gripper.append(rb2e[:3, :3].T)
        t_base2gripper.append(xyz[i])

        # 处理R_target2cam和t_target2cam
        # 获标定板在相机坐标系下的旋转矩阵,把旋转向量转成旋转矩阵
        dst1, jacobian1 = cv.Rodrigues(rvec[i])
        # 相机坐标系旋转矩阵转置
        R_target2cam.append(dst1.T)
        # 写入相机坐标系平移向量,平移向量需要乘原本的负的旋转矩阵
        t_target2cam.append(np.matmul(-dst1.T, tvec[i]))

    # 核心方法,前面都是为了得到该方法的参数,获得转换矩阵
    r_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam,
                                                       method=cv.CALIB_HAND_EYE_TSAI)
    # 拼接出转换矩阵
    rt = np.vstack((np.hstack((r_cam2gripper, t_cam2gripper)), np.array([0, 0, 0, 1])))
    # results = np.zeros((3, 4, 4))
    for i in range(len(t_base2gripper)):
        print(str(i) + " ")
        base = np.column_stack((R_base2gripper[i], t_base2gripper[i]))
        base = np.row_stack((base, np.array([0, 0, 0, 1])))

        gripper = np.column_stack((R_target2cam[i], t_target2cam[i]))
        gripper = np.row_stack((gripper, np.array([0, 0, 0, 1])))

        result = np.matmul(np.matmul(base, rt), gripper)
        # result[i] = result
        print(repr(result))
    print('相机相对于末端的变换矩阵为：')
    print(rt)
    # 内参数矩阵
    print("cameraMatrix内参矩阵:\n", camera_matrix)
    # np.savetxt(RESULT_PATH, results, delimiter=',')
