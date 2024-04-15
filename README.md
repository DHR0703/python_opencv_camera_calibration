# UR5机器人手眼标定----基于python opencv(眼在手外)
## 环境
系统:WIN11
IDE:pycharm
opencv版本:4.6.0(低于4.1.0没有camera_calibration函数)

## main.py
主要负责拍照部分,弹出窗口后,先输入拍照数量
然后按下s拍照
拍到足够数目的照片后会调用camera_calibration.py进行运算
## camera_calibration.py
自动读取拍照得到的数据进行运算  
核心的函数是camera_calibration()  
参数说明:  
R_base2gripper: 从基座到手臂末端的旋转矩阵,由机器人读数得到的rxryrz得到  
程序读到的是旋转矢量,需要先转换成旋转矩阵,然后和xyz拼接在一起,求逆矩阵,取求出来的逆矩阵的[:3,:3],然后再转置传入!  

t_base2gripper: 从基座到手臂末端的平移向量,由机器人读数得到的xyz得到单位是mm,直接传入  

R_target2cam: 标定板在相机坐标系下的旋转矩阵,由对照片进行标定后的得到,先用solvePnP函数获得,solvePnP得到的是旋转矢量,先用罗德里格斯公式
转成旋转矩阵,然后再转置再传入  

t_target2cam: 标定板在相机坐标系下的平移向量,由对照片进行标定后的得到,先用solvePnP函数获得,需要乘上负的标定板旋转矩阵的转置再传入

method=cv.CALIB_HAND_EYE_TSAI: 指定用什么方法来进行计算,TSAI最快