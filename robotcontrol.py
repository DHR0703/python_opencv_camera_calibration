import numpy as np
import util
import math
import rtde_control
import rtde_receive
import rtde_io

# UR5连接参数
HOST = "192.168.31.32"
PORT = 30003


class RobotControl:

    def __init__(self):
        # 初始化socket来获得数据
        self.tool_acc = 0.5  # Safe: 0.5
        self.tool_vel = 0.2  # Safe: 0.2
        # UR官方的RTDE接口,可用于控制和读取数据
        # rtde_c复制UR5的控制
        self.rtde_c = rtde_control.RTDEControlInterface(HOST)
        # rtde_r负责UR5的数据读取
        self.rtde_r = rtde_receive.RTDEReceiveInterface(HOST)
        # rtde_io负责控制机器臂的数字输出等
        self.rtde_io = rtde_io.RTDEIOInterface(HOST)

    def __del__(self):
        self.rtde_r.disconnect()
        self.rtde_c.disconnect()

    def get_current_tcp(self):
        """ 获得XYZ RXRYRZ,XYZ单位是M,示教器上单位是mm.RXYZ和示教器上一致 """
        return self.rtde_r.getActualTCPPose()

    def get_speed(self):
        """ 获得机器臂的运动速度 """
        return self.rtde_r.getActualTCPSpeed()

    def get_current_angle(self):
        """ 获得各个关节的角度,返回数组,依次为机座,肩部,肘部,手腕1 2 3 """
        # 获得弧度数组
        actual = np.array(self.rtde_r.getActualQ())
        # 转化为角度
        actual = actual * 180 / math.pi
        return actual

    def get_current_radian(self):
        """ 返回各个关节的弧度，返回数组,依次为机座,肩部,肘部,手腕1 2 3 """
        return self.rtde_r.getActualQ()

    def get_current_pos(self):
        """ x, y, theta """
        tcp = self.get_current_tcp()
        rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
        return np.asarray([tcp[0], tcp[1], rpy[-1]])

    def get_current_pos_same_with_simulation(self):
        tcp = self.get_current_tcp()
        rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
        return np.asarray([tcp[1], tcp[0], rpy[-1]])

    def move_up(self, z):
        """机械臂末端向上移动多少mm"""
        tcp = self.get_current_tcp()
        tcp[2] = tcp[2] + z / 1000
        self.rtde_c.moveL(tcp, speed=self.tool_vel, acceleration=self.tool_acc)

    def move_down(self, z):
        """机械臂末端向下移动多少mm"""
        tcp = self.get_current_tcp()
        tcp[2] = tcp[2] - z / 1000
        self.rtde_c.moveL(tcp, speed=self.tool_vel, acceleration=self.tool_acc)

    def reset(self, tool_vel=0.8, tool_acc=0.5):
        """机器臂复位"""
        self.rtde_c.moveJ(
            q=[-0.785398157435008, -1.570796314870016, -1.570796314870016, -1.570796314870016, 1.5707963705062866, 0.0],
            speed=tool_vel, acceleration=tool_acc)

    def moveJ_Angle(self, angles, tool_vel=0.8, tool_acc=0.5):
        """机器臂复位"""
        self.rtde_c.moveJ(q=[angles[0] / 180 * math.pi, angles[1] / 180 * math.pi, angles[2] / 180 * math.pi,
                             angles[3] / 180 * math.pi, angles[4] / 180 * math.pi, angles[5] / 180 * math.pi],
                          speed=tool_vel, acceleration=tool_acc)
