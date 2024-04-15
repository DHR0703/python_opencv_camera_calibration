# -*- coding: utf-8 -*-
"""
@Time ： 2022/9/4 16:40
@Auth ： 邓浩然
@File ：opencv_camera.py
@IDE ：PyCharm
@Description：负责拍照的类,使用线程
"""
import time
import pyrealsense2 as rs
import numpy as np
import cv2


class Camera:
    def __init__(self):
        self.flag = True
        # 用于存放帧集合
        self.frame_list = []
        # 时间滤波器
        self.temporal = rs.temporal_filter()
        # 孔洞填充过滤器
        # self.hole_filling = rs.hole_filling_filter()
        # 空间过滤器
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 1)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
        self.spatial.set_option(rs.option.filter_smooth_delta, 50)
        # 叠加孔洞填充
        self.spatial.set_option(rs.option.holes_fill, 3)
        # 深度图着色器
        self.colorizer = rs.colorizer()
        # 点云处理工具
        self.pc = rs.pointcloud()

    # 负责对照片进行处理
    def take_photo(self, color_image):
        # 时间滤波
        temp_filtered = None
        for x in range(10):
            temp_filtered = self.temporal.process(self.frame_list[x])
        filtered_depth = self.spatial.process(temp_filtered)
        filtered_depth_image = np.asanyarray(filtered_depth.get_data())

        # 保存待识别的图片
        cv2.imwrite('color.png', color_image)
        # 保存深度图
        cv2.imwrite('depth.png', filtered_depth_image)

    def run(self):
        # 配置深度和颜色流
        pipeline = rs.pipeline()
        camera_config = rs.config()

        # 获取设备产品线，用于设置支持分辨率
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = camera_config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        # 创建对齐对象（深度对齐颜色）
        align = rs.align(rs.stream.color)
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # 深度参数
        camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # 颜色参数
        camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 开始流式传输
        pipeline.start(camera_config)
        time.sleep(2)
        try:
            while len(self.frame_list) < 10:
                # 等待一对连贯的帧：深度和颜色
                frames = pipeline.wait_for_frames()
                # 对齐后再获取
                aligned_frames = align.process(frames)

                # 需要进行拍照,就开始存放帧
                self.frame_list.append(aligned_frames.get_depth_frame())

                # 获得对齐后的深度和颜色帧
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # 将图像转换为 numpy 数组,也成为了图像
                color_image = np.asanyarray(color_frame.get_data())

                # 在深度图像上应用颜色图
                depth_colormap = self.colorizer.colorize(depth_frame).get_data()

                # 需要进行拍照,我们会获取10帧来进行彩色图,深度图和点云的计算
                if len(self.frame_list) == 10:
                    # 存完了10帧，停止拍照
                    self.take_photo(color_image)
                    # 拍摄完成
                    print('拍照完成')

                images = np.hstack((color_image, depth_colormap))

                # 显示图像
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)

        finally:
            # 停止流式传输
            pipeline.stop()
