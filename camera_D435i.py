import threading
import time
import pyrealsense2 as rs
import numpy as np
import cv2



class RealSenseCamera:
    def __init__(self, output_dir="catch_result/photo"):
        self._display_thread = None
        self.last_depth_image = None
        self.last_color_image = None
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # 获取设备产品线，用于设置支持分辨率
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        # 创建对齐对象（深度对齐颜色）
        self.align = rs.align(rs.stream.color)
        self.output_dir = output_dir
        self.is_running = False

        self.image_ready = threading.Condition(threading.Lock())  # 添加条件锁

        # 设置配置：RGB与深度流均为640x480
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # 创建深度处理管道，包括temporal和spatial滤波器
        self.depth_pipeline = rs.pipeline_profile()
        # 孔洞填充过滤器
        self.hole_filling = rs.hole_filling_filter()
        # 时间滤波器
        self.temporal = rs.temporal_filter()
        # 边缘滤波
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 5)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 8)
        # 创建深度处理管道，包括temporal和spatial滤波器
        self.depth_pipeline = rs.pipeline_profile()
        self.depth_filters = [
            self.hole_filling,
            self.temporal,
            self.spatial
        ]

        self.colorizer = rs.colorizer()

    def start_display_and_capture(self):
        self.is_running = True
        self.pipeline.start(self.config)
        self._display_thread = threading.Thread(target=self._display_and_capture)
        self._display_thread.start()

    def _display_and_capture(self):
        try:
            time.sleep(3)
            while self.is_running:
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)

                # 获取并处理深度数据
                depth_frame = frames.get_depth_frame()
                for filter_ in self.depth_filters:
                    depth_frame = filter_.process(depth_frame)
                # 保存原深度图
                depth = np.asanyarray(depth_frame.get_data())

                # 在深度图像上应用颜色图
                depth_colormap = self.colorizer.colorize(depth_frame)
                depth_image = np.asanyarray(depth_colormap.get_data())

                # 获取彩色图像
                color_frame = frames.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                # 因为opencv使用的是BGR,但是相机用的是RGB,所以要转换
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                # 将RGB图像和深度图像堆叠在一起
                combined_image = np.hstack((color_image, depth_image))

                # 显示堆叠后的图像
                cv2.imshow("RGB & Depth Images", combined_image)

                with self.image_ready:  # 在这里获取锁
                    self.last_color_image = color_image
                    self.last_depth_image = depth

                    self.image_ready.notify_all()  # 通知所有等待的线程，图像已准备好

                cv2.waitKey(1)
        except Exception as e:
            print(f"Error during display and capture: {e}")
            self.is_running = False
        finally:
            cv2.destroyAllWindows()
            self.pipeline.stop()

    def save_screenshot(self, photo_path) -> None:
        with self.image_ready:
            while not (self.last_color_image is not None and self.last_depth_image is not None):
                self.image_ready.wait()  # 等待图像准备好
            # 保存图片
            # 保存待识别的图片
            cv2.imwrite(photo_path, self.last_color_image)

    def stop(self):
        self.is_running = False
