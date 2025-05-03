import logging
import weakref

import cv2
import numpy as np
import pygame

import carla

from carla import ColorConverter as cc

from tool.hud import get_actor_display_name


# Carla 碰撞传感器
class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor # Ego 车辆，碰撞传感器将被附加到这辆车上
        self.hud = hud              # HUD，用于在UI上显示消息
        self.is_collision = False
        world = self._parent.get_world()    # 获取 Carla 世界
        bp = world.get_blueprint_library().find('sensor.other.collision')  # 获取碰撞传感器的蓝图
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent) # 在世界中生成碰撞传感器, 并将其附加到 Ego 车辆上
        # We need to pass the lambda a weak reference to self to avoid circular reference.
        # 创建 self 的弱引用，避免循环引用导致的内存泄漏
        weak_self = weakref.ref(self)
        # 监听碰撞事件，每当发生碰撞，CARLA 会调用 _on_collision() 方法
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    # 碰撞处理
    @staticmethod # 静态方法，不会隐式传入 self，需要手动获取 self
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.is_collision = True
        # 获取碰撞对象的可读名称
        actor_type = get_actor_display_name(event.other_actor)
        # 在控制台和 HUD 上打印碰撞信息
        logging.info('Collision with %s; Restart task', actor_type)
        self.hud.notification('Collision with %r; Restart task' % actor_type)


# 管理摄像头传感器
# 绑定摄像头到车辆
# 切换不同的摄像头模式（RGB、深度、分割、LiDAR）
# 解析图像数据并渲染
# 记录图像并保存视频
class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor # Ego 车辆，摄像头传感器将被附加到这辆车上
        self.hud = hud

        bound_y = 0.5 + self._parent.bounding_box.extent.y # 车辆的 y 方向边界，用于确定摄像头安装在车的侧面时的位置
        Attachment = carla.AttachmentType
        # 摄像头的位置预设
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),  # 后视角，车尾远处，SpringArm 传感器相对车辆的位移不固定，传感器可在一定范围内移动
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),                                  # 第一人称视角，驾驶员位置，Rigid 直接附加到车上
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),                       # 右侧视角，车右前方
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),  # 俯视角，车顶上方
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]                       # 左侧视角，车左侧
        self.transform_index = 1 # 默认选择第一人称视角

        # 传感器列表，包括['传感器蓝图 ID', '转换类型', '传感器名称', '自定义参数']
        self.sensors = [
            # RGB 相机
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            # 深度相机
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            # 语义分割相机
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)', {}],
            # LiDAR 雷达                                         最大扫描距离 50 米
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            # DVS 相机，动态视觉传感器（运动检测）
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            # RGB 畸变相机
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
             {'lens_circle_multiplier': '3.0',
              'lens_circle_falloff': '3.0',
              'chromatic_aberration_intensity': '0.5',
              'chromatic_aberration_offset': '0'}]]
        
        # 传感器蓝图配置
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        # 遍历传感器列表，为每个传感器配置蓝图
        for item in self.sensors:
            bp = bp_library.find(item[0])
            # 设置相机
            if item[0].startswith('sensor.camera'):
                # 修改分辨率
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                # 设置自定义参数
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            # 设置雷达
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50  # 雷达范围

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            # 每个 sensor 附加对应的蓝图 bp
            item.append(bp)
        self.index = None

        self._images = []

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    # 切换/创建摄像头传感器，并将其附加到 Ego 车辆上，同时监听传感器数据流
    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors) #  确保索引不会越界
        # 判断是否需要重新创建传感器
        # 如果 self.index 为空（表示当前没有传感器），则一定需要创建新传感器
        # 如果 force_respawn=True（强制重建传感器），则重新创建传感器
        # 如果新传感器的类型与当前不同，则需要重新创建
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        # 重新创建传感器
        if needs_respawn:
            # 先销毁旧的传感器
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            # 再创建新传感器
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],                            # 传感器蓝图
                self._camera_transforms[self.transform_index][0],   # 传感器位姿
                attach_to=self._parent,                             # 附加到 Ego 车辆
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid circular reference.
            # 创建 self 的弱引用，避免循环引用导致的内存泄漏
            # 监听传感器数据流，每当传感器有数据流入，CARLA 会调用 _parse_image() 方法
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index # 更新传感器索引

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def save_video(self, save_path):
        if len(self._images) == 0:
            return
        height, width = self._images[0].shape[:2]
        video_path = save_path / 'task.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(video_path), fourcc, 20.0, (width,  height))
        for image in self._images:
            out.write(image)
        out.release()

    def clear_saved_images(self):
        self._images.clear()

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))  # BGRA - 4 channels
            array = array[:, :, :3]  # only keep BGR
            array = array[:, :, ::-1]  # convert to RGB
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

            image_array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            self._images.append(image_array)
