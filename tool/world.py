import logging
import sys
import random
import re
import math
from queue import Queue, Empty

import numpy as np
import carla

from data_generation.hud import HUD, get_actor_display_name
from data_generation.sensors import CollisionSensor, CameraManager
from data_generation import parking_position
from data_generation.bev_render import BevRender


# 获取 CARLA 所有预设天气参数，返回列表
def find_weather_presets():
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), x) for x in presets]


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    sensor_queue.put((sensor_data, sensor_name))


# CARLA 仿真环境的核心管理器
# 用于：
# 1.初始化 CARLA 世界，并设置同步模式     2.管理停车场目标点
# 3.管理自车（Ego Vehicle）及传感器      4.管理天气、相机、HUD 显示
# 5.存储车辆及传感器数据
class World(object):
    def __init__(self, carla_world, args):
        # set carla in sync + fixed time step mode
        self._world = carla_world
        self._client = carla.Client(args.host, args.port)
        settings = self._world.get_settings()
        settings.fixed_delta_seconds = float(1 / 30)  # 固定仿真步长，设置帧率为 30 FPS
        settings.synchronous_mode = True              # 同步模式，确保仿真每帧都会等待客户端指令，避免数据丢失或不一致
        self._world.apply_settings(settings)
        self.is_restart=True
        self.over=False
        self.data=[]

        # 选择地图并加载停车位
        if args.map == 'Town04_Opt':
            self._parking_spawn_points = parking_position.parking_vehicle_locations_Town04.copy()
        elif args.map == 'Town07_Opt':
            self._parking_spawn_points = parking_position.parking_vehicle_locations_Town07.copy()
        elif args.map == 'Town10HD_Opt':
            self._parking_spawn_points = parking_position.parking_vehicle_locations_Town10.copy()
        else:
            logging.error('Invalid map %s', args.map)
            sys.exit(1)

        # 获取 CARLA 地图对象
        try:
            self._map = self._world.get_map()
        except RuntimeError as error:
            logging.error('RuntimeError: {}'.format(error))
            logging.error('The server could not send the OpenDRIVE (.xodr) file:')
            logging.error('Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        self._step = -1

        # 存储所有可能的停车目标点
        self._all_parking_goals = []

        self._hud = HUD(args.width, args.height)
        self._keyboard_restart_task = False

        # 传感器管理
        self._collision_sensor = None
        self._camera_manager = None
        self._weather_presets = find_weather_presets() # 获取所有天气预设
        # del self._weather_presets[5]                 # 删除第 5 个天气预设
        self._weather_presets = [                      # 删除带有 Rain 和 Night 的天气预设
            preset for preset in self._weather_presets if not re.search(r"Rain|Night", preset[1])
        ]
        self._weather_index = 0                        # 天气预设索引
        self._gamma = args.gamma

        # 停车场中其他车辆列表
        self._actor_list = []
        # ego 自车
        self._player = None
        # 摄像机观察者
        self._spectator = None
        # 自车的传感器
        self._sensor_list = []
        # sensor data queue on each frame; used for sensor callback 传感器数据的队列
        self._sensor_queue = Queue()
        # sensor data on each frame 传感器数据的字典
        self._sensor_data_frame = {}

        # 在 CARLA 每帧更新时，调用 self._hud.on_world_tick() 方法更新 HUD 信息
        self._world.on_tick(self._hud.on_world_tick)

        # 是否随机化静态车辆和天气
        self._shuffle_static_vhe = args.shuffle_veh    # True
        self._shuffle_weather = args.shuffle_weather   # False

        # Render BEV Segmentation map
        self._bev_render_device = args.bev_render_device  # cpu
        self._bev_render = None

        # 相机参数
        self._cam_config = {}
        self._cam_center = None
        self._cam_specs = {}
        self._intrinsic = None
        self._cam2pixel = None
        self._veh2cam_dict = {}

        # 目标位置偏差
        self._x_diff_to_goal = 0            # x 方向偏差
        self._y_diff_to_goal = 0            # y 方向偏差
        self._distance_diff_to_goal = 0     # 距离偏差
        self._rotation_diff_to_goal = 0     # 朝向偏差

        # 标记是否需要初始化 Ego 车辆状态
        self._need_init_ego_state = True

    # 重启场景，重新初始化玩家和相关环境
    def restart(self, seed, target_index, ego_transform, args):

        # 在停车场内生成静态 NPC 车辆
        if self._shuffle_static_vhe:
            self.init_static_npc(seed, target_index, args)

        if hasattr(self, '_pedestrian') and self._pedestrian is not None:
            self._pedestrian.destroy()
            self._pedestrian = None
            logging.info("Pedestrian has been destroyed.")

        if args.place_pedestrians == True:
            if args.map == "Town04_Opt":
                random_y = random.uniform(-30.0, -10.0)
                start_location = ego_transform.location + carla.Location(x=2.0, y=random_y, z=0.3)
                if start_location.y < -239.8:
                    start_location.y = -239.8
                target_location = carla.Location(x=287.0, y=-183.20, z=0.3)
            elif args.map == "Town07_Opt":
                random_x = random.uniform(10.0, 30.0)
                start_location = ego_transform.location + carla.Location(x=random_x, y=-3.0, z=0.3)
                target_location = carla.Location(x=-76.0, y=-65.5, z=0.1)
            elif args.map == "Town10HD_Opt":
                random_x = random.uniform(10.0, 30.0)
                start_location = ego_transform.location + carla.Location(x=random_x, y=3.0, z=0.3)
                target_location = carla.Location(x=-22.8, y=127.2, z=0.1)
            
            self._pedestrain_paths = [(start_location, target_location)]
            self.init_dynamic_pedestrians(self._pedestrain_paths, args)

        # 初始化 ego 位置
        self._player.set_transform(ego_transform)
        self._player.apply_control(carla.VehicleControl())
        self._player.set_target_velocity(carla.Vector3D(0, 0, 0))
        
        # self._spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(z=50),
        #                                               carla.Rotation(pitch=-90)))

        actor_type = get_actor_display_name(self._player)
        self._hud.notification(actor_type)

        if self._shuffle_weather:
            self.next_weather()

        self._camera_manager.clear_saved_images()

        self._need_init_ego_state = True

    # 在停车场中生成玩家控制的车辆
    def init_ego_vehicle(self, ego_transform,args):

        ego_vehicle_bp = self._world.get_blueprint_library().find('vehicle.tesla.model3')
        # 生成自车实例并放置到指定位置和方向
        self._player = self._world.spawn_actor(ego_vehicle_bp, ego_transform)

        # 初始化俯视图渲染器
        self._bev_render = BevRender(self, self._bev_render_device)

        self._spectator = self._world.get_spectator()
        # 把摄像机放置在 ego_transform 之上 50m，然后 俯视（pitch=-90°），摄像机始终跟随自车
        # self._spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(z=50),
        #                                               carla.Rotation(pitch=-90)))

        # 摄像头位置，将摄像机固定在 (x=283.825165, y=-210.039487, z=35.0) 的位置，并俯视   Town04_Opt 停车场最中间的位置
        if args.map == "Town04_Opt":
            self._spectator.set_transform(carla.Transform(carla.Location(x=283.825165, y=-210.039487, z=35.0),
                                                      carla.Rotation(pitch=-90, yaw = 0)))
        
        # Town07_Opt 停车场最中间的位置
        elif args.map == "Town07_Opt":
            self._spectator.set_transform(carla.Transform(carla.Location(x=-47.637909, y=-74.943932, z=35.0),
                                                      carla.Rotation(pitch=-90, yaw = -91)))
        elif args.map == "Town10HD_Opt":
            self._spectator.set_transform(carla.Transform(carla.Location(x=-12.230294, y=127.928894, z=35.0),
                                                      carla.Rotation(pitch=-90)))

        # 在 HUD 显示车辆信息
        actor_type = get_actor_display_name(self._player)
        self._hud.notification(actor_type) 

    # 在停车场中生成静态 NPC（非玩家控制）车辆
    def init_static_npc(self, seed, target_index, args):

        random.seed(seed)

        # 获取目标停车位置
        target_parking_goal = self._parking_spawn_points[target_index]

        # 记录停车场中的总停车点数量 get all possible spawn points in the parking lot 
        logging.info("total parking points: %d", len(self._parking_spawn_points))

        # 生成静态车辆数量，在总停车点数量的 1/3 到 1 之间随机生成
        # static_vehicle_num = random.randint(int(len(self._parking_spawn_points) / 3),
        #                                     len(self._parking_spawn_points) - 1)
        static_vehicle_num = len(self._parking_spawn_points) - 1
        # static_vehicle_num = 0
        # static_vehicle_num = 2
        logging.info("spawn %d static vehicle in parking lot", static_vehicle_num)

        # 随机打乱停车点
        parking_points_copy = self._parking_spawn_points.copy()
        random.shuffle(parking_points_copy)

        # 从 CARLA 世界的蓝图库中获取四轮车的蓝图 choose only 4 wheels vehicles
        blueprints = self._world.get_blueprint_library().filter('vehicle')
        blueprints = [bp for bp in blueprints if 'firetruck' not in bp.id.lower() and 'fusorosa' not in bp.id.lower() and 'cybertruck' not in bp.id.lower() and 'ambulance' not in bp.id.lower()]
        blueprints = [x for x in blueprints if self.valid_vehicle(x)]

        if args.map == "Town04_Opt" or args.map == "Town10HD_Opt":
            parking_vehicle_rotation = [
                carla.Rotation(yaw=180),
                carla.Rotation(yaw=0)
            ]
        elif args.map == "Town07_Opt":
            parking_vehicle_rotation = [
                carla.Rotation(yaw=90),
                carla.Rotation(yaw=-90)
            ]


        # 遍历前 static_vehicle_num 个停车点，生成静态 NPC 车辆
        for index in range(static_vehicle_num):
            spawn_point = parking_points_copy[index]

            if spawn_point == target_parking_goal:
                self._all_parking_goals.append(spawn_point)
                continue

            # 生成 NPC 车辆的位置和朝向
            npc_transform = carla.Transform(spawn_point, rotation=random.choice(parking_vehicle_rotation))
            npc_bp = random.choice(blueprints)
            npc = self._world.try_spawn_actor(npc_bp, npc_transform)
            if npc is not None:
                npc.set_simulate_physics(False)
                self._actor_list.append(npc)
            else:
                # logging.info("try_spawn_actor %s at (%.3f, %.3f, %.3f) failed!",
                #              npc_bp.id, spawn_point.x, spawn_point.y, spawn_point.z)
                self._all_parking_goals.append(spawn_point)

        # 遍历剩余未用的停车点，将它们设置为停车目标 set parking goal
        for index in range(static_vehicle_num, len(parking_points_copy)):
            self._all_parking_goals.append(parking_points_copy[index])

        logging.info('set %d parking goal', len(self._all_parking_goals))

    # 初始化传感器
    def init_sensors(self):
        self._collision_sensor = CollisionSensor(self._player, self._hud)  # 碰撞传感器
        self._camera_manager = CameraManager(self._player, self._hud, self._gamma) # 摄像头管理器
        self._camera_manager.transform_index = 0    # 选择默认的摄像机视角，后视角
        self._camera_manager.set_sensor(0, notify=False) # 切换/创建摄像头传感器，并将其附加到 Ego 车辆上，同时监听传感器数据流

        # init sensors on ego_vehicle
        self.setup_sensors()


    def init_dynamic_pedestrians(self, pedestrian_paths, args):
        # 获取行人蓝图
        pedestrian_blueprints = self._world.get_blueprint_library().filter('walker.pedestrian.*')

        logging.info("Spawning a dynamic pedestrian")

        # 只取第一个路径（只有一个行人）
        start_location, _ = pedestrian_paths[0]

        # 选择随机行人蓝图
        walker_bp = random.choice(pedestrian_blueprints)
        walker_bp.set_attribute('is_invincible', 'false')  # 行人不是无敌的，会被车辆撞倒
        spawn_transform = carla.Transform(start_location, carla.Rotation(yaw=90))  # 设置初始位置和朝向

        # 生成行人对象
        result = self._client.apply_batch_sync([carla.command.SpawnActor(walker_bp, spawn_transform)], False)
    
        if result[0].error:
            logging.warning("Failed to spawn pedestrian: %s", result[0].error)
            return

        # 获取生成的行人对象
        self._pedestrian = self._world.get_actor(result[0].actor_id)

        if not self._pedestrian:
            logging.warning("No pedestrian was spawned!")
            return

        logging.info("Pedestrian spawned successfully")

        # 使用 WalkerControl 控制行人
        walker_control = carla.WalkerControl()

        # 计算目标方向
        if args.map == "Town04_Opt":
            walker_control.direction = carla.Vector3D(0,1,0).make_unit_vector() # 朝向终点的单位向量
        elif args.map == "Town07_Opt":
            walker_control.direction = carla.Vector3D(-1,0,0).make_unit_vector()
        elif args.map == "Town10HD_Opt":
            walker_control.direction = carla.Vector3D(-1,0,0).make_unit_vector()
        walker_control.speed = 1.2  # 设置速度

        self._pedestrian.apply_control(walker_control)  # 应用控制

        logging.info("Pedestrian is moving towards the target location")


    # def _avoid_vehicles(self, walker):
    #     """
    #     如果行人与自车的 y 坐标差值小于 1 米，首先向 (1, 0, 0) 方向移动 3.5 米，
    #     然后向 (0, 1, 0) 方向移动 3.5 米。
    #     """
    #     ego_vehicle = self._player  # 获取自车
    #     walker_location = walker.get_location()
    #     vehicle_location = ego_vehicle.get_location()

    #     # 计算行人与自车 y 坐标的差值
    #     y_difference = abs(walker_location.y - vehicle_location.y)

    #     if y_difference < 1.0:  # 如果 y 坐标差值小于 1 米
    #         logging.info("Walker moving due to y difference with ego vehicle")

    #         # 向 (1, 0, 0) 方向移动 3.5 米
    #         move_direction = carla.Vector3D(1, 0, 0)  # (1, 0, 0) 方向
    #         new_location = walker_location + move_direction * 3.5  # 移动 3.5 米
    #         walker_control = carla.WalkerControl()
    #         walker_control.direction = (new_location - walker.get_location()).make_unit_vector()
    #         walker_control.speed = 1.0
    #         walker.apply_control(walker_control)

    #         # 然后向 (0, 1, 0) 方向移动 3.5 米
    #         move_direction = carla.Vector3D(0, 1, 0)  # (0, 1, 0) 方向
    #         new_location = walker_location + move_direction * 3.5  # 移动 3.5 米
    #         walker_control = carla.WalkerControl()
    #         walker_control.direction = (new_location - walker.get_location()).make_unit_vector()
    #         walker_control.speed = 1.0  # 设置随机速度
    #         walker.apply_control(walker_control)


    def valid_vehicle(self, vehicle_bp):
        # is_no_bmw_isetta = (vehicle_bp.id != 'vehicle.bmw.isetta')
        # is_no_cybertruck = (vehicle_bp.id != 'vehicle.tesla.cybertruck')
        # is_no_carlacola = (vehicle_bp.id != 'vehicle.carlamotors.carlacola')
        is_four_wheels = int(vehicle_bp.get_attribute('number_of_wheels')) == 4
        return is_four_wheels

    def soft_restart(self, ego_transform):

        # clear cache
        self.sensor_data_frame.clear()
        self._sensor_queue = Queue()
        self._step = -1

        # init the player position
        self._player.set_transform(ego_transform)
        self._player.apply_control(carla.VehicleControl())
        self._player.set_target_velocity(carla.Vector3D(0, 0, 0))
        # self._spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(z=50),
        #                                               carla.Rotation(pitch=-90)))

        self._camera_manager.clear_saved_images()

        self._need_init_ego_state = True

    # 初始化并配置车辆传感器
    def setup_sensors(self):

        # gnss
        bp_gnss = self._world.get_blueprint_library().find('sensor.other.gnss')             # 蓝图
        gnss = self._world.spawn_actor(bp_gnss, carla.Transform(), attach_to=self._player,  # 生成传感器实例，附加到 Ego 车辆上
                                       attachment_type=carla.AttachmentType.Rigid)
        gnss.listen(lambda data: sensor_callback(data, self._sensor_queue, "gnss")) # 监听数据流
        self._sensor_list.append(gnss)

        # imu
        bp_imu = self._world.get_blueprint_library().find('sensor.other.imu')
        imu = self._world.spawn_actor(bp_imu, carla.Transform(), attach_to=self._player,
                                      attachment_type=carla.AttachmentType.Rigid)
        imu.listen(lambda data: sensor_callback(data, self._sensor_queue, "imu"))
        self._sensor_list.append(imu)

        # camera
        self._cam_config = {
            'width': 400,
            'height': 300,
            'fov': 100,     # 视场角
        }
        self._cam_center = np.array([self._cam_config['width'] / 2.0, self._cam_config['height'] / 2.0])
        # 定义 RGB 相机 和 深度相机 的 位置、角度、类型
        self._cam_specs = {
            'rgb_front': {
                'x': 1.5, 'y': 0.0, 'z': 1.5,           # 位置相对车辆中心：x 轴：前后（+前，-后）；y 轴：左右（+右，-左）；z 轴：上下（+上，-下）
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,  
                'type': 'sensor.camera.rgb',
            },
            'rgb_left': {
                'x': 0.0, 'y': -0.8, 'z': 1.5,
                'roll': 0.0, 'pitch': -40.0, 'yaw': -90.0,
                'type': 'sensor.camera.rgb',
            },
            'rgb_right': {
                'x': 0.0, 'y': 0.8, 'z': 1.5,
                'roll': 0.0, 'pitch': -40.0, 'yaw': 90.0,
                'type': 'sensor.camera.rgb',
            },
            'rgb_rear': {
                'x': -2.2, 'y': 0.0, 'z': 1.5,
                'roll': 0.0, 'pitch': -30.0, 'yaw': 180.0,
                'type': 'sensor.camera.rgb',
            },
            'depth_front': {
                'x': 1.5, 'y': 0.0, 'z': 1.5,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'type': 'sensor.camera.depth',
            },
            'depth_left': {
                'x': 0.0, 'y': -0.8, 'z': 1.5,
                'roll': 0.0, 'pitch': -40.0, 'yaw': -90.0,
                'type': 'sensor.camera.depth',
            },
            'depth_right': {
                'x': 0.0, 'y': 0.8, 'z': 1.5,
                'roll': 0.0, 'pitch': -40.0, 'yaw': 90.0,
                'type': 'sensor.camera.depth',
            },
            'depth_rear': {
                'x': -2.2, 'y': 0.0, 'z': 1.5,
                'roll': 0.0, 'pitch': -30.0, 'yaw': 180.0,
                'type': 'sensor.camera.depth',
            },
        }

        # 将相机附着到车辆上
        for key, value in self._cam_specs.items():
            self.spawn_rgb_camera(key, value)

        # 计算相机内参矩阵
        w = self._cam_config['width']
        h = self._cam_config['height']
        fov = self._cam_config['fov']
        f = w / (2 * np.tan(fov * np.pi / 360))  # 焦距
        Cu = w / 2  # 光心，主点坐标
        Cv = h / 2  
        # 内参矩阵
        self._intrinsic = np.array([
            [f, 0, Cu],
            [0, f, Cv],
            [0, 0, 1]
        ], dtype=np.float)

        # CARLA 相机坐标系到像素坐标系的变换
        self._cam2pixel = np.array([[0, 1, 0, 0],
                                    [0, 0, -1, 0],
                                    [1, 0, 0, 0],
                                    [0, 0, 0, 1]], dtype=float)

        # 遍历相机，只处理 RGB 相机
        for cam_id, cam_spec in self._cam_specs.items():
            if cam_id.startswith('rgb'):
                # 相机到车辆的变换
                cam2veh = carla.Transform(carla.Location(x=cam_spec['x'], y=cam_spec['y'], z=cam_spec['z']),
                                          carla.Rotation(yaw=cam_spec['yaw'], pitch=cam_spec['pitch'],
                                                         roll=cam_spec['roll']))
                # 车辆坐标系到像素坐标系的变换
                veh2cam = self._cam2pixel @ np.array(cam2veh.get_inverse_matrix())
                # 存储每个相机的 veh2cam 变换矩阵
                self._veh2cam_dict[cam_id] = veh2cam

    # 将相机附着到车辆上
    def spawn_rgb_camera(self, sensor_id, sensor_spec):
        # 在蓝图库中找到制定类型的传感器
        blueprint_library = self._world.get_blueprint_library()
        bp = blueprint_library.find(sensor_spec['type'])
        # 设置传感器参数
        bp.set_attribute('image_size_x', str(self._cam_config['width']))
        bp.set_attribute('image_size_y', str(self._cam_config['height']))
        bp.set_attribute('fov', str(self._cam_config['fov']))
        # 设置摄像头位置和朝向
        sensor_location = carla.Location(x=sensor_spec['x'],
                                         y=sensor_spec['y'],
                                         z=sensor_spec['z'])
        sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                         roll=sensor_spec['roll'],
                                         yaw=sensor_spec['yaw'])
        sensor_transform = carla.Transform(sensor_location, sensor_rotation)
        # 生成传感器实例，并附着到 Ego 车辆上
        cam = self._world.spawn_actor(bp, sensor_transform, attach_to=self._player,
                                      attachment_type=carla.AttachmentType.Rigid)
        cam.listen(lambda data: sensor_callback(data, self._sensor_queue, sensor_id))
        self._sensor_list.append(cam)

    def spawn_lidar(self, lidar_specs):
        blueprint_library = self.world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('rotation_frequency', str(lidar_specs['rotation_frequency']))
        lidar_bp.set_attribute('points_per_second', str(lidar_specs['points_per_second']))
        lidar_bp.set_attribute('channels', str(lidar_specs['channels']))
        lidar_bp.set_attribute('upper_fov', str(lidar_specs['upper_fov']))
        lidar_bp.set_attribute('atmosphere_attenuation_rate', str(lidar_specs['atmosphere_attenuation_rate']))
        lidar_bp.set_attribute('dropoff_general_rate', str(lidar_specs['dropoff_general_rate']))
        lidar_bp.set_attribute('dropoff_intensity_limit', str(lidar_specs['dropoff_intensity_limit']))
        lidar_bp.set_attribute('dropoff_zero_intensity', str(lidar_specs['dropoff_zero_intensity']))
        lidar_location = carla.Location(x=lidar_specs['x'],
                                        y=lidar_specs['y'],
                                        z=lidar_specs['z'])
        lidar_rotation = carla.Rotation(pitch=lidar_specs['pitch'],
                                        roll=lidar_specs['roll'],
                                        yaw=lidar_specs['yaw'])
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.player,
                                       attachment_type=carla.AttachmentType.Rigid)
        lidar.listen(lambda data: sensor_callback(data, self._sensor_queue, "lidar"))
        self._sensor_list.append(lidar)

    # 切换天气预设
    def next_weather(self, reverse=False):
        # 获取天气索引，确保不越界
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self._hud.notification('Weather: %s' % preset[1]) # 更新 HUD 信息
        self._player.get_world().set_weather(preset[0])   # 更新天气
        self._weather_index += -1 if reverse else 1       # 更新天气索引

    def world_tick(self):
        self._world.tick()

    @property
    def map(self):
        return self._map

    @property
    def step(self):
        return self._step

    @property
    def player(self):
        return self._player

    @property
    def world(self):
        return self._world

    @property
    def hud(self):
        return self._hud

    @property
    def camera_manager(self):
        return self._camera_manager

    @property
    def sensor_data_frame(self):
        return self._sensor_data_frame

    @property
    def bev_state(self):
        return self._bev_render.get_bev_states()

    @property
    def ego_transform(self):
        return self._player.get_transform()

    @property
    def all_parking_goals(self):
        return self._all_parking_goals

    @property
    def x_diff_to_goal(self):
        return self._x_diff_to_goal

    @x_diff_to_goal.setter
    def x_diff_to_goal(self, diff):
        self._x_diff_to_goal = diff

    @property
    def y_diff_to_goal(self):
        return self._y_diff_to_goal

    @y_diff_to_goal.setter
    def y_diff_to_goal(self, diff):
        self._y_diff_to_goal = diff

    @property
    def distance_diff_to_goal(self):
        return self._distance_diff_to_goal

    @distance_diff_to_goal.setter
    def distance_diff_to_goal(self, diff):
        self._distance_diff_to_goal = diff

    @property
    def rotation_diff_to_goal(self):
        return self._rotation_diff_to_goal

    @rotation_diff_to_goal.setter
    def rotation_diff_to_goal(self, diff):
        self._rotation_diff_to_goal = diff

    @property
    def cam_config(self):
        return self._cam_config

    @property
    def intrinsic(self):
        return self._intrinsic

    @property
    def veh2cam_dict(self):
        return self._veh2cam_dict

    @property
    def keyboard_restart_task(self):
        return self._keyboard_restart_task

    @keyboard_restart_task.setter
    def keyboard_restart_task(self, activate):
        self._keyboard_restart_task = activate

    @property
    def need_init_ego_state(self):
        return self._need_init_ego_state

    @need_init_ego_state.setter
    def need_init_ego_state(self, need_init_ego_state):
        self._need_init_ego_state = need_init_ego_state

    def render_BEV_from_state(self, state):
        return self._bev_render.render_BEV_from_state(state)

    def render_BEV(self):
        return self._bev_render.render_BEV()

    def save_video(self, path):
        self._camera_manager.save_video(path)

    # 仿真循环的“心跳”函数，在每一帧调用一次。负责：
    # 1.收集车辆的状态信息（位置、速度、控制输入）  2.获取并存储所有传感器的数据
    # 3.更新 HUD（用户界面）                    4.在仿真环境中绘制目标停车位的标记
    # 5.调整摄像机视角                         6.检测碰撞，并在发生碰撞时触发重新开始
    def tick(self, clock, target_index, args):
        try:
            # 采集车辆的状态信息
            t = self._player.get_transform()    # 位姿
            v = self._player.get_velocity()     # 速度
            c = self._player.get_control()      # 控制
            self._sensor_data_frame['veh_transfrom'] = t
            self._sensor_data_frame['veh_velocity'] = v
            self._sensor_data_frame['veh_control'] = c

            # 采集行人数据
            try:
                t_p = self._pedestrian.get_transform()
                t_v = self._pedestrian.get_velocity()
                self._sensor_data_frame['ped_transform'] = t_p
                self._sensor_data_frame['ped_velocity'] = t_v
            except:
                pass

            # 采集传感器的数据
            for i in range(0, len(self._sensor_list)):
                s_data = self._sensor_queue.get(block=True, timeout=1.0)
                self._sensor_data_frame[s_data[1]] = s_data[0]

                # if s_data[1] == 'rgb_left':
                #     target_ego = convert_veh_coord(self.target_parking_goal.x, self.target_parking_goal.y, self.target_parking_goal.z, t)
                #     self.image_process(target_ego, cam_id=s_data[1], image=s_data[0])

        except Empty:
            logging.error("Some of the sensor information is missed")

        # 更新 HUD
        self._hud.tick(self, clock)

        # 绘制目标停车位 T
        target_parking_goal = self._parking_spawn_points[target_index]
        self._world.debug.draw_string(target_parking_goal, 'T', draw_shadow=True, color=carla.Color(255, 0, 0))

        # 更新观察者（spectator）视角，更新界面里地图的显示方向
        t = self._player.get_transform().location  # 玩家位置
        if args.map == "Town04_Opt":
            self._spectator.set_transform(carla.Transform(t + carla.Location(z=30), carla.Rotation(pitch=-90, yaw = 0)))
        elif args.map == "Town07_Opt":
            self._spectator.set_transform(carla.Transform(t + carla.Location(z=30), carla.Rotation(pitch=-90, yaw = -91)))
        elif args.map == "Town10HD_Opt":
            self._spectator.set_transform(carla.Transform(t + carla.Location(z=30), carla.Rotation(pitch=-90, yaw = -91)))
        self._step += 1

        # detect collision
        if self._collision_sensor.is_collision or self._keyboard_restart_task:
            self._collision_sensor.is_collision = False
            self._keyboard_restart_task = False
            return True

        return False

    def render(self, display):
        self._camera_manager.render(display)
        self._hud.render(display)

    # 清理和销毁传感器、玩家、actors 等对象，确保它们的资源被正确释放，避免内存泄漏
    def destroy(self):
        sensors = self._sensor_list
        if self._camera_manager is not None:
            sensors.append(self._camera_manager.sensor)
        if self._collision_sensor is not None:
            sensors.append(self._collision_sensor.sensor)
        # 遍历传感器，依次停止并销毁
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        self._sensor_list.clear()       # 清空传感器列表
        self._camera_manager = None     # 置空摄像头管理器
        self._collision_sensor = None   # 置空碰撞传感器

        if hasattr(self, '_pedestrian') and self._pedestrian is not None:
            self._pedestrian.destroy()
            self._pedestrian = None
            logging.info("Pedestrian has been destroyed.")

        # 销毁玩家车辆
        if self._player is not None:
            self._player.destroy()

        # 销毁停车场中的静态 NPC 车辆
        for actor in self._actor_list:
            actor.destroy()
        self._actor_list.clear()

        # 清空传感器数据和队列
        self._sensor_data_frame.clear()
        self._sensor_queue = Queue()
        self._step = -1

    def soft_destroy(self):
        if self._shuffle_static_vhe:
            for actor in self._actor_list:
                actor.destroy()
            self._actor_list.clear()

        self._sensor_data_frame.clear()
        self._sensor_queue = Queue()
        self._step = -1
        if self._shuffle_static_vhe:
            self._all_parking_goals.clear()
