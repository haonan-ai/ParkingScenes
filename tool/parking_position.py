import random
import carla

town04_bound = {
    "x_min": 264.0,
    "x_max": 304.0,
    "y_min": -241.0,
    "y_max": -178.0,
}

slot_id = [
    '2-1',   # 0
    '2-3',
    '2-5',
    '2-7',
    '2-9',
    '2-11',
    '2-13',
    '2-15',
    '3-1',
    '3-3',
    '3-5',
    '3-7',
    '3-9',
    '3-11',
    '3-13',
    '3-15',  # 15
]

parking_vehicle_locations_Town04 = [
    # row 1
    carla.Location(x=298.5, y=-235.73, z=0.3),  # 1-1
    carla.Location(x=298.5, y=-232.73, z=0.3),  # 1-2
    carla.Location(x=298.5, y=-229.53, z=0.3),  # 1-3
    carla.Location(x=298.5, y=-226.43, z=0.3),  # 1-4
    carla.Location(x=298.5, y=-223.43, z=0.3),  # 1-5
    carla.Location(x=298.5, y=-220.23, z=0.3),  # 1-6
    carla.Location(x=298.5, y=-217.23, z=0.3),  # 1-7
    carla.Location(x=298.5, y=-214.03, z=0.3),  # 1-8
    carla.Location(x=298.5, y=-210.73, z=0.3),  # 1-9
    carla.Location(x=298.5, y=-207.30, z=0.3),  # 1-10
    carla.Location(x=298.5, y=-204.23, z=0.3),  # 1-11
    carla.Location(x=298.5, y=-201.03, z=0.3),  # 1-12
    carla.Location(x=298.5, y=-198.03, z=0.3),  # 1-13
    carla.Location(x=298.5, y=-194.90, z=0.3),  # 1-14
    carla.Location(x=298.5, y=-191.53, z=0.3),  # 1-15
    carla.Location(x=298.5, y=-188.20, z=0.3),  # 1-16

    # row 2
    carla.Location(x=290.9, y=-235.73, z=0.3),  # 2-1
    carla.Location(x=290.9, y=-232.73, z=0.3),  # 2-2
    carla.Location(x=290.9, y=-229.53, z=0.3),  # 2-3
    carla.Location(x=290.9, y=-226.43, z=0.3),  # 2-4
    carla.Location(x=290.9, y=-223.43, z=0.3),  # 2-5
    carla.Location(x=290.9, y=-220.23, z=0.3),  # 2-6
    carla.Location(x=290.9, y=-217.23, z=0.3),  # 2-7
    carla.Location(x=290.9, y=-214.03, z=0.3),  # 2-8
    carla.Location(x=290.9, y=-210.73, z=0.3),  # 2-9
    carla.Location(x=290.9, y=-207.30, z=0.3),  # 2-10
    carla.Location(x=290.9, y=-204.23, z=0.3),  # 2-11
    carla.Location(x=290.9, y=-201.03, z=0.3),  # 2-12
    carla.Location(x=290.9, y=-198.03, z=0.3),  # 2-13
    carla.Location(x=290.9, y=-194.90, z=0.3),  # 2-14
    carla.Location(x=290.9, y=-191.53, z=0.3),  # 2-15
    carla.Location(x=290.9, y=-188.20, z=0.3),  # 2-16

    # row 3
    carla.Location(x=280.0, y=-235.73, z=0.3),  # 3-1
    carla.Location(x=280.0, y=-232.73, z=0.3),  # 3-2
    carla.Location(x=280.0, y=-229.53, z=0.3),  # 3-3
    carla.Location(x=280.0, y=-226.43, z=0.3),  # 3-4
    carla.Location(x=280.0, y=-223.43, z=0.3),  # 3-5
    carla.Location(x=280.0, y=-220.23, z=0.3),  # 3-6
    carla.Location(x=280.0, y=-217.23, z=0.3),  # 3-7
    carla.Location(x=280.0, y=-214.03, z=0.3),  # 3-8
    carla.Location(x=280.0, y=-210.73, z=0.3),  # 3-9
    carla.Location(x=280.0, y=-207.30, z=0.3),  # 3-10
    carla.Location(x=280.0, y=-204.23, z=0.3),  # 3-11
    carla.Location(x=280.0, y=-201.03, z=0.3),  # 3-12
    carla.Location(x=280.0, y=-198.03, z=0.3),  # 3-13
    carla.Location(x=280.0, y=-194.90, z=0.3),  # 3-14
    carla.Location(x=280.0, y=-191.53, z=0.3),  # 3-15
    carla.Location(x=280.0, y=-188.20, z=0.3),  # 3-16

    # row 4
    carla.Location(x=272.5, y=-235.73, z=0.3),  # 4-1
    carla.Location(x=272.5, y=-232.73, z=0.3),  # 4-2
    carla.Location(x=272.5, y=-229.53, z=0.3),  # 4-3
    carla.Location(x=272.5, y=-226.43, z=0.3),  # 4-4
    carla.Location(x=272.5, y=-223.43, z=0.3),  # 4-5
    carla.Location(x=272.5, y=-220.23, z=0.3),  # 4-6
    carla.Location(x=272.5, y=-217.23, z=0.3),  # 4-7
    carla.Location(x=272.5, y=-214.03, z=0.3),  # 4-8
    carla.Location(x=272.5, y=-210.73, z=0.3),  # 4-9
    carla.Location(x=272.5, y=-207.30, z=0.3),  # 4-10
    carla.Location(x=272.5, y=-204.23, z=0.3),  # 4-11
    carla.Location(x=272.5, y=-201.03, z=0.3),  # 4-12
    carla.Location(x=272.5, y=-198.03, z=0.3),  # 4-13
    carla.Location(x=272.5, y=-194.90, z=0.3),  # 4-14
    carla.Location(x=272.5, y=-191.53, z=0.3),  # 4-15
    carla.Location(x=272.5, y=-188.20, z=0.3),  # 4-16
]

parking_vehicle_locations_Town07 = [
    # row 1
    carla.Location(x=-29.3, y=-70.5, z=0.1),    # 1-1
    carla.Location(x=-32.0, y=-70.5, z=0.1),    # 1-2
    carla.Location(x=-34.5, y=-70.5, z=0.1),    # 1-3
    carla.Location(x=-37.2, y=-70.5, z=0.1),    # 1-4
    carla.Location(x=-40.0, y=-70.5, z=0.1),    # 1-5
    carla.Location(x=-42.6, y=-70.5, z=0.1),    # 1-6
    carla.Location(x=-45.5, y=-70.5, z=0.1),    # 1-7
    carla.Location(x=-48.2, y=-70.5, z=0.1),    # 1-8
    carla.Location(x=-50.8, y=-70.5, z=0.1),    # 1-9
    carla.Location(x=-53.5, y=-70.5, z=0.1),    # 1-10
    carla.Location(x=-56.2, y=-70.5, z=0.1),    # 1-11
    carla.Location(x=-58.9, y=-70.5, z=0.1),    # 1-12
    carla.Location(x=-61.6, y=-70.5, z=0.1),    # 1-13
    carla.Location(x=-64.4, y=-70.5, z=0.1),    # 1-14
    carla.Location(x=-67.2, y=-70.5, z=0.1),    # 1-15
    carla.Location(x=-69.9, y=-70.5, z=0.1),    # 1-16
    carla.Location(x=-72.9, y=-70.5, z=0.1),    # 1-17
    carla.Location(x=-76.0, y=-70.5, z=0.1),    # 1-18

    # row 2
    carla.Location(x=-33.4, y=-83.5, z=0.1),  # 2-1
    carla.Location(x=-37.1, y=-83.5, z=0.1),  # 2-2
    carla.Location(x=-40.9, y=-83.5, z=0.1),  # 2-3
    carla.Location(x=-44.7, y=-83.5, z=0.1),  # 2-4
    carla.Location(x=-48.4, y=-83.5, z=0.1),  # 2-5
    carla.Location(x=-52.2, y=-83.5, z=0.1),  # 2-6
    carla.Location(x=-56.0, y=-83.5, z=0.1),  # 2-7
    carla.Location(x=-59.7, y=-83.5, z=0.1),  # 2-8
]

parking_vehicle_locations_Town10 = [
    carla.Location(x=-22.8, y=127.2, z=0.1),  # 1
    carla.Location(x=-16.9, y=127.2, z=0.1),  # 2
    carla.Location(x=-11.3, y=127.2, z=0.1),  # 3
    carla.Location(x=- 5.2, y=127.2, z=0.1),  # 4
    carla.Location(x=- 0.7, y=127.2, z=0.1),  # 5
    carla.Location(x=  3.8, y=127.2, z=0.1)   # 6
]



# class EgoPosTown04:
#     def __init__(self):
#         self.x = 285.600006   # 2-1 slot.x
#         self.y = -243.729996  # 2-1 slot.y - 8.0
#         self.z = 0.32682
#         self.yaw = 90.0

#         self.yaw_to_r = 90.0
#         self.yaw_to_l = -90.0

#         self.goal_y = None
#         self.y_max = None
#         self.y_min = None
#         self.y_step = None

#     def get_cur_ego_transform(self):
#         return carla.Transform(carla.Location(x=self.x, y=self.y, z=self.z),
#                                carla.Rotation(pitch=0.0, yaw=self.yaw, roll=0.0))

#     def get_init_ego_transform(self):
#         return self.get_cur_ego_transform()

#     def update_y_scope(self, goal_y):
#         self.goal_y = goal_y
#         self.y_max = self.goal_y + 8
#         self.y_min = self.goal_y - 8

#     def update_data_gen_goal_y(self, goal_y):
#         self.update_y_scope(goal_y)

#     def update_eva_goal_y(self, goal_y, every_parking_num, parking_idx):
#         self.update_y_scope(goal_y)

#         self.y = self.y_min
#         self.yaw = self.yaw_to_r if parking_idx < (every_parking_num / 2) else self.yaw_to_l

#         if every_parking_num > 1:
#             self.y_step = (self.y_max - self.y_min) / (every_parking_num - 1)
#             self.y = self.y_min
#         else:
#             self.y_step = 0.0
#             self.y = self.goal_y

#     def get_data_gen_ego_transform(self):
#         # 随机选择一个 y 坐标
#         self.y = random.uniform(self.y_min, self.y_max)
#         # 根据 y 的位置来决定车辆朝向
#         # self.yaw = self.yaw_to_r if self.y < self.goal_y else self.yaw_to_l
#         self.yaw = self.yaw_to_r if self.y > self.goal_y else self.yaw_to_l
#         return self.get_cur_ego_transform()

#     def get_eva_ego_transform(self, every_parking_num, parking_idx):
#         self.yaw = self.yaw_to_r if parking_idx < (every_parking_num / 2) else self.yaw_to_l
#         ego_transform = self.get_cur_ego_transform()
#         self.y += self.y_step
#         return ego_transform


class EgoPos:
    def __init__(self, map_name):
        # **存储不同地图的自车初始位置**
        ego_positions = {
            "Town04_Opt": {
                "x": 285.600006, "y": -243.729996, "z": 0.32682, "yaw": 90.0,   # 2-1 slot 附近   自车 y 坐标变化
                "yaw_to_r": 90.0, "yaw_to_l": -90.0,
                "goal_y": None, "y_max": None, "y_min": None, "y_step": None
            },
            "Town07_Opt": {
                "x": -39.6, "y": -63.5, "z": 0.3, "yaw": 0,                  # 1-7 slot 附近   自车 x 坐标变化
                "yaw_to_r": 180.0, "yaw_to_l": 0,
                "goal_x": None, "x_max": None, "x_min": None, "x_step": None
            },
            "Town10HD_Opt": {
                "x": -17.9, "y": 129.6, "z": 0.3, "yaw": 90,                  # 1-7 slot 附近   自车 x 坐标变化
                "yaw_to_r": 180.0, "yaw_to_l": 0,
                "goal_x": None, "x_max": None, "x_min": None, "x_step": None
            }
            
        }

        # **获取当前地图的参数**
        if map_name not in ego_positions:
            raise ValueError(f"Unknown map name: {map_name}")

        ego_data = ego_positions[map_name]

        # **赋值自车初始位置**
        self.x = ego_data["x"]
        self.y = ego_data["y"]
        self.z = ego_data["z"]
        self.yaw = ego_data["yaw"]
        self.yaw_to_r = ego_data["yaw_to_r"]
        self.yaw_to_l = ego_data["yaw_to_l"]

        if map_name == "Town04_Opt":
            self.goal_y = ego_data["goal_y"]
            self.y_max = ego_data["y_max"]
            self.y_min = ego_data["y_min"]
            self.y_step = ego_data["y_step"]
        elif map_name == "Town07_Opt" or map_name == "Town10HD_Opt":
            self.goal_x = ego_data["goal_x"]
            self.x_max = ego_data["x_max"]
            self.x_min = ego_data["x_min"]
            self.x_step = ego_data["x_step"]            

    def get_cur_ego_transform(self):
        return carla.Transform(carla.Location(x=self.x, y=self.y, z=self.z),
                            #   (carla.Location(x=self.x, y=-248.7, z=self.z), 
                               carla.Rotation(pitch=0.0, yaw=self.yaw, roll=0.0))

    def get_init_ego_transform(self):
        return self.get_cur_ego_transform()

    def update_y_scope(self, goal_y):
        self.goal_y = goal_y
        self.y_max = self.goal_y + 8
        self.y_min = self.goal_y - 8

    def update_x_scope(self, goal_x):
        self.goal_x = goal_x
        # self.x_max = self.goal_x + 8
        self.x_max = self.goal_x - 8
        self.x_min = self.goal_x - 8

    def update_data_gen_goal_y(self, goal_y):
        self.update_y_scope(goal_y)

    def update_data_gen_goal_x(self, goal_x):
        self.update_x_scope(goal_x)

    def update_eva_goal_y(self, goal_y, every_parking_num, parking_idx):
        self.update_y_scope(goal_y)

        self.y = self.y_min
        self.yaw = self.yaw_to_r if parking_idx < (every_parking_num / 2) else self.yaw_to_l

        if every_parking_num > 1:
            self.y_step = (self.y_max - self.y_min) / (every_parking_num - 1)
            self.y = self.y_min
        else:
            self.y_step = 0.0
            self.y = self.goal_y

    def get_data_gen_ego_transform_y(self):
        # 随机选择一个 y 坐标
        self.y = random.uniform(self.y_min, self.y_max)
        # 根据 y 的位置来决定车辆朝向
        # self.yaw = self.yaw_to_r if self.y < self.goal_y else self.yaw_to_l
        # self.yaw = self.yaw_to_r if self.y > self.goal_y else self.yaw_to_l
        self.yaw = self.yaw_to_l
        return self.get_cur_ego_transform()
    
    def get_data_gen_ego_transform_x(self):
        # 随机选择一个 x 坐标
        self.x = random.uniform(self.x_min, self.x_max)
        self.x=self.goal_x
        # 根据 y 的位置来决定车辆朝向
        self.yaw = self.yaw_to_r if self.x < self.goal_x else self.yaw_to_l
        self.yaw = self.yaw_to_r if self.x > self.goal_x else self.yaw_to_l
        return self.get_cur_ego_transform()
    
    def get_data_gen_ego_transform(self):
        # 随机选择一个 x 坐标
        self.x = random.uniform(self.x_min, self.x_max)
        self.yaw = 180
        return self.get_cur_ego_transform()

    def get_eva_ego_transform(self, every_parking_num, parking_idx):
        self.yaw = self.yaw_to_r if parking_idx < (every_parking_num / 2) else self.yaw_to_l
        ego_transform = self.get_cur_ego_transform()
        self.y += self.y_step
        return ego_transform