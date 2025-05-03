import sys
import json
import math
import random
import logging
import pathlib
import carla

import numpy as np
import cv2

from datetime import datetime
from threading import Thread

from tool import parking_position
from tool.tools import encode_npy_to_pil
from tool.world import World
import os
import csv


class DataGenerator:
    def __init__(self, carla_world, args):
        self._seed = args.random_seed  # 随机种子，默认为 0
        random.seed(args.random_seed)  # 固定 Python random 生成的随机数，以便复现实验结果

        # 初始化 CARLA 世界对象，封装移除静态车辆的地图和仿真配置 args
        self._world = World(carla_world, args)

        parking_goal_indices = {
            "Town04_Opt":17,  # 2-2
            "Town07_Opt":1,    # 1-2
            "Town10HD_Opt":0
            # "Town10HD_Opt"
        }
        
        # 泊车任务配置地图
        # 根据地图选择对应的索引
        self._task_index = 0
        
        if args.map == "Town04_Opt":
            self._parking_goal_index = self._task_index*2+17
            self._parking_goal = parking_position.parking_vehicle_locations_Town04[self._parking_goal_index] # 目标停车位置，预定义列表
            self._ego_transform_generator = parking_position.EgoPos(args.map) # 自车位置生成器
        
        elif args.map == "Town07_Opt":
            self._parking_goal = parking_position.parking_vehicle_locations_Town07[self._parking_goal_index]
            self._ego_transform_generator = parking_position.EgoPos(args.map)

        elif args.map == "Town10HD_Opt":
            self._parking_goal_index = self._task_index
            self._parking_goal = parking_position.parking_vehicle_locations_Town10[self._parking_goal_index]
            self._ego_transform_generator = parking_position.EgoPos(args.map)
        # self._parking_goal_index = 17  # 泊车目标的索引值，列表中2-2的索引
        # self._parking_goal = parking_position.parking_vehicle_locations_Town04[self._parking_goal_index] # 目标停车位置，预定义列表
        # self._ego_transform_generator = parking_position.EgoPos() # 自车位置生成器

        # 创建保存结果的目录
        now = datetime.now()
        result_dir = '_'.join(map(lambda x: '%02d' % x, (now.month, now.day, now.hour, now.minute, now.second)))
        self._save_path = pathlib.Path(args.save_path) / args.map / result_dir
        self._save_path.mkdir(parents=True, exist_ok=False)

        self._save_frequency = 3  # 数据保存频率，0.1s 一次 save sensor data for every 3 steps 0.1s

        self._num_tasks = args.task_num
        self._world.index=self._task_index

        self._distance_diff_to_goal = 10000
        self._rotation_diff_to_goal = 10000
        # 到达终点误差阈值
        self._goal_reach_distance_diff = 0.5  # meter
        self._goal_reach_rotation_diff = 3.5  # degree

        # 车辆需要连续在目标区域内 2 秒（以 30Hz 计算为 60 帧）才能认为任务完成。
        # number of frames needs to get into the parking goal in order to consider task completed
        self._num_frames_goal_needed = 30
        self._num_frames_out_time_need = 90
        self._num_frames_in_goal = 0


        # 创建一个空列表，用于临时存储传感器数据，稍后会将这些数据保存到磁盘。
        self._batch_data_frames = []

        self.init(args)

    @property
    def world(self):
        return self._world

    def world_tick(self):
        self._world.world_tick()

    def render(self, display):
        self._world.render(display)

    # 初始化单个任务的仿真环境
    def init(self,args):
        logging.info('*****Init environment for task %d*****', self._task_index)

        # 清理上一次仿真任务中的残留设置和资源
        self.destroy()

        # 初始化停车场中的静态车辆
        self._world.init_static_npc(self._seed, self._parking_goal_index,args)

        # Spawn the player.
        if args.map == "Town04_Opt":
            self._ego_transform_generator.update_data_gen_goal_y(self._parking_goal.y) # 更新自车 y 坐标范围，[goal_y - 8, goal_y + 8]
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform_y() # 生成自车位置和朝向
        elif args.map == "Town07_Opt":
            self._ego_transform_generator.update_data_gen_goal_x(self._parking_goal.x)
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform_x() 
        elif args.map == "Town10HD_Opt":
            self._ego_transform_generator.update_data_gen_goal_x(self._parking_goal.x)
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform()

        self._world.init_ego_vehicle(ego_transform,args) # 生成自车

        # 初始化传感器
        self._world.init_sensors()

        if args.place_pedestrians == True:
            # 初始化动态行人
            if args.map == "Town04_Opt":
                random_y = random.uniform(-30.0, -10.0)
                start_location = ego_transform.location + carla.Location(x=2.0, y=random_y, z=0.3)
                if start_location.y < -239.8:
                    start_location.y = -239.8
                target_location = carla.Location(x=287.0, y=-183.20, z=0.3)
            # if args.map == "Town04_Opt":
            #     start_location = ego_transform.location + carla.Location(x=2.0, y=-15.0, z=0.3)
            #     target_location = carla.Location(x=287.0, y=-183.20, z=0.3)
            elif args.map == "Town07_Opt":
                random_x = random.uniform(10, 30)
                start_location = ego_transform.location + carla.Location(x=random_x, y=-3.0, z=0.3)
                target_location = carla.Location(x=-76.0, y=-65.5, z=0.1)
            elif args.map == "Town10HD_Opt":
                random_x = random.uniform(10.0, 30.0)
                start_location = ego_transform.location + carla.Location(x=random_x, y=3.0, z=0.3)
                target_location = carla.Location(x=-22.8, y=127.2, z=0.1)
            self._pedestrain_paths = [(start_location, target_location)]
            self._world.init_dynamic_pedestrians(self._pedestrain_paths, args)

        
        # 初始化天气
        self._world.next_weather()

        logging.info('*****Init environment for task %d done*****', self._task_index)

    def destroy(self):
        # 清空数据
        self._batch_data_frames.clear()
        self._num_frames_in_goal = 0

        # 清理和销毁传感器、玩家、演员（actors）等对象
        self._world.destroy()

    def soft_destroy(self):
        self._batch_data_frames.clear()
        self._num_frames_in_goal = 0

        self._world.soft_destroy()

    # 每一帧调用，主要作用：
    # 1.更新与目标停车位的距离和角度信息  2.检查是否发生碰撞
    # 3.按照一定的频率保存传感器数据     4.检查车辆是否到达目标停车位
    def tick(self, clock, args):

        self._world.distance_diff_to_goal = self._distance_diff_to_goal
        self._world.rotation_diff_to_goal = self._rotation_diff_to_goal

        is_collision = self._world.tick(clock, self._parking_goal_index, args)
        if is_collision:
            print("collision")
            self.soft_restart(args)
            return

        # 按照固定帧率（self.save_frequency frame）保存传感器数据 
        step = self._world.step
        if step % self._save_frequency == 0: # 0.1s 保存一次数据
            sensor_data_frame = self._world.sensor_data_frame
            sensor_data_frame['bev_state'] = self._world.bev_state
            self._batch_data_frames.append(sensor_data_frame.copy())

        # 检查是否到达目标停车位
        self.check_goal(args)
        self.check_out_time(args)

    def check_out_time(self, args):
        if self._world.over:
            self._num_frames_in_goal += 1
        else:
            self._num_frames_in_goal = 0
        if self._num_frames_in_goal >= self._num_frames_out_time_need:
            self._world.over=False
            self.soft_restart(args)

    # 检查是否到达目标停车位
    def check_goal(self, args):
        t = self._world.ego_transform
        p = t.location      # 车辆位置
        r = t.rotation      # 车辆姿态

        all_parking_goals = self._world.all_parking_goals

        # 找到最近的目标停车位
        self._distance_diff_to_goal = sys.float_info.max
        closest_goal = [0.0, 0.0, 0.0]  # (x, y, yaw)
        for parking_goal in all_parking_goals:
            if p.distance(parking_goal) < self._distance_diff_to_goal:
                self._distance_diff_to_goal = p.distance(parking_goal)
                closest_goal[0] = parking_goal.x
                closest_goal[1] = parking_goal.y
                closest_goal[2] = r.yaw

        # 方向误差
        if args.map == "Town04_Opt" or args.map == "Town10HD_Opt": 
            self._rotation_diff_to_goal = math.sqrt(min(abs(r.yaw), 180 - abs(r.yaw)) ** 2 + r.roll ** 2 + r.pitch ** 2)
        elif args.map == "Town07_Opt": 
            self._rotation_diff_to_goal = math.sqrt(min(abs(r.yaw - 90), 270 - abs(r.yaw)) ** 2 + r.roll ** 2 + r.pitch ** 2)

        # 检查停车位是否达到目标
        # if (self._distance_diff_to_goal < self._goal_reach_distance_diff and self._rotation_diff_to_goal < self._goal_reach_rotation_diff)\
        #         or(self._distance_diff_to_goal < (90 - self._goal_reach_distance_diff) and self._rotation_diff_to_goal < (90 - self._goal_reach_rotation_diff)):
        if self._world.over:
            self._num_frames_in_goal += 1
        else:
            self._num_frames_in_goal = 0

        if self._num_frames_in_goal > self._num_frames_goal_needed:
            logging.info('task %d goal reached; ready to save sensor data', self._task_index)
            self.save_sensor_data(closest_goal)
            logging.info('*****task %d done*****', self._task_index)
            self._task_index += 1
            self._world.index = self._task_index
            if self._task_index >= self._num_tasks:
                csv_file_path=os.path.join('/home/jack/e2e-parking-carla/e2e_parking/Town04_Opt/CARLA.csv')
                with open(csv_file_path, 'w', newline='') as csv_file:
                    csv_writer = csv.writer(csv_file)
                    for da in self._world.data:
                        csv_writer.writerow(da)
                logging.info('completed all tasks; Thank you!')
                exit(0)
            self.restart(args)

    # 重新初始化环境，为新的任务配置停车目标、重置车辆状态，更新模拟环境
    def restart(self, args):
        self._world.is_restart = True
        self._world.over=False
        logging.info('*****Config environment for task %d*****', self._task_index)

        # clear all previous setting
        self.soft_destroy()

        #spawn static vehicles in the parking lot
        # if self._task_index >= 16:
        #     self._parking_goal_index = 17
        # else:
        #     self._parking_goal_index += 2

        # Town04_Opt 的车位按顺序给出
        if args.map == "Town04_Opt": 
            if self._task_index >= 16:
                self._parking_goal_index = 17
            else:
                self._parking_goal_index += 2
            self._parking_goal = parking_position.parking_vehicle_locations_Town04[self._parking_goal_index]
            self._ego_transform_generator.update_data_gen_goal_y(self._parking_goal.y)
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform_y() 
        # Town07_Opt 的目标车位随机生成
        elif args.map == "Town07_Opt": 
            if self._task_index >= 16:
                self._parking_goal_index = 1
            else:
                self._parking_goal_index += 1
            self._parking_goal_index = random.randint(0, 17)
            self._parking_goal = parking_position.parking_vehicle_locations_Town07[self._parking_goal_index]
            self._ego_transform_generator.update_data_gen_goal_x(self._parking_goal.x)
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform_x() 
        # Town10HD_Opt 的目标车位随机生成
        elif args.map == "Town10HD_Opt": 
            if self._task_index >= 6:
                self._parking_goal_index = 1
            else:
                self._parking_goal_index += 1
            # self._parking_goal_index = random.randint(0, 4)
            self._parking_goal = parking_position.parking_vehicle_locations_Town10[self._parking_goal_index]
            self._ego_transform_generator.update_data_gen_goal_x(self._parking_goal.x)
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform() 
        
        # self._seed += 1 # 增加种子数，确保新的环境生成的内容有所变化
        self._world.restart(self._seed,self._parking_goal_index,ego_transform,args)

        logging.info('*****Config environment for task %d done*****', self._task_index)

    def soft_restart(self, args):
        self._world.is_restart = True
        self._world.over=False
        logging.info('*****Restart task %d*****', self._task_index)
        if args.map == "Town04_Opt":
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform_y() 
        elif args.map == "Town07_Opt":
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform_x() 
        elif args.map == "Town10HD_Opt":
            ego_transform = self._ego_transform_generator.get_data_gen_ego_transform()
        self._world.soft_restart(ego_transform)

        # clear cache
        self._batch_data_frames.clear()
        self._num_frames_in_goal = 0

        logging.info('*****Restart task %d done*****', self._task_index)

    # 保存当前泊车任务的传感器数据
    def save_sensor_data(self, parking_goal):
        # create dirs
        cur_save_path = pathlib.Path(self._save_path) / ('task' + str(self._task_index))
        cur_save_path.mkdir(parents=True, exist_ok=False)
        (cur_save_path / 'measurements').mkdir()
        (cur_save_path / 'measurements_pedestrain').mkdir()
        (cur_save_path / 'lidar').mkdir()
        (cur_save_path / 'parking_goal').mkdir()
        (cur_save_path / 'topdown').mkdir()
        for sensor in self._batch_data_frames[0].keys():
            if sensor.startswith('rgb') or sensor.startswith('depth'):
                (cur_save_path / sensor).mkdir()

        # 多线程保存数据
        total_frames = len(self._batch_data_frames)
        thread_num = 10
        frames_for_thread = total_frames // thread_num
        thread_list = []
        # 每个线程处理一部分的数据
        for t_idx in range(thread_num):
            start = t_idx * frames_for_thread
            if t_idx == thread_num - 1:
                end = total_frames
            else:
                end = (t_idx + 1) * frames_for_thread
            t = Thread(target=self.save_unit_data, args=(start, end, cur_save_path))
            t.start()   # 启动线程
            thread_list.append(t)

        # 等待所有线程完成，使用 thread.join() 让主线程在所有子线程完成之前阻塞
        for thread in thread_list:
            thread.join()

        # save Parking Goal
        measurements_file = cur_save_path / 'parking_goal' / '0001.json'
        with open(measurements_file, 'w') as f:
            data = {'x': parking_goal[0],
                    'y': parking_goal[1],
                    'yaw': parking_goal[2]}
            json.dump(data, f, indent=4)
        


        # save vehicle video
        self._world.save_video(cur_save_path)

        logging.info('saved sensor data for task %d in %s', self._task_index, str(cur_save_path))

    # 保存单个数据帧
    def save_unit_data(self, start, end, cur_save_path):
        for index in range(start, end):
            data_frame = self._batch_data_frames[index]

            # 传感器数据 save camera / lidar
            for sensor in data_frame.keys():
                if sensor.startswith('rgb'):
                    # _, image = self.image_process(self.target_parking_goal, cam_id=sensor, image=data_frame[sensor])
                    # image = Image.fromarray(image)
                    # image.save(str(cur_save_path / sensor / ('%04d.png' % index)))
                    data_frame[sensor].save_to_disk(
                        str(cur_save_path / sensor / ('%04d.png' % index)))
                elif sensor.startswith('depth'):
                    data_frame[sensor].save_to_disk(
                        str(cur_save_path / sensor / ('%04d.png' % index)))
                elif sensor.startswith('lidar'):
                    data_frame[sensor].save_to_disk(
                        str(cur_save_path / 'lidar' / ('%04d.ply' % index)))

            # 车辆相关信息 save measurements
            imu_data = data_frame['imu']
            gnss_data = data_frame['gnss']
            vehicle_transform = data_frame['veh_transfrom']
            vehicle_velocity = data_frame['veh_velocity']
            vehicle_control = data_frame['veh_control']


            data = {
                'x': vehicle_transform.location.x,
                'y': vehicle_transform.location.y,
                'z': vehicle_transform.location.z,
                'pitch': vehicle_transform.rotation.pitch,
                'yaw': vehicle_transform.rotation.yaw,
                'roll': vehicle_transform.rotation.roll,
                'speed': (3.6 * math.sqrt(vehicle_velocity.x ** 2 + vehicle_velocity.y ** 2 + vehicle_velocity.z ** 2)),  # km/h
                'speed_x': vehicle_velocity.x,
                'speed_y': vehicle_velocity.y,
                'Throttle': vehicle_control.throttle,
                'Steer': vehicle_control.steer,
                'Brake': vehicle_control.brake,
                'Reverse': vehicle_control.reverse,
                'Hand brake': vehicle_control.hand_brake,
                'Manual': vehicle_control.manual_gear_shift,
                'Gear': {-1: 'R', 0: 'N'}.get(vehicle_control.gear, vehicle_control.gear),
                'acc_x': imu_data.accelerometer.x,
                'acc_y': imu_data.accelerometer.y,
                'acc_z': imu_data.accelerometer.z,
                'gyr_x': imu_data.gyroscope.x,
                'gyr_y': imu_data.gyroscope.y,
                'gyr_z': imu_data.gyroscope.z,
                'compass': imu_data.compass,
                'lat': gnss_data.latitude,
                'lon': gnss_data.longitude
            }

            measurements_file = cur_save_path / 'measurements' / ('%04d.json' % index)
            with open(measurements_file, 'w') as f:
                json.dump(data, f, indent=4)
            try:
                ped_transform = data_frame['ped_transform']
                ped_velocity = data_frame['ped_velocity']
                data_pedestrain = {
                    'x': ped_transform.location.x,
                    'y': ped_transform.location.y,
                    'z': ped_transform.location.z,
                    'speed_x': ped_velocity.x,
                    'speed_y': ped_velocity.y
                }
            except:
                pass
            measurements_pedestrain_file = cur_save_path / 'measurements_pedestrain' / ('%04d.json' % index)
            with open(measurements_pedestrain_file, 'w') as f:
                try:
                    json.dump(data_pedestrain, f, indent=4)
                except:
                    pass

            # 保存顶部视图
            def save_img(image, keyword=""):
                img_save = np.moveaxis(image, 0, 2)
                save_path = str(cur_save_path / 'topdown' / ('encoded_%04d' % index + keyword + '.png'))
                cv2.imwrite(save_path, img_save)

            keyword = ""
            bev_view1 = self._world.render_BEV_from_state(data_frame['bev_state'])
            img1 = encode_npy_to_pil(np.asarray(bev_view1.squeeze().cpu()))
            save_img(img1, keyword)
