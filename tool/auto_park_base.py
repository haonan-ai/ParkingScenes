'''
auto_park_base.py
基本功能
'''

import carla
import logging
from data_generation import parking_position
import pygame
import math

class Auto_Park(object):
    def __init__(self, world):
        self._world = world
        if isinstance(self._world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

    def main(self, client, world, clock):
        if self._world.need_init_ego_state:
            self._control = carla.VehicleControl()
            self._world.need_init_ego_state = False
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
        if isinstance(self._control, carla.VehicleControl):

            self._parking_goal_index = 17  # 2-2; index 15+2=17
            self._parking_goal = parking_position.parking_vehicle_locations_Town04[self._parking_goal_index]

            t = world.player.get_transform()
            v = world.player.get_velocity()
            c = world.player.get_control()
            acc = world.player.get_acceleration()

            #停车位坐标
            self._parking_goal.x
            self._parking_goal.y
            #自车坐标
            t.location.x
            t.location.y
            #自车航向角
            t.rotation.yaw
            #位置偏差
            world.distance_diff_to_goal
            #角度偏差
            world.rotation_diff_to_goal
            #车速
            speed= math.sqrt(v.x**2 + v.y**2 + v.z**2)
            #加速度
            acc.x
            acc.y
            acc.z
            #油门 （0，1）
            self._control.throttle
            #刹车
            self._control.brake
            #方向盘 （-0.7，0.7）
            self._control.steer
            #是否倒车 True：倒车 False：前进
            self._control.reverse
            #是否停车
            self._control.hand_brake
            #控制信号应用到ego vehicle
            world.player.apply_control(self._control)
            #画出一个点
            self._world.world.debug.draw_point(self._parking_goal,life_time=0)
        # logging.info('parking_goal y: %.6f' % self._parking_goal.y)
