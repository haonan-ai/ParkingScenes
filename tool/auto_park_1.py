'''
auto_park_1
基本功能
'''
import os,sys
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
import carla
from tool import parking_position
import pygame
import math
import csv
from tool.AutomatedValetParking.plan import plan
from tool.MotionPlanning.Control.MPC import Motion
import numpy as np


class Auto_Park(object):
    def __init__(self, world):
        self._world = world
        if isinstance(self._world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        self._planning= True
        self._coordinate_rotation=False
        self._motion=None
        self._walker_mode=0
    def read_specific_rows_from_csv(self,file_path):
        x_coords = []
        y_coords = []
        
        with open(file_path, mode='r', newline='') as csv_file:
            csv_reader = csv.reader(csv_file,delimiter='\t')
            for i, row in enumerate(csv_reader):
                if i % self.step == 0:
                    try:
                        x = float(row[1])  # 第2个元素（索引为1）
                        y = float(row[2])  # 第3个元素（索引为2）
                        x_coords.append(x)
                        y_coords.append(y)
                    except (IndexError, ValueError):
                        print(f"Skipping row {i} due to missing or invalid data.")
        return x_coords, y_coords
    def rotate_vehicle_boxes(self,vehicle_boxes):
        """
        将 vehicle_boxes 中每个边界的坐标（格式：(x1,y1,x2,y2,x3,y3,x4,y4)）绕原点顺时针旋转90度，
        旋转公式： (x, y) -> (y, -x)
        
        参数：
            vehicle_boxes: list of tuples，每个 tuple 包含 8 个数字，依次为四个顶点的 (x, y) 坐标
            
        返回：
            rotated_boxes: list of tuples，每个 tuple 为旋转后的坐标，格式同上
        """
        rotated_boxes = []
        for box in vehicle_boxes:
            rotated_box = [
                -box[1], box[0],  # 旋转前第一个点 (x1, y1) -> (y1, -x1)
                -box[3], box[2],  # 旋转前第二个点 (x2, y2) -> (y2, -x2)
                -box[5], box[4],  # 旋转前第三个点 (x3, y3) -> (y3, -x3)
                -box[7], box[6]   # 旋转前第四个点 (x4, y4) -> (y4, -x4)
            ]
            rotated_boxes.append(rotated_box)
        return rotated_boxes
    
    def get_wall_vertices(self,x, y, yaw, length, width, thickness):
        """
        根据停车场中心 (x,y)，朝向 yaw（出口方向），
        长度 length，宽度 width，墙厚 thickness，
        计算三个墙面（左、后、右）的四个顶点，返回一个列表：
        [
            左侧墙顶点,  # 顺时针或逆时针排列
            后侧墙顶点,
            右侧墙顶点
        ]
        局部坐标系定义：
        - 原点为停车场中心
        - v 轴指向停车场后侧（与出口相反），出口在 v = -length/2
        - u 轴指向右侧，左侧为 u = -width/2，右侧为 u = width/2
        转换公式：先计算局部坐标，再通过旋转和平移到全局，
        旋转角 theta = radians(yaw + 90) 保证局部负 v 轴与全局出口方向一致。
        """
        # 半长与半宽
        half_length = length / 2.0
        half_width = width / 2.0
        # 计算旋转角（单位：弧度）
        theta = math.radians(yaw + 90)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        out=[]
        
        # 坐标变换函数：局部 (u, v) -> 全局 (X, Y)
        def transform(u, v):
            X = x + u * cos_t - v * sin_t
            Y = y + u * sin_t + v * cos_t
            return [X, Y]
        
        # 左侧墙：依附于 u = -half_width，向左延伸 thickness（即 u 从 -half_width 到 -half_width - thickness）
        left_wall = [
            transform(-half_width, -half_length),                     # 内边界下端
            transform(-half_width - thickness, -half_length),           # 外边界下端
            transform(-half_width - thickness, half_length),            # 外边界上端
            transform(-half_width, half_length)                         # 内边界上端
        ]
        
        # 后侧（顶侧）墙：依附于 v = half_length，向后延伸 thickness（即 v 从 half_length 到 half_length + thickness）
        top_wall = [
            transform(-half_width, half_length),                        # 内边界左端
            transform(-half_width, half_length + thickness),            # 外边界左端
            transform(half_width, half_length + thickness),             # 外边界右端
            transform(half_width, half_length)                          # 内边界右端
        ]
        
        # 右侧墙：依附于 u = half_width，向右延伸 thickness（即 u 从 half_width 到 half_width + thickness）
        right_wall = [
            transform(half_width, half_length),                         # 内边界上端
            transform(half_width + thickness, half_length),             # 外边界上端
            transform(half_width + thickness, -half_length),            # 外边界下端
            transform(half_width, -half_length)                         # 内边界下端
        ]
        for x,y in left_wall:
            out.append(x)
            out.append(y)
            # location=carla.Location(x=x,y=y,z=0.3)
            # self._world.world.debug.draw_point(location,life_time=0,size=0.05)
        for x,y in top_wall:
            out.append(x)
            out.append(y)
            # location=carla.Location(x=x,y=y,z=0.3)
        #     self._world.world.debug.draw_point(location,life_time=0,size=0.05)
        for x,y in right_wall:
            out.append(x)
            out.append(y)
            # location=carla.Location(x=x,y=y,z=0.3)
            # self._world.world.debug.draw_point(location,life_time=0,size=0.05)
        return out

    def rotate_walls(self,walls):
        rotated_box = [
            -walls[1], walls[0],  # 旋转前第一个点 (x1, y1) -> (y1, -x1)
            -walls[3], walls[2],  # 旋转前第二个点 (x2, y2) -> (y2, -x2)
            -walls[5], walls[4],  # 旋转前第三个点 (x3, y3) -> (y3, -x3)
            -walls[7], walls[6],   # 旋转前第四个点 (x4, y4) -> (y4, -x4)
            -walls[9], walls[8],
            -walls[11], walls[10],
            -walls[13], walls[12],
            -walls[15], walls[14],
            -walls[17], walls[16],
            -walls[19], walls[18],
            -walls[21], walls[20],
            -walls[23], walls[22]
            ]
        return rotated_box
    
    def pi_2_pi(self,angle):
        while((angle > math.pi) or (angle < -math.pi)):
            if angle > math.pi:
                angle -= 2.0 * math.pi
            elif angle < -math.pi:
                angle += 2.0 * math.pi
        return angle

    def calculate_rear_wheel_center(self,x, y, yaw, wheelbase):
        """
        计算汽车后轮中心的坐标

        参数:
        x (float): 车辆中心的x坐标
        y (float): 车辆中心的y坐标
        yaw (float): 车辆的航向角（弧度）
        wheelbase (float): 车辆的轴距

        返回:
        tuple: 后轮中心的坐标 (rear_x, rear_y)
        """
        # 计算后轮中心相对于车辆中心的偏移量
        yaw=self.pi_2_pi(yaw)
        dx = -wheelbase * math.cos(yaw)
        dy = -wheelbase * math.sin(yaw)

        # 计算后轮中心的坐标
        rear_x = x + dx
        rear_y = y + dy

        return rear_x, rear_y

    def perception(self,world,t,args):
        if args.map=='Town04_Opt':
            self._parking_goal = parking_position.parking_vehicle_locations_Town04[self._parking_goal_index]
            if int(self._parking_goal_index-1)//16%2==0:
                goal_yaw=0
            else:
                goal_yaw=180
            walls=self.get_wall_vertices(self._parking_goal.x, self._parking_goal.y,goal_yaw, 4, 2.8, 0.1)            
        elif args.map=='Town10HD_Opt':
            self._parking_goal = parking_position.parking_vehicle_locations_Town10[self._parking_goal_index]
            goal_yaw=t.rotation.yaw
        move_x, move_y = self.calculate_rear_wheel_center(self._parking_goal.x, self._parking_goal.y,\
                                            math.radians(goal_yaw),self.move_back)        
        vehicle_boxes = []
        vehicles = world.world.get_actors().filter('vehicle.*')

        for vehicle in vehicles:
            if vehicle.id == world.player.id:
                continue
            start_distance = vehicle.get_location().distance(t.location)
            end_distance = vehicle.get_location().distance(self._parking_goal)
            if (start_distance < 20.0)or (end_distance < 20.0):
                bb = vehicle.bounding_box       # 本地边界框
                trans = vehicle.get_transform() # Actor 的全局变换
                front_left  = carla.Location(x=bb.location.x + bb.extent.x, y=bb.location.y - bb.extent.y, z=2)
                front_right = carla.Location(x=bb.location.x + bb.extent.x, y=bb.location.y + bb.extent.y, z=2)
                rear_right  = carla.Location(x=bb.location.x - bb.extent.x, y=bb.location.y + bb.extent.y, z=2)
                rear_left   = carla.Location(x=bb.location.x - bb.extent.x, y=bb.location.y - bb.extent.y, z=2)
                
                # 将本地顶点转换为世界坐标（忽略 z 方向信息）
                world_front_left  = trans.transform(front_left)
                world_front_right = trans.transform(front_right)
                world_rear_right  = trans.transform(rear_right)
                world_rear_left   = trans.transform(rear_left)
                
                # 存储 2D 坐标 (x,y) 到一个元组中，按照顺时针顺序
                box = (
                    world_front_left.x,  world_front_left.y,
                    world_front_right.x, world_front_right.y,
                    world_rear_right.x,  world_rear_right.y,
                    world_rear_left.x,   world_rear_left.y
                )
                vehicle_boxes.append(box)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file_path=os.path.join(current_dir, 'AutomatedValetParking/BenchmarkCases/CARLA.csv')
        os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
        with open(csv_file_path, 'w', newline='') as csv_file:
            row_goal_x,row_goal_y=self.calculate_rear_wheel_center(self._parking_goal.x, self._parking_goal.y,\
                                                                   t.rotation.yaw,self.move_back)
            if (t.rotation.yaw>80 and t.rotation.yaw<100) or (t.rotation.yaw<-80 and t.rotation.yaw>-100):
                self._coordinate_rotation=True
                start_x=-t.location.y
                start_y=t.location.x
                start_yaw=t.rotation.yaw+90
                goal_x=-move_y
                goal_y=move_x
                if int(self._parking_goal_index-1)//16%2==0:
                    goal_yaw=90
                else:
                    goal_yaw=180+90
                # goal_yaw=goal_yaw+90
            else:
                self._coordinate_rotation=False
                start_x=t.location.x
                start_y=t.location.y
                start_yaw=t.rotation.yaw
                goal_x=move_x
                goal_y=move_y
            csv_writer = csv.writer(csv_file)
            data=[]
            data.extend([start_x, start_y,self.pi_2_pi(math.radians(start_yaw)), goal_x, goal_y, self.pi_2_pi(math.radians(goal_yaw))])
            vehicle_num=len(vehicle_boxes)
            if vehicle_num > 0:
                if args.map=='Town04_Opt':
                    data.extend([vehicle_num+3])
                    for i in range(vehicle_num+3):
                        data.extend([4])                    
                else:
                    data.extend([vehicle_num])
                    for i in range(vehicle_num):
                        data.extend([4])
                if self._coordinate_rotation:
                    vehicle_boxes = self.rotate_vehicle_boxes(vehicle_boxes)
                    if args.map=='Town04_Opt':
                        walls=self.rotate_walls(walls)
                for box in vehicle_boxes:
                    box=list(box)
                    data.extend(box)
            else:
                if args.map=='Town04_Opt':
                    data.extend([3,4,4,4])
                else:
                    data.extend([0])
            if args.map=='Town04_Opt':  
                data.extend(walls)
            csv_writer.writerows([data])
        pass

    def plot_traj(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file_path=os.path.join(current_dir, 'AutomatedValetParking/solution/CARLA.csv')
        x_coords, y_coords = self.read_specific_rows_from_csv(csv_file_path)
        for i in range(len(x_coords)):
            if i%3==0:
                location = carla.Location(x=x_coords[i], y=y_coords[i], z=0.2)
                self._world.world.debug.draw_point(location,life_time=0,size=0.05,color=carla.Color(255, 255, 0))
            # location = carla.Location(x=x_coords[i], y=y_coords[i], z=0.2)
            # self._world.world.debug.draw_point(location,life_time=0,size=0.05,color=carla.Color(255, 255, 0))
    def need_to_stop(self, world, t, args):
        if args.map == "Town10HD_Opt":
            # 侧方停车任务中，行人检测不需要考虑
            return False
        # 初始化 _stop_state 属性（如果尚未定义）
        if not hasattr(self, '_stop_state'):
            self._stop_state = False

        if not args.place_pedestrians:
            self._stop_state = False
            # print("place_pedestrians is False. No pedestrian detection.")
            return False

        yaw = t.rotation.yaw
        vehicle_x = t.location.x
        vehicle_y = t.location.y
        direct = self._motion.node.direct  # 1: 前进, -1: 倒车

        # 只考虑第一个行人
        walkers = world.world.get_actors().filter('walker.pedestrian.*')
        if not walkers:
            self._stop_state = False
            # print("No pedestrians detected.")
            return False
        walker = walkers[0]
        pedestrian_location = walker.get_location()
        walker_x = pedestrian_location.x
        walker_y = pedestrian_location.y

        # 车辆尺寸参数（单位：米）
        VEHICLE_LENGTH = 4.0
        VEHICLE_WIDTH = 2.0
        half_length = VEHICLE_LENGTH / 2.0  # 2.0
        half_width = VEHICLE_WIDTH / 2.0    # 1.0

        # 安全边界参数（单位：米）
        safe_margin = 0.5

        # 根据运动方向调整有效朝向：倒车时加 180°
        effective_yaw = yaw if direct == 1 else yaw + 180
        theta = math.radians(effective_yaw)

        # 将行人位置转换到车辆运动坐标系（x轴：车辆运动方向，正值表示前方）
        dx = walker_x - vehicle_x
        dy = walker_y - vehicle_y
        x_eff = dx * math.cos(theta) + dy * math.sin(theta)
        y_eff = -dx * math.sin(theta) + dy * math.cos(theta)

        # 打印调试信息
        # print("Vehicle position: ({:.2f}, {:.2f})".format(vehicle_x, vehicle_y))
        # print("Vehicle yaw: {:.2f}, effective_yaw: {:.2f}, theta (rad): {:.2f}".format(yaw, effective_yaw, theta))
        # print("Walker position: ({:.2f}, {:.2f})".format(walker_x, walker_y))
        # print("Relative (dx, dy): ({:.2f}, {:.2f})".format(dx, dy))
        # print("Transformed (x_eff, y_eff): ({:.2f}, {:.2f})".format(x_eff, y_eff))

        if direct == 1:
            # 前进状态下的停止与恢复阈值
            forward_stop_distance = half_length + safe_margin       # 2.0 + 0.5 = 2.5
            forward_side_stop = half_width + safe_margin              # 1.0 + 0.5 = 1.5
            forward_resume_distance = forward_stop_distance + 1.0      # 3.5
            forward_side_resume = forward_side_stop + 0.5              # 2.0

            if not self._stop_state:
                # 未处于停车状态：只要行人进入“停止区域”就立即停车
                if x_eff <= forward_stop_distance and abs(y_eff) <= forward_side_stop:
                    self._stop_state = True
                    # print("Pedestrian in stop zone (forward). Stopping vehicle.")
                    return True
                else:
                    self._stop_state = False
                    # print("Pedestrian not in stop zone (forward). Vehicle can move.")
                    return False
            else:
                # 已经停车：只有当行人完全离开“恢复区域”后才允许车辆继续行驶
                if x_eff > forward_resume_distance or abs(y_eff) > forward_side_resume:
                    self._stop_state = False
                    # print("Pedestrian cleared resume zone (forward). Vehicle can move.")
                    return False
                else:
                    self._stop_state = True
                    # print("Pedestrian still in resume zone (forward). Vehicle remains stopped.")
                    return True
        else:
            # 倒车状态下：首先判断行人是否在倒车方向上
            # 当车辆倒车时，正的 x_eff 表示倒车方向（危险区域），负的 x_eff 表示车辆前部区域（安全区域）
            if x_eff < 0:
                self._stop_state = False
                # print("Pedestrian is in front of the vehicle (reverse mode). Vehicle can move.")
                return False

            # 设置倒车的停止与恢复阈值
            reverse_stop_distance = 12.0       # 停止阈值
            reverse_side_stop = half_width + safe_margin  # 1.0 + 0.5 = 1.5
            reverse_resume_distance = 14.0       # 恢复阈值
            reverse_side_resume = half_width + safe_margin + 0.5  # 1.0 + 0.5 + 0.5 = 2.0

            if not self._stop_state:
                # 未处于停车状态：行人进入倒车方向的停止区域则停车
                if x_eff <= reverse_stop_distance and abs(y_eff) <= reverse_side_stop:
                    self._stop_state = True
                    # print("Pedestrian in stop zone (reverse). Stopping vehicle.")
                    return True
                else:
                    self._stop_state = False
                    # print("Pedestrian not in stop zone (reverse). Vehicle can move.")
                    return False
            else:
                # 已经停车：只有当行人完全脱离倒车方向的恢复区域后，车辆才能继续运动
                if x_eff > reverse_resume_distance or abs(y_eff) > reverse_side_resume:
                    self._stop_state = False
                    # print("Pedestrian cleared resume zone (reverse). Vehicle can move.")
                    return False
                else:
                    self._stop_state = True
                    # print("Pedestrian still in resume zone (reverse). Vehicle remains stopped.")
                    return True

    def main(self, client, world, clock,index,args):
        self.step=1
        self._parking_goal_index=index
        self.move_back=1.4
        k_brake=0.1
        k_steer=1
        k_throttle=0.4
        throttle_min=0
        if self._world.need_init_ego_state:
            self._control = carla.VehicleControl()
            self._world.need_init_ego_state = False
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
        if isinstance(self._control, carla.VehicleControl):
            t = world.player.get_transform()
            v = world.player.get_velocity()
            c = world.player.get_control()
            acc = world.player.get_acceleration()
            t.location.x,t.location.y=self.calculate_rear_wheel_center(t.location.x, t.location.y,\
                                        math.radians(t.rotation.yaw), self.move_back)            
            if world.is_restart==True:
                self._planning=True
                self._walker_mode=0
                try:
                    self._world.data.append(self._motion.data)
                except:
                    pass
                world.is_restart=False
            if self._planning:
                self.perception(world,t,args)
                plan(self._coordinate_rotation,world.index)
                # self.plot_traj()
                self._motion=Motion()
                self._planning = False
            else:
                if self.need_to_stop(world,t,args):
                # if False:
                    self._control.brake=1
                    self._control.throttle=0
                    self._control.steer=0
                    self._control.hand_brake=True
                else:
                    speed= math.sqrt(v.x**2 + v.y**2)*self._motion.node.direct
                    throttle, steer,hand_brake,world.over=self._motion.control(t.location.x,t.location.y,\
                                                            math.radians(t.rotation.yaw),speed,args)
                    if hand_brake:
                        self._control.hand_brake=True
                    else:
                        self._control.hand_brake=False
                        self._control.steer=steer*k_steer
                        self._control.gear = 1
                        if self._motion.node.direct==1:
                            self._control.reverse=False
                            if throttle>0:
                                self._control.throttle=throttle*k_throttle+throttle_min
                                self._control.brake=0.0
                            else:
                                self._control.brake=-throttle*k_brake
                                self._control.throttle=0.0
                        else: 
                            self._control.reverse=True
                            if throttle>0:
                                self._control.brake=throttle*k_brake
                                self._control.throttle=0.0
                            else:
                                self._control.throttle=-throttle*k_throttle+throttle_min
                                self._control.brake=0.0
                        if self._control.steer>1:
                            self._control.steer=1
                        elif self._control.steer<-1:
                            self._control.steer=-1
                        if self._control.throttle>1:
                            self._control.throttle=1
                        elif self._control.throttle<-1:
                            self._control.throttle=-1
                if c.gear==0:
                    self._control.gear=1
                    self._control.reverse=True
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_control(self._control)
                
                