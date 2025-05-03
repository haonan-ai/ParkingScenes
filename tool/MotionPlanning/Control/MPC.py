import csv
import os
import sys
import math
import cvxpy
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../MotionPlanning/")

import Control.draw as draw
import CurvesGenerator.reeds_shepp as rs
import CurvesGenerator.cubic_spline as cs


class P:
    # System config
    NX = 4  # state vector: z = [x, y, v, phi]
    NU = 2  # input vector: u = [acceleration, steer]
    T = 6  # finite time horizon length

    # MPC config
    Q = np.diag([2.0, 2.2, 1.0, 0])  # penalty for states
    Qf = np.diag([2.0, 2.2, 1.0, 0])  # penalty for end state
    R = np.diag([0.01, 0.01])  # penalty for inputs
    Rd = np.diag([0.01, 0.01])  # penalty for change of inputs

    dist_stop = 0.05  # stop permitted when dist to goal < dist_stop
    speed_stop = 0.01  # stop permitted when speed < speed_stop
    time_max = 500.0  # max simulation time
    iter_max = 5  # max iteration
    target_speed = 0.8  # target speed
    N_IND = 10  # search index number
    dt = 0.2  # time step
    d_dist = 1.0  # dist step
    du_res = 0.1  # threshold for stopping iteration

    # vehicle config
    RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width

    steer_max = np.deg2rad(60.0)  # max steering angle [rad]
    steer_change_max = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    speed_max = 55.0 / 3.6  # maximum speed [m/s]
    speed_min = -20.0 / 3.6  # minimum speed [m/s]
    acceleration_max = 1.0  # maximum acceleration [m/s2]


class Node:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct
        self.current_part=0
        self.shift_index=[]

    def update(self,x,y,yaw,v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
    @staticmethod
    def limit_input_delta(delta):
        if delta >= P.steer_max:
            return P.steer_max

        if delta <= -P.steer_max:
            return -P.steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= P.speed_max:
            return P.speed_max

        if v <= P.speed_min:
            return P.speed_min

        return v


class PATH:
    def __init__(self, cx, cy, cyaw):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.length = len(cx)
        self.ind_old = 0

    def nearest_index(self, node):
        """
        calc index of the nearest node in N steps
        :param node: current information
        :return: nearest index, lateral distance to ref point
        """
        foward_ind = 0
        if (self.ind_old + P.N_IND)>node.shift_index[0][node.current_part]:
            foward_ind = node.shift_index[0][node.current_part]
        else:
            foward_ind = self.ind_old + P.N_IND
        # if node.current_part==1 and (self.ind_old + P.N_IND)>node.shift_index:
        #     foward_ind = node.shift_index+1
        # else:
        #     foward_ind = self.ind_old + P.N_IND

        dx = [node.x - x for x in self.cx[self.ind_old: foward_ind]]
        dy = [node.y - y for y in self.cy[self.ind_old: foward_ind]]
        dist = np.hypot(dx, dy)

        ind_in_N = int(np.argmin(dist))
        ind = self.ind_old + ind_in_N
        self.ind_old = ind

        rear_axle_vec_rot_90 = np.array([[math.cos(node.yaw + math.pi / 2.0)],
                                         [math.sin(node.yaw + math.pi / 2.0)]])

        vec_target_2_rear = np.array([[dx[ind_in_N]],
                                      [dy[ind_in_N]]])#报错了

        er = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)
        er = er[0][0]
        ind=min(node.shift_index[0][node.current_part],ind+2)
        return ind, er


def calc_ref_trajectory_in_T_step(node, ref_path, sp):
    """
    calc referent trajectory in T steps: [x, y, v, yaw]
    using the current velocity, calc the T points along the reference path
    :param node: current information
    :param ref_path: reference path: [x, y, yaw]
    :param sp: speed profile (designed speed strategy)
    :return: reference trajectory
    """

    z_ref = np.zeros((P.NX, P.T + 1))
    length = ref_path.length

    ind, _ = ref_path.nearest_index(node)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = sp[ind]
    z_ref[3, 0] = rs.pi_2_pi(ref_path.cyaw[ind])

    dist_move = 0.0

    for i in range(1, P.T + 1):
        dist_move += abs(node.v) * P.dt
        ind_move = int(round(dist_move / P.d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = sp[index]
        z_ref[3, i] = rs.pi_2_pi(ref_path.cyaw[index])

    return z_ref, ind


def linear_mpc_control(z_ref, z0, a_old, delta_old):
    """
    linear mpc controller
    :param z_ref: reference trajectory in T steps
    :param z0: initial state vector
    :param a_old: acceleration of T steps of last time
    :param delta_old: delta of T steps of last time
    :return: acceleration and delta strategy based on current information
    """

    if a_old is None or delta_old is None:
        a_old = [0.0] * P.T
        delta_old = [0.0] * P.T

    x, y, yaw, v = None, None, None, None

    for k in range(P.iter_max):
        z_bar = predict_states_in_T_step(z0, a_old, delta_old, z_ref)
        a_rec, delta_rec = a_old[:], delta_old[:]
        a_old, delta_old, x, y, yaw, v = solve_linear_mpc(z_ref, z_bar, z0, delta_old)

        du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
        du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])

        if max(du_a_max, du_d_max) < P.du_res:
            break

    return a_old, delta_old, x, y, yaw, v


def predict_states_in_T_step(z0, a, delta, z_ref):
    """
    given the current state, using the acceleration and delta strategy of last time,
    predict the states of vehicle in T steps.
    :param z0: initial state
    :param a: acceleration strategy of last time
    :param delta: delta strategy of last time
    :param z_ref: reference trajectory
    :return: predict states in T steps (z_bar, used for calc linear motion model)
    """

    z_bar = z_ref * 0.0

    for i in range(P.NX):
        z_bar[i, 0] = z0[i]

    node = Node(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3])

    for ai, di, i in zip(a, delta, range(1, P.T + 1)):
        z_bar[0, i] = node.x
        z_bar[1, i] = node.y
        z_bar[2, i] = node.v
        z_bar[3, i] = node.yaw

    return z_bar


def calc_linear_discrete_model(v, phi, delta):
    """
    calc linear and discrete time dynamic model.
    :param v: speed: v_bar
    :param phi: angle of vehicle: phi_bar
    :param delta: steering angle: delta_bar
    :return: A, B, C
    """

    A = np.array([[1.0, 0.0, P.dt * math.cos(phi), - P.dt * v * math.sin(phi)],
                  [0.0, 1.0, P.dt * math.sin(phi), P.dt * v * math.cos(phi)],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, P.dt * math.tan(delta) / P.WB, 1.0]])

    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [P.dt, 0.0],
                  [0.0, P.dt * v / (P.WB * math.cos(delta) ** 2)]])

    C = np.array([P.dt * v * math.sin(phi) * phi,
                  -P.dt * v * math.cos(phi) * phi,
                  0.0,
                  -P.dt * v * delta / (P.WB * math.cos(delta) ** 2)])

    return A, B, C


def solve_linear_mpc(z_ref, z_bar, z0, d_bar):
    """
    solve the quadratic optimization problem using cvxpy, solver: OSQP
    :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
    :param z_bar: predicted states in T steps
    :param z0: initial state
    :param d_bar: delta_bar
    :return: optimal acceleration and steering strategy
    """

    z = cvxpy.Variable((P.NX, P.T + 1))
    u = cvxpy.Variable((P.NU, P.T))

    cost = 0.0
    constrains = []

    for t in range(P.T):
        cost += cvxpy.quad_form(u[:, t], P.R)
        cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], P.Q)

        A, B, C = calc_linear_discrete_model(z_bar[2, t], z_bar[3, t], d_bar[t])

        constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]

        if t < P.T - 1:
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], P.Rd)
            constrains += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= P.steer_change_max * P.dt]

    cost += cvxpy.quad_form(z_ref[:, P.T] - z[:, P.T], P.Qf)

    constrains += [z[:, 0] == z0]
    constrains += [z[2, :] <= P.speed_max]
    constrains += [z[2, :] >= P.speed_min]
    constrains += [cvxpy.abs(u[0, :]) <= P.acceleration_max]
    constrains += [cvxpy.abs(u[1, :]) <= P.steer_max]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
    prob.solve(solver=cvxpy.OSQP)

    a, delta, x, y, yaw, v = None, None, None, None, None, None

    if prob.status == cvxpy.OPTIMAL or \
            prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = z.value[0, :]
        y = z.value[1, :]
        v = z.value[2, :]
        yaw = z.value[3, :]
        a = u.value[0, :]
        delta = u.value[1, :]
    else:
        print("Cannot solve linear mpc!")

    return a, delta, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    design appropriate speed strategy
    :param cx: x of reference path [m]
    :param cy: y of reference path [m]
    :param cyaw: yaw of reference path [m]
    :param target_speed: target speed [m/s]
    :return: speed profile
    """

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi

    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle

def find_shift_indices_and_signs(vs):
    """
    找出列表 vs 中所有换档点的下标以及对应区间的速度正负值。
    换档点定义为：当一个区间（由连续的有效速度正负确定）结束时，
    用该区间最后一个元素的下标作为换档点，最后一段的结束点为列表最后一个下标。
    每个区间的正负由该区间内第一个非 0 的速度决定（正值为 1，负值为 -1）。
    若某段全为 0，则默认延续前一段的状态；若全部为 0，则默认正向（1）。
    
    参数:
        vs: 一个列表，元素为速度（正值表示向前，负值表示后退，0视为延续前一段状态）
    
    返回:
        一个包含两个子列表的列表：
         - 第一个子列表为各区间（换档点及最后一个点）的下标
         - 第二个子列表为对应区间的速度正负（1 或 -1）
    
    例如:
        vs = [1.0, 2.0, 0.0, -1.0, -2.0, 0.0, 1.0]
        返回 [[2, 5, 6], [1, -1, 1]]
    """
    n = len(vs)
    if n == 0:
        return [[], []]
    
    # 辅助函数：返回非零值的符号
    def get_sign(x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

    # 确定第一段的有效方向：找第一个非零的元素；若全为零，默认正向（1）
    current_sign = None
    for v in vs:
        if v != 0:
            current_sign = get_sign(v)
            break
    if current_sign is None:
        current_sign = 1

    indices = []  # 用于存储换档点下标（各区间的结束点，下标）
    signs = []    # 存储对应区间的方向（1或-1）
    segment_start = 0  # 当前区间的起始索引

    # 遍历整个列表
    for i in range(n):
        # 当前元素若为 0，则视作延续当前区间的方向
        effective_sign = get_sign(vs[i]) if vs[i] != 0 else current_sign
        # 当遇到非0值且其符号与当前区间方向不同，则说明前一段结束
        if vs[i] != 0 and effective_sign != current_sign:
            # 记录前一区间的结束点（即 i-1）
            indices.append(i - 1)
            signs.append(current_sign)
            # 更新当前区间的起始点和方向
            segment_start = i
            current_sign = effective_sign
    # 最后记录最后一段的结束点（最后一个下标）及其方向
    indices.append(n - 1)
    signs.append(current_sign)
    
    return [indices, signs]

def read_specific_rows_from_csv(file_path, step=10):
    x_coords = []
    y_coords = []
    yaws = []
    vs=[]
    v_k=1.0
    v_b=1.5
    with open(file_path, mode='r', newline='') as csv_file:
        csv_reader = csv.reader(csv_file,delimiter='\t')
        shift_index=[]
        for i, row in enumerate(csv_reader):
            if i % step == 0:
                try:
                    num=int(i/step)
                    x = float(row[1])  # 第2个元素（索引为1）
                    y = float(row[2])  # 第3个元素（索引为2）
                    yaw = pi_2_pi(float(row[3]))  # 第4个元素（索引为3）
                    v=float(row[4])*v_k
                    if v>0:
                        v+=v_b
                    elif v<0:
                        v-=v_b
                    else:
                        if vs[-2] >0:
                            v=v_b
                        else:
                            v=-v_b
                    x_coords.append(x)
                    y_coords.append(y)
                    yaws.append(yaw)
                    vs.append(v)
                except (IndexError, ValueError):
                    print(f"Skipping row {i} due to missing or invalid data.")
    shift_index=find_shift_indices_and_signs(vs)
    return x_coords, y_coords,yaws,vs,shift_index

class Motion:
    def __init__(self):
        self.move_back=1.2
        here = os.path.dirname(os.path.abspath(__file__))
        data_gen_dir = os.path.abspath(os.path.join(here, os.pardir, os.pardir))
        csv_file_path = os.path.join(data_gen_dir,'AutomatedValetParking','solution','CARLA.csv')
        self.cx, self.cy, self.cyaw,self.sp, shift_index = \
            read_specific_rows_from_csv(csv_file_path, step=1)
        '''
        read_specific_rows_from_csv返回为空
        '''
        self.ref_path = PATH(self.cx, self.cy, self.cyaw)
        self.node = Node(x=self.cx[1], y=self.cy[1], yaw=self.cyaw[1], v=0.0)
        self.time = 0.0
        self.x = [self.node.x]
        self.y = [self.node.y]
        self.yaw = [self.node.yaw]
        self.v = [self.node.v]
        self.t = [0.0]
        self.d = [0.0]
        self.a = [0.0]
        self.delta_opt, self.a_opt = None, None
        self.a_exc, self.delta_exc = 0.0, 0.0
        self.node.shift_index=shift_index
        self.k_p=20.0
        self.data = [0,0]

    def control(self, x,y,yaw,v,args):
        over=False
        yaw=rs.pi_2_pi(yaw)
        hand_back = False
        self.node.update(x,y,yaw,v)
        z_ref, target_ind = calc_ref_trajectory_in_T_step(self.node, self.ref_path, self.sp)
        direct=self.node.shift_index[1][self.node.current_part]
        # if self.sp[self.node.shift_index[self.node.current_part]]<0:
        #     direct=-1
        # else:
        #     direct=1
        self.node.direct=direct
        z0 = [x,y,v,yaw]
        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = \
            linear_mpc_control(z_ref, z0, self.a_opt, self.delta_opt)
        if delta_opt is not None:
            delta_exc, a_exc = delta_opt[0], a_opt[0]
        self.x.append(self.node.x)
        self.y.append(self.node.y)
        self.yaw.append(self.node.yaw)
        self.v.append(self.node.v)
        self.d.append(delta_exc)
        self.a.append(a_exc)
        target_x=self.cx[self.node.shift_index[0][self.node.current_part]]
        target_y=self.cy[self.node.shift_index[0][self.node.current_part]]
        target_yaw=self.cyaw[self.node.shift_index[0][self.node.current_part]]
        dist=math.hypot(self.node.x - target_x, self.node.y - target_y)
        yaw_diff=pi_2_pi(self.node.yaw-target_yaw)
        vector_x = target_x - self.node.x
        vector_y = target_y - self.node.y
        target_vector_x = math.cos(target_yaw)
        target_vector_y = math.sin(target_yaw)
        dot_product = (vector_x * target_vector_x + vector_y * target_vector_y)*self.node.direct
        if ((dist < P.dist_stop) or (dot_product < 0)) and (target_ind==self.node.shift_index[0][self.node.current_part]):
            if self.node.current_part==len(self.node.shift_index[0])-1:
                if self.data==[0,0]:
                    self.data[0]=dist
                    self.data[1]=yaw_diff
                hand_back = True
                over=True
            else:
                self.ref_path.ind_old=self.node.shift_index[0][self.node.current_part] +1
                self.node.current_part += 1        
        if args.map=='Town04_Opt':
            if (self.node.current_part==len(self.node.shift_index[0])-1):
                if yaw<-2.9 or yaw>2.9:
                    delta_exc=-pi_2_pi(yaw-np.pi)*self.k_p*direct
                elif yaw<np.pi-2.9 and yaw>2.9-np.pi:
                    delta_exc=-pi_2_pi(yaw)*self.k_p*direct
        elif args.map=='Town10HD_Opt':
            if (self.node.current_part==len(self.node.shift_index[0])-1) and ((len(self.cx) - target_ind) < 30):
                if yaw<-2.9 or yaw>2.9:
                    delta_exc=-pi_2_pi(yaw-np.pi)*self.k_p*direct
        return a_exc, delta_exc,hand_back,over

if __name__ == '__main__':
    motion=Motion()
    x,y,yaw,v=motion.cx[0],motion.cy[0],motion.cyaw[0],motion.sp[0]
    a,d,hand_back=motion.control(x,y,yaw,v)
