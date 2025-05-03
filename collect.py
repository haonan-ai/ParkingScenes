import argparse
import logging
import carla    # CARLA 仿真平台的核心 Python API
import pygame   # 用于渲染窗口和处理用户输入

from data_generation.data_generator import DataGenerator
from data_generation.keyboard_control import KeyboardControl
from data_generation.auto_park_1 import Auto_Park


def game_loop(args):
    # 初始化 Pygame
    pygame.init()
    pygame.font.init() # 初始化 Pygame 字体

    # 数据生成器
    data_generator = None

    try:
        client = carla.Client(args.host, args.port) # 连接到 CARLA 仿真服务器
        client.set_timeout(5.0)
        logging.info('Load Map %s', args.map)
        carla_world = client.load_world(args.map)   # 加载指定地图
        carla_world.unload_map_layer(carla.MapLayer.ParkedVehicles) # 移除地图中的静态车辆

        # 初始化数据生成器和键盘控制器
        data_generator = DataGenerator(carla_world, args)   # 数据生成器
        controller = Auto_Park(data_generator.world)  # 键盘控制器  

        # 创建 Pygame 显示窗口
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        # 创建 Pygame 时钟对象，用于限制帧率，保持渲染的流畅性
        clock = pygame.time.Clock()

        # 主循环
        while True:
            data_generator.world_tick() # 更新 CARLA 世界状态，包括车辆位置、速度等
            clock.tick_busy_loop(60)    # 限制帧率为 60 FPS
            # 键盘控制，操控车辆
            if controller.main(client, data_generator.world, clock,data_generator._parking_goal_index,args):
                return
            data_generator.tick(clock, args)  # 更新数据生成器状态
            data_generator.render(display)  # 将当前仿真场景渲染到 Pygame 显示窗口
            pygame.display.flip()       # 刷新显示窗口

    # 当程序异常退出时，关闭 Pygame 显示窗口
    finally:

        if data_generator:
            client.stop_recorder() # 停止数据记录

        if data_generator is not None:
            data_generator.destroy() # 销毁数据生成器，释放资源

        pygame.quit() # 退出 Pygame，释放资源


# 将字符串形式的布尔值转换为 Python 的布尔值
def str2bool(v):
    if v.lower() in ('yes', 'true', 'True', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'False', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Unsupported value encountered.')


def main():
    # 参数解析器实例，用于解析命令行参数
    argparser = argparse.ArgumentParser(
        description='CARLA Data Generation')
    # 在运行脚本时通过添加 -v 或 --verbose 输出调试信息
    # 如果在运行脚本时使用了 -v/--verbose，则 args.debug 会被设置为 True，否则为 False
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    # CARLA 仿真服务器的 IP 地址
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1', # 默认为本地地址
        help='IP of the host server (default: 127.0.0.1)')
    # 与 CARLA 服务器的通信端口
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    # 窗口分辨率
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='860x480',
        help='window resolution (default: 860x480)')
    # Gamma 校正，用于调整图像亮度
    argparser.add_argument(
        '--gamma',
        default=0.0,
        type=float,
        help='Gamma correction of the camera (default: 0.0)')
    # 数据保存路径
    argparser.add_argument(
        '--save_path',
        default='./e2e_parking/',
        help='path to save sensor data (default: ./e2e_parking/)')
    # 是否随机化静态车辆
    argparser.add_argument(
        '--shuffle_veh',
        default=True,
        type=str2bool,
        help='shuffle static vehicles between tasks (default: True)')
    # 是否随机化天气条件
    argparser.add_argument(
        '--shuffle_weather',
        default=False,
        type=str2bool,
        help='shuffle weather between tasks (default: False)')
    # 随机种子
    argparser.add_argument(
        '--random_seed',
        default=0,
        help='random seed to initialize env; if sets to 0, use current timestamp as seed (default: 0)')
    # BEV 的渲染设备
    argparser.add_argument(
        '--bev_render_device',
        default='cpu',
        help='device used for BEV Rendering (default: cpu)',
        choices=['cpu', 'cuda'])
    # 是否放置行人
    argparser.add_argument(
        '--place_pedestrians',
        default=True,
        # default=False,
        type=str2bool,
        help='place pedestrians in the simulation (default: False)')
    # 地图选择
    argparser.add_argument(
        '--map',
        # default='Town04_Opt',
        default='Town10HD_Opt',
        help='map of carla (Town04_Opt:倒车入库, Town10HD_Opt:侧方停车)',
        # Town05_Opt 还没有加入
        choices=['Town04_Opt','Town10HD_Opt'])
    # 泊车任务数量
    argparser.add_argument(
        '--task_num',
        default=6,
        type=int,
        help='number of parking task (Town04_Opt max task_num:16,Town10HD_Opt max task_num:6)')
    
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    # 设置日志级别
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    # 调用主仿真循环，使用 try-except 语句捕获异常
    try:
        game_loop(args)

    # 当用户按 Ctrl+C 终止程序时，记录退出日志
    except KeyboardInterrupt:
        logging.info('Cancelled by user. Bye!')


if __name__ == '__main__':
    main()
