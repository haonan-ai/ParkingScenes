import carla

def visualize_parking_locations(client, world):
    # 获取 CARLA 调试工具
    debug = world.debug

    # 定义停车位坐标
    parking_vehicle_locations_Town07 = [
        carla.Location(x=-76.0, y=-65.5, z=0.1)
    ]

    # 遍历所有停车点并在 CARLA 地图上可视化
    for loc in parking_vehicle_locations_Town07:
        debug.draw_string(loc, "T", draw_shadow=False,
                          color=carla.Color(255, 0, 0), life_time=20.0,
                          persistent_lines=True)
        # debug.draw_point(loc, size=0.3, color=carla.Color(0, 255, 0), life_time=10.0)

if __name__ == "__main__":
    client = carla.Client('localhost', 2000)  # 连接到 CARLA 服务器
    client.set_timeout(10.0)
    world = client.get_world()
    
    visualize_parking_locations(client, world)
