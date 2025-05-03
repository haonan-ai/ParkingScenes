import carla
 
client = carla.Client('localhost', 2000)
carla_world = client.get_world()
 
#获取CARLA世界中的spectator
spectator = carla_world.get_spectator()
 
# 获取观察者的当前变换（包括位置、旋转等）
transform = spectator.get_transform()
print(transform)