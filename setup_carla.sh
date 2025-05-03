#!/usr/bin/env bash
set -e

# 1. 创建并切换到 carla 目录
mkdir -p carla
cd carla

# 2. 下载并解压主程序包
echo "==> Downloading CARLA 0.9.14..."
#wget -q https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.14.tar.gz
echo "==> Extracting CARLA..."
tar -xf CARLA_0.9.14.tar.gz
rm CARLA_0.9.14.tar.gz

# 3. 下载额外地图包到 Import/ 下
echo "==> Downloading AdditionalMaps 0.9.14..."
#wget -q -O Import/AdditionalMaps_0.9.14.tar.gz https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/AdditionalMaps_0.9.14.tar.gz

# 4. 运行 ImportAssets.sh，将所有 Import/ 下的 .tar.gz 解包
echo "==> Importing all assets (including AdditionalMaps)..."
chmod +x ImportAssets.sh
./ImportAssets.sh

# 5. 安装 Python API
echo "==> Installing Python API..."
cd PythonAPI/carla/dist/
unzip -q carla-0.9.14-py3.7-linux-x86_64.egg \
      -d carla-0.9.14-py3.7-linux-x86_64
cd carla-0.9.14-py3.7-linux-x86_64/
cat > setup.py <<EOF
from distutils.core import setup
setup(name='carla', version='0.9.14', py_modules=['carla'],)
EOF
cd ..
pip install --user -e carla-0.9.14-py3.7-linux-x86_64

echo "==> CARLA 0.9.14 + AdditionalMaps installed successfully!"
