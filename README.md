## 内容

1. Nav2定位替换：可以选择Cartographer纯定位/amcl定位（无Cartographer）

## 安装步骤

1. 克隆本仓库源码

2. 使用鱼香ROS一键安装的国内版rosdepc安装依赖（到~/cartographer_ws下）：

wget http://fishros.com/install -O fishros && . fishros

rosdepc update

rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

3. 构建

colcon build --symlink-install