#!/bin/bash

# echo "Starting the first Docker container..."
# gnome-terminal -- bash -c "source ~/shared_dir/start.sh; exec bash"

# 启动 Docker 容器
echo "Starting the Docker container..."
gnome-terminal -- bash -c "cd ~/vehicleToEverything && bash docker_run_test.sh; exec bash"

# 进入第一个 Docker 容器并启动相关服务
echo "进入容器，启动组合 LAVIDA 导航驱动......"
gnome-terminal -- bash -c "docker exec -it xiaoche-humble /bin/bash -c 'source ros2pkg/install/setup.bash; ros2 launch chcnav demo_1.py; exec bash'"
# gnome-terminal -- bash -c "docker exec -it xiaoche /bin/bash -c 'source ros2pkg/install/setup.bash && ros2 bag play --l langyi_bag/langyi_bag2/'"

# 进入第二个 Docker 容器并启动 GPS 服务
echo "进入容器，并启动 GPS 服务......"
gnome-terminal -- bash -c "docker exec -it xiaoche-humble /bin/bash -c 'source pm_v2x/install/setup.bash; ros2 launch gps_to_xyz shiche.launch.py; exec bash'"


# 启动 grpc_bridge_gui
echo "Starting grpc_bridge_gui..."
# gnome-terminal -- bash -c "docker exec -it autoware-humble /bin/bash -c 'source /grpc/grpc_bridge_gui/install/setup.bash; ros2 run grpc_bridge_gui grpc_bridge_gui; exec bash'"
gnome-terminal -- bash -c "docker exec -it xiaoche-humble /bin/bash -c 'source grpc/grpc_bridge_gui/install/setup.bash && ros2 run grpc_bridge_gui grpc_bridge_gui; exec bash'"

# 启动 GUI 客户端
echo "Starting GUI client..."
gnome-terminal -- bash -c "/usr/local/gui-obu-client/bin/gui-obu-client.sh; exec bash"

