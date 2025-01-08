---
outline: deep
---

# ROS2 示例


### 编写发布和订阅包 (Python)

创建名为 `pubsub_py` 的包
```shell
cd demo
ros2 pkg create --build-type ament_python pubsub_py
```
得到如下内容
```shell
demo/pubsub_py
├── package.xml
├── pubsub_py
│   ├── __init__.py
│   ├── publisher.py    # 创建该文件，发布者
│   └── subscriber.py   # 创建该文件，订阅者
├── resource
│   └── pubsub_py
├── setup.cfg
├── setup.py
└── test  #   如果不需要测试，可以删除
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

仍然在你的工作空间的根，建立你的新的包:

```shell
colcon build --packages-select pubsub_py
```

打开一个终端激活环境，并运行订阅者
```bash
. install/setup.bash
ros2 run pubsub_py subscriber
```

打开另外一个新终端激活环境，并运行发布者
```bash
. install/setup.bash
ros2 run pubsub_py publisher
```

### 编写发布和订阅包 (C++)

创建名为 `pubsub` 的包，使用 `ament_cmake` 构建类型
```shell
cd src
ros2 pkg create --build-type ament_cmake pubsub
```

在包 `pubsub` 目录下添加 `app/publisher.cpp` 和 `app/subscriber.cpp` 文件，以及 `CMakeLists.txt` 文件，添加包依赖
```cmake
cmake_minimum_required(VERSION 3.8)
project(pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # 添加依赖

# [Node] publisher : 编译 链接 ROS 安装
add_executable(publisher # 可执行文件名，也就是节点(运行的文件)名
  app/publisher.cpp # 源文件
)
ament_target_dependencies(publisher
  rclcpp    # 添加 ROS 包: rclcpp
  std_msgs  # 添加 ROS 包: std_msgs
)
install(TARGETS     publisher           # 安装的可执行文件
        DESTINATION lib/${PROJECT_NAME} # 安装的位置
)

# [Node] subscriber : 编译 链接 ROS 安装
add_executable(subscriber app/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)
install(TARGETS subscriber DESTINATION lib/${PROJECT_NAME})

ament_package()
```

> ![TIP]
> 可以看到 `CMakeLists.txt` 中，将可执行文件安装到 `lib/${PROJECT_NAME}` 目录下，一般来说，二进制文件会安装到 `bin` 目录下，ROS 中的习惯貌似是安装到 `lib` 目录下，这里仍然遵循这个习惯。

编译
```bash
colcon build --packages-select pubsub
```

打开一个终端激活环境，并运行订阅者
```bash
. install/setup.bash
ros2 run pubsub subscriber
```

打开另外一个新终端激活环境，并运行发布者
```bash
. install/setup.bash
ros2 run pubsub publisher
```

### 使用 launch 文件启动节点

> 参考 [_Integrating launch files into ROS 2 packages_](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-system.html)

launch 文件能组织节点的启动顺序，启动多个节点，设置节点参数等。
```bash
cd demo
ros2 pkg create --build-type ament_cmake py_launch_demo
```

在 `py_launch_demo` 包下创建 `launch` 目录，添加 `_launch.py` 文件

```python
import launch
import launch_ros.actions
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="pubsub_py",
            executable="subscriber",
            name="demo_subscriber",
        ),
        launch_ros.actions.Node(
            package="pubsub_py",
            executable="publisher",
            name="demo_publisher",
        ),
    ])
```

然后在 `setup.py` 中添加依赖
```python
import os
from glob import glob
# Other imports ...
package_name = 'py_launch_demo'
setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ]
)
```

编译启动
```bash
colcon build --packages-select py_launch_demo
. install/setup.bash
ros2 launch py_launch_demo _launch.py
```