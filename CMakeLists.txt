# ============================================================================
# desktop_arm_description / CMakeLists.txt
# 第6组 · 桌面小管家 · 课程代码 (2026-2027-1)-S0610350-04
#
# 【文件性质】新增文件。原始工程包确认为 catkin 功能包，但 CMakeLists.txt 未随
# 交付材料提供；本文件按 ROS1 catkin 规范补写，作用是：
#   1) 让 catkin_make 能识别本包（否则 $(find desktop_arm_description) 找不到）；
#   2) 把 scripts/ 下两个 Python 节点 install 成可执行；
#   3) 把 urdf/ launch/ rviz/ meshes/ 安装到 share 目录，devel 空间下即可 roslaunch。
# 本包不含 C++ 代码，因此没有 add_executable / target_link_libraries。
# ============================================================================
cmake_minimum_required(VERSION 3.0.2)
project(desktop_arm_description)

find_package(catkin REQUIRED)

# 纯资源包：声明依赖即可，不导出 C++ 头文件/库
catkin_package()

# 算法节点（见 launch/stack.launch）。脚本首行需为 #!/usr/bin/env python3
catkin_install_python(PROGRAMS
  scripts/color_sort_node.py
  scripts/gesture_tcp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 模型、启动文件、RViz 配置与源码一并安装到包 share 目录
install(DIRECTORY urdf launch rviz src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        FILES_MATCHING
          PATTERN "*.urdf"
          PATTERN "*.STL"
          PATTERN "*.launch"
          PATTERN "*.rviz"
          PATTERN "*.py"
          PATTERN "*.md"
          PATTERN "*.ipynb"
)

# 配置文件说明（标定产物 T_arm2ws.txt / object.yaml / z_error_polyfit.txt 需自行生成）
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL
)
