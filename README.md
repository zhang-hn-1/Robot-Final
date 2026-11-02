# "桌面小管家"——机械臂目标识别、自动分拣与手势搬运一体化系统

> **课程代码**：(2026-2027-1)-S0610350-04　|　**课程**：《机器人感知与控制综合实践》
> **组号**：第 6 组　|　**学期**：2026-2027 学年第一学期　|　**提交时间**：2026 年 11 月
> **开源仓库**：<https://github.com/zhang-hn-1/Robot-Final>
> **组员**：张航宁 24061733（组长，40%）、张浩轩 24061720（32%）、吴锐超 24061734（28%）

---

## 0. 一页速览：这个代码包给了你什么

| 交付内容 | 位置 | 性质（重要，请先看这一列） |
|---|---|---|
| URDF 模型（13 连杆 / 12 关节）+ 14 个 STL 网格 | `urdf/arm5dof.urdf`、`urdf/meshes/` | **复原版**：参数取自本组原始 URDF（PyBullet 版，见交付材料 `src/code/1.txt`）与第二章实验报告表2/表3；原工程为 SolidWorks 导出 |
| URDF 离线校验脚本 | `urdf/check_urdf.py` | **新增**（本次整理时编写），纯 Python，无需 ROS；运行输出贴在本包 `代码包自检.md` |
| launch 启动文件 3 份 | `launch/*.launch` | **复原版**：原始 launch 未随交付材料提供，按第二章报告第 5、7 节给出的片段复原 |
| RViz 配置 | `rviz/workcell.rviz` | **复原版**：按"RobotModel + TF + Grid + Image"的最小可用配置重写 |
| 机械臂 SDK / 应用层 | `src/arm_sdk/*.py` | **原始文件**（仅加文件头注释，正文一字未改，MD5 见自检文档） |
| 手势识别客户端与服务端 | `src/gesture/new.py`、`我的通讯识别.ipynb` | `new.py` 为**派生文件**（改写自 GPL-3.0 开源项目）；notebook 为**原始文件** |
| 颜色分拣 / 追踪 / 抓取实验 | `src/color_sort/`、`src/tracking/`、`src/lab/` | **原始文件**（notebook 原样保留，未改一个字节） |
| catkin 包清单与编译脚本 | `package.xml`、`CMakeLists.txt` | **新增**：原始包确认为 catkin 功能包，但这两份文件未随材料提供，按 ROS1 规范补写 |
| ROS 联调薄封装节点 2 个 | `scripts/color_sort_node.py`、`gesture_tcp_node.py` | **新增**：已过语法检查，**未在 ROS 环境逐帧回归** |
| 标定产物（3 个 txt/yaml） | `config/` | ❌ **未提供**，见 `config/README_配置文件说明.md`，需按说明重新标定 |

> **一句话结论**：模型、启动文件、标定配置属于"按报告参数与原始 URDF 复原"，SDK 与算法 notebook 属于"原样保留"。
> 本包**不声称**"开箱即跑"：缺 DICE 私有模块与标定文件时，真机链路需要先补齐（详见本文第 12 节"已知限制"与 `代码包自检.md`）。

---

## 1. 项目简介

"桌面小管家"是一台由 **DICE 五自由度串口舵机机械臂**（实验箱 6 号）+ **Jetson Nano** + **USB 摄像头**组成的桌面服务机器人。
它把课程内多组实验串成一条完整任务链：先在 SolidWorks/URDF 里建立机械臂模型并在 RViz / Gazebo 中验证，
再通过关节空间与笛卡尔空间的点到点运动完成抓取与放置，然后用 **HSV 颜色分割**识别工作台上的红/黄/绿/蓝四色物块并**自动分拣**到对应垃圾桶，
最后用 **MediaPipe 手势识别**把"抬手、摆手、握拳"翻译成机械臂的转向、抓取与停止指令，实现**非接触式搬运**。

一句话：**看得到（建模与可视化）→ 动得准（运动控制）→ 认得出（颜色/手势识别）→ 干得稳（状态机互锁与系统集成）**。

---

## 2. 功能清单

| # | 功能 | 实现载体（本包内文件） | 对应实验 / 报告章节 |
|---|---|---|---|
| 1 | **URDF 建模与 RViz 可视化**：13 连杆、12 关节、mimic 双指联动、TF 树、四方向核对 | `urdf/arm5dof.urdf`、`launch/display.launch`、`rviz/workcell.rviz` | 第二章 机械臂建模与可视化 |
| 2 | **Gazebo 物理加载**：同一份 URDF 注入物理引擎，`-J joint2 -1.5708` 预置静息位 | `launch/gazebo.launch` | 第二章 第 7 节 |
| 3 | **关节与点到点运动**：关节角下发、位姿到关节角、最小加加速度（六段式）轨迹 | `src/arm_sdk/arm5dof_uservo.py`、`arm_test.py`、`minimum_jerk.py` | 第一章、第三章 |
| 4 | **HSV 四色识别**：中心 ROI + 高斯加权 + 四色阈值判决 + 5 帧锁存 | `src/color_sort/我的垃圾分类.ipynb`、`scripts/color_sort_node.py` | 实验六 基于颜色识别的垃圾桶分拣 |
| 5 | **自动分拣（四色归位）**：颜色 → 垃圾桶映射 + 状态机 INIT/OBSERVE/ACTION/IDLE | `src/color_sort/我的垃圾分类.ipynb`、`src/lab/分发.ipynb` | 实验六、第三章 |
| 6 | **手势搬运**：MediaPipe 21 关键点 → 手指位掩码 → 5 类手势 → 机械臂动作 | `src/gesture/new.py`、`src/gesture/我的通讯识别.ipynb`、`scripts/gesture_tcp_node.py` | 实验七 手势识别与机械臂指令控制 |
| 7 | **状态机互锁与通信**：双级确认（5 帧手势确认 + TCP 侧 `catch_busy` 互锁）、`latest_cmd` 覆盖写 | `src/gesture/我的通讯识别.ipynb`、`src/gesture/new.py` | 实验七、报告第 4 章 |
| 8 | **物体追踪（扩展）**：HSV 双区间掩码 + 最大轮廓 + 误差比例控制 pan/tilt | `src/tracking/14.OpenCV摄像头机械臂物体追踪.ipynb` | 颜色追踪扩展实验 |

---

## 3. 代码包目录树

```text
第6组_张航宁_张浩轩_吴锐超_代码包/
├─ README.md                          ← 本文件（环境/编译/启动/分工/开源引用）
├─ 代码包自检.md                       ← 原始文件 vs 复原文件逐项清单 + URDF 校验输出
├─ package.xml                        ← catkin 功能包清单（新增，包名 desktop_arm_description）
├─ CMakeLists.txt                     ← catkin 编译脚本（新增，资源包，无 C++ 代码）
├─ urdf/
│  ├─ arm5dof.urdf                    ← 13 连杆 / 12 关节模型（复原版，含逐段中文注释）
│  ├─ check_urdf.py                   ← URDF 离线校验脚本（新增，纯 Python）
│  └─ meshes/                         ← 14 个 STL（11 个用于机械臂本体，3 个为场景道具）
├─ launch/
│  ├─ display.launch                  ← 模型 + 滑块 + RSP + RViz
│  ├─ stack.launch                    ← display.launch + 算法节点（联调入口）
│  └─ gazebo.launch                   ← empty_world + spawn_model
├─ rviz/
│  └─ workcell.rviz                   ← RobotModel / TF / Grid / Image 最小可用配置
├─ scripts/
│  ├─ color_sort_node.py              ← 颜色判决 ROS 薄封装（新增）
│  └─ gesture_tcp_node.py             ← 手势 TCP 服务端 ROS 薄封装（新增）
├─ src/
│  ├─ arm_sdk/
│  │  ├─ arm5dof_uservo.py            ← DICE SDK 真机驱动（原始）
│  │  ├─ config.py                    ← 关节/位姿/轨迹参数（原始，最终标定版）
│  │  ├─ arm_test.py                  ← 抓取/释放应用层（原始）
│  │  ├─ arm_test_origin.py           ← import 兼容层（新增，一行转发）
│  │  ├─ minimum_jerk.py              ← 最小加加速度轨迹（复原实现）
│  │  └─ transform.py                 ← 4×4 齐次变换（复原实现）
│  ├─ gesture/
│  │  ├─ new.py                       ← 手势 → 机械臂 TCP 客户端（派生文件）
│  │  └─ 我的通讯识别.ipynb            ← 手势服务端 + 执行线程 + 互锁（原始）
│  ├─ color_sort/
│  │  ├─ 我的垃圾分类.ipynb            ← HSV 四色识别 + 自动分拣状态机（原始）
│  │  └─ config.py                    ← 与 arm_sdk/config.py 同一份文件（原始）
│  ├─ tracking/
│  │  └─ 14.OpenCV摄像头机械臂物体追踪.ipynb   ← 颜色追踪（原始）
│  └─ lab/
│     ├─ 分发.ipynb                    ← 四方位抓取点动（原始）
│     └─ 07.机械臂抓取工作台物块测试.ipynb     ← 定点抓取测试（原始）
└─ config/
   └─ README_配置文件说明.md           ← 缺失的 3 个标定产物怎么重新生成（新增）
```

> 本目录的同级还会带一个 `第6组_张航宁_张浩轩_吴锐超_代码包.zip`，它就是上面这棵目录树的压缩包（40 个文件、约 3.1 MB，压缩后约 0.70 MB），内容与目录树完全一致。

**关于 `urdf/meshes/`**：原工程的网格放在功能包根目录 `meshes/`；本包按任务书要求的目录树放在 `urdf/meshes/`，
因此 URDF 中的引用写成 `package://desktop_arm_description/urdf/meshes/xxx.STL`。若你按原工程结构放置网格，
只需把 URI 里的 `urdf/` 去掉，其余不变。

---

## 4. 环境要求

### 4.1 机器人端（真机）

| 项 | 版本 / 参数 | 说明 |
|---|---|---|
| 主控 | **Jetson Nano**，Ubuntu **18.04** LTS | 实验箱 6 号配套 |
| 操作系统内核 / 串口 | `/dev/ttyTHS1`，**115200-8-N-1** | 6 路串口舵机（ID0~ID5） |
| ROS | **ROS1 Melodic**（Ubuntu 18.04）或 **Noetic**（Ubuntu 20.04） | 本包按 ROS1 编写；Noetic 下 `python3` 直接可用 |
| Python | **3.6+**（Jetson 18.04 自带 3.6.9） | `rospy`、`cv_bridge` 按 ROS 发行版安装 |
| 机械臂 | DICE 5DoF 串口舵机机械臂（6 舵机） | joint1 底座 / joint2 大臂 / joint3 肘 / joint4 腕摆 / joint5 腕回转 / 夹爪 |
| 相机 | USB 摄像头 **640×480@30fps**，索引 0 | 实验五装于末端，实验六固定俯视工作台 |
| 视觉库 | **OpenCV 4.x**、**NumPy**、**MediaPipe**（`mediapipe>=0.8`，Jetson 上建议用官方 aarch64 wheel 或源码编译） | 手势模块需要 `google.protobuf`（MediaPipe 依赖） |
| 其他 | **PyYAML**、**imutils**、**Jupyter Notebook**、**ipywidgets**（notebook 里用了 `widgets.Image`） | `jetcam` 仅追踪 notebook 显示用，可省 |

### 4.2 PC 端（跑手势客户端 / 看 RViz）

| 项 | 版本 |
|---|---|
| 系统 | Windows 10/11 或 Ubuntu 18.04+（Windows 上只跑 `src/gesture/new.py` 与 Jupyter，不需要 ROS） |
| Python | 3.8 左右（MediaPipe 对 3.6~3.11 支持较好；上游项目 README 用的是 3.8.5） |
| 依赖 | `opencv-python`、`mediapipe`、`numpy`、`protobuf` |
| 网络 | PC 与 Jetson 同一局域网，Jetson 侧 TCP **9999** 端口可通 |

---

## 5. 编译步骤

本包是一个标准 ROS1 catkin 功能包（包名 `desktop_arm_description`，见 `package.xml`）。**它不能直接放在 U 盘/exFAT 上编译**，必须先复制进工作空间：

```bash
# 1) 建工作空间
mkdir -p ~/arm2025_ws/src
cd ~/arm2025_ws/src

# 2) 把整个代码包目录复制进来（目录名带中文没问题，但建议改成英文别名）
cp -r /media/usb/第6组_张航宁_张浩轩_吴锐超_代码包 ~/arm2025_ws/src/desktop_arm_description
# 如果是从 zip 解压：unzip 第6组_张航宁_张浩轩_吴锐超_代码包.zip -d ~/arm2025_ws/src/

# 3) 编译
cd ~/arm2025_ws
catkin_make
# 或：catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3   （Noetic / 手动指定 python3 时）

# 4) 生效
source ~/arm2025_ws/devel/setup.bash
echo "source ~/arm2025_ws/devel/setup.bash" >> ~/.bashrc   # 可选，长期生效

# 5) 验证包是否被 ROS 找到
rospack find desktop_arm_description      # 应打印 ~/arm2025_ws/src/desktop_arm_description

# 6) 给脚本执行权限（catkin_install_python 已处理，手动运行时用）
chmod +x scripts/*.py urdf/check_urdf.py
```

### 5.1 常见报错与处理

| 现象 | 原因 | 解决 |
|---|---|---|
| `CMake Error: ... Symbolic link ... not supported` / `catkin_make` 在 U 盘路径下失败 | U 盘为 **exFAT**，不支持符号链接，catkin 无法在 U 盘上建 devel 空间 | 把功能包复制到 `~/arm2025_ws/src` 再编译，U 盘只作源码留存（报告第二章"问题2"） |
| `[rospack] Error: package 'desktop_arm_description' not found` | 没 `source devel/setup.bash`，或包不在 `src/` 下 | 先 `source`；确认 `package.xml` 与 `CMakeLists.txt` 与本包同级 |
| TF 时有时无 / RViz 模型闪烁 | 上一次 `roslaunch` 未完全退出，同名 `robot_state_publisher` 顶号 | `rosnode list` / `rosnode kill` 清理残留，必要时 `killall -9 rosmaster roslaunch` 后重来（报告"问题3"） |
| RViz 启动弹出 **ROS1 EOL 警告**挡住画面 | ROS1 已 EOL，RViz 每次提示 | `export DISABLE_ROS1_EOL_WARNINGS=1` 后重启 RViz（报告"问题4"） |
| RViz 里模型"白板 / 一片空白"、网格不显示 | mesh 的 `package://` 路径没被解析，或网格不在 `urdf/meshes/` | 先跑 `python3 urdf/check_urdf.py` 看 mesh 是否全部命中；再确认包路径与 URI 前缀一致 |
| `ImportError: No module named uservo` / `arm5dof_kinematic` | DICE SDK 私有模块缺失（本包未提供，见第 12 节） | 从课程发放的完整 SDK 目录复制 `uservo.py`、`arm5dof_kinematic.py` 到 `src/arm_sdk/` |
| `FileNotFoundError: config/z_error_polyfit.txt` | 标定产物缺失（本包未提供） | 按 `config/README_配置文件说明.md` 重新标定后放入 `config/` |

---

## 6. 启动命令（4 组，各有预期现象）

> 所有命令都在 **Jetson** 的 `~/arm2025_ws` 下执行，且已 `source devel/setup.bash`。
> 真机命令（第 3、4 条）需要串口权限：`sudo usermod -aG dialout $USER` 后重新登录。

### ① 模型可视化（RViz + 六滑块）——只动模型，不动真机

```bash
roslaunch desktop_arm_description display.launch
```

**预期现象**：RViz 自动打开，世界坐标系（`world`）下出现机械臂模型（网格颜色沿用原始 URDF 材质：link2/link4 与夹指为绿色 `0 0.6 0`、link1/link3/link5 为灰色 `0.6 0.6 0.6`、底座深灰；若要改成报告正文描述的红色主体，改 `urdf/arm5dof.urdf` 里对应 `<material>` 的 rgba 即可）；左侧 `joint_state_publisher_gui`
弹出 6 个滑块（joint1~joint5 + left_gripper_joint），拖动 joint1 时整条臂绕竖直轴回转，
拖动 left_gripper_joint 时**左右夹指反向联动**（mimic），勾选 TF 后可看到每个关节的 RGB 三色坐标轴
（Marker Scale 0.25）。默认位形 `joint2=-1.5708`、`left_gripper_joint=0.6`，即大臂水平前伸、夹爪半张。

> 只想看模型、不启动 RViz：`roslaunch desktop_arm_description display.launch rviz:=false`。

### ② 颜色分拣（HSV 四色识别 + 自动归位）

```bash
# 真机流程（推荐，与原实验一致）：Jetson 上开 Jupyter，运行颜色分拣 notebook
cd ~/arm2025_ws/src/desktop_arm_description/src/color_sort
jupyter notebook 我的垃圾分类.ipynb        # 按 cell 顺序执行（Run All）

# 无真机 / 只做 ROS 联调时：起相机 + 一键联调
roslaunch usb_cam usb_cam.launch            # 或任意发布 /usb_cam/image_raw 的相机节点
roslaunch desktop_arm_description stack.launch
```

**预期现象**：笔记本里出现 600×480 的实时画面与中心 50×50 的紫色 ROI 框，下方两行标签显示"中心颜色：red/yellow/green/blue"与
"状态：观察中→A 观察中→S 预处理→有害垃圾/其他垃圾/厨余垃圾/可回收垃圾"；机械臂依次完成
"回观察位 → 抓取物块 → 抬起 → 转运到对应垃圾桶 → 松爪 → 回观察位"，识别到 5 帧一致才锁存颜色、本轮完成才进入下一轮。
ROS 联调模式下，终端打印 `[color_sort] 锁存颜色: red（连续 5 帧）`，RViz 的 `Image (color_sort)` 面板显示带框的调试图。

> 颜色阈值在 `src/color_sort/我的垃圾分类.ipynb` 的 `COLOR_RANGES` 与 `scripts/color_sort_node.py` 顶部，
> 搬动工作台或换灯光后需重新核一遍（报告第二章"问题6"就是蓝色/绿色色相被误判的排错记录）。

### ③ 手势搬运（PC 端识别 → TCP → Jetson 执行）

```bash
# 终端 A（Jetson，真机执行端）：先起服务端，再开真机 SDK
cd ~/arm2025_ws/src/desktop_arm_description/src/gesture
jupyter notebook 我的通讯识别.ipynb        # 依次执行：建 arm → 定义服务端 → run_server()
#   无真机时可改用 ROS 联调：roslaunch desktop_arm_description stack.launch

# 终端 B（PC，手势识别端）：把 ARM_HOST 改成 Jetson 的 IP
python new.py
```

**预期现象**：PC 上弹出 `Arm Gesture` 窗口，画面左右翻转，手部画出 MediaPipe 骨架，左上角两行文字
`gest: INDEX/FIST/PALM/...` 与 `act: target1/target2/catch/stop/hold`；Jetson 终端出现
`[Jupyter] 客户端接入: ('192.168.x.x', 5xxxx)` 与 `[CMD] target1 → pan=xx.x°`。
手势 → 动作约定（与 `new.py` 的 `GEST_TO_ACTION` 一致）：**食指（INDEX）→ `target1`，joint1 步进 +3°**、
**剪刀手（V_GEST / FIRST2 / TWO_FINGER_CLOSED）→ `target2`，joint1 步进 −3°**、
**握拳（FIST）→ `stop` 回 home**、**五指张开（PALM）/四指（LAST4）→ `catch` 抓取当前位置**。
持续保持方向手势会按 0.1 s 间隔连续步进，因此底座看起来是平滑转动；朝向哪一侧取决于 joint1 的装配正方向，现场按实物确认。
抓取执行期间 `catch_busy=True`，TCP 侧新指令只会覆盖 `latest_cmd` 而不会打断正在进行的抓取（互锁）。

> 端到端延迟约 0.4 s（采集 8 ms + 推理 18 ms + 5 帧确认 167 ms + 3 帧确认 100 ms + TCP 15 ms + 舵机 90 ms），
> 详细实测与拆解见报告第 5 章。

### ④ 机械臂点动调试（不开 RViz，直接串口驱真机）

```bash
cd ~/arm2025_ws/src/desktop_arm_description/src/arm_sdk
python3 -c "
from arm5dof_uservo import Arm5DoFUServo
arm = Arm5DoFUServo(is_debug=True)        # 构造时自动 home（T=2.0 s 软运动）
arm.set_joint_angle_soft('joint1', 0.3, T=1.0)   # joint1 转 0.3 rad ≈ 17°
arm.gripper_open(angle=0.5, T=0.5)                # 夹爪张开 0.5 rad（也可 set_joint_angle_soft('left_gripper_joint', 0.5, T=0.5)）
"
# 抓取点位动（工作台坐标系）：见 src/lab/分发.ipynb 与 07.机械臂抓取工作台物块测试.ipynb
```

**预期现象**：终端先打印 `设置舵机角度, 舵机#0 目标角度 ...` 与到位日志，机械臂回零点后底座顺时针/逆时针转约 17°、
夹爪张开；`is_debug=True` 时会打印目标位姿与求解出的关节角。
若报 `该设备不存在,请重新填写UART舵机转接板的端口号`，说明 `/dev/ttyTHS1` 不对——PC 上跑请把
`src/arm_sdk/config.py` 里 `DEVICE_PORT_DEFAULT` 改成 `COM6`（Windows）或 `/dev/ttyUSB0`（Ubuntu）。

---

## 7. 参数文件说明

### 7.1 `src/arm_sdk/config.py`（原始文件，最终标定版）

| 变量 | 当前值 | 含义与用法 |
|---|---|---|
| `DEVICE_PORT_DEFAULT` | `'/dev/ttyTHS1'` | 串口设备；文件里以注释形式给出 `COM6` / `/dev/ttyUSB0` / `/dev/ttyS0` 等可选值 |
| `SERVO_NUM` | `6` | 舵机个数（ID0~ID5：joint1~joint5 + 夹爪） |
| `SERVO_SPEED_DEFAULT` | `100` | 舵机默认平均转速（°/s） |
| `JOINT_SERVO_ID_DICT` | joint1→0 … left_gripper_joint→5 | **关节名 → 舵机 ID** 映射；关节名必须与 URDF 一致 |
| `JOINT2SERVO_K` | `[-59.142, 57.678, -52.075, -55.513, 0.318, 57.296]` | 关节弧度 → 舵机原始角度的**比例系数 k** |
| `JOINT2SERVO_B` | `[-0.400, 92.900, 2.000, 3.200, 1.300, 1.600]` | 同上，**零点偏移 b**；即 `舵机角 = k·θ + b` |
| `ARM_POSE_DICT['home']` | `[150, 0, 200, None, π]` | 零点位姿 `[x, y, z, pitch, roll]`（单位 mm / rad，Z-Y-Z 欧拉角，`None` 表示自动求） |
| `ARM_POSE_DICT['object_release']` | `[0, 185, 80, None, π]` | 物块释放点 |
| `ARM_POSE_DICT['capture_image']` | `[105, 0.0, 60.0, 2.47, π]` | 拍摄工作台图像时的位姿（末端相机朝下俯视） |
| `ARM_POSE_DICT['object_green/yellow/red/blue']` | 见文件 | 四色物块的测试抓取位姿（pitch 用 `np.radians` 写成 125°~140°） |
| `PITCH_PANEL_A/B/C` | `-0.002490 / -0.004985 / 3.048636` | 工作台平面俯仰角**拟合系数**（`pitch = A·x + B·y + C`），让夹爪始终垂直于台面抓取 |
| `TRAJECTORY_DELTA_T` | `0.012` s | 轨迹插补采样周期（与 `minimum_jerk.py` 默认值一致） |
| `RAD_PER_STEP` | `0.05` rad | 关节插补的单步角度增量（约 2.86°） |

**注意**：`JOINT2SERVO_K/B` 在文件里以注释保留了 **4 组历史标定值**，末行未注释的才是最终采用值；
`lab3/lab4` 那份代码包里的 `config.py` 是更早一版（K 第 1 项 `-57.678`），本包只收最终版。

### 7.2 标定产物（本包**未提供**，需自行生成）

| 文件 | 使用者 | 内容 |
|---|---|---|
| `config/z_error_polyfit.txt` | `arm5dof_uservo.py` | Z 轴误差补偿的一次拟合系数 `[k, b]`（`np.loadtxt(delimiter=",")`） |
| `config/T_arm2ws.txt` | `arm_test.py` | 工作台 → 机械臂基座的 4×4 齐次变换矩阵 |
| `config/object.yaml` | `arm_test.py` | 物块高度 `height`（米）等抓取参数 |

生成方法、格式与目录放置方式见 **`config/README_配置文件说明.md`**。这三个文件是实测标定结果，
本包不填猜测值，否则机械臂会以错误位姿抓取。

### 7.3 手势与通信参数

| 参数 | 位置 | 当前值 | 说明 |
|---|---|---|---|
| `ARM_HOST` / `ARM_PORT` | `src/gesture/new.py` | `"192.168.55.1"` / `9999` | Jetson 的 IP 与端口，需按现场网络修改 |
| `send_interval` | `src/gesture/new.py` | `0.05` s | 方向类指令最小发送间隔（限频） |
| `STABLE_THRESHOLD` | `src/gesture/new.py` | `3` | 连续 3 帧动作一致才确认（一级防抖） |
| `frame_cnt > 4` | `src/gesture/new.py` `HandRecog.get_gest` | `4` | 手势稳定 5 帧才更新（二级防抖） |
| `TURN_STEP_RAD` | `我的通讯识别.ipynb` | `math.radians(3.0)` | 每条 target1/target2 指令转 3° |
| `PAN_RAD_MIN/MAX` | `我的通讯识别.ipynb` | `±90°` | joint1 软件限位（与 URDF ±1.5708 一致） |
| `MOVE_INTERVAL` | `我的通讯识别.ipynb` | `0.1` s | 服务端方向指令最小间隔 |
| `catch_busy` | `我的通讯识别.ipynb` | `False` | 抓取执行期间为 `True`，新 catch 指令被丢弃（互锁） |
| `latest_cmd` | `我的通讯识别.ipynb` | `None` | 新指令**永远覆盖**旧指令，避免指令队列堆积导致的"延迟执行" |

---

## 8. 组员分工

| 组员 | 学号 | 贡献度 | 负责模块 | 本包内对应文件 | 可被提问的技术点 |
|---|---|---|---|---|---|
| **张航宁**（组长） | 24061733 | **40%** | URDF 建模与 RViz / Gazebo 验证；**机械臂运动控制与 Pick & Place** | `urdf/arm5dof.urdf`、`urdf/check_urdf.py`、`launch/display.launch`、`launch/gazebo.launch`、`rviz/workcell.rviz`、`src/arm_sdk/arm5dof_uservo.py`、`src/arm_sdk/arm_test.py`、`src/arm_sdk/config.py`、`src/arm_sdk/minimum_jerk.py`、`src/arm_sdk/transform.py`、`src/lab/分发.ipynb` | joint2 的 rpy=(-1.5708,0,0) 为什么必需；limit/axis 改错会怎样（axis 反向 TCP 偏 0.596 m）；mimic 双指联动；TF 树连乘；六段式最小加加速度轨迹；抓取/释放的 2 mm / 3 mm 数值门限 |
| **张浩轩** | 24061720 | **32%** | **手势识别**（MediaPipe 与手指判据）；**TCP 通信与指令服务端** | `src/gesture/new.py`、`src/gesture/我的通讯识别.ipynb`、`scripts/gesture_tcp_node.py` | 手指位掩码 Gest 的 5 位编码；`sign_dist` 判据为什么对旋转鲁棒；双级防抖（3 帧 + 5 帧）；`latest_cmd` 覆盖写与 `catch_busy` 互锁；JSON 行协议分包/粘包处理 |
| **吴锐超** | 24061734 | **28%** | **HSV 颜色识别与自动分拣**；**状态机互锁与系统集成测试** | `src/color_sort/我的垃圾分类.ipynb`、`src/color_sort/config.py`、`src/tracking/14.OpenCV摄像头机械臂物体追踪.ipynb`、`scripts/color_sort_node.py` | 三通道阈值怎么定（红跨 0/180 要用双区间）；中心 ROI + 高斯加权的理由；5 帧锁存与 `pending_color` 为什么不能用实时颜色；INIT/OBSERVE/ACTION/IDLE 状态机；蓝色被误判为红的色相排错过程 |

**协作机制**：源码与报告用 Git 管理（分支 `feat/urdf`、`feat/gesture`、`feat/color`，合并前互审）；
每周一次联调（记录"现象—原因—修改—验证"四列）；演示脚本与测试数据（JSON）统一放在实验记录目录。

---

## 9. 外部开源引用与来源

> 说明：本项目在**开源项目的手势编码思路**与**公开 SDK/库**基础上完成，下面逐条写清来源与"我们改了什么"，
> 不把上游代码伪装成原创。

| # | 名称 | 来源网址 | 许可证 | 在本项目中的作用 | 本组做了哪些修改 / 使用程度 |
|---|---|---|---|---|---|
| 1 | **Gesture-Controlled-Virtual-Mouse**（手势虚拟鼠标） | <https://github.com/Viral-Doshi/Gesture-Controlled-Virtual-Mouse>（上游 README 给出的原始地址为 <https://github.com/xenon-19/Gesture-Controlled-Virtual-Mouse>，现已重定向到前者） | **GPL-3.0** | 提供"手指伸直/弯曲 → 5 位手指位掩码 → 手势枚举"的编码思路与 `HandRecog` 判据 | **派生改写**（本包 `src/gesture/new.py`）：① 输出从"虚拟鼠标/键盘动作"改为"机械臂动作" `target1/target2/catch/stop/hold`；② 新增 `ArmClient`，用 TCP + JSON 行协议与 Jetson 通信；③ 新增 3 帧防抖（`STABLE_THRESHOLD`）与方向类 50 ms 限频；④ `catch/stop` 只在动作变化时发一次，避免重复触发。**未采用**其语音助手（`Proton.py`）、网页 UI（`src/web/`）、鼠标控制（`Gesture_Controller.py`）与演示 GIF |
| 2 | **MediaPipe Hands**（Google） | <https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker>、<https://github.com/google/mediapipe> | Apache-2.0 | 手部 21 个关键点检测与左右手分类（`multi_handedness` 置信度） | 直接调用 Python API（`mp.solutions.hands`，`max_num_hands=2`），未改算法 |
| 3 | **OpenCV** | <https://opencv.org/>、<https://docs.opencv.org/> | Apache-2.0 | HSV 转换、`inRange` 阈值分割、形态学腐蚀/膨胀、轮廓与绘制、摄像头采集 | 直接调用；本项目自写的只有阈值区间、ROI 加权核与状态机 |
| 4 | **DICE 5DoF 机械臂 SDK** | <https://www.dice.com.cn/> | 课程随实验箱发放（版权归 DICE） | 串口舵机驱动、关节角↔舵机角换算、运动学、轨迹、抓取应用层 | **原样使用**（`src/arm_sdk/` 下 4 个原始文件只加文件头注释）；本组补写了缺失的 `minimum_jerk.py`、`transform.py` 接口（复原实现） |
| 5 | **ROS / URDF 文档** | <https://wiki.ros.org/urdf>、<https://wiki.ros.org/urdf/Tutorials>、<https://docs.ros.org/en/melodic/> | BSD / CC BY 3.0 | URDF 标签语义（link/joint/origin/axis/limit/mimic）、launch 结构、RViz 配置格式 | 按规范撰写模型与 launch；`check_urdf.py` 用标准库 `xml.etree` 自行实现检查 |
| 6 | **Minimum Jerk 轨迹**（Flash & Hogan, 1985） | <https://doi.org/10.1523/JNEUROSCI.05-07-01688.1985> | 学术引用 | 六段式平滑轨迹的公式来源：`s(τ)=10τ³−15τ⁴+6τ⁵` | 在 `src/arm_sdk/minimum_jerk.py` 中实现（原 SDK 实现未随材料提供，本文件为按公式复原） |
| 7 | **本项目开源仓库** | <https://github.com/zhang-hn-1/Robot-Final> | GPL-3.0 | 报告与代码包的公开归档地址 | 与报告第 6.4 节一致 |

**在上游基础上我们到底改了什么（逐条对照，便于答辩时被追问）**

1. **输出对象换了**：上游把手势映射成虚拟鼠标动作（移动光标、左/右击、滚轮、音量/亮度），本项目映射成机械臂动作 `target1 / target2 / catch / stop / hold`，并新增 `ArmClient` 用 TCP + JSON 行协议把指令发到 Jetson。
2. **加了双级确认（防误触发）**：第一级在客户端 `new.py`——动作连续 3 帧一致才算确认（`STABLE_THRESHOLD=3`），手势本身还要稳定 5 帧（`frame_cnt > 4`）；第二级在服务端——方向类指令最小间隔 `MOVE_INTERVAL=0.1 s`，且颜色/手势判决都有"连续 N 帧一致才执行"的门限。
3. **加了 BUSY 互锁**：服务端 `catch_busy` 在执行抓取的整段时间内为 `True`，期间到达的新 `catch` 指令直接丢弃，避免"抓取还没抬起来又被要求抓一次"把物块甩出去。
4. **加了 `latest_cmd` 覆盖写**：TCP 收包线程只把最新一条指令写进 `latest_cmd`（加锁覆盖），执行线程取走即清空——这样即使网络抖动堆了十几条指令，机械臂也只会执行最新的那一条，不会出现"动作延迟十几秒才做完"的排队现象。
5. **删掉了与本项目无关的部分**：上游的语音助手（`Proton.py`）、网页交互界面（`src/web/`）、手套版识别（`Gesture_Controller_Gloved.py`）与鼠标控制（`Gesture_Controller.py`）都**没有**纳入本包；本包只保留"手指编码 + 手势枚举 + 机械臂动作映射"这一条主线。

---

## 10. 许可证与致谢

- **许可证**：本代码包整体以 **GNU General Public License v3.0（GPL-3.0）** 发布。
  原因：`src/gesture/new.py` 派生自 GPL-3.0 的上游项目，按 GPL 的传染性要求，派生作品须同样以 GPL-3.0 发布。
  `src/arm_sdk/` 下的 DICE SDK 原始文件版权归 **DICE（<https://www.dice.com.cn/>）** 所有，按其随课程发放的使用条款使用，不因本包重新授权。
- **致谢**：
  - 感谢 **DICE** 提供五自由度机械臂、SDK 与实验箱；
  - 感谢上游项目 **Gesture-Controlled-Virtual-Mouse** 的作者团队（Viral Doshi、Nishiket Bidawat、Ankit Sharma、Parth Sakariya）开源其手势编码实现；
  - 感谢 **Google MediaPipe** 与 **OpenCV** 社区；
  - 感谢课程指导教师在 URDF 建模、Gazebo 加载与排错过程中的指导。

---

## 11. 版本记录

| 版本 | 日期 | 内容 |
|---|---|---|
| v1.0 | 2026-11 | 综合设计交付版：整理 4 个实验目录源码；按第二章报告表2/表3 与原始 URDF 复原 `arm5dof.urdf`（13 连杆 / 12 关节）；复原 3 份 launch 与 RViz 配置；新增 `check_urdf.py`、`package.xml`、`CMakeLists.txt`、2 个 ROS 薄封装节点；补齐 `minimum_jerk.py`、`transform.py` 说明性实现；完成 README 与《代码包自检.md》 |

---

## 12. 已知限制（如实声明，请勿据此高估本包）

1. **DICE 私有模块缺失**：`uservo.py`、`arm5dof_kinematic.py` 未随本次交付材料提供，
   所以 `src/arm_sdk/` 的 SDK **能被阅读、在完整 SDK 环境能运行，但仅凭本包不能独立 `import` 跑真机**。
   `minimum_jerk.py`、`transform.py` 是本组按公式/调用点复原的兼容实现，**不保证与原实现逐位一致**。
2. **标定产物缺失**：`z_error_polyfit.txt`、`T_arm2ws.txt`、`object.yaml` 三个实测标定文件未提供，
   真机抓取前必须按 `config/README_配置文件说明.md` 重新生成，否则 SDK 会在构造函数处 `FileNotFoundError`。
3. **URDF 是复原版**：原始工程为 SolidWorks 导出，本包依据本组原始 URDF（`src/code/1.txt`，PyBullet 版）
   与报告表2/表3 复原；`world` 与 `tool_center_frame` 的 `inertial`/`collision` 为**补充的占位近似值**，
   其余连杆的惯量取自原始文件（非估算）。`camera` 连杆的 inertial/visual/collision 在原始文件中被注释，本包恢复启用。
4. **launch / rviz 是复原版**：原始 launch 未随材料提供，本包按报告第 5、7 节片段复原；
   报告提到的 `desktop_scene`（桌面、料筒、物块参数化场景）属于配套包 `desktop_helper_scene`，**未随本包提供**。
5. **未做 ROS 回归**：本包在**无 ROS 的 Windows 机器**上整理，`scripts/` 下两个节点与 3 份 launch 只做了
   `py_compile` 语法检查、XML/YAML 结构与 `check_urdf.py` 逻辑校验，**没有在真 ROS 环境里 roslaunch 跑过**，
   因此不能声称"已验证可运行"。
6. **Gazebo 未接 ros_control**：模型中没有 `<transmission>` 与 controller 配置，
   关节角只能由 `-J` 参数或 `gz joint` 命令指定（报告第 11 节已写明这一局限）。
7. **颜色阈值依赖现场光照**：`COLOR_RANGES` 是在本组实验台灯光下标定的，换环境需重新核对（尤其蓝色的 V 下界）。
