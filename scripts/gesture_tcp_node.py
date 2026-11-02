#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =====================================================================
# 【新增文件｜ROS 薄封装节点，非原始交付材料】
# 文件名 : gesture_tcp_node.py
# 用途   : 实验七的手势服务端在 Jupyter 里运行（见 src/gesture/我的通讯识别.ipynb）。
#          本节点把同一套 TCP 9999 端口 / JSON 行协议搬到 ROS 节点里，供
#          launch/stack.launch 一键联调，并与手势客户端 src/gesture/new.py 配对。
# 协议   : 每行一个 JSON：{"action": "target1|target2|catch|stop|hold", "step": 3.0}
# 行为   : target1/target2 按 3°/步 调整 joint1（±90° 限位），并把结果发布到
#          /desktop_arm/manual_joint_states（display.launch 的 joint_state_publisher_gui
#          把它列入 source_list，RViz 无真机也能看到底座跟手转动）；
#          catch/stop/hold 转发到 /desktop_arm/target_cmd。
# 参数   : ~tcp_port (默认 9999)、~use_arm (默认 false，true 时才连真机 SDK)
# 说明   : 已通过 python3 -m py_compile 语法检查；未在 ROS 环境逐帧回归。
# =====================================================================
import json
import math
import socket
import threading
import time

try:
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
except ImportError as _exc:  # 允许在无 ROS 的 PC 上做语法检查
    rospy = None
    String = JointState = None
    _ROS_IMPORT_ERROR = _exc

# ---- 与 src/gesture/我的通讯识别.ipynb 一致的控制参数 ----
TURN_STEP_RAD = math.radians(3.0)
PAN_RAD_MIN = math.radians(-90.0)
PAN_RAD_MAX = math.radians(90.0)
MOVE_INTERVAL = 0.1          # 方向类指令最小间隔（s）
HOME_JOINT2 = -1.5708        # display.launch 的 zeros 预设
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "left_gripper_joint"]


class GestureTcpNode(object):
    def __init__(self):
        self.port = int(rospy.get_param("~tcp_port", 9999))
        self.use_arm = bool(rospy.get_param("~use_arm", False))

        self.lock = threading.Lock()
        self.pan_rad = 0.0
        self.angles = [0.0, HOME_JOINT2, 0.0, 0.0, 0.0, 0.0]
        self.last_move_time = 0.0
        self.catch_busy = False
        self.running = True

        self.cmd_pub = rospy.Publisher("/desktop_arm/target_cmd", String, queue_size=10)
        self.js_pub = rospy.Publisher("/desktop_arm/manual_joint_states",
                                      JointState, queue_size=10)

        self.arm = None
        if self.use_arm:
            self._try_init_arm()

        # 20 Hz 发布关节状态：joint_state_publisher_gui 的 source_list 会合并它
        rospy.Timer(rospy.Duration(0.05), self.publish_joint_state)

        # TCP 服务端放在独立线程，避免阻塞 rospy.spin()
        self.thread = threading.Thread(target=self.serve_forever)
        self.thread.daemon = True
        self.thread.start()
        rospy.loginfo("[gesture_tcp] 监听 0.0.0.0:%d，use_arm=%s", self.port, self.use_arm)

    # ---------------- 真机（可选） ----------------
    def _try_init_arm(self):
        try:
            from arm5dof_uservo import Arm5DoFUServo
            self.arm = Arm5DoFUServo(is_debug=False)
            rospy.loginfo("[gesture_tcp] 已连接真机 SDK")
        except Exception as exc:
            rospy.logwarn("[gesture_tcp] 未连接真机（%s），退回纯话题模式", exc)
            self.arm = None

    # ---------------- 指令处理 ----------------
    def handle_command(self, cmd):
        action = cmd.get("action")
        step = float(cmd.get("step", 3.0))
        now = time.time()

        if action == "hold":
            return

        if action == "stop":
            with self.lock:
                self.pan_rad = 0.0
                self.angles[0] = 0.0
            self.cmd_pub.publish(String(data="stop"))
            rospy.loginfo("[gesture_tcp] stop → joint1 回零")
            if self.arm is not None:
                try:
                    self.arm.home()
                except Exception as exc:
                    rospy.logwarn("[gesture_tcp] 真机 home 失败: %s", exc)
            return

        if action in ("target1", "target2"):
            if now - self.last_move_time < MOVE_INTERVAL:
                return
            self.last_move_time = now
            delta = TURN_STEP_RAD * (1.0 if action == "target1" else -1.0)
            with self.lock:
                self.pan_rad = max(PAN_RAD_MIN, min(PAN_RAD_MAX, self.pan_rad + delta))
                self.angles[0] = self.pan_rad
                pan_deg = math.degrees(self.pan_rad)
            self.cmd_pub.publish(String(data=action))
            rospy.loginfo("[gesture_tcp] %s → joint1=%.1f°", action, pan_deg)
            if self.arm is not None:
                try:
                    self.arm.set_joint_angle_soft("joint1", self.pan_rad, T=0.3)
                except Exception as exc:
                    rospy.logwarn("[gesture_tcp] 真机转动失败: %s", exc)
            return

        if action == "catch":
            if self.catch_busy:
                rospy.logwarn_throttle(2.0, "[gesture_tcp] 上一轮 catch 未完成，忽略本条")
                return
            self.catch_busy = True
            try:
                self.cmd_pub.publish(String(data="catch"))
                rospy.loginfo("[gesture_tcp] catch → 抓取当前位置")
                if self.arm is not None:
                    try:
                        self.arm.gripper_close(T=0.5)
                    except Exception as exc:
                        rospy.logwarn("[gesture_tcp] 真机抓取失败: %s", exc)
            finally:
                self.catch_busy = False
            return

        rospy.loginfo("[gesture_tcp] 未知动作: %r", action)

    def publish_joint_state(self, _event=None):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        with self.lock:
            msg.name = list(JOINT_NAMES)
            msg.position = list(self.angles)
        self.js_pub.publish(msg)

    # ---------------- TCP 服务端 ----------------
    def serve_forever(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.settimeout(1.0)
        try:
            srv.bind(("0.0.0.0", self.port))
            srv.listen(1)
        except OSError as exc:
            rospy.logerr("[gesture_tcp] 端口 %d 绑定失败: %s", self.port, exc)
            return

        while self.running and not rospy.is_shutdown():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            rospy.loginfo("[gesture_tcp] 客户端接入: %s", str(addr))
            buf = b""
            try:
                conn.settimeout(1.0)
                while self.running and not rospy.is_shutdown():
                    try:
                        data = conn.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buf += data
                    lines = buf.split(b"\n")
                    buf = lines[-1]
                    for line in lines[:-1]:
                        if not line.strip():
                            continue
                        try:
                            cmd = json.loads(line.decode("utf-8"))
                        except ValueError as exc:
                            rospy.logwarn("[gesture_tcp] JSON 解析失败: %s / %r", exc, line)
                            continue
                        try:
                            self.handle_command(cmd)
                        except Exception as exc:  # 单条指令异常不影响连接
                            rospy.logwarn("[gesture_tcp] 指令执行失败: %s", exc)
            except Exception as exc:
                rospy.logwarn("[gesture_tcp] 连接异常: %s", exc)
            finally:
                conn.close()
                rospy.loginfo("[gesture_tcp] 客户端断开")
        srv.close()


def main():
    if rospy is None:
        raise SystemExit("本节点需要 ROS1 环境（rospy），当前 python 无法 import：%s"
                         % _ROS_IMPORT_ERROR)
    rospy.init_node("gesture_tcp_node")
    GestureTcpNode()
    rospy.spin()


if __name__ == "__main__":
    main()
