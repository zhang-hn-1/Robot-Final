# =====================================================================
# 【派生文件｜本组在开源项目基础上改写，非纯原创】
# 文件名 : new.py（手势 → 机械臂 TCP 指令 客户端）
# 来源   : 本组交付的 code.zip 中 src/new.py；
#          其手势编码（Gest 手指位掩码、HandRecog.set_finger/get_gest）与判定思路
#          改写自开源项目 Gesture-Controlled-Virtual-Mouse（GPL-3.0，原 README 给出的
#          仓库为 https://github.com/xenon-19/Gesture-Controlled-Virtual-Mouse ，
#          现已重定向到 https://github.com/Viral-Doshi/Gesture-Controlled-Virtual-Mouse ）。
# 本组改动: ① 把「手势 → 虚拟鼠标/键盘动作」改为「手势 → 机械臂动作」（target1/target2/
#          catch/stop/hold），新增 ArmClient 走 TCP 9999 端口；② 新增 3 帧防抖
#          （STABLE_THRESHOLD）与方向类指令限频（send_interval=50 ms）；③ catch/stop 只在
#          动作变化时发一次，避免机械臂被重复触发。正文中文注释与打印信息为本组所加。
# 许可   : 上游为 GPL-3.0，本文件作为派生作品同样按 GPL-3.0 发布，见包根目录 README.md。
# =====================================================================
# ====================== 导入 ======================
import cv2
import mediapipe as mp
import math
import socket
import json
import time
from enum import IntEnum
from google.protobuf.json_format import MessageToDict

mp_drawing = mp.solutions.drawing_utils
mp_hands   = mp.solutions.hands

# ====================== 机械臂通信 ======================
ARM_HOST = "192.168.55.1"
ARM_PORT = 9999

class ArmClient:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        print(f"[VS Code] 已连接机械臂 {host}:{port}")

    def send(self, action, step=3.0):
        try:
            msg = json.dumps({"action": action, "step": step}) + "\n"
            self.sock.sendall(msg.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"[VS Code] 发送失败: {e}")

    def close(self):
        self.sock.close()

# ====================== 手势编码 ======================
class Gest(IntEnum):
    FIST               = 0
    PINKY              = 1
    RING               = 2
    MID                = 4
    LAST3              = 7
    INDEX              = 8
    FIRST2             = 12
    LAST4              = 15
    THUMB              = 16
    PALM               = 31
    V_GEST             = 33
    TWO_FINGER_CLOSED  = 34

class HLabel(IntEnum):
    MINOR = 0
    MAJOR = 1

# ====================== 手部识别 ======================
class HandRecog:
    def __init__(self, label):
        self.label      = label
        self.finger     = Gest.PALM
        self.pre_gest   = Gest.PALM
        self.ori_gest   = Gest.PALM
        self.frame_cnt  = 0
        self.hand_res   = None

    def update(self, res):
        self.hand_res = res

    def dist(self, pts):
        x1 = self.hand_res.landmark[pts[0]].x
        y1 = self.hand_res.landmark[pts[0]].y
        x2 = self.hand_res.landmark[pts[1]].x
        y2 = self.hand_res.landmark[pts[1]].y
        return math.hypot(x1 - x2, y1 - y2)

    def sign_dist(self, pts):
        d = self.dist(pts)
        if self.hand_res.landmark[pts[0]].y < self.hand_res.landmark[pts[1]].y:
            return d
        return -d

    def set_finger(self):
        if self.hand_res is None:
            return
        points = [[8, 5, 0], [12, 9, 0], [16, 13, 0], [20, 17, 0]]
        finger_int = 0
        for p in points:
            denom = self.sign_dist(p[1:])
            if abs(denom) < 1e-6:
                finger_int = finger_int << 1
                continue
            r = self.sign_dist(p[:2]) / denom
            finger_int = finger_int << 1
            if r > 0.5:
                finger_int = finger_int | 1
        try:
            self.finger = Gest(finger_int)
        except ValueError:
            self.finger = Gest.PALM

    def get_gest(self):
        if self.hand_res is None:
            return Gest.PALM

        if self.finger == Gest.FIRST2:
            denom = self.dist([5, 9])
            if denom < 1e-6:
                cur = self.finger
            else:
                r = self.dist([8, 12]) / denom
                if r > 1.7:
                    cur = Gest.V_GEST
                else:
                    z_diff = abs(self.hand_res.landmark[8].z -
                                 self.hand_res.landmark[12].z)
                    cur = Gest.TWO_FINGER_CLOSED if z_diff < 0.1 else Gest.MID
        else:
            cur = self.finger

        if cur == self.pre_gest:
            self.frame_cnt += 1
        else:
            self.frame_cnt = 0
        self.pre_gest = cur
        if self.frame_cnt > 4:
            self.ori_gest = cur
        return self.ori_gest

# ====================== 手势 → 机械臂动作 ======================
GEST_TO_ACTION = {
    Gest.FIST:              "stop",
    Gest.INDEX:             "target1",
    Gest.FIRST2:            "target2",
    Gest.V_GEST:            "target2",
    Gest.TWO_FINGER_CLOSED: "target2",
    Gest.LAST4:             "catch",
    Gest.PALM:              "catch",
}

def gest_to_action(gest):
    return GEST_TO_ACTION.get(gest, "hold")

# ====================== 主控 ======================
class ArmGestureController:
    def __init__(self, arm_host, arm_port):
        self.cap = cv2.VideoCapture(0)
        self.w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.major = HandRecog(HLabel.MAJOR)
        self.minor = HandRecog(HLabel.MINOR)

        self.client = ArmClient(arm_host, arm_port)
        self.step = 3.0

        # ---- 发送控制 ----
        self.last_send_time = 0
        self.send_interval  = 0.05    # 每 50ms 发一次

        # ---- 防抖 ----
        self.stable_action  = None
        self.stable_count   = 0
        self.STABLE_THRESHOLD = 3     # 连续 3 帧相同才确认

        # ---- catch 只发一次 ----
        self.last_sent_action = None

    def classify(self, res):
        left_hand = right_hand = None
        for idx, hand_land in enumerate(res.multi_hand_landmarks):
            info = MessageToDict(res.multi_handedness[idx])
            label = info['classification'][0]['label']
            if label == 'Right':
                right_hand = hand_land
            else:
                left_hand = hand_land
        self.major.update(right_hand)
        self.minor.update(left_hand)

    def start(self):
        with mp_hands.Hands(max_num_hands=2,
                            min_detection_confidence=0.5,
                            min_tracking_confidence=0.5) as hands:
            while self.cap.isOpened():
                ok, img = self.cap.read()
                if not ok:
                    continue

                img = cv2.flip(img, 1)
                rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                out = hands.process(rgb)

                action = "hold"
                gest_name = "NO HAND"

                if out.multi_hand_landmarks:
                    self.classify(out)
                    self.minor.set_finger()
                    self.major.set_finger()

                    current_gest = self.major.get_gest()
                    gest_name = current_gest.name if isinstance(current_gest, Gest) else f"UNK({current_gest})"
                    action = gest_to_action(current_gest)

                    for hand_lm in out.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            img, hand_lm, mp_hands.HAND_CONNECTIONS)

                # ---- 防抖 ----
                if action == self.stable_action:
                    self.stable_count += 1
                else:
                    self.stable_count = 0
                    self.stable_action = action

                if self.stable_count < self.STABLE_THRESHOLD:
                    action = self.last_sent_action if self.last_sent_action else "hold"

                # ---- 发送逻辑 ----
                now = time.time()
                if now - self.last_send_time >= self.send_interval:
                    if action in ("target1", "target2"):
                        # 方向类：持续发
                        self.client.send(action, step=self.step)
                        self.last_send_time = now
                        self.last_sent_action = action
                    elif action == "catch":
                        # 抓取：只在变化时发一次
                        if action != self.last_sent_action:
                            self.client.send(action, step=self.step)
                            print(f"[VS Code] 手势={gest_name:20s} → 动作={action}")
                            self.last_sent_action = action
                        self.last_send_time = now
                    elif action in ("stop", "hold"):
                        # 停/保持：只在变化时发一次
                        if action != self.last_sent_action:
                            self.client.send(action, step=self.step)
                            print(f"[VS Code] 手势={gest_name:20s} → 动作={action}")
                            self.last_sent_action = action
                        self.last_send_time = now

                cv2.putText(img, f"gest: {gest_name}", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.putText(img, f"act : {action}", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
                cv2.imshow("Arm Gesture", img)

                if cv2.waitKey(5) == 13:
                    break

        self.cap.release()
        cv2.destroyAllWindows()
        self.client.close()


if __name__ == "__main__":
    gc = ArmGestureController(ARM_HOST, ARM_PORT)
    gc.start()