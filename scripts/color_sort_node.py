#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =====================================================================
# 【新增文件｜ROS 薄封装节点，非原始交付材料】
# 文件名 : color_sort_node.py
# 用途   : 把实验六《我的垃圾分类.ipynb》里的颜色判决逻辑搬到 ROS 话题上，
#          供 launch/stack.launch 一键联调：订阅相机图像 → 中心 ROI 四色判决
#          → 连续 N 帧一致后锁存 → 发布 /desktop_arm/color（std_msgs/String）
# 参数   : ~image_topic (默认 /usb_cam/image_raw)
#          ~confirm_frames (默认 5，与 notebook 的 CONFIRM_FRAMES 一致)
#          ~use_arm (默认 false；true 时尝试调用真机 SDK，缺模块会告警退回纯话题)
# 颜色阈值: 直接取自 notebook / 实验六报告的 HSV 区间（红 0-8 ∪ 170-180、
#          黄 15-40、绿 35-90、蓝 90-140；S/V 下界同 notebook）
# 说明   : 已通过 python3 -m py_compile 语法检查；未在 ROS 环境逐帧回归，
#          正式实验流程仍以 Jupyter Notebook 驱动真机为准（见 README 第 6 节）。
# =====================================================================
import threading

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
except ImportError as _exc:  # 允许在无 ROS 的 PC 上做语法/单元检查
    rospy = None
    Image = String = CvBridge = None
    _ROS_IMPORT_ERROR = _exc

import cv2

# ---- HSV 四色阈值（与 我的垃圾分类.ipynb 的 COLOR_RANGES 完全一致）----
COLOR_RANGES = {
    "red":    [((0, 60, 40), (8, 255, 255)),
               ((170, 60, 40), (180, 255, 255))],
    "yellow": [((15, 50, 100), (40, 255, 255))],
    "green":  [((35, 50, 50), (90, 255, 255))],
    "blue":   [((90, 50, 50), (140, 255, 255))],
}

ROI_HALF = 25          # 中心 ROI 半宽（像素），notebook 原值
SCORE_RATIO = 0.03     # 最佳得分低于 3% 权重和则判为「无有效颜色」，notebook 原值


class ColorSortNode(object):
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 5))
        self.use_arm = bool(rospy.get_param("~use_arm", False))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_color = None
        self.stable_color = None
        self.stable_count = 0

        # 中心加权核：越靠近光心权重越大（notebook 的高斯权重做法）
        yy, xx = np.mgrid[0:ROI_HALF * 2, 0:ROI_HALF * 2]
        c0 = ROI_HALF
        w = np.exp(-(((xx - c0) ** 2 + (yy - c0) ** 2) / (2 * (ROI_HALF * 0.6) ** 2)))
        self.weight = (w / w.max()).astype(np.float32)
        self.weight_sum = float(self.weight.sum())

        self.arm = None
        if self.use_arm:
            self._try_init_arm()

        self.color_pub = rospy.Publisher("/desktop_arm/color", String, queue_size=10)
        self.debug_pub = rospy.Publisher("/color_sort/debug_image", Image, queue_size=2)
        rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=2)
        rospy.loginfo("[color_sort] 订阅 %s，确认帧数 %d，use_arm=%s",
                      self.image_topic, self.confirm_frames, self.use_arm)

    # ---------------- 真机（可选） ----------------
    def _try_init_arm(self):
        try:
            from arm5dof_uservo import Arm5DoFUServo
            self.arm = Arm5DoFUServo(is_debug=False)
            rospy.loginfo("[color_sort] 已连接真机 SDK")
        except Exception as exc:  # ImportError / SerialException 都会落到这里
            rospy.logwarn("[color_sort] 未连接真机（%s），退回纯话题模式", exc)
            self.arm = None

    # ---------------- 颜色判决 ----------------
    def detect_center_color(self, frame):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        roi = frame[max(0, cy - ROI_HALF):cy + ROI_HALF,
                    max(0, cx - ROI_HALF):cx + ROI_HALF]
        if roi.size == 0:
            return None
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[..., 1] = np.clip(hsv[..., 1] * 1.6, 0, 255)
        hsv[..., 2] = np.clip(hsv[..., 2] * 0.8, 0, 255)
        hsv = hsv.astype(np.uint8)

        best_color, best_score = None, 0.0
        for name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)
            score = float(np.sum(mask.astype(np.float32) * self.weight))
            if score > best_score:
                best_score, best_color = score, name
        if best_score < self.weight_sum * SCORE_RATIO:
            return None
        return best_color

    # ---------------- 回调 ----------------
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[color_sort] 图像转换失败: %s", exc)
            return

        color = self.detect_center_color(frame)

        with self.lock:
            if color is not None and color == self.last_color:
                self.stable_count += 1
            else:
                self.stable_count = 1 if color is not None else 0
            self.last_color = color
            if self.stable_count >= self.confirm_frames and self.stable_color != color:
                self.stable_color = color
                self.color_pub.publish(String(data=color))
                rospy.loginfo("[color_sort] 锁存颜色: %s（连续 %d 帧）",
                              color, self.stable_count)

        # 调试图像：画中心框并标注当前判决
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.rectangle(frame, (cx - ROI_HALF, cy - ROI_HALF),
                      (cx + ROI_HALF, cy + ROI_HALF), (255, 0, 255), 2)
        cv2.putText(frame, color if color else "none",
                    (max(0, cx - ROI_HALF), max(20, cy - ROI_HALF - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
        except Exception:
            pass


def main():
    if rospy is None:
        raise SystemExit("本节点需要 ROS1 环境（rospy / cv_bridge），当前 python 无法 import：%s"
                         % _ROS_IMPORT_ERROR)
    rospy.init_node("color_sort_node")
    ColorSortNode()
    rospy.spin()


if __name__ == "__main__":
    main()
