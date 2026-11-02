# -*- coding: utf-8 -*-
# =====================================================================
# 【兼容层｜新增文件，非原始文件】
# 文件名 : arm_test_origin.py
# 背景   : 实验六《我的垃圾分类.ipynb》第 3 个 cell 写的是
#              from arm_test_origin import ArmApplication
#          而实验三/实验四的 notebook 写的是
#              from arm_test import ArmApplication
#          交付材料里 arm_test_origin.py 与 arm_test.py 的 MD5 完全相同
#          （9bea1dc50c6b6bbff9a908bf82c3edfa），只是文件名不同。
# 处理   : 本包只保留一份实体 arm_test.py（任务书目录要求），另用本文件做
#          一行 re-export，好处是：① 不改动原始 notebook 一个字节；
#          ② 两份文件永远保持同一份实现，不会出现「改了 A 忘了 B」。
# =====================================================================
from arm_test import *          # noqa: F401,F403  —— 原样转发 ArmApplication
from arm_test import ArmApplication  # noqa: F401  —— 显式导入，便于 IDE 跳转
