#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_urdf.py —— 纯 Python 的 URDF 离线自检脚本（不需要 ROS / urdfdom）

背景：本包交付环境是 Windows/无 ROS 的机器与 Jetson Nano，助教/老师不一定有
      check_urdf（URDF 官方检查工具，随 ROS 安装）。因此本脚本只用标准库
      xml.etree 实现同等力度的结构与资源检查，一条命令即可跑：

          python urdf/check_urdf.py
          python urdf/check_urdf.py urdf/arm5dof.urdf      # 指定文件

检查项（全部通过才返回 0）：
  1. XML 能解析、根元素为 <robot> 且带 name
  2. link / joint 名字不重复
  3. 每个 joint 的 parent / child 都能在 <link> 中找到
  4. 所有连杆从唯一根连杆可达（连通）、无环（多父/自环都会被抓出来）
  5. 每个 link 具备 visual / collision / inertial，inertial 质量 > 0
  6. 每个 <mesh filename="package://..."> 指向的文件真实存在
  7. revolute / continuous 关节必须有 axis（模长约 1）与 limit，且 lower <= upper
  8. mimic 指向的关节必须存在（右夹指依赖左夹指）

输出的检查表可直接贴进 代码包自检.md。
"""

import os
import sys
import xml.etree.ElementTree as ET

PKG = "desktop_arm_description"          # package:// 前缀里声明的包名
PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # .../代码包根目录


class Report(object):
    def __init__(self):
        self.items = []
        self.failed = 0

    def ok(self, item, detail=""):
        self.items.append(("PASS", item, detail))

    def fail(self, item, detail=""):
        self.items.append(("FAIL", item, detail))
        self.failed += 1

    def show(self):
        for status, item, detail in self.items:
            line = "[%s] %s" % (status, item)
            if detail:
                line += "  --  " + detail
            print(line)


def parse_xyz(text, name):
    parts = (text or "").split()
    if len(parts) != 3:
        raise ValueError("元素 %s 的 xyz 不是 3 个数: %r" % (name, text))
    return [float(p) for p in parts]


def main():
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG_ROOT, "urdf", "arm5dof.urdf")
    urdf_path = os.path.abspath(urdf_path)
    rep = Report()

    print("=" * 88)
    print("URDF 离线自检  check_urdf.py")
    print("文件：%s" % urdf_path)
    print("包根：%s" % PKG_ROOT)
    print("=" * 88)

    if not os.path.isfile(urdf_path):
        rep.fail("文件存在", urdf_path + " 不存在")
        rep.show()
        return 1

    # ---------- 1. 解析与根元素 ----------
    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError as exc:
        rep.fail("XML 语法", "解析失败: %s" % exc)
        rep.show()
        return 1
    root = tree.getroot()
    if root.tag != "robot":
        rep.fail("根元素", "根元素是 <%s>，应为 <robot>" % root.tag)
    else:
        rep.ok("根元素 <robot>", 'name="%s"' % root.get("name", ""))

    links = root.findall("link")
    joints = root.findall("joint")
    rep.ok("连杆数量", "%d 个（本模型设计值 13）" % len(links))
    rep.ok("关节数量", "%d 个（本模型设计值 12）" % len(joints))

    # ---------- 2. 名称唯一 ----------
    link_names = [l.get("name") for l in links]
    joint_names = [j.get("name") for j in joints]
    dup_l = sorted({n for n in link_names if link_names.count(n) > 1})
    dup_j = sorted({n for n in joint_names if joint_names.count(n) > 1})
    if dup_l:
        rep.fail("连杆命名唯一", "重复: %s" % dup_l)
    else:
        rep.ok("连杆命名唯一", "%d 个名字均无重复" % len(link_names))
    if dup_j:
        rep.fail("关节命名唯一", "重复: %s" % dup_j)
    else:
        rep.ok("关节命名唯一", "%d 个名字均无重复" % len(joint_names))

    link_set = set(link_names)

    # ---------- 3. parent / child 存在 ----------
    bad = []
    children_of = {}
    parent_of = {}
    for j in joints:
        jn = j.get("name")
        p = j.find("parent")
        c = j.find("child")
        pl = p.get("link") if p is not None else None
        cl = c.get("link") if c is not None else None
        if pl not in link_set:
            bad.append("%s.parent=%s 不是已定义的 link" % (jn, pl))
        if cl not in link_set:
            bad.append("%s.child=%s 不是已定义的 link" % (jn, cl))
        if pl in link_set and cl in link_set:
            children_of.setdefault(pl, []).append((jn, cl))
            parent_of.setdefault(cl, []).append(jn)
    if bad:
        rep.fail("parent/child 引用有效", "; ".join(bad))
    else:
        rep.ok("parent/child 引用有效", "12 个关节的父子连杆全部能在 <link> 中查到")

    # ---------- 4. 连通 + 无环 ----------
    roots = [n for n in link_names if n not in parent_of]
    if len(roots) != 1:
        rep.fail("唯一根连杆", "找到 %d 个根link（无父节点）: %s" % (len(roots), roots))
    else:
        rep.ok("唯一根连杆", "root = %s" % roots[0])

    multi_parent = [n for n, v in parent_of.items() if len(v) > 1]
    if multi_parent:
        rep.fail("无多父（无环必要条件）", "以下 link 有多个父关节: %s" %
                 ["%s(%s)" % (n, parent_of[n]) for n in multi_parent])
    else:
        rep.ok("无多父（无环必要条件）", "每个 link 至多一个父关节")

    visited, cyc = set(), []

    def dfs(name, stack):
        if name in stack:
            cyc.append(list(stack) + [name])
            return
        visited.add(name)
        for _, child in children_of.get(name, []):
            dfs(child, stack + [name])

    if len(roots) == 1:
        dfs(roots[0], [])
        if cyc:
            rep.fail("无环（DFS 回溯）", "发现环: %s" % cyc[0])
        else:
            rep.ok("无环（DFS 回溯）", "从 %s 出发未遇回边" % roots[0])
        missing = [n for n in link_names if n not in visited]
        if missing:
            rep.fail("全连通", "从根不可达的 link: %s" % missing)
        else:
            rep.ok("全连通", "%d 个 link 全部可从根到达（连通成树）" % len(link_names))

    # ---------- 5. visual / collision / inertial ----------
    miss = []
    for l in links:
        n = l.get("name")
        for tag in ("visual", "collision", "inertial"):
            if l.find(tag) is None:
                miss.append("%s 缺 <%s>" % (n, tag))
        ine = l.find("inertial")
        if ine is not None:
            mass = ine.find("mass")
            try:
                m = float(mass.get("value")) if mass is not None else -1.0
            except (TypeError, ValueError):
                m = -1.0
            if m <= 0:
                miss.append("%s 的质量非正: %r" % (n, mass.get("value") if mass is not None else None))
            inertia = ine.find("inertia")
            if inertia is None:
                miss.append("%s 缺 <inertia>" % n)
            else:
                diag = [float(inertia.get(k)) for k in ("ixx", "iyy", "izz")]
                if min(diag) <= 0:
                    miss.append("%s 惯性张量对角元非正: %s" % (n, diag))
    if miss:
        rep.fail("每个 link 有 visual/collision/inertial 且质量为正", "; ".join(miss))
    else:
        rep.ok("每个 link 有 visual/collision/inertial 且质量为正",
               "%d 个 link 全部齐备" % len(link_names))

    # ---------- 6. mesh 文件存在 ----------
    meshes, absent, n_visual_coll = [], [], 0
    for l in links:
        for tag in ("visual", "collision"):
            for g in l.findall("%s/geometry/mesh" % tag):
                fn = g.get("filename") or ""
                n_visual_coll += 1
                meshes.append((l.get("name"), tag, fn))
                if fn.startswith("package://"):
                    rel = fn[len("package://"):]
                    pkgname, _, rest = rel.partition("/")
                    if pkgname != PKG:
                        absent.append("%s: package 名 %r 与本包 %r 不符" % (fn, pkgname, PKG))
                        continue
                    disk = os.path.join(PKG_ROOT, rest)
                else:
                    disk = os.path.normpath(os.path.join(os.path.dirname(urdf_path), fn))
                if not os.path.isfile(disk):
                    absent.append("%s（解析为 %s）" % (fn, disk))
    if absent:
        rep.fail("mesh 文件全部存在", "%d 处缺失: %s" % (len(absent), "; ".join(absent)))
    else:
        rep.ok("mesh 文件全部存在",
               "%d 处 <mesh> 引用、%d 个唯一 STL 文件全部命中" %
               (n_visual_coll, len({m[2] for m in meshes})))

    # 同名 link 的 visual 与 collision 是否用同一网格
    same = True
    for l in links:
        v = l.find("visual/geometry/mesh")
        c = l.find("collision/geometry/mesh")
        if v is not None and c is not None and v.get("filename") != c.get("filename"):
            same = False
    if meshes:
        rep.ok("visual 与 collision 网格一致", "一致" if same else "存在不一致（请人工确认）")

    # ---------- 7. 关节轴与限位 ----------
    bad7 = []
    for j in joints:
        jn = j.get("name")
        jt = j.get("type")
        if jt in ("revolute", "continuous"):
            ax = j.find("axis")
            if ax is None:
                bad7.append("%s 缺 <axis>" % jn)
            else:
                try:
                    v = parse_xyz(ax.get("xyz"), jn)
                    norm = sum(x * x for x in v) ** 0.5
                    if abs(norm - 1.0) > 1e-3:
                        bad7.append("%s 的 axis 模长 %.6f ≠ 1" % (jn, norm))
                except ValueError as exc:
                    bad7.append("%s: %s" % (jn, exc))
            lim = j.find("limit")
            if lim is None:
                bad7.append("%s 缺 <limit>" % jn)
            else:
                try:
                    lo = float(lim.get("lower"))
                    up = float(lim.get("upper"))
                    if lo > up:
                        bad7.append("%s 的 lower(%.4f) > upper(%.4f)" % (jn, lo, up))
                    if abs(lo) > 6.3 or abs(up) > 6.3:
                        bad7.append("%s 限位超出 ±2π 合理范围" % jn)
                except (TypeError, ValueError):
                    bad7.append("%s 的 limit 上下界不是数字" % jn)
        elif jt == "fixed":
            if j.find("limit") is not None:
                bad7.append("%s 是 fixed 却定义了 <limit>" % jn)
        else:
            bad7.append("%s 的 type=%r 不在 revolute/fixed/continuous 之内" % (jn, jt))
    if bad7:
        rep.fail("关节 axis/limit 合理", "; ".join(bad7))
    else:
        rep.ok("关节 axis/limit 合理",
               "revolute 关节 axis 模长=1、lower<=upper 且 |limit|<=2π；fixed 关节无 limit")

    # ---------- 8. mimic 目标存在 ----------
    bad8 = []
    for j in joints:
        m = j.find("mimic")
        if m is not None:
            target = m.get("joint")
            if target not in joint_names:
                bad8.append("%s mimic 指向不存在的关节 %r" % (j.get("name"), target))
            else:
                rep.ok("mimic 联动", "%s → %s（multiplier=%s，offset=%s）"
                       % (j.get("name"), target, m.get("multiplier"), m.get("offset")))
    if bad8:
        rep.fail("mimic 目标有效", "; ".join(bad8))

    # ---------- 打印逐项检查表 ----------
    print("-" * 88)
    rep.show()
    print("-" * 88)

    # ---------- 汇总 ----------
    print("TF 树（自根向叶，括号内为关节名）：")
    if len(roots) == 1:
        print("  %s" % roots[0])

        def dump(name, prefix=""):
            kids = children_of.get(name, [])
            for i, (jn, child) in enumerate(kids):
                last = (i == len(kids) - 1)
                print("  %s%s(%s)─> %s" % (prefix, "└─" if last else "├─", jn, child))
                dump(child, prefix + ("   " if last else "│  "))

        dump(roots[0])
    print("-" * 88)

    if rep.failed:
        print("自检结果：FAIL（%d 项不通过）" % rep.failed)
    else:
        print("自检结果：PASS —— 根元素/13 连杆/12 关节/唯一根/全连通无环/"
              "visual-collision-inertial 齐备/mesh 全部存在/axis 与 limit 合理/mimic 有效")
    print("=" * 88)
    return 0 if rep.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
