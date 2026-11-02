# 配置文件说明（本目录与 DICE SDK 的 `config/` 目录对应）

> 第6组 · 桌面小管家 · 课程代码 (2026-2027-1)-S0610350-04 · 组员：张航宁 24061733 / 张浩轩 24061720 / 吴锐超 24061734

## 1. 为什么这里只有一份说明

DICE SDK 的代码在运行时，会从 `config_folder="config"` 这个目录读 3 个**标定产物**文件：

| 文件 | 谁在用 | 内容 | 本次是否随材料提供 |
|---|---|---|---|
| `z_error_polyfit.txt` | `arm5dof_uservo.py` 构造函数 | Z 轴误差补偿的一次拟合系数 `k,b`（两列，`np.loadtxt(..., delimiter=",")`） | ❌ 未提供 |
| `T_arm2ws.txt` | `arm_test.py` 构造函数 | 工作台坐标系 → 机械臂坐标系的 4×4 齐次变换矩阵（逗号分隔） | ❌ 未提供 |
| `object.yaml` | `arm_test.py` 构造函数 | 物块高度 `height`（以及抓取相关尺寸） | ❌ 未提供 |

交付材料里只包含 `.py`、`.ipynb`、`.STL` 与报告，**没有**这三个文件，所以本目录只放这份说明，
不放凭猜测填出来的数值——写错一个标定值，机械臂就会以错误位姿抓取，属于必须如实说明的事情。

## 2. 缺文件时的现象

直接运行 SDK / notebook 会得到（这是预期行为，不是代码 bug）：

```text
FileNotFoundError: [Errno 2] No such file or directory: 'config/z_error_polyfit.txt'
FileNotFoundError: [Errno 2] No such file or directory: 'config/T_arm2ws.txt'
FileNotFoundError: [Errno 2] No such file or directory: 'config/object.yaml'
```

## 3. 如何重新生成（在实验箱上做一遍即可）

1. **`z_error_polyfit.txt`**：把探针/标定块放在工作台几个已知高度，记录 `set_tool_pose` 目标 z 与
   TF 实测 z 的差值 Δz，做一次线性拟合 `Δz = k·z + b`，把 `k,b` 写成一行逗号分隔的两列：

   ```text
   k,b
   ```

   报告里引用的补偿模型就是这条一次多项式（`z_error_polyfit`）。

2. **`T_arm2ws.txt`**：把 ArUco/标定板放在工作台原点，分别读机械臂基坐标系下的位姿与工作台坐标系下的位姿，
   `T_arm2ws = T_arm^-1 · T_ws`，写成 4 行 4 列逗号分隔的矩阵。

3. **`object.yaml`**：物块高度（米），以及抓取时需要的偏移量，例如：

   ```yaml
   height: 0.03
   ```

   注意 `arm_test.py` 会读 `object_config["height"]`，缺这个键会 KeyError。

## 4. 目录该怎么放

`arm5dof_uservo.py` / `arm_test.py` 里默认 `config_folder="config"`，即**相对于运行目录**的 `config/`。
所以在 Jetson 上跑 notebook 时的推荐布局是：

```text
~/arm2025_ws/src/desktop_arm_description/
├─ src/arm_sdk/        ← 把 .py 放这里后，notebook 与本目录同级运行
└─ config/             ← 放 z_error_polyfit.txt / T_arm2ws.txt / object.yaml
```

即：先在 `src/arm_sdk/` 里 `jupyter notebook`（或把 `src/arm_sdk` 加入 `PYTHONPATH`），
再在**上一级**建 `config/`；也可以直接改 `Arm5DoFUServo(config_folder="你的路径")`。
