# acados 生成版 MPCC 集成说明

本文档记录 `quad_midbridge_mpcc` 中 acados generated solver 的集成方式、运行检查方法和常见故障定位。当前目标不是替代 PX4 底层控制器，而是让 MPCC 作为路径条件化参考生成核心，为后级 governor 提供可解释、可验证的 raw reference。

运行成功时，forward 阶段的调试话题应出现：

```text
/midbridge/mpcc_debug.status == generated_code_solved
```

如果仍然出现 `acados_generated_fallback:`，说明节点已经回退到 fallback reference generator，需要继续排查 generated solver 的构建、链接或运行时环境。

## 1. OCP 模型

生成的 acados OCP 名称为：

```text
quad_midbridge_mpcc_ocp
```

状态量：

```text
x = [px, py, pz, vx, vy, vz, ax, ay, az, psi, s]
```

控制量：

```text
u = [jx, jy, jz, psi_dot, s_dot]
```

连续动力学：

```text
p_dot   = v
v_dot   = a
a_dot   = j
psi_dot = psi_dot
s_dot   = s_dot
```

代价函数主要包含：

```text
contour error
lag error
yaw error
speed error
acceleration
jerk
```

其中 `-w_progress * s_dot` 用于鼓励沿路径前进，但最终安全整形仍由 governor 完成。

不等式约束 `h` 与 C++ 端参数布局一致：

```text
h[0] contour_xy       in [0, corridor_half_width]
h[1] vertical_err     in [-corridor_half_height, corridor_half_height]
h[2] tilt             in [0, tilt_max_rad]
h[3] thrust           in [thrust_min, thrust_max]
h[4] psi_dot          in [-yaw_rate_max, yaw_rate_max]
h[5] s_dot            in [0 or -speed_max, speed_max]
h[6] speed_norm       in [0, speed_max]
h[7] accel_norm       in [0, accel_max]
h[8] jerk_norm        in [0, jerk_max]
```

flatness 相关约束使用的近似为：

```text
f_d = [-a_x, -a_y, g - a_z]
thrust = ||f_d||
tilt = atan2(||a_xy||, g - a_z)
```

这里的局部位置遵循 PX4 NED 约定，`z` 向下为正，因此高度上升对应更小的 `z`。

## 2. 生成 acados solver

在已经安装 acados Python 接口，并且 acados 本体位于 `/home/zjnujza/acados` 的情况下，可以执行：

```bash
cd /home/zjnujza/quad_midbridge_ws/src/quad_midbridge_mpcc
python3 tools/generate_mpcc_acados_solver.py \
  --horizon-steps 20 \
  --horizon-dt 0.05 \
  --export-dir third_party/mpcc_acados_generated/c_generated_code \
  --build
```

生成参数需要和 ROS 参数保持一致：

```yaml
horizon_steps: 20
horizon_dt: 0.05
```

如果修改 `horizon_steps` 或 `horizon_dt`，需要重新生成 solver，并重新构建 ROS 包。

## 3. 生成文件检查

生成完成后，应能看到：

```text
quad_midbridge_mpcc/
  third_party/mpcc_acados_generated/
    c_generated_code/
      acados_solver_quad_midbridge_mpcc_ocp.h
      libacados_ocp_solver_quad_midbridge_mpcc_ocp.so
```

其中 header 用于 C++ 编译，shared library 用于运行时链接。

## 4. 构建 generated 后端

推荐在 workspace 根目录执行：

```bash
cd /home/zjnujza/quad_midbridge_ws
export ACADOS_SOURCE_DIR=/home/zjnujza/acados
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select quad_midbridge_mpcc \
  --cmake-args -DQUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED=ON
```

当前 CMake 会根据 `ACADOS_SOURCE_DIR` 自动补充：

```text
acados/include
acados/lib
acados/external/blasfeo/include
acados/external/hpipm/include
```

并链接以下库：

```text
acados_ocp_solver_quad_midbridge_mpcc_ocp
acados
hpipm
blasfeo
qpOASES_e
m
```

如果 acados 安装位置不同，可以显式传入：

```bash
-DQUAD_MIDBRIDGE_MPCC_ACADOS_INCLUDE_DIR=/path/to/acados/include
-DQUAD_MIDBRIDGE_MPCC_ACADOS_LIB_DIR=/path/to/acados/lib
-DQUAD_MIDBRIDGE_MPCC_ACADOS_EXTRA_INCLUDE_DIRS=/path/to/blasfeo/include;/path/to/hpipm/include
```

## 5. 运行前环境

建议使用工作区里的环境脚本：

```bash
source /home/zjnujza/quad_midbridge_ws/setup_midbridge_acados.bash
```

该脚本应至少保证：

```text
ACADOS_SOURCE_DIR=/home/zjnujza/acados
LD_LIBRARY_PATH 包含 acados/lib
ROS 环境和当前 workspace install 已 source
```

## 6. 快速冒烟测试

不启动完整 PX4/SITL 时，可以先确认节点能加载 generated 后端：

```bash
cd /home/zjnujza/quad_midbridge_ws
source setup_midbridge_acados.bash
timeout 3s install/quad_midbridge_mpcc/lib/quad_midbridge_mpcc/mpcc_node \
  --ros-args -p solver_backend:=acados_generated
```

如果输出类似下面内容，说明运行时动态库已经能被找到：

```text
mpcc_node started with backend=acados_generated
```

完整运行时，需要在 forward 阶段检查：

```bash
ros2 topic echo --once /midbridge/mpcc_debug
```

目标状态为：

```text
status: generated_code_solved
```

## 7. 常见故障

### 7.1 generated_bindings_not_compiled

含义：ROS 包没有以 `QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED=ON` 构建，或生成的 header/source 没有被编译进 `quad_midbridge_mpcc`。

处理：

```bash
export ACADOS_SOURCE_DIR=/home/zjnujza/acados
colcon build --symlink-install \
  --packages-select quad_midbridge_mpcc \
  --cmake-args -DQUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED=ON
```

### 7.2 找不到 acados_solver_quad_midbridge_mpcc_ocp.h

含义：生成代码目录不存在，或 CMake 没有找到正确的 generated include 路径。

处理：重新运行 `tools/generate_mpcc_acados_solver.py`，并检查 `third_party/mpcc_acados_generated/c_generated_code`。

### 7.3 找不到 libacados_ocp_solver_quad_midbridge_mpcc_ocp.so

含义：生成 solver 的 shared library 未生成，或运行时 `LD_LIBRARY_PATH`/RPATH 没覆盖到该目录。

处理：使用 `--build` 重新生成 solver，然后重新构建 ROS 包。

### 7.4 找不到 libacados.so、libhpipm.so、libblasfeo.so 或 libqpOASES_e.so

含义：acados 本体库路径没有进入链接或运行时搜索路径。

处理：确认：

```bash
export ACADOS_SOURCE_DIR=/home/zjnujza/acados
ls /home/zjnujza/acados/lib
```

然后重新构建，并在运行前 source `setup_midbridge_acados.bash`。

### 7.5 create_failed 或 solve_failed

含义：generated solver 已经被编译和加载，但 OCP 创建、参数写入或求解失败。

处理顺序：

```text
1. 检查 horizon_steps / horizon_dt 是否与生成 solver 一致
2. 检查参数维度是否与 Python 生成脚本一致
3. 检查 initial state、path samples、bounds 是否存在 NaN/Inf
4. 查看 /midbridge/mpcc_debug.status 的详细 fallback 原因
```

## 8. 当前验证状态

当前代码已经完成以下验证：

```text
默认 fallback 构建通过
generated-enabled 构建通过
mpcc_node 可以以 solver_backend:=acados_generated 启动
离线 smoke test 出现 generated_code_solved
```

下一步应在 PX4/SITL 完整链路中确认 forward 阶段不再回退，并把新的 generated backend bag 与之前 fallback 稳定基线对比。
