# Stereo ArUco Calibration

独立的 ROS 2 双目外参标定工具。标定器只依赖标准的 `sensor_msgs/msg/Image` 和
`sensor_msgs/msg/CameraInfo` 话题，不绑定具体相机驱动。C++ 标定器可以独立启动，也可以由
上层 bringup 包与相机驱动组合到同一个 `component_container_mt` 中；独立 GUI 只接收
缩放 JPEG 预览和轻量状态消息。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 标定板

默认配置使用 `DICT_6X6_50`，四个 20 cm ArUco 的中心距为水平 110 cm、垂直 70 cm：

```text
ID 1 --------- ID 2
 |              |
 |              | 0.70 m
 |              |
ID 3 --------- ID 4
       1.10 m
```

四个标记必须采用相同朝向。板坐标系原点位于标记中心的几何中心，`+x` 向右、
`+y` 向下。配置中的间距是中心距，不是边缘净距。

## 配置

- `config/calibration.yaml`：输入话题、ArUco 字典、板尺寸、采样条件、预览参数和输出路径。

左右相机内参由各自的 `CameraInfo` 话题提供。相机序列号、曝光、帧率、触发方式和内参文件
属于相机驱动或上层 bringup 包的配置，不由本工具管理。

## 启动 GUI 标定

```bash
ros2 launch stereo_aruco_calibration calibration.launch.py
```

启动前必须已有左右图像和内参话题。默认输入为：

```text
/left_camera/image_raw
/left_camera/camera_info
/right_camera/image_raw
/right_camera/camera_info
```

其他话题名称可在 `config/calibration.yaml` 中配置。图像和对应的 `CameraInfo` 必须具有一致的
分辨率和 `frame_id`，左右图像的 `header.stamp` 必须处于同一时间基准。

窗口显示：

- 左右识别图像；
- 绿色的左右共同 ArUco、黄色的单侧 ArUco；
- 有效样本数和求解门槛；
- 左右时间差和 PnP 重投影误差；
- 位置、距离和旋转覆盖度；
- 最近一次接受或拒绝原因；
- 求解后的 RMS、极线误差和基线。

按钮和快捷键：

| 操作     | 按钮  | 快捷键              |
| -------- | ----- | ------------------- |
| 开始采样 | START | Space（非采样状态） |
| 停止采样 | STOP  | Space（采样状态）   |
| 清空样本 | RESET | R                   |
| 求解外参 | SOLVE | C                   |
| 保存结果 | SAVE  | S                   |
| 退出 GUI | —     | Q / Esc             |

求解按钮在样本达到 `sampling.minimum_samples` 后启用；保存按钮在求解成功后启用。

无图形界面运行：

```bash
ros2 launch stereo_aruco_calibration calibration.launch.py gui:=false
```

## 命令行控制

GUI 和命令行调用同一组服务：

```bash
ros2 service call /stereo_calibration/start std_srvs/srv/Trigger {}
ros2 service call /stereo_calibration/stop  std_srvs/srv/Trigger {}
ros2 service call /stereo_calibration/reset std_srvs/srv/Trigger {}
ros2 service call /stereo_calibration/solve  std_srvs/srv/Trigger {}
ros2 service call /stereo_calibration/save   std_srvs/srv/Trigger {}
```

调试话题：

```text
/stereo_calibration/preview/compressed
/stereo_calibration/status
```

## 采集方法

对于自由运行的相机，采样器会要求左右 Marker 角点在完整稳定窗口内保持静止；界面的
`STABILIZING` 百分比达到 100% 后才会评估并接受样本。接受后状态切换为
`WAITING_FOR_MOTION`，需要先移动标定板，再在新姿态停稳。`pose_too_similar` 表示该姿态
与已有样本过于接近，不是识别失败。

STOP 只暂停样本采集，不暂停左右预览；START 会清空旧的配对与稳定窗口，但保留已接受样本。
稳定窗口由 `sampling.stability.*` 参数控制，默认要求至少 6 对图像、持续 0.4 秒且整个
窗口内左右角点 RMS 位移不超过 1 px。

建议采集 20–40 个多样姿态。至少满足配置中的 10 个样本后才能求解。

结果保存位置由 `config/calibration.yaml` 中的 `output_path` 指定，变换约定为：

```text
X_right = R_right_left * X_left + t_right_left
```

平移单位为米。工具不会自动修改其他项目的 URDF。
