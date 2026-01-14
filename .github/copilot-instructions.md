# PX4-Avoidance（ROS1/catkin）AI 编码指引

## 环境版本（以 Noetic 为主）
- 本地开发/运行以 **Ubuntu 20.04 + ROS Noetic** 为主（见 [README.md](../README.md)）。
- CI（Jenkins）使用 `px4io/px4-dev-ros-melodic:*` 镜像构建（见 [Jenkinsfile](../Jenkinsfile)）；如果改动涉及 ROS API/依赖版本差异，需同时考虑 Noetic/Melodic 兼容性。

## 工程大图景（先选一个 planner）
- 这是 PX4 避障/规划算法的 ROS1 封装，包含 4 个 catkin 包：`avoidance/`（共享库）、`local_planner/`（VFH+*）、`global_planner/`（OctoMap 全局图）、`safe_landing_planner/`（安全降落）。三种 planner **互相独立**，不要混用同一套运行链路（见 [README.md](../README.md)）。
- 共享依赖与通用工具主要在 `avoidance/`：例如 `AvoidanceNode` 负责 MAVROS 参数/任务信息与 companion 状态上报（见 [avoidance/src/avoidance_node.cpp](../avoidance/src/avoidance_node.cpp)）。

## 构建工作流（catkin_tools + wstool）
- 推荐使用 `catkin_tools`（README/CI 都是该路径）：在 catkin 工作区执行 `catkin build`。
- CI（Jenkins）典型构建方式：在 `catkin_ws/src/avoidance` 下用 `wstool init src src/avoidance/dependencies.rosinstall && wstool update -t src` 拉取额外依赖（见 [Jenkinsfile](../Jenkinsfile) 与 [dependencies.rosinstall](../dependencies.rosinstall)）。
- 可选编译开关：`-DDISABLE_SIMULATION` 会关闭 Gazebo/可视化相关代码路径（多个包的 `CMakeLists.txt` 都会检查该宏）。

## 运行/集成要点（MAVROS + TF + 点云）
- `local_planner` 默认以 nodelet 运行：插件名 `LocalPlannerNodelet`（见 [local_planner/nodelets.xml](../local_planner/nodelets.xml)），可执行入口用 `nodelet::Loader` 加载（见 [local_planner/src/nodes/local_planner_node_main.cpp](../local_planner/src/nodes/local_planner_node_main.cpp)）。
- 坐标系约定：常用机体系 frame 为 `fcu`，相机到 `fcu` 的静态 TF 通常在 launch 中声明（例：[local_planner/launch/local_planner_sitl_3cam.launch](../local_planner/launch/local_planner_sitl_3cam.launch)）。
- 点云订阅由参数驱动：`LocalPlannerNodelet::readParams()` 读取 `pointcloud_topics` 并按数组创建多个订阅与 transform 线程（见 [local_planner/src/nodes/local_planner_nodelet.cpp](../local_planner/src/nodes/local_planner_nodelet.cpp)）。修改相机数量/话题时优先改 launch 的 `rosparam`。
- MAVROS topic/消息流在 README 有完整表格（见 [README.md](../README.md) 的 “Message Flows”）：例如 local planner 订阅 `mavros/local_position/pose`、发布 `mavros/obstacle/send`（`sensor_msgs/LaserScan`）与 `mavros/companion_process/status`。

## 项目内“改一处要跟着改”的约定
- PX4 运动学/规划相关参数结构体是 `ModelParameters`（见 [avoidance/include/avoidance/common.h](../avoidance/include/avoidance/common.h)）。如果新增/修改 PX4 参数：
  - 更新 `AvoidanceNode::px4ParamsCallback()` 的解析分支；
  - 更新 `AvoidanceNode::checkPx4Parameters()` 的定期 `ParamGet` 请求（见 [avoidance/src/avoidance_node.cpp](../avoidance/src/avoidance_node.cpp)）。
- 动态参数使用 `dynamic_reconfigure`：cfg 位于 `*/cfg/*.cfg`，server 在节点/ nodelet 的 `onInit()` 中绑定回调（例：`local_planner` 见 [local_planner/CMakeLists.txt](../local_planner/CMakeLists.txt) 与 [local_planner/src/nodes/local_planner_nodelet.cpp](../local_planner/src/nodes/local_planner_nodelet.cpp)）。

## 代码风格与测试
- 代码风格：Google 风格、`ColumnLimit: 120`。仓库脚本会调用 `clang-format-3.8`（见 [tools/fix_style.sh](../tools/fix_style.sh) 与 [tools/check_code_format.sh](../tools/check_code_format.sh)）。
- 单元测试：各包使用 `catkin_add_gtest`（`*/test/`）。覆盖率脚本示例见 [tools/generate_coverage.sh](../tools/generate_coverage.sh)。

## 常用命令速查（在你的 catkin_ws 下执行）
- 构建：`catkin build`（或 `catkin build local_planner`）
- 测试：`catkin test local_planner && catkin test avoidance`（需要启用测试配置）
- 启动仿真（示例）：`roslaunch local_planner local_planner_sitl_3cam.launch`
- 代码格式化：`./tools/fix_style.sh ..`

## 常见排障入口（优先按这几步查）
- RViz 里只有飞机/无环境：先看是否有点云话题与数据：`rostopic list | grep camera`、`rostopic echo /camera/depth/points`（见 [README.md](../README.md) “Troubleshooting”）。
- Gazebo 没发布仿真时间：`rostopic echo /clock`；没有输出通常说明 Gazebo/world 没正常起来。
- local_planner 无动作/不避障：确认 PX4 进入 `OFFBOARD`/已解锁；并检查 `mavros/local_position/pose` 是否有数据（见 [README.md](../README.md) “Troubleshooting” / “Message Flows”）。
- 想看更详细日志：launch 已通过 `ROSCONSOLE_CONFIG_FILE` 指向 [local_planner/resource/custom_rosconsole.conf](../local_planner/resource/custom_rosconsole.conf)（例：[local_planner/launch/local_planner_sitl_3cam.launch](../local_planner/launch/local_planner_sitl_3cam.launch)）；把其中 `log4j.logger.ros.local_planner` 设为 `DEBUG`。
