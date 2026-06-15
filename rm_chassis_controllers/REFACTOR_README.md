# Bipedal Wheel Controller State Machine Refactoring Guide

本指南记录了将 `bipedal_wheel_controller` 中的状态机 (`controller_mode`) 从 ROS 强耦合重构为纯 C++ (ROS-independent) 模块的完整流程。您可以按照以下步骤自行重现此重构过程。

## 目标
实现核心控制逻辑与 ROS 中间件的解耦，使得状态机可以独立编译、在非 ROS 环境中进行单元测试，并能直接移植到 ROS 2 或其他嵌入式/仿真平台。

---

## 阶段 1：定义解耦的数据结构与接口

首先，我们需要切断状态机内部对 ROS 特有类型（如 `ros::Time`, `geometry_msgs` 等）和控制库 (`control_toolbox::Pid`) 的依赖。

### 1.1 修改基础定义 (`definitions.h`)
* 引入 `<Eigen/Dense>`。
* 将 `ChassisState` 结构体中的 `geometry_msgs::Vector3` 替换为 `Eigen::Vector3d`。

### 1.2 创建逻辑输入输出结构 (`logic_types.h`)
新建 `include/.../bipedal_wheel_controller/logic_types.h`：
* **`UserInstruction`**: 封装用户的指令（期望 X 速度、Yaw 速度、腿长、跳跃指令）。
* **`LogicInput`**: 状态机的唯一输入源。包含：
  * `time`, `period` (使用 `double` 类型，单位秒)
  * `ChassisState`, 左右 `LegState`
  * 左右轮速 `left_wheel_vel`, `right_wheel_vel`
  * 所有通过智能指针传递的参数集 (`ModelParams`, `ControlParams`, `BiasParams`, `SpringParams` 等)
  * 其他控制标志位（`base_state`, `overturn`, `complete_stand` 等）
* **`LogicOutput`**: 状态机的输出结果。包含：
  * 计算出的控制指令：`leg_cmd[2]` 和 `wheel_cmd[2]`
  * 状态切换请求标志：`next_mode`（-1 表示不切换）、`state_changed_requested`、`clear_status_requested` 等，供外层包装器处理。

### 1.3 抽象 PID 接口 (`pid_interface.h`)
新建 `include/.../bipedal_wheel_controller/pid_interface.h`：
* 定义纯虚基类 `PidInterface`，包含 `virtual double computeCommand(double error, double period) = 0;`。
* 定义 `PidSet` 结构体，打包所有状态机所需的 `PidInterface*` 指针，方便传参。

### 1.4 提取数学助手函数 (`math_helpers.h`)
新建 `include/.../bipedal_wheel_controller/math_helpers.h`，将散落在各处的数学计算抽取出来：
* `clamp(val, min, max)`
* `shortest_angular_distance(from, to)`
* `f_spring_force(...)`
* `get_LM(l)` 和 `get_theta_leg_offset(l)`

---

## 阶段 2：重构基类与管理器

### 2.1 修改 `ModeBase` (`mode_base.h` / `mode_base.cpp`)
* 移除 `BipedalControllerInterface*` 成员变量（彻底切断对控制器大类的依赖）。
* 修改核心接口：`virtual LogicOutput execute(const LogicInput& input) = 0;`

### 2.2 修改 `ModeManager` (`mode_manager.h` / `mode_manager.cpp`)
* 构造函数改为接收 `const PidSet& pids`，不再接收 ROS NodeHandle 或硬件句柄。
* 提供 `LogicOutput execute(const LogicInput& input)` 方法，直接转发给当前的 `mode_impl`。
* 内部实例化所有模式子类时，传入 `PidSet` 中对应的指针。

---

## 阶段 3：重构所有模式子类 (核心工作)

对 `Normal`, `StandUp`, `Protect`, `Recover`, `SitDown`, `Upstairs` 执行以下一致的重构：

1. **构造函数**：将原先的 `control_toolbox::Pid*` 替换为 `PidInterface*`。
2. **`execute` 方法**：
   * 签名改为 `LogicOutput execute(const LogicInput& input) override;`
   * 创建局部的 `LogicOutput output;` 并最终返回它。
   * 将所有 `controller->getXXX()` 替换为从 `input` 中读取（例如 `input.chassis_state`）。
   * 将所有修改控制器状态的操作（例如 `controller->setMode(...)` 或 `controller->setStateChange(...)`）替换为设置 `output` 中的对应标志位。
   * 将计算出的力矩不再直接发送给硬件 (`setJointCommands`)，而是赋值给 `output.leg_cmd` 和 `output.wheel_cmd`。

> **⚠️ 关键 Debug 修复（防止 Gazebo 物理崩溃）：**
> 在重写调用 `f_spring_force` 时，原代码中传参存在隐患，重构时必须严格修正，否则会导致 Gazebo 的 TF 崩溃和模型坍缩：
> * 必须传入真实的连杆长度 `vmc->getL1()` 和 `vmc->getL2()`，而不是统一的 `model_params_->l`。
> * **在 `math_helpers.h` 中必须为 `f_spring_force` 增加安全保护**：
>   ```cpp
>   double cos_theta3 = (l1 * l1 + l2 * l2 - L0 * L0) / (2 * l1 * l2);
>   clamp(cos_theta3, -1.0, 1.0); // 必须 clamp，否则浮点误差会导致 acos 报 NaN
>   double theta3 = acos(cos_theta3);
>   double ls = sqrt(...);
>   double sin_theta3 = sin(theta3);
>   if (ls == 0.0 || sin_theta3 == 0.0) return 0.0; // 防止除 0 导致 NaN
>   ```
> * 在 `Normal::calculateSupportForce` 中加入周期保护：`if (period <= 0.0) period = 0.001;`

> **⚠️ 状态枚举硬编码修正：**
> 在原先脱离了 ROS 宏的环境中，如果需要判断 `FALLEN` 状态，请确保与 `rm_msgs::ChassisCmd` 保持一致。`FALLEN` 的值是 **4** 而不是 2。

---

## 阶段 4：实现 ROS 包装层 (`BipedalController`)

在 `controller.h` 和 `controller.cpp` 中实现 ROS 到纯逻辑的桥梁。

### 4.1 实现 `RosPidWrapper`
```cpp
class RosPidWrapper : public PidInterface {
public:
  RosPidWrapper(control_toolbox::Pid* pid) : pid_(pid) {}
  double computeCommand(double error, double period) override {
    return pid_->computeCommand(error, ros::Duration(period));
  }
private:
  control_toolbox::Pid* pid_;
};
```

### 4.2 更新 `BipedalController`
* **成员变量**：声明大量的 `std::unique_ptr<RosPidWrapper>` 实例，用于包裹 `control_toolbox::Pid` 实例。
* **初始化 (`init`)**：
  * 在加载完 PID 参数后，实例化所有的 Wrapper。
  * 将所有 Wrapper 指针打包进 `PidSet`，用其初始化 `ModeManager`。
* **主控循环 (`moveJoint`)**：
  1. 调用 `updateEstimation` 获取最新状态。
  2. 构造 `LogicInput` 结构体，将时间、硬件状态 (`chassis_state_`, `leg_state_`，轮速)、用户指令（订阅的话题）、各种参数指针打包。
  3. 调用 `LogicOutput output = mode_manager_->execute(input);`。
  4. 解析 `output`：
     * 根据 `output.next_mode` 调用 `mode_manager_->switchMode(...)`。
     * 处理 `output.state_changed_requested` 等标志位，同步更新 `BipedalController` 的内部状态。
     * 调用 `setJointCommands(joint_handles_, output.leg_cmd[LEFT], ...)` 下发给真实（或仿真）硬件接口。

### 4.3 修正遗留的语法错误
* 将 `chassis_state_.angular_vel.z` 修正为 `chassis_state_.angular_vel.z()`（因为现在是 Eigen 类型）。
* 确保移除了各个 `cpp` 文件中因 `-Werror=unused-variable` 导致的未使用的局部变量（例如没用上的 `model_params_`）。

---
遵循上述流程，即可将 `bipedal_wheel_controller` 中的状态机完全剥离成高内聚、低耦合、易于测试的核心算法模块。