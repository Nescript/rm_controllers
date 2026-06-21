//
// Created by Antigravity on 2026-06-21.
//

#pragma once
#include <urdf_model/joint.h>
#include <Eigen/Dense>

namespace bipedal_wheel_core
{

// 机器人状态机枚举
enum class RobotMode
{
  HANGING = 0, // 挂起/悬空状态
  FALLEN,      // 倾覆/倒地状态
  GETTING_UP,  // 试图起立中
  STAND        // 平衡站立控制状态
};

// 左右腿枚举
enum LegIndex
{
  LEFT = 0,
  RIGHT = 1
};

// 物理与算法配置参数（由 ROS 包装层读取后传入）
struct LqrModelParams
{
  double L_weight;   // 虚拟腿长权重 (轮轴方向)
  double Lm_weight;  // 虚拟腿长权重 (质心方向)
  double l;          // 腿部松弛状态自然长度
  double m_w;        // 单侧轮子质量
  double m_p;        // 单侧腿部连杆质量
  double M;          // 机器人底盘机体质量
  double i_w;        // 轮子转动惯量
  double i_p;        // 腿部绕髋关节转动惯量
  double i_m;        // 机体绕俯仰轴转动惯量
  double r;          // 轮子半径
  double g;          // 重力加速度
  double f_gravity;  // 抵消重力的前馈力
};

struct ControllerParams
{
  LqrModelParams model_params{};
  // 您可以在此处添加 LQR 增益矩阵 (Q, R)、状态机阈值、滤波器协方差等其他参数
};

// 核心算法内部估计的机器人绝对状态（可通过 Getter 接口暴露供 ROS 观测）
struct LQRStatus
{
  double theta = 0.0;    // 虚拟腿绝对摆角（相对世界坐标系）
  double d_theta = 0.0;  // 虚拟腿绝对摆角速度
  double x = 0.0;        // 绝对水平位移
  double dx = 0.0;       // 绝对水平速度
  double pitch = 0.0;    // 底盘绝对俯仰角
  double d_pitch = 0.0;  // 底盘绝对俯仰角速度
};

// 纯粹的硬件执行器状态（输入）
struct JointState
{
  double position = 0.0;
  double vel = 0.0;
  double effort = 0.0;
};

struct LegState
{
  JointState hip;
  JointState knee;
  JointState wheel;
};

// 纯粹的底盘惯导及几何状态（输入）
struct ChassisSensorState
{
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero(); // 转换到 base_link 坐标系下的角速度
  Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();  // 转换到 base_link 坐标系下的线加速度
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// 最终汇总的传感器输入包
struct SensorMeasurements
{
  LegState leg_state[2]{};
  ChassisSensorState chassis_state{};
};

// 纯粹的关节控制器目标（输出指令）
struct JointCommand
{
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0; // 目标关节力矩
  double kp = 0.0;     // 刚度增益
  double kd = 0.0;     // 阻尼增益
};

struct LegCommand
{
  JointCommand hip;
  JointCommand knee;
  JointCommand wheel;
};

// 实时安全的调试数据包
struct DebugData
{
  double virtual_force = 0.0;
  double virtual_torque = 0.0;
  double lqr_error[6]{}; // 存储状态偏差量以供绘制曲线
  RobotMode mode = RobotMode::HANGING;
};

// 算法更新的最终输出包
struct ControlOutput
{
  LegCommand leg_cmd[2]{};
  DebugData debug_data{};
};

}  // namespace bipedal_wheel_core
