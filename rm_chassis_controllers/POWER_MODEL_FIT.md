# 功率辨识全局指标

`/power_model/fit_metrics`（`rm_msgs/PowerModelFit`）与 CSV 保存同一快照：RMSE（W）、R²、bias（W，正值为高估）、当前生效的 12 维 c、样本数及模型版本。原 `/power_model/c` 保留，采用锁存发布，并按指标发布频率刷新，供 manual 做超时保护。

全局范围是模型 init 以来所有通过时间对齐、工况、功率范围和有限值检查的入模样本。静止样本按 `stationary_sample_period` 降采样保留，用于约束 c0。用当前已接受的 c 重新评估整段历史，不施加训练用的遗忘权重。只有同时通过激励矩阵、RMSE 和 R² 门槛的模型才发布并增加 `model_version`。指标属于样本内拟合质量。

消息 `header.stamp` 是最后一个入模样本时间，`last_solve_stamp` 是最后成功求解时间。没有有效模型时 metrics_valid=false，浮点指标为 NaN；实测功率方差近零时 r2_valid=false，R² 为 NaN。`last_solve_success` 表示最近一次矩阵求解是否成功，`model_accepted` 表示该结果是否同时通过质量门槛并成为当前生效模型。

控制器参数（无需修改配置即可使用默认值）：

```yaml
power_model:
  identification_enabled: true
  require_normal_state: true
  reject_protected_samples: true
  lambda: 0.999
  alpha_ridge: 0.001
  filter_time_constant: 0.02
  measurement_delay: 0.0
  max_alignment_error: 0.05
  stationary_sample_period: 0.2
  min_measurement: 0.0
  max_measurement: 500.0
  minimum_samples: 500
  min_normalized_eigenvalue: 1.0e-6
  min_r2: 0.5
  max_rmse: 20.0
  metrics_rate: 1.0
  log_enabled: true
  # initial_coefficients: [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11]
  # log_directory: /absolute/path/power_model
```

`require_normal_state=true` 时仅接受正常站立、FOLLOW、未跳跃、未翻倒且未打滑的样本。`initial_coefficients` 用于加载已审核的回退模型；该模型会通过锁存的 `/power_model/c` 立即提供给晚启动或重启的 manual。在线候选未通过质量门槛时继续保留上一组已接受系数。

manual 会发布 `/power_limit/active`、`/power_limit/scale`、`/power_limit/allowed_power` 和 `/power_limit/predicted_command_power`，可直接检查使能状态、降额系数、动态功率上限与限幅前预测功率。任一裁判/电容输入或模型系数超时、非法，或底盘姿态不为 NORMAL 时，速度模型降额自动旁路，原有底盘安全控制仍保留。

功率订阅回调触发指标发布，以墙钟限频，最高 metrics_rate Hz；没有功率消息时不额外发布。话题保留最新消息。CSV 默认在 `$ROS_LOG_DIR/power_model`，未设置时使用 `$ROS_HOME/log/power_model` 或 `$HOME/.ros/log/power_model`，实际路径在启动日志中输出。每次 init 生成带随机后缀的会话文件。

CSV 由独立线程写入，每行 flush；析构时排空队列。控制器 stop 不关闭文件，后续同一实例继续使用同一辨识会话。最多缓存 256 条快照，队列满丢弃新记录并报错，磁盘错误停止记录并报错，话题发布继续。记录的是限频快照，不是每次求解结果。

c0…c11 对应特征依次为：1、v*a、omega*alpha、a²、alpha²、|v|、|omega|、v²、omega²、|a|、|alpha|、|v*omega|。

验证命令：`catkin build rm_chassis_controllers -j2` 与 `catkin test rm_chassis_controllers -j2`。


## 预测功率

`/power_model/predicted_power`（`std_msgs/Float64`，`data` 单位 W）在每次功率测量通过对齐和工况检查时发布，不受 metrics_rate 限频。使用本次测量入模前的有效 c 计算 `cᵀφ`；尚无有效模型、对齐失败或预测非有限值时不发布。消息无时间戳，适合实时观察；不是未来时刻功率预测。
