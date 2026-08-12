// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_

/**
 * @class PID
 * @brief 离散型 PID 控制器，用于实现速度/位置闭环控制
 *
 * 实现标准的位置式 PID 算法（Positional PID），计算公式为：
 *   u(t) = Kp*e(t) + Ki*∑e(t)*dt + Kd*(e(t)-e(t-1))/dt
 *
 * 其中：
 *   - Kp：比例系数，对当前误差进行比例放大，决定响应速度
 *   - Ki：积分系数，累积历史误差以消除稳态误差
 *   - Kd：微分系数，根据误差变化率进行预测，抑制超调和振荡
 *   - dt：控制周期（采样间隔），用于积分项和微分项的计算
 *
 * 输出值会被钳位在 [min, max] 范围内，积分项限幅可由调用者配置。
 */
class PID
{
public:
  /**
   * @brief 构造 PID 控制器并设置所有参数
   *
   * @param dt  控制周期（秒），即两次 calculate() 调用的时间间隔
   * @param max 控制器输出的最大值上限（钳位用）
   * @param min 控制器输出的最小值下限（钳位用）
   * @param kp  比例增益系数，值越大响应越快但可能超调
   * @param kd  微分增益系数，用于抑制超调和振荡
   * @param ki  积分增益系数，用于消除稳态误差，过大会导致积分饱和
   */
  PID(double dt, double max, double min, double kp, double kd, double ki);

  /**
   * @brief 根据设定值和当前值计算控制输出
   *
   * 核心 PID 计算函数，每轮控制循环调用一次。
   * 内部自动完成比例、积分、微分三项的计算，并对积分项和输出值进行限幅。
   *
   * @param set_point 目标设定值（期望值），例如期望速度或期望角度
   * @param pv        当前过程值（实际值），即传感器反馈的当前状态
   * @return          计算后的控制输出值，已钳位在 [min, max] 范围
   */
  double calculate(double set_point, double pv);

  void setGains(double kp, double kd, double ki);

  void setLimits(double max, double min);

  void setIntegralLimit(double limit);

  void reset();

  /**
   * @brief 手动设置积分项的累积误差值
   *
   * 通常用于控制器刚启动时初始化积分项，或切换控制模式时重置积分累积。
   *
   * @param sum_error 要设置的积分累积值
   */
  void setSumError(double sum_error);

  ~PID();

private:
  double dt_;        ///< 控制周期（秒），即两次 PID 计算之间的时间间隔
  double max_;       ///< 控制器输出的上限钳位值
  double min_;       ///< 控制器输出的下限钳位值
  double kp_;        ///< 比例增益系数（Proportional Gain）
  double kd_;        ///< 微分增益系数（Derivative Gain）
  double ki_;        ///< 积分增益系数（Integral Gain）
  double pre_error_; ///< 上一次的误差值，用于计算微分项
  double integral_;  ///< 累积积分误差，用于计算积分项
  double integral_limit_{1.0};  ///< 积分累积误差的绝对值上限
};

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
