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

#include "pb_omni_pid_pursuit_controller/pid.hpp"

/**
 * @brief PID 构造函数，通过初始化列表设置所有控制参数并清零误差状态
 *
 * @param dt  控制周期（秒），决定积分项和微分项的时间尺度
 * @param max 输出上限，防止控制量过大
 * @param min 输出下限，允许负向控制（如后退）
 * @param kp  比例系数
 * @param kd  微分系数
 * @param ki  积分系数
 */
PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0)
{
}

/**
 * @brief PID 核心计算函数，执行一次完整的 PID 控制循环
 *
 * 计算过程分为四个步骤：
 *   1. 比例项 (P)：Kp * error —— 对当前误差的直接线性响应
 *   2. 积分项 (I)：Ki * ∫error*dt —— 累积历史误差消除稳态偏差
 *   3. 微分项 (D)：Kd * d(error)/dt —— 根据误差变化趋势进行预测性调节
 *   4. 输出限幅：将最终结果钳位在 [min, max] 之间
 *
 * @param set_point 目标期望值（如期望线速度或角度）
 * @param pv        当前过程变量值（如当前实际速度或角度）
 * @return          经过限幅后的控制输出值
 */
double PID::calculate(double set_point, double pv)
{
  // 计算当前误差：期望值 - 实际值
  double error = set_point - pv;

  // 比例项 (Proportional)：对误差的即时比例响应
  double p_out = kp_ * error;

  // 积分项 (Integral)：将当前误差乘以时间步长累加到积分器中
  // 以消除长期存在的稳态误差
  integral_ += error * dt_;
  double i_out = ki_ * integral_;

  // 积分限幅：防止积分饱和 (Integral Windup)
  // 当累积误差过大时，积分项失去调节意义，将积分值限制在 [-1, 1]
  if (integral_ > 1) {
    integral_ = 1;
  } else if (integral_ < -1) {
    integral_ = -1;
  }

  // 微分项 (Derivative)：基于误差变化率进行预测
  // 除以 dt 将误差差量转换为变化率（单位：误差/秒）
  double derivative = (error - pre_error_) / dt_;
  double d_out = kd_ * derivative;

  // 总输出 = P项 + I项 + D项
  double output = p_out + i_out + d_out;

  // 输出限幅：确保控制量在允许范围内
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // 保存当前误差，供下一次循环计算微分项使用
  pre_error_ = error;

  return output;
}

void PID::setGains(double kp, double kd, double ki)
{
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
}

void PID::setLimits(double max, double min)
{
  max_ = max;
  min_ = min;
}

void PID::reset()
{
  pre_error_ = 0.0;
  integral_ = 0.0;
}

/**
 * @brief 设置积分累积值，用于初始化或重置积分器状态
 *
 * 典型用途：
 *   - 控制器刚启动时将积分项清零
 *   - 切换控制模式后重置历史累积误差，避免积分饱和
 *
 * @param sum_error 新的积分累积值
 */
void PID::setSumError(double sum_error) { integral_ = sum_error; }

PID::~PID() {}
