"""AI 控制器调参工具 —— 目标点发送节点。

按照预设的调参阶段（平移、前视、旋转、弯道、接近、综合），
依次向 Nav2 发送导航目标点，用于系统化调试全向 PID 追踪控制器参数。
支持通过 ROS 参数和 /ai_controller_tuner/current_stage 话题动态切换阶段。
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# 目标点类型：(x, y, yaw)，x/y 单位为米，yaw 为弧度
Goal = Tuple[float, float, float]

# 各调参阶段的预设目标点序列
# 通过依次导航到这些目标点，可以针对性地观察和调试不同控制器参数
STAGE_GOALS = {
    # 2. 平移跟踪调试：直线往返，主要观察 translation_kp/ki/kd。
    "translation": [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
    # 3. 前视距离调试：更长直线，观察 lookahead_dist / lookahead_time 是否平滑。
    "lookahead": [(2.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
    # 4. 旋转跟踪调试：位置变化小，主要观察 rotation_kp/ki/kd。
    "rotation": [(0.15, 0.0, math.pi / 2.0), (0.15, 0.0, -math.pi / 2.0), (0.15, 0.0, 0.0)],
    # 5. 弯道/曲率限速调试：L 形和折线路径，观察弯道是否降速且不过度抖动。
    "curvature": [(1.0, 0.0, 0.0), (1.0, 1.0, math.pi / 2.0), (0.0, 1.0, math.pi)],
    # 6. 终点减速调试：短距离目标，观察 approach_velocity_scaling_dist 和最小接近速度。
    "approach": [(0.45, 0.0, 0.0), (0.0, 0.0, math.pi)],
    # 7. 综合场景验证：直线、横移、转弯、回到起点。
    "comprehensive": [
        (1.0, 0.0, 0.0),
        (1.0, 1.0, math.pi / 2.0),
        (0.0, 1.0, math.pi),
        (0.0, 0.0, -math.pi / 2.0),
    ],
}

# 所有可用的调参阶段列表（按推荐顺序排列）
ROUTE_STAGES = [
    "translation",
    "lookahead",
    "rotation",
    "curvature",
    "approach",
    "comprehensive",
    "custom",
]
TUNING_COMPLETE_STAGE = "complete"

# 各阶段未配置路由参数时的默认兜底目标点
DEFAULT_ROUTE_FALLBACK = {
    "custom": [(1.0, 0.0, 0.0)],
}


def yaw_to_quaternion(yaw: float):
    """将偏航角（绕 Z 轴旋转）转换为四元数的 z/w 分量。"""
    half_yaw = yaw * 0.5
    return {
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


class AiControllerGoalSender(Node):
    """AI 控制器调参目标发送节点。

    按阶段依次发送导航目标点给 Nav2，支持：
    - 内置调参阶段（平移/前视/旋转/弯道/接近/综合）
    - 通过话题动态跟随 tuner 的阶段切换
    - 自定义目标点序列
    - 目标点循环发送
    """

    def __init__(self) -> None:
        super().__init__("ai_controller_goal_sender")

        # ---- ROS 参数声明 ----
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("debug_stage", "translation")
        self.declare_parameter("goal_x", 1.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("custom_goal_xs", [1.0])
        self.declare_parameter("custom_goal_ys", [0.0])
        self.declare_parameter("custom_goal_yaws", [0.0])
        self.declare_parameter("loop_goals", False)
        self.declare_parameter("loop_delay_sec", 2.0)
        self.declare_parameter("use_stage_routes", True)
        self.declare_parameter("follow_tuner_stage", True)
        self.declare_parameter("stage_topic", "/ai_controller_tuner/current_stage")
        self.declare_parameter("keep_alive_for_stage_updates", True)
        self.declare_parameter("stop_on_tuning_complete", True)

        # 为每个阶段声明可配置的路由参数（route_<stage>_xs/ys/yaws）
        for stage in ROUTE_STAGES:
            defaults = STAGE_GOALS.get(stage, DEFAULT_ROUTE_FALLBACK.get(stage, [(0.0, 0.0, 0.0)]))
            self.declare_parameter(
                f"route_{stage}_xs", [float(goal[0]) for goal in defaults]
            )
            self.declare_parameter(
                f"route_{stage}_ys", [float(goal[1]) for goal in defaults]
            )
            self.declare_parameter(
                f"route_{stage}_yaws", [float(goal[2]) for goal in defaults]
            )

        # ---- 读取并缓存参数值 ----
        self.action_name = self.get_parameter("action_name").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.debug_stage = self.get_parameter("debug_stage").value
        self.loop_goals = bool(self.get_parameter("loop_goals").value)
        self.loop_delay_sec = float(self.get_parameter("loop_delay_sec").value)
        self.use_stage_routes = bool(self.get_parameter("use_stage_routes").value)
        self.follow_tuner_stage = bool(self.get_parameter("follow_tuner_stage").value)
        self.stage_topic = self.get_parameter("stage_topic").value
        self.keep_alive_for_stage_updates = bool(
            self.get_parameter("keep_alive_for_stage_updates").value
        )
        self.stop_on_tuning_complete = bool(
            self.get_parameter("stop_on_tuning_complete").value
        )

        # ---- 内部状态 ----
        self.goals = self._load_goals()          # 当前阶段的目标点列表
        self._goal_index = 0                     # 当前发送到第几个目标点
        self._goal_active = False                # 是否有目标正在执行中
        self._goal_handle = None                 # 当前活跃目标的句柄
        self._ignore_next_goal_result = False    # 是否忽略下一个目标结果（阶段切换取消时使用）
        self._sequence_done = False              # 当前阶段序列是否已完成
        self._loop_timer = None                  # 循环重发定时器
        self._shutdown_timer = None              # 调参完成后的退出定时器
        self._stopped_for_tuning = False         # 调参完成后不再发送新目标

        # ---- Action 客户端 ----
        self._client = ActionClient(self, NavigateToPose, self.action_name)

        # 如果跟随 tuner 阶段，订阅阶段切换话题（使用 TRANSIENT_LOCAL 以获取最后一条留存消息）
        if self.follow_tuner_stage:
            stage_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, self.stage_topic, self._on_stage_update, stage_qos)

        # 定时检查并发送下一个目标点（0.5s 间隔轮询）
        self.create_timer(0.5, self._maybe_send_goal)
        self.get_logger().info(
            f"Waiting for Nav2 action server: {self.action_name}; "
            f"stage={self.debug_stage}; frame={self.goal_frame}; goals={len(self.goals)}; "
            f"follow_tuner_stage={self.follow_tuner_stage}"
        )

    def _on_stage_update(self, msg: String) -> None:
        """接收 tuner 的阶段切换消息，动态切换当前调参阶段。

        如果当前有正在执行的目标，会取消它并切换到新阶段的目标序列。
        """
        stage = msg.data.strip()
        if stage == TUNING_COMPLETE_STAGE:
            if self.stop_on_tuning_complete:
                self._stop_for_tuning_complete()
            return
        # 空消息或阶段未变化则忽略
        if not stage or self._stopped_for_tuning or stage == self.debug_stage:
            return
        if stage not in ROUTE_STAGES:
            self.get_logger().warn(f"Ignoring unknown tuner stage update: {stage}")
            return

        old_stage = self.debug_stage
        self.debug_stage = stage
        # 重新加载新阶段的目标点序列
        self.goals = self._load_goals()
        self._goal_index = 0
        self._sequence_done = False
        # 取消循环定时器
        if self._loop_timer is not None:
            self._loop_timer.cancel()
            self._loop_timer = None
        # 如果有正在执行的目标，取消它
        if self._goal_active and self._goal_handle is not None:
            self.get_logger().info(
                f"Tuner stage changed {old_stage} -> {stage}; canceling active goal and switching route."
            )
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: None)
            self._ignore_next_goal_result = True  # 忽略被取消目标的完成回调
        else:
            self.get_logger().info(
                f"Tuner stage changed {old_stage} -> {stage}; loaded {len(self.goals)} goal(s)."
            )
        self._goal_active = False
        self._goal_handle = None

    def _load_goals(self) -> List[Goal]:
        """根据当前阶段加载目标点列表。

        优先级：ROS 参数路由 > custom 参数 > 内置 STAGE_GOALS > 单目标兜底。
        """
        # 优先使用通过 ROS 参数配置的阶段路由
        if self.use_stage_routes:
            route_goals = self._route_goals_from_stage_params(self.debug_stage)
            if route_goals:
                self.get_logger().info(
                    f"Loaded configured route for stage '{self.debug_stage}': {route_goals}"
                )
                return route_goals

        # custom 阶段：从 custom_goal_xs/ys/yaws 参数读取
        if self.debug_stage == "custom":
            xs = list(self.get_parameter("custom_goal_xs").value)
            ys = list(self.get_parameter("custom_goal_ys").value)
            yaws = list(self.get_parameter("custom_goal_yaws").value)
            if not (len(xs) == len(ys) == len(yaws)) or not xs:
                self.get_logger().warn(
                    "custom_goal_xs/custom_goal_ys/custom_goal_yaws must be non-empty and equal length; "
                    "falling back to goal_x/goal_y/goal_yaw."
                )
                return [self._single_goal_from_params()]
            return [(float(x), float(y), float(yaw)) for x, y, yaw in zip(xs, ys, yaws)]

        # 使用内置的预设阶段目标
        if self.debug_stage in STAGE_GOALS:
            return STAGE_GOALS[self.debug_stage]

        # 未知阶段：降级为单目标
        self.get_logger().warn(
            f"Unknown debug_stage '{self.debug_stage}', falling back to single configured goal."
        )
        return [self._single_goal_from_params()]

    def _route_goals_from_stage_params(self, stage: str) -> List[Goal]:
        """从 ROS 参数中读取指定阶段的路由目标点序列。

        参数命名规则：route_<stage>_xs, route_<stage>_ys, route_<stage>_yaws。
        """
        names = {
            "xs": f"route_{stage}_xs",
            "ys": f"route_{stage}_ys",
            "yaws": f"route_{stage}_yaws",
        }
        if not all(self.has_parameter(name) for name in names.values()):
            return []

        values: Dict[str, List[float]] = {}
        for key, name in names.items():
            raw_values = list(self.get_parameter(name).value)
            values[key] = [float(value) for value in raw_values]

        # 三个数组均为空则视为未配置
        if not values["xs"] and not values["ys"] and not values["yaws"]:
            return []
        # 三个数组长度必须一致
        if not (
            len(values["xs"]) == len(values["ys"]) == len(values["yaws"])
        ):
            self.get_logger().warn(
                f"Ignoring route for stage '{stage}' because route arrays have "
                f"different lengths: xs={len(values['xs'])}, ys={len(values['ys'])}, "
                f"yaws={len(values['yaws'])}."
            )
            return []
        if not values["xs"]:
            return []
        return list(zip(values["xs"], values["ys"], values["yaws"]))

    def _single_goal_from_params(self) -> Goal:
        """从 goal_x/goal_y/goal_yaw 参数构造单个目标点（兜底方案）。"""
        return (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_yaw").value),
        )

    def _maybe_send_goal(self) -> None:
        """定时轮询：如果空闲且 Action 服务器就绪，发送下一个目标点。"""
        if self._stopped_for_tuning or self._goal_active or self._sequence_done:
            return
        if not self._client.server_is_ready():
            self._client.wait_for_server(timeout_sec=0.1)
            return

        goal = self.goals[self._goal_index]
        self._goal_active = True
        goal_msg = self._make_goal_msg(goal)

        self.get_logger().info(
            f"Sending stage goal {self._goal_index + 1}/{len(self.goals)}: "
            f"x={goal[0]:.3f}, y={goal[1]:.3f}, yaw={goal[2]:.3f}"
        )
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _make_goal_msg(self, goal: Goal) -> NavigateToPose.Goal:
        """将 (x, y, yaw) 目标点构造为 NavigateToPose Action 消息。"""
        x, y, yaw = goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        quat = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.z = quat["z"]
        goal_msg.pose.pose.orientation.w = quat["w"]
        return goal_msg

    def _goal_response_callback(self, future) -> None:
        """Action 目标发送后的响应回调：检查是否被 Nav2 接受。"""
        goal_handle = future.result()
        if self._stopped_for_tuning:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async().add_done_callback(lambda _: None)
            return
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            self._finish_current_goal()
            return

        self._goal_handle = goal_handle
        self.get_logger().info("Goal accepted by Nav2.")
        # 异步等待目标执行结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        """目标执行完成回调：记录状态并推进到下一个目标点。"""
        result = future.result()
        self.get_logger().info(f"Goal finished with status: {result.status}")
        if self._stopped_for_tuning:
            return
        # 如果当前结果属于被取消的目标（阶段切换时），跳过处理
        if self._ignore_next_goal_result:
            self._ignore_next_goal_result = False
            return
        self._finish_current_goal()

    def _finish_current_goal(self) -> None:
        """完成当前目标点，推进索引或结束序列。

        根据配置决定下一步：
        - 还有剩余目标点：返回等待下一次轮询发送
        - 序列完成 + 跟随 tuner + 保持存活：等待阶段切换
        - 序列完成 + 不循环：关闭节点
        - 序列完成 + 循环模式：启动延迟定时器后重新开始
        """
        self._goal_active = False
        self._goal_handle = None
        self._goal_index += 1

        # 还有未发送的目标点
        if self._goal_index < len(self.goals):
            return

        # 序列全部完成
        if not self.loop_goals:
            # 跟随 tuner 模式：保持节点存活，等待下一阶段切换
            if self.follow_tuner_stage and self.keep_alive_for_stage_updates:
                self.get_logger().info(
                    "Stage goal sequence complete; waiting for tuner stage updates."
                )
                self._sequence_done = True
                return
            # 非跟随模式：发送完成，关闭节点
            self.get_logger().info("Stage goal sequence complete; shutting down goal sender.")
            self._sequence_done = True
            rclpy.shutdown()
            return

        # 循环模式：延迟后重新开始
        self._sequence_done = True
        self.get_logger().info(
            f"Stage sequence complete; loop enabled, restarting in {self.loop_delay_sec:.1f}s."
        )
        self._loop_timer = self.create_timer(self.loop_delay_sec, self._reset_for_loop)

    def _stop_for_tuning_complete(self) -> None:
        """调参完成后取消目标并退出 goal sender。"""
        if self._stopped_for_tuning:
            return
        self._stopped_for_tuning = True
        self._sequence_done = True
        if self._loop_timer is not None:
            self._loop_timer.cancel()
            self._loop_timer = None
        if self._goal_active and self._goal_handle is not None:
            self.get_logger().info("Tuning complete; canceling active navigation goal.")
            self._ignore_next_goal_result = True
            self._goal_handle.cancel_goal_async().add_done_callback(lambda _: None)
        self._goal_active = False
        self._goal_handle = None
        self.get_logger().info("Tuning complete; shutting down goal sender.")
        self._shutdown_timer = self.create_timer(0.1, self._shutdown_after_tuning)

    def _shutdown_after_tuning(self) -> None:
        """让取消请求先提交，再退出 ROS spin。"""
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
        if rclpy.ok():
            rclpy.shutdown()

    def _reset_for_loop(self) -> None:
        """循环模式：重置索引和状态，重新开始当前阶段的目标序列。"""
        if self._loop_timer is not None:
            self._loop_timer.cancel()
            self._loop_timer = None
        self._goal_index = 0
        self._sequence_done = False


def main(args: Optional[list] = None) -> None:
    """入口函数：初始化 ROS 2 并启动目标发送节点。"""
    rclpy.init(args=args)
    node = AiControllerGoalSender()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
