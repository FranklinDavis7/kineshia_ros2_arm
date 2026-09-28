#!/usr/bin/env python3
"""
controller_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a ROS 2 node that owns the arm state, accepts a target, plans a
time-parameterized joint trajectory, and streams joint states as it executes.

Suggested interface (you may adapt, but document any changes in your README):
    - Publishes:   /joint_states   (sensor_msgs/JointState)   at a fixed rate
    - Service:     /move_to_target (your choice of srv; e.g. a Point target)
      OR Topic:    /target_pose    (geometry_msgs/PointStamped)
    - Parameters:  publish_rate_hz, control_mode, trajectory_duration, ...

Core requirements (see the task brief):
    1. Run IK on the incoming target (use PlanarArm.inverse_kinematics).
    2. Generate a smooth joint-space trajectory from the current q to the goal
       q (trapezoidal or quintic — your choice; explain it).
    3. Step along the trajectory in a timer callback and publish JointState.
    4. Respect joint limits and the ground constraint.
    5. Design the command path so a hardware backend (e.g. Dynamixel) could be
       swapped in later without rewriting the planner.

Stretch (optional, rewarded): velocity / current control modes, a PID
trajectory-tracking loop with an error signal, a ROS 2 action for the full
pick-and-place with feedback/cancel.
"""
import time
from std_msgs.msg import Bool
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from planar_arm_interfaces.action import PickAndPlace
from rclpy.node import Node
from sensor_msgs.msg import JointState
# The provided kinematics library — do not modify it.
from planar_arm_control.planar_arm import PlanarArm
from geometry_msgs.msg import PointStamped    
import numpy as np 
LINK_LENGTHS = [3.0, 2.0, 1.5]

class JointCommandSink:
    def send(self, q, stamp):
        raise NotImplementedError


class RosPublisherSink(JointCommandSink):
    def __init__(self, node):
        self._pub = node.create_publisher(JointState, "/joint_states", 10)

    def send(self, q, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = list(q)
        self._pub.publish(msg)


class DynamixelSink(JointCommandSink):
    """Reference implementation — not wired up. Would talk to a real bus."""
    def __init__(self, port="/dev/ttyUSB0", baudrate=1_000_000):
        self.port = port
        self.baudrate = baudrate

    def send(self, q, stamp):
        # Placeholder — would call the Dynamixel SDK here.
        pass
class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        # TODO: declare parameters, create publishers/services, timers, state.
        self.get_logger().info("controller_node started (stub — implement me).")
        self._pending_target = None      # target to pursue after reaching neutral
        self.grasp_pub = self.create_publisher(Bool, "/grasp_state", 10)
        self.current_q = [0.0,0.0,0.0]
                #Creating a publisher to /joint_states topic 
        self.sink = RosPublisherSink(self)
        self.timer = self.create_timer(1.0/20.0,self.tick)
        self.target_sub = self.create_subscription(
            PointStamped, "/target_pose", self.target_callback, 10
        )

        self.q_start = [0.0, 0.0, 0.0]
        self.q_goal = [0.0, 0.0, 0.0]
        self.motion_start_time = None
        self.motion_duration = 2.0

        self._action_server = ActionServer(
        self,
        PickAndPlace,
        "/pick_and_place",
        execute_callback=self.execute_pick_and_place,)
        self._motion_done = False        # set to True when step_toward_goal finishes ALL legs
    def execute_pick_and_place(self, goal_handle):
        """Runs when the GUI sends a goal. Blocks until complete or cancelled."""
        self.get_logger().info("Pick-and-place: start")

        pick = (goal_handle.request.pick.x, goal_handle.request.pick.y)
        place = (goal_handle.request.place.x, goal_handle.request.place.y)

        feedback = PickAndPlace.Feedback()

        def move_to(target_xy):
            if not self._command_target(target_xy):
                return False
            self._motion_done = False
            while not self._motion_done:
                if goal_handle.is_cancel_requested:
                    return False
                time.sleep(0.02)
            return True

        # Phase 1: Move to pick
        feedback.phase = "moving_to_pick"
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)
        if not move_to(pick):
            goal_handle.abort()
            result = PickAndPlace.Result()
            result.success = False
            result.message = "Failed to reach pick"
            return result

        # Phase 2: Grasp
        feedback.phase = "grasping"
        feedback.progress = 0.4
        goal_handle.publish_feedback(feedback)
        time.sleep(0.5)
        self._grasp_callback(True)

        # Phase 3: Move to place
        feedback.phase = "moving_to_place"
        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)
        if not move_to(place):
            goal_handle.abort()
            result = PickAndPlace.Result()
            result.success = False
            result.message = "Failed to reach place"
            return result

        # Phase 4: Release
        feedback.phase = "releasing"
        feedback.progress = 0.9
        goal_handle.publish_feedback(feedback)
        time.sleep(0.3)
        self._grasp_callback(False)

        # Done
        feedback.phase = "done"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = PickAndPlace.Result()
        result.success = True
        result.message = f"Picked at {pick}, placed at {place}"
        self.get_logger().info("Pick-and-place: complete")
        return result
    
    def _command_target(self, target_xy):
        """Move to target_xy, routing through a neutral pose if needed."""
        ok, reason = self._try_direct(target_xy)
        if ok:
            return True

        # Direct failed. Try routing through a neutral "unfold" pose.
        self.get_logger().info(
            f"Direct IK failed for {target_xy}: {reason}. Trying waypoint route."
        )

        # Pick a neutral pose that is reachable from both sides.
        # (6.0, 0.5) is far out on +x — arm extended, near q = [~5°, 0°, 0°].
        neutral = (6.0, 0.5)

        ok_neutral, reason_neutral = self._try_direct(neutral)
        if not ok_neutral:
            self.get_logger().warn(
                f"Neutral waypoint {neutral} unreachable: {reason_neutral}"
            )
            self._last_ik_failure = reason
            return False

        # Motion to neutral is now running. Wait for it, then retry the target.
        # We defer the "second leg" by setting a pending target.
        self._pending_target = target_xy
        self._last_ik_failure = None
        self.get_logger().info(
            f"Routing via {neutral}, target queued: {target_xy}"
        )
        return True
    def _try_direct(self, target_xy):
            """Attempt a direct move. Returns (success, reason)."""
            target = tuple(self.arm.reachable_target(target_xy))

            seeds = [
                self.current_q,
                [0.0, 0.0, 0.0],
                [np.pi / 2, 0.0, 0.0],
                [np.pi / 4, np.pi / 2, -np.pi / 2],
                [np.pi, 0.0, 0.0],
                [np.pi / 2, -np.pi / 2, np.pi / 2],
                [np.pi / 2, np.pi / 2, -np.pi / 2],
                [3 * np.pi / 4, 0.0, 0.0],
            ]

            best_q = None
            best_cost = float("inf")

            for seed in seeds:
                try:
                    candidate = self.arm.inverse_kinematics(
                        target, initial_guess=seed
                    )
                except Exception:
                    continue

                candidate = list(candidate)
                for i in range(3):
                    delta = candidate[i] - self.current_q[i]
                    candidate[i] = self.current_q[i] + PlanarArm.wrap_angle(delta)

                if not self.arm.within_joint_limits(candidate):
                    continue
                if not self.arm.arm_above_base(candidate):
                    continue

                cost = sum(abs(candidate[i] - self.current_q[i]) for i in range(3))
                if cost < best_cost:
                    best_cost = cost
                    best_q = candidate

            if best_q is None:
                return False, (
                    f"No valid IK for ({target[0]:.2f}, {target[1]:.2f}) "
                    f"within joint limits"
                )

            self.q_start = list(self.current_q)
            self.q_goal = list(best_q)
            self.motion_start_time = self.get_clock().now()
            self._motion_done = False                        # ← set HERE, only on success
            self.get_logger().info(
                f"IK accepted (deg): "
                f"[{np.degrees(best_q[0]):+.1f}, "
                f"{np.degrees(best_q[1]):+.1f}, "
                f"{np.degrees(best_q[2]):+.1f}], cost={best_cost:.3f}"
            )
            return True, "ok"
    def _tick(self):
        """Runs at 20 Hz. Advances the trajectory and sends a command."""
        self.step_toward_goal()

        stamp = self.get_clock().now().to_msg()
        self.sink.send(self.current_q, stamp)
    def target_callback(self, msg):
        target = (msg.point.x, msg.point.y)
        self.get_logger().info(f"Target received: ({target[0]:.2f}, {target[1]:.2f})")
        self._command_target(target)

    @staticmethod
    def quintic_alpha(t_norm):
        return 10 * t_norm**3 - 15 * t_norm**4 + 6 * t_norm**5

    def step_toward_goal(self):
        if self.motion_start_time is None:
            return

        elapsed = (self.get_clock().now() - self.motion_start_time).nanoseconds / 1e9
        t_norm = min(elapsed / self.motion_duration, 1.0)
        alpha = self.quintic_alpha(t_norm)

        self.current_q = [
            self.q_start[i] + alpha * (self.q_goal[i] - self.q_start[i])
            for i in range(3)
        ]

        if alpha < 1.0:
            return

        # --- Current leg complete ---
        self.current_q = list(self.q_goal)
        self.motion_start_time = None

        # --- If another leg is queued, start it ---
        if self._pending_target is not None:
            next_target = self._pending_target
            self._pending_target = None
            ok, reason = self._try_direct(next_target)
            if not ok:
                self._last_ik_failure = reason
                self.get_logger().warn(f"Second leg failed: {reason}")
                self._motion_done = True
            # else: motion_start_time is now set → we'll keep stepping next tick
        else:
            self._motion_done = True
            self.get_logger().info("Motion complete.")

    def _grasp_callback(self, is_grasping):
        msg = Bool()
        msg.data = bool(is_grasping)
        self.grasp_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()