#!/usr/bin/env python3
"""
gui_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a PyQt5 + PyQtGraph node that is a live client of the controller.

Suggested behaviour (adapt as you like, document changes):
    - Subscribes to /joint_states and renders the arm live (links + joints).
    - Plots joint angles (and, if you do PID tracking, tracking error) over
      time with PyQtGraph.
    - Lets the operator enter pick and place targets and send them to the
      controller (service call or topic publish).
    - Shows telemetry: end-effector position, current mode, status.

THE KEY CHALLENGE: the ROS 2 executor and the Qt event loop must run together
without either one blocking the other. Spinning ROS in a background thread or
driving rclpy.spin_once from a QTimer are both acceptable — your handling of
this is a graded signal (multithreading / timer synchronisation).

You may reuse the look and feel of a standard PyQtGraph arm plot; the point of
this task is the ROS 2 integration, not pixel-perfect styling.
"""
from PyQt5 import QtCore
import sys
from sensor_msgs.msg import JointState    # at the top of the file
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QTimer
from PyQt5 import QtWidgets  # noqa: F401  (import here so missing deps fail loudly)
import pyqtgraph as pg  # noqa: F401
from collections import deque
from planar_arm_control.planar_arm import PlanarArm
import time
LINK_LENGTHS = [3.0, 2.0, 1.5]
from geometry_msgs.msg import PointStamped 
from rclpy.action import ActionClient
from planar_arm_interfaces.action import PickAndPlace
from std_msgs.msg import Bool

class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        self.window = None
        self.current_q = [0.0, 0.0, 0.0]

        self.sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.target_pub = self.create_publisher(
            PointStamped, "/target_pose", 10
        )

        # Action client for pick-and-place
        self._action_client = ActionClient(
            self, PickAndPlace, "/pick_and_place"
        )

        # Grasp state from the controller
        self.grasp_sub = self.create_subscription(
            Bool, "/grasp_state", self._on_grasp_state, 10
        )

        self._current_goal_handle = None
        self.get_logger().info("gui_node started")

    # -----------------------------------------------------------------
    def joint_state_callback(self, msg):
        self.current_q = list(msg.position)
        if self.window is not None:
            self.window.update_arm(self.current_q)

    # -----------------------------------------------------------------
    def send_target(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.target_pub.publish(msg)
        self.get_logger().info(f"Sent target ({x:.2f}, {y:.2f})")

    # -----------------------------------------------------------------
    def _on_grasp_state(self, msg):
        print(f"[GRASP] received {msg.data}")
        if self.window is not None:
            self.window.object_picked = msg.data
    # -----------------------------------------------------------------
    def send_pick_and_place(self, pick_xy, place_xy):
        """Send an action goal to /pick_and_place."""
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Pick-and-place server not available")
            if self.window is not None:
                self.window.status.setText("Action server not available")
            return

        goal = PickAndPlace.Goal()
        goal.pick.x = float(pick_xy[0])
        goal.pick.y = float(pick_xy[1])
        goal.pick.z = 0.0
        goal.place.x = float(place_xy[0])
        goal.place.y = float(place_xy[1])
        goal.place.z = 0.0

        self.get_logger().info(
            f"Sending pick-and-place: {pick_xy} -> {place_xy}"
        )
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._on_action_feedback,
        )
        send_future.add_done_callback(self._on_goal_accepted)

    # -----------------------------------------------------------------
    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Pick-and-place goal rejected")
            if self.window is not None:
                self.window.status.setText("Action goal rejected")
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info("Pick-and-place goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    # -----------------------------------------------------------------
    def _on_action_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        if self.window is not None:
            self.window.status.setText(
                f"[{fb.phase}] {int(fb.progress * 100)}%"
            )

    # -----------------------------------------------------------------
    def _on_action_result(self, future):
        result = future.result().result
        self._current_goal_handle = None
        if self.window is not None:
            if result.success:
                self.window.status.setText(f"Done: {result.message}")
            else:
                self.window.status.setText(f"FAILED: {result.message}")
        self.get_logger().info(
            f"Pick-and-place finished: success={result.success}, "
            f"message={result.message}"
        )


# =====================================================================
# Qt side
# =====================================================================
class Arm(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3-DOF Arm Viewer")
        self.resize(700, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # ---- Interaction state -------------------------------------
        self.mode = "idle"                 # "idle" | "spawn"
        self.object_pos = None             # (x, y) or None
        self.object_picked = False         # True while the arm carries it
        self.object_marker = None          # pyqtgraph item, created on spawn
        self.target_sender = None          # filled in by main()
        self.action_sender = None          # filled in by main()

        # ---- Arm plot -----------------------------------------------
        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("bottom", "X")
        self.plot.setLabel("left", "Y")

        vb = self.plot.getViewBox()
        vb.enableAutoRange(False, False)
        vb.setAutoVisible(False)
        vb.setLimits(xMin=-11, xMax=11, yMin=-11, yMax=11)

        self.plot.setXRange(-11, 11, padding=0)
        self.plot.setYRange(-11, 11, padding=0)
        self.plot.setMouseEnabled(x=False, y=False)

        layout.addWidget(self.plot)

        # Arm curve + base marker
        self.arm_curve = self.plot.plot(
            pen=pg.mkPen(color="c", width=4),
            symbol="o", symbolSize=10, symbolBrush="y",
        )
        self.plot.plot(
            [0], [0],
            symbol="s", symbolSize=14, symbolBrush="r", pen=None,
        )

        # Arm model (for FK in update_arm)
        self.arm = PlanarArm(LINK_LENGTHS)

        # ---- Angle plot ---------------------------------------------
        self.angle_plot = pg.PlotWidget()
        self.angle_plot.showGrid(x=True, y=True)
        self.angle_plot.setLabel("bottom", "time (s)")
        self.angle_plot.setLabel("left", "angle (rad)")
        self.angle_plot.addLegend()

        self.angle_curves = [
            self.angle_plot.plot(pen=pg.mkPen("r", width=2), name="q1"),
            self.angle_plot.plot(pen=pg.mkPen("g", width=2), name="q2"),
            self.angle_plot.plot(pen=pg.mkPen("b", width=2), name="q3"),
        ]
        layout.addWidget(self.angle_plot)

        # History buffers for the angle plot
        self.t_history = deque(maxlen=2000)
        self.q_history = [deque(maxlen=2000) for _ in range(3)]
        self.t0 = None

        # ---- Buttons -------------------------------------------------
        self.button_row = QtWidgets.QHBoxLayout()

        self.spawn_button = QtWidgets.QPushButton("Spawn Object")
        self.button_row.addWidget(self.spawn_button)

        self.run_pp_button = QtWidgets.QPushButton("Run Pick & Place")
        self.button_row.addWidget(self.run_pp_button)

        layout.addLayout(self.button_row)

        # ---- Telemetry label ----------------------------------------
        self.info = QtWidgets.QLabel("Telemetry: waiting for /joint_states…")
        self.info.setAlignment(QtCore.Qt.AlignCenter)
        self.info.setFrameShape(QtWidgets.QFrame.Box)
        self.info.setMinimumHeight(32)
        layout.addWidget(self.info)

        # ---- Status label (transient mode messages) ------------------
        self.status = QtWidgets.QLabel("Ready")
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        self.status.setMinimumHeight(28)
        self.status.setStyleSheet(
            "color: #ffe066; background: #2b2b2b; padding: 4px;"
        )
        layout.addWidget(self.status)

        # ---- Signal connections -------------------------------------
        self.spawn_button.clicked.connect(self._on_spawn_clicked)
        self.run_pp_button.clicked.connect(self._on_run_pp_clicked)
        self.plot.scene().sigMouseClicked.connect(self._on_plot_click)


    # -----------------------------------------------------------------
    # Button handlers
    # -----------------------------------------------------------------
    def _on_spawn_clicked(self):
        self.mode = "spawn"
        self.status.setText("Mode: SPAWN — click on the plot to place the object")

    def _on_run_pp_clicked(self):
        if self.action_sender is None:
            self.status.setText("Action client not wired")
            return

        # Auto-spawn at the fixed pick location if no object exists yet
        if self.object_pos is None:
            pick = (4.0, 2.0)
            self._handle_spawn(pick[0], pick[1])
        else:
            pick = self.object_pos

        place = (-3.0, 3.0)      # fixed place target from the brief
        self.status.setText(
            f"Sending pick-and-place: {pick} -> {place}"
        )
        self.action_sender(pick, place)

    # -----------------------------------------------------------------
    # Plot click dispatch
    # -----------------------------------------------------------------
    def _on_plot_click(self, event):
        """Dispatch a plot click based on the current mode."""
        if not self.plot.sceneBoundingRect().contains(event.scenePos()):
            return

        pos = self.plot.getViewBox().mapSceneToView(event.scenePos())
        x, y = pos.x(), pos.y()

        if y < 0:
            self.status.setText(
                f"Ignored click below ground: ({x:.2f}, {y:.2f})"
            )
            return

        if self.mode == "spawn":
            self._handle_spawn(x, y)
        else:
            # Idle mode — send target directly (free-form motion)
            if self.target_sender is not None:
                self.target_sender(x, y)

    # -----------------------------------------------------------------
    # Spawn handler
    # -----------------------------------------------------------------
    def _handle_spawn(self, x, y):
        """Create the object marker at (x, y)."""
        if self.object_marker is not None:
            self.plot.removeItem(self.object_marker)
            self.object_marker = None

        self.object_pos = (x, y)
        self.object_picked = False

        self.object_marker = self.plot.plot(
            [x], [y],
            symbol="s", symbolSize=18,
            symbolBrush="g", pen=None,
        )
        self.status.setText(f"Object spawned at ({x:.2f}, {y:.2f})")
        self.mode = "idle"

    # -----------------------------------------------------------------
    # Called by GuiNode on every /joint_states message
    # -----------------------------------------------------------------
    def update_arm(self, q):
        # 1. Draw the arm
        points = self.arm.forward_kinematics(q)
        self.arm_curve.setData(
            [p[0] for p in points],
            [p[1] for p in points],
        )

        # 2. Telemetry label
        ee_x, ee_y = self.arm.end_effector(q)
        self.info.setText(
            f"q = [{q[0]:+.2f}, {q[1]:+.2f}, {q[2]:+.2f}] rad   "
            f"EE = ({ee_x:+.2f}, {ee_y:+.2f})"
        )

        # 3. Angle history
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        self.t_history.append(t)
        for i in range(3):
            self.q_history[i].append(q[i])

        t_list = list(self.t_history)
        for i in range(3):
            self.angle_curves[i].setData(
                t_list, list(self.q_history[i])
            )

        # 4. If the object is being carried, its marker follows the EE
        if self.object_picked and self.object_marker is not None:
            ee = self.arm.end_effector(q)
            print(f"[FOLLOW] picked=True, EE={ee}")
            self.object_marker.setData([ee[0]], [ee[1]])
            self.object_pos = (ee[0], ee[1])


# =====================================================================
# main
# =====================================================================
def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    window = Arm()
    window.show()

    node = GuiNode()
    node.window = window
    window.target_sender = node.send_target
    window.action_sender = node.send_pick_and_place

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    exit_code = app.exec_()

    ros_timer.stop()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

