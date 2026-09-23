import csv
import math
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)


class AdvancedIndoorOffboardTest(Node):
    """Small indoor body-relative cross-pattern flight for PX4 Offboard testing.

    The node never sends arm or disarm commands. The pilot must arm in Position
    mode. Moving the RC mode switch out of Offboard immediately ends external
    setpoint publication. The PX4 kill switch remains independent of this node.
    """

    WAIT_FOR_POSE = "WAIT_FOR_POSE"
    WAIT_FOR_ARM = "WAIT_FOR_ARM"
    WAIT_FOR_OFFBOARD = "WAIT_FOR_OFFBOARD"
    PRE_TAKEOFF = "PRE_TAKEOFF"
    TAKEOFF = "TAKEOFF"
    TAKEOFF_HOLD = "TAKEOFF_HOLD"
    PATTERN = "PATTERN"
    LAND_REQUESTED = "LAND_REQUESTED"
    WAIT_FOR_DISARM = "WAIT_FOR_DISARM"
    PILOT_TAKEOVER = "PILOT_TAKEOVER"
    ABORTED = "ABORTED"
    COMPLETE = "COMPLETE"

    def __init__(self):
        super().__init__("advanced_indoor_offboard_test")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos,
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.vehicle_status_callback,
            qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.command_ack_callback,
            qos,
        )

        # ---------------- User-adjustable test settings ----------------
        self.takeoff_height_m = 1
        self.takeoff_speed_mps = 0.15
        self.takeoff_accel_mps2 = 0.15
        self.takeoff_brake_accel_mps2 = 0.15
        self.takeoff_braking_margin_m = 0.05
        self.takeoff_reached_tolerance_m = 0.08
        self.takeoff_reached_speed_mps = 0.08
        self.takeoff_reacquire_tolerance_m = 0.15
        self.takeoff_timeout_s = 15.0
        self.pre_takeoff_delay_s = 3.0
        self.takeoff_hold_s = 3.0

        self.move_distance_m = 1.00
        self.move_speed_mps = 0.15
        # XY motion uses velocity control with acceleration-limited commands.
        # The permitted speed is reduced from the physical braking-distance
        # relation v^2 = 2*a*d, then position hold captures the endpoint.
        self.move_accel_mps2 = 0.15
        self.move_brake_accel_mps2 = 0.15
        self.braking_margin_m = 0.05
        self.point_reached_tolerance_m = 0.08
        self.point_reached_speed_mps = 0.10
        self.point_reacquire_tolerance_m = 0.15
        self.outer_hold_s = 1.5
        self.center_hold_s = 1.0
        self.pattern_cycles = 1

        # Hard safety envelope around the locked start pose.
        self.max_horizontal_radius_m = 1.20
        self.max_height_m = 1.5
        self.max_horizontal_speed_mps = 1.00
        self.max_vertical_speed_mps = 0.80
        self.max_horizontal_tracking_error_m = 0.35
        self.max_vertical_tracking_error_m = 1
        self.max_yaw_error_rad = math.radians(25.0)
        self.local_position_timeout_s = 0.50
        # ---------------------------------------------------------------

        self.timer_period_s = 0.10

        # Flight-path recording. Matplotlib is imported only after disarming,
        # so plotting never competes with the flight-control callback.
        self.recording_started = False
        self.record_start_time_s = None
        self.flight_samples = []
        self.results_saved = False
        self.log_directory = Path.home() / "robotx_ws" / "flight_logs"
        self.exit_timer = None

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)
        # Stream the complete XYZ setpoint independently at 10 Hz.  XY is
        # therefore resent continuously just like altitude, including while a
        # pattern point is being held.
        self.setpoint_timer = self.create_timer(
            self.timer_period_s,
            self.setpoint_stream_callback,
        )
        self.record_timer = self.create_timer(
            self.timer_period_s,
            self.record_sample,
        )

        self.state = self.WAIT_FOR_POSE
        self.state_enter_time = self.now_s()
        self.last_status_log_time = -math.inf

        self.vehicle_status = None
        self.local_position = None
        self.last_local_position_rx_s = None

        # A short stationary window is required before locking the origin.
        self.pose_window = deque(maxlen=50)
        self.start_x = None
        self.start_y = None
        self.start_z = None
        self.start_yaw = None
        self.locked_xy_reset_counter = None
        self.locked_z_reset_counter = None
        self.locked_heading_reset_counter = None

        self.cmd_x = None
        self.cmd_y = None
        self.cmd_z = None
        self.cmd_yaw = None
        self.cmd_vx = math.nan
        self.cmd_vy = math.nan
        self.cmd_vz = math.nan
        self.xy_position_hold_active = True
        self.z_position_hold_active = True

        self.offboard_request_started_s = None
        self.last_offboard_request_s = -math.inf
        self.last_land_request_s = -math.inf
        self.offboard_was_active = False

        self.pattern_points = []
        self.pattern_index = 0
        self.point_reached_time_s = None

        self.position_error_start_s = None
        self.yaw_error_start_s = None

        self.get_logger().warning(
            "PROPELLERS-OFF TEST FIRST. Start in Position mode with SF at Ready. "
            "This program never sends arm or disarm commands."
        )
        self.get_logger().warning(
            "Emergency order: use SC for Position/Altitude takeover first; "
            "use SH Kill only for imminent collision or total loss of control."
        )

    # ------------------------------------------------------------------
    # Basic helpers
    # ------------------------------------------------------------------

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    @staticmethod
    def wrap_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def enter_state(self, new_state):
        self.state = new_state
        self.state_enter_time = self.now_s()

        if new_state == self.COMPLETE:
            self.record_sample()
            self.save_results(make_plot=True)
            if self.exit_timer is None:
                self.exit_timer = self.create_timer(
                    0.5,
                    self.shutdown_after_complete,
                )

    def shutdown_after_complete(self):
        if self.exit_timer is not None:
            self.exit_timer.cancel()
        if rclpy.ok():
            rclpy.shutdown()

    def is_armed(self):
        return (
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

    def is_offboard(self):
        return (
            self.vehicle_status is not None
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_position_mode(self):
        return (
            self.vehicle_status is not None
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_POSCTL
        )

    def local_position_is_fresh(self):
        if self.last_local_position_rx_s is None:
            return False
        return self.now_s() - self.last_local_position_rx_s <= self.local_position_timeout_s

    @staticmethod
    def local_position_is_valid(msg):
        return (
            msg is not None
            and msg.xy_valid
            and msg.z_valid
            and msg.v_xy_valid
            and msg.v_z_valid
            and msg.heading_good_for_control
            and not msg.dead_reckoning
            and all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, msg.heading))
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def local_position_callback(self, msg):
        self.local_position = msg
        self.last_local_position_rx_s = self.now_s()

        if self.state == self.WAIT_FOR_POSE and self.local_position_is_valid(msg):
            self.pose_window.append((msg.x, msg.y, msg.z, msg.heading))
        elif self.state == self.WAIT_FOR_POSE:
            self.pose_window.clear()

    # ------------------------------------------------------------------
    # Expected/actual path recording and plotting
    # ------------------------------------------------------------------

    def ned_to_start_body(self, x, y, z):
        """Convert local NED position to start-heading forward/right/up."""
        dx = x - self.start_x
        dy = y - self.start_y
        forward = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
        right = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)
        up = self.start_z - z
        return forward, right, up

    def record_sample(self):
        if (
            not self.recording_started
            or self.record_start_time_s is None
            or self.results_saved
            or self.start_x is None
            or self.local_position is None
        ):
            return

        msg = self.local_position
        actual_valid = all(
            math.isfinite(v) for v in (msg.x, msg.y, msg.z)
        )
        if not actual_valid:
            return

        actual_forward, actual_right, actual_up = self.ned_to_start_body(
            msg.x,
            msg.y,
            msg.z,
        )

        expected_available = (
            self.is_offboard()
            and self.cmd_x is not None
            and self.cmd_y is not None
            and self.cmd_z is not None
        )

        if expected_available:
            expected_forward, expected_right, expected_up = self.ned_to_start_body(
                self.cmd_x,
                self.cmd_y,
                self.cmd_z,
            )
            horizontal_error = math.hypot(
                msg.x - self.cmd_x,
                msg.y - self.cmd_y,
            )
            vertical_error = abs(msg.z - self.cmd_z)
            expected_x = self.cmd_x
            expected_y = self.cmd_y
            expected_z = self.cmd_z
        else:
            expected_forward = math.nan
            expected_right = math.nan
            expected_up = math.nan
            horizontal_error = math.nan
            vertical_error = math.nan
            expected_x = math.nan
            expected_y = math.nan
            expected_z = math.nan

        self.flight_samples.append(
            {
                "time_s": self.now_s() - self.record_start_time_s,
                "state": self.state,
                "offboard_active": int(self.is_offboard()),
                "expected_x_ned_m": expected_x,
                "expected_y_ned_m": expected_y,
                "expected_z_ned_m": expected_z,
                "actual_x_ned_m": msg.x,
                "actual_y_ned_m": msg.y,
                "actual_z_ned_m": msg.z,
                "expected_forward_m": expected_forward,
                "expected_right_m": expected_right,
                "expected_up_m": expected_up,
                "actual_forward_m": actual_forward,
                "actual_right_m": actual_right,
                "actual_up_m": actual_up,
                "actual_vx_mps": msg.vx,
                "actual_vy_mps": msg.vy,
                "actual_vz_mps": msg.vz,
                "horizontal_error_m": horizontal_error,
                "vertical_error_m": vertical_error,
            }
        )

    def save_results(self, make_plot=True):
        if self.results_saved:
            return

        if not self.flight_samples:
            self.get_logger().warning(
                "No flight samples were recorded; no CSV or plot was created."
            )
            self.results_saved = True
            return

        # Set this before file operations to prevent duplicate writes from
        # callbacks arriving during shutdown.
        self.results_saved = True
        self.log_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self.log_directory / f"advanced_offboard_{stamp}.csv"
        png_path = self.log_directory / f"advanced_offboard_{stamp}.png"

        fieldnames = list(self.flight_samples[0].keys())
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.flight_samples)

        self.get_logger().info(f"Flight data saved: {csv_path}")

        if not make_plot:
            self.get_logger().warning(
                "Plot skipped because the program stopped while armed. "
                "CSV data was saved."
            )
            return

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().error(
                "CSV was saved, but the PNG could not be created because "
                f"matplotlib is unavailable: {exc}"
            )
            return

        times = [s["time_s"] for s in self.flight_samples]
        expected_forward = [s["expected_forward_m"] for s in self.flight_samples]
        expected_right = [s["expected_right_m"] for s in self.flight_samples]
        expected_up = [s["expected_up_m"] for s in self.flight_samples]
        actual_forward = [s["actual_forward_m"] for s in self.flight_samples]
        actual_right = [s["actual_right_m"] for s in self.flight_samples]
        actual_up = [s["actual_up_m"] for s in self.flight_samples]
        horizontal_error = [s["horizontal_error_m"] for s in self.flight_samples]
        vertical_error = [s["vertical_error_m"] for s in self.flight_samples]

        figure, axes = plt.subplots(1, 3, figsize=(16, 5))

        # Top view in the heading frame: right on x, forward on y.
        # 根据数据量决定标记间隔，整条轨迹大约显示25个标记
        marker_step = max(
        1,
        len(self.flight_samples) // 25,
        )

        # 期望路径：蓝色粗虚线、圆形标记
        axes[0].plot(
            expected_right,
            expected_forward,
            color="#0066CC",
            linestyle="--",
            linewidth=3.0,
            marker="o",
            markersize=5,
            markerfacecolor="white",
            markevery=marker_step,
            alpha=0.90,
            zorder=2,
            label="Expected",
            )

        # 实际路径：红色细实线、叉形标记
        axes[0].plot(
            actual_right,
            actual_forward,
            color="#E31A1C",
            linestyle="-",
            linewidth=1.5,
            marker="x",
            markersize=6,
            markevery=marker_step,
            alpha=0.90,
            zorder=3,
            label="Actual",
            )
            
        axes[0].scatter([0.0], [0.0], marker="o", s=45, label="Start")
        axes[0].set_title("Top View Path")
        axes[0].set_xlabel("Right from start (m)")
        axes[0].set_ylabel("Forward from start (m)")
        axes[0].axis("equal")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(times, expected_up, "--", linewidth=2.0, label="Expected")
        axes[1].plot(times, actual_up, "-", linewidth=1.5, label="Actual")
        axes[1].set_title("Height")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Up from start (m)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()

        axes[2].plot(times, horizontal_error, label="Horizontal error")
        axes[2].plot(times, vertical_error, label="Vertical error")
        axes[2].axhline(
            self.max_horizontal_tracking_error_m,
            linestyle="--",
            linewidth=1.0,
            label="Horizontal limit",
        )
        axes[2].axhline(
            self.max_vertical_tracking_error_m,
            linestyle=":",
            linewidth=1.0,
            label="Vertical limit",
        )
        axes[2].set_title("Tracking Error")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel("Error (m)")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()

        figure.suptitle(
            "Advanced Indoor Offboard: Expected vs Actual",
            fontsize=14,
        )
        figure.tight_layout()
        figure.savefig(png_path, dpi=160, bbox_inches="tight")
        plt.close(figure)
        self.get_logger().info(f"Flight plot saved: {png_path}")

    def vehicle_status_callback(self, msg):
        previously_armed = self.is_armed()
        self.vehicle_status = msg

        if previously_armed and not self.is_armed():
            if self.state not in (self.WAIT_FOR_POSE, self.WAIT_FOR_ARM, self.COMPLETE):
                self.get_logger().info("Vehicle disarmed. Test complete.")
                self.enter_state(self.COMPLETE)

    def command_ack_callback(self, msg):
        results = {
            0: "ACCEPTED",
            1: "TEMPORARILY_REJECTED",
            2: "DENIED",
            3: "UNSUPPORTED",
            4: "FAILED",
            5: "IN_PROGRESS",
            6: "CANCELLED",
        }
        result = results.get(msg.result, f"UNKNOWN({msg.result})")

        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.get_logger().info(f"Offboard acknowledgement: {result}")
        elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.get_logger().info(f"Land acknowledgement: {result}")

    # ------------------------------------------------------------------
    # PX4 publications
    # ------------------------------------------------------------------

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.mode_pub.publish(msg)

    def publish_position_setpoint(self):
        if None in (self.cmd_x, self.cmd_y, self.cmd_z, self.cmd_yaw):
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        if self.xy_position_hold_active:
            # Endpoint capture/hold: pure XY position control, with no velocity
            # feed-forward that could cause repeated overshoot.
            position_x = float(self.cmd_x)
            position_y = float(self.cmd_y)
            velocity_x = math.nan
            velocity_y = math.nan
        else:
            # Travel/braking: pure XY velocity control.  The endpoint remains
            # in cmd_x/cmd_y for guidance, safety checks and plotting only.
            position_x = math.nan
            position_y = math.nan
            velocity_x = float(self.cmd_vx)
            velocity_y = float(self.cmd_vy)

        if self.z_position_hold_active:
            position_z = float(self.cmd_z)
            velocity_z = math.nan
        else:
            # Takeoff travel/braking: direct vertical velocity control.  In
            # local NED coordinates, a negative velocity commands climb.
            position_z = math.nan
            velocity_z = float(self.cmd_vz)

        msg.position = [position_x, position_y, position_z]
        msg.velocity = [velocity_x, velocity_y, velocity_z]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = float(self.cmd_yaw)
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def setpoint_stream_callback(self):
        """Continuously publish the latest complete position target at 10 Hz."""
        streaming_states = (
            self.WAIT_FOR_ARM,
            self.WAIT_FOR_OFFBOARD,
            self.PRE_TAKEOFF,
            self.TAKEOFF,
            self.TAKEOFF_HOLD,
            self.PATTERN,
        )

        if self.state in streaming_states:
            self.publish_offboard_heartbeat()
            self.publish_position_setpoint()
        elif self.state == self.LAND_REQUESTED and self.is_offboard():
            # Keep streaming until PX4 has actually changed from Offboard to
            # Land mode, avoiding an Offboard-loss race during the handover.
            self.publish_offboard_heartbeat()
            self.publish_position_setpoint()

    def publish_vehicle_command(self, command_id, param1=math.nan, param2=math.nan):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command_id)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_offboard(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.last_offboard_request_s = self.now_s()

    def request_land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.last_land_request_s = self.now_s()
        self.get_logger().info("LAND command sent.")

    # ------------------------------------------------------------------
    # Pose lock and pattern construction
    # ------------------------------------------------------------------

    def stationary_pose_ready(self):
        if len(self.pose_window) < self.pose_window.maxlen:
            return False

        xs = [p[0] for p in self.pose_window]
        ys = [p[1] for p in self.pose_window]
        zs = [p[2] for p in self.pose_window]
        yaws = [p[3] for p in self.pose_window]

        horizontal_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        vertical_span = max(zs) - min(zs)

        sin_mean = sum(math.sin(v) for v in yaws) / len(yaws)
        cos_mean = sum(math.cos(v) for v in yaws) / len(yaws)
        mean_yaw = math.atan2(sin_mean, cos_mean)
        yaw_span = max(abs(self.wrap_pi(v - mean_yaw)) for v in yaws)

        return (
            horizontal_span <= 0.05
            and vertical_span <= 0.05
            and yaw_span <= math.radians(5.0)
        )

    def lock_start_pose(self):
        msg = self.local_position
        self.start_x = float(msg.x)
        self.start_y = float(msg.y)
        self.start_z = float(msg.z)
        self.start_yaw = float(msg.heading)

        self.locked_xy_reset_counter = msg.xy_reset_counter
        self.locked_z_reset_counter = msg.z_reset_counter
        self.locked_heading_reset_counter = msg.heading_reset_counter

        self.cmd_x = self.start_x
        self.cmd_y = self.start_y
        self.cmd_z = self.start_z
        self.cmd_yaw = self.start_yaw
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.xy_position_hold_active = True
        self.z_position_hold_active = True

        self.build_pattern()

        self.get_logger().info(
            "Start pose locked: "
            f"x={self.start_x:.3f}, y={self.start_y:.3f}, "
            f"z={self.start_z:.3f}, yaw={self.start_yaw:.3f} rad. "
            "Remain in Position mode and arm manually with SF."
        )
        self.enter_state(self.WAIT_FOR_ARM)

    def body_offset_to_ned(self, forward_m, right_m):
        # PX4 local NED: x=north, y=east; yaw positive clockwise.
        dx = forward_m * math.cos(self.start_yaw) - right_m * math.sin(self.start_yaw)
        dy = forward_m * math.sin(self.start_yaw) + right_m * math.cos(self.start_yaw)
        return self.start_x + dx, self.start_y + dy

    def build_pattern(self):
        d = self.move_distance_m
        one_cycle = [
            ("forward", d, 0.0, self.outer_hold_s),
            ("center", 0.0, 0.0, self.center_hold_s),
            ("backward", -d, 0.0, self.outer_hold_s),
            ("center", 0.0, 0.0, self.center_hold_s),
            ("left", 0.0, -d, self.outer_hold_s),
            ("center", 0.0, 0.0, self.center_hold_s),
            ("right", 0.0, d, self.outer_hold_s),
            ("center", 0.0, 0.0, self.center_hold_s),
        ]

        self.pattern_points = []
        for cycle in range(1, self.pattern_cycles + 1):
            for name, forward_m, right_m, hold_s in one_cycle:
                x, y = self.body_offset_to_ned(forward_m, right_m)
                self.pattern_points.append(
                    {
                        "name": name,
                        "cycle": cycle,
                        "x": x,
                        "y": y,
                        "hold_s": hold_s,
                    }
                )

    # ------------------------------------------------------------------
    # Motion and safety
    # ------------------------------------------------------------------

    def set_xy_endpoint(self, target_x, target_y):
        """Start acceleration-limited velocity guidance to one XY endpoint."""
        self.cmd_x = float(target_x)
        self.cmd_y = float(target_y)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.xy_position_hold_active = False
        return self.update_xy_velocity_command()

    def update_xy_velocity_command(self):
        """Generate a speed command that can brake smoothly at the endpoint."""
        if self.local_position is None:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            return False

        dx = self.cmd_x - self.local_position.x
        dy = self.cmd_y - self.local_position.y
        distance = math.hypot(dx, dy)
        horizontal_speed = math.hypot(
            self.local_position.vx,
            self.local_position.vy,
        )

        # Once the low-speed position-capture phase has started, leave the
        # endpoint to PX4's position loop.  Fall back to velocity guidance only
        # if the vehicle escapes the wider reacquisition radius.
        if self.xy_position_hold_active:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            if distance > self.point_reacquire_tolerance_m:
                self.xy_position_hold_active = False
            else:
                return (
                    distance <= self.point_reached_tolerance_m
                    and horizontal_speed <= self.point_reached_speed_mps
                )

        if distance <= self.point_reached_tolerance_m:
            desired_vx = 0.0
            desired_vy = 0.0
        else:
            # Reserve both the capture radius and a delay/noise margin, then
            # calculate the largest speed that can stop with the configured
            # braking acceleration: v_allow = sqrt(2*a*d_remaining).
            braking_distance = max(
                distance
                - self.point_reached_tolerance_m
                - self.braking_margin_m,
                0.0,
            )
            allowed_speed = math.sqrt(
                2.0 * self.move_brake_accel_mps2 * braking_distance
            )
            desired_speed = min(self.move_speed_mps, allowed_speed)
            desired_vx = desired_speed * dx / distance
            desired_vy = desired_speed * dy / distance

        # Limit the change of the velocity vector on every 10 Hz update.  This
        # avoids step changes, including a sudden full-speed reversal after an
        # overshoot.
        delta_vx = desired_vx - self.cmd_vx
        delta_vy = desired_vy - self.cmd_vy
        delta_speed = math.hypot(delta_vx, delta_vy)
        current_speed_command = math.hypot(self.cmd_vx, self.cmd_vy)
        desired_speed_command = math.hypot(desired_vx, desired_vy)
        reversing = self.cmd_vx * desired_vx + self.cmd_vy * desired_vy < 0.0
        if desired_speed_command < current_speed_command or reversing:
            acceleration_limit = self.move_brake_accel_mps2
        else:
            acceleration_limit = self.move_accel_mps2

        max_delta_speed = acceleration_limit * self.timer_period_s
        if delta_speed > max_delta_speed and delta_speed > 1e-9:
            scale = max_delta_speed / delta_speed
            self.cmd_vx += delta_vx * scale
            self.cmd_vy += delta_vy * scale
        else:
            self.cmd_vx = desired_vx
            self.cmd_vy = desired_vy

        position_capture_radius = (
            self.point_reached_tolerance_m + self.braking_margin_m
        )
        if (
            distance <= position_capture_radius
            and horizontal_speed <= self.point_reached_speed_mps
        ):
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.xy_position_hold_active = True
            return distance <= self.point_reached_tolerance_m

        return False

    def start_takeoff_velocity_control(self):
        """Begin direct vertical-speed guidance toward the takeoff height."""
        self.cmd_vz = 0.0
        self.z_position_hold_active = False

    def update_takeoff_velocity_command(self):
        """Generate a climb command that brakes before the target height."""
        if self.local_position is None:
            self.cmd_vz = 0.0
            return False

        target_z = self.start_z - self.takeoff_height_m
        z_error = target_z - self.local_position.z
        distance = abs(z_error)
        vertical_speed = abs(self.local_position.vz)

        # Once low-speed position capture starts, let PX4 settle at the exact
        # height.  Re-enter velocity guidance only after a large escape.
        if self.z_position_hold_active:
            self.cmd_vz = 0.0
            if distance > self.takeoff_reacquire_tolerance_m:
                self.z_position_hold_active = False
            else:
                return (
                    distance <= self.takeoff_reached_tolerance_m
                    and vertical_speed <= self.takeoff_reached_speed_mps
                )

        if distance <= self.takeoff_reached_tolerance_m:
            desired_vz = 0.0
        else:
            braking_distance = max(
                distance
                - self.takeoff_reached_tolerance_m
                - self.takeoff_braking_margin_m,
                0.0,
            )
            allowed_speed = math.sqrt(
                2.0 * self.takeoff_brake_accel_mps2 * braking_distance
            )
            desired_speed = min(self.takeoff_speed_mps, allowed_speed)
            desired_vz = math.copysign(desired_speed, z_error)

        current_speed_command = abs(self.cmd_vz)
        desired_speed_command = abs(desired_vz)
        reversing = self.cmd_vz * desired_vz < 0.0
        if desired_speed_command < current_speed_command or reversing:
            acceleration_limit = self.takeoff_brake_accel_mps2
        else:
            acceleration_limit = self.takeoff_accel_mps2

        delta_vz = desired_vz - self.cmd_vz
        max_delta_vz = acceleration_limit * self.timer_period_s
        if abs(delta_vz) > max_delta_vz:
            self.cmd_vz += math.copysign(max_delta_vz, delta_vz)
        else:
            self.cmd_vz = desired_vz

        position_capture_radius = (
            self.takeoff_reached_tolerance_m
            + self.takeoff_braking_margin_m
        )
        if (
            distance <= position_capture_radius
            and vertical_speed <= self.takeoff_reached_speed_mps
        ):
            self.cmd_z = target_z
            self.cmd_vz = 0.0
            self.z_position_hold_active = True
            return distance <= self.takeoff_reached_tolerance_m

        return False

    def safety_problem(self):
        if not self.local_position_is_fresh():
            return "local-position update timeout"

        msg = self.local_position
        if not self.local_position_is_valid(msg):
            return "local position/velocity/heading became invalid"

        if msg.xy_reset_counter != self.locked_xy_reset_counter:
            return "EKF horizontal-position reset detected"
        if msg.z_reset_counter != self.locked_z_reset_counter:
            return "EKF vertical-position reset detected"
        if msg.heading_reset_counter != self.locked_heading_reset_counter:
            return "EKF heading reset detected"

        horizontal_radius = math.hypot(msg.x - self.start_x, msg.y - self.start_y)
        height = self.start_z - msg.z
        horizontal_speed = math.hypot(msg.vx, msg.vy)

        if horizontal_radius > self.max_horizontal_radius_m:
            return f"horizontal safety radius exceeded ({horizontal_radius:.2f} m)"
        if height > self.max_height_m:
            return f"height safety limit exceeded ({height:.2f} m)"
        if height < -0.15:
            return f"unexpected downward displacement ({height:.2f} m)"
        if horizontal_speed > self.max_horizontal_speed_mps:
            return f"horizontal speed too high ({horizontal_speed:.2f} m/s)"
        if abs(msg.vz) > self.max_vertical_speed_mps:
            return f"vertical speed too high ({msg.vz:.2f} m/s)"

        horizontal_error = math.hypot(msg.x - self.cmd_x, msg.y - self.cmd_y)
        vertical_error = abs(msg.z - self.cmd_z)
        # During velocity-guided travel, distance to the final endpoint is an
        # intentional remaining distance rather than a tracking failure.  The
        # horizontal position-error timeout applies after position capture;
        # the radius and speed envelopes remain active throughout travel.
        horizontal_tracking_problem = (
            self.xy_position_hold_active
            and horizontal_error > self.max_horizontal_tracking_error_m
        )
        if (
            horizontal_tracking_problem
            or vertical_error > self.max_vertical_tracking_error_m
        ):
            if self.position_error_start_s is None:
                self.position_error_start_s = self.now_s()
            elif self.now_s() - self.position_error_start_s > 1.5:
                return (
                    "setpoint tracking error persisted "
                    f"(xy={horizontal_error:.2f} m, z={vertical_error:.2f} m)"
                )
        else:
            self.position_error_start_s = None

        yaw_error = abs(self.wrap_pi(msg.heading - self.start_yaw))
        if yaw_error > self.max_yaw_error_rad:
            if self.yaw_error_start_s is None:
                self.yaw_error_start_s = self.now_s()
            elif self.now_s() - self.yaw_error_start_s > 1.0:
                return f"yaw error persisted ({math.degrees(yaw_error):.1f} deg)"
        else:
            self.yaw_error_start_s = None

        return None

    def trigger_landing(self, reason):
        if self.state in (self.LAND_REQUESTED, self.WAIT_FOR_DISARM, self.COMPLETE):
            return
        # Stop any velocity-only travel immediately and hold the current XY
        # location until PX4 accepts the Land command.
        if self.local_position is not None:
            self.cmd_x = float(self.local_position.x)
            self.cmd_y = float(self.local_position.y)
            self.cmd_z = float(self.local_position.z)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.xy_position_hold_active = True
        self.z_position_hold_active = True
        self.get_logger().warning(f"Landing triggered: {reason}")
        self.enter_state(self.LAND_REQUESTED)
        self.request_land()

    def handle_unexpected_offboard_exit(self):
        if self.offboard_was_active and not self.is_offboard():
            self.get_logger().warning(
                "Offboard exited by pilot or failsafe. External heartbeat and "
                "setpoints have stopped; RC/PX4 now has control."
            )
            self.enter_state(self.PILOT_TAKEOVER)
            return True
        return False

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------

    def timer_callback(self):
        now = self.now_s()

        if self.state == self.WAIT_FOR_POSE:
            if self.stationary_pose_ready():
                self.lock_start_pose()
            elif now - self.last_status_log_time >= 2.0:
                self.get_logger().info(
                    "Waiting for a fresh, valid and stationary local pose..."
                )
                self.last_status_log_time = now
            return

        if self.state == self.COMPLETE:
            return

        if self.state == self.ABORTED:
            if not self.is_armed():
                self.get_logger().info("Vehicle is disarmed. Abort complete.")
                self.enter_state(self.COMPLETE)
            return

        if self.state == self.PILOT_TAKEOVER:
            # Deliberately publish nothing: pilot/PX4 owns the vehicle now.
            if not self.is_armed():
                self.get_logger().info("Vehicle disarmed after pilot takeover.")
                self.enter_state(self.COMPLETE)
            return

        if self.state == self.WAIT_FOR_DISARM:
            # PX4 Land mode owns the vehicle. Do not continue Offboard messages.
            if not self.is_armed():
                self.get_logger().info("Vehicle disarmed. Test complete.")
                self.enter_state(self.COMPLETE)
            return

        if self.state == self.WAIT_FOR_ARM:
            if self.is_armed():
                if not self.is_position_mode():
                    self.get_logger().error(
                        "Vehicle armed outside Position mode. Set SF to Ready, "
                        "disarm, select Position mode, and restart this program."
                    )
                    self.enter_state(self.ABORTED)
                    return

                self.get_logger().info(
                    "ARMED in Position mode. Requesting Offboard."
                )
                self.offboard_request_started_s = now
                self.request_offboard()
                self.enter_state(self.WAIT_FOR_OFFBOARD)
            return

        if self.state == self.WAIT_FOR_OFFBOARD:
            if not self.is_armed():
                self.get_logger().warning(
                    "Vehicle disarmed before Offboard became active. Test aborted."
                )
                self.enter_state(self.COMPLETE)
                return

            if self.is_offboard():
                self.offboard_was_active = True
                if not self.recording_started:
                    self.recording_started = True
                    self.record_start_time_s = now
                    self.record_sample()
                self.get_logger().info(
                    "Offboard active. Center the throttle now. "
                    f"Takeoff ramp starts after {self.pre_takeoff_delay_s:.1f} seconds."
                )
                self.enter_state(self.PRE_TAKEOFF)
                return

            if now - self.last_offboard_request_s >= 1.0:
                self.request_offboard()

            if now - self.offboard_request_started_s > 5.0:
                self.get_logger().error(
                    "Offboard did not become active within 5 seconds. "
                    "External setpoints stopped; disarm manually with SF."
                )
                self.enter_state(self.ABORTED)
            return

        if self.state in (
            self.PRE_TAKEOFF,
            self.TAKEOFF,
            self.TAKEOFF_HOLD,
            self.PATTERN,
        ):
            if self.handle_unexpected_offboard_exit():
                return

            problem = self.safety_problem()
            if problem is not None:
                self.trigger_landing(problem)
                return

        if self.state == self.PRE_TAKEOFF:
            if now - self.state_enter_time >= self.pre_takeoff_delay_s:
                self.get_logger().info(
                    f"Beginning {self.takeoff_height_m:.2f} m takeoff ramp."
                )
                self.start_takeoff_velocity_control()
                self.enter_state(self.TAKEOFF)
            return

        if self.state == self.TAKEOFF:
            target_z = self.start_z - self.takeoff_height_m
            reached = self.update_takeoff_velocity_command()

            # Integrate the actual velocity command for the expected-height
            # plot.  This includes the acceleration and braking phases instead
            # of drawing an unrealistically immediate constant-speed ramp.
            if not self.z_position_hold_active:
                self.cmd_z = min(
                    self.start_z,
                    max(
                        target_z,
                        self.cmd_z + self.cmd_vz * self.timer_period_s,
                    ),
                )

            if reached:
                self.cmd_z = target_z
                self.cmd_vz = 0.0
                self.z_position_hold_active = True
                self.get_logger().info(
                    "Takeoff target reached and settled. "
                    "Holding before movement pattern."
                )
                self.enter_state(self.TAKEOFF_HOLD)
            elif now - self.state_enter_time > self.takeoff_timeout_s:
                self.trigger_landing(
                    f"takeoff did not settle within {self.takeoff_timeout_s:.1f} s"
                )
            return

        if self.state == self.TAKEOFF_HOLD:
            if now - self.state_enter_time >= self.takeoff_hold_s:
                self.pattern_index = 0
                self.point_reached_time_s = None
                first = self.pattern_points[0]
                self.set_xy_endpoint(first["x"], first["y"])
                self.get_logger().info(
                    f"Starting pattern: cycle 1/{self.pattern_cycles}, "
                    f"moving {first['name']} {self.move_distance_m:.2f} m."
                )
                self.enter_state(self.PATTERN)
            return

        if self.state == self.PATTERN:
            point = self.pattern_points[self.pattern_index]
            if self.point_reached_time_s is None:
                reached = self.update_xy_velocity_command()
                if reached:
                    self.cmd_vx = 0.0
                    self.cmd_vy = 0.0
                    self.point_reached_time_s = now
                    self.get_logger().info(
                        f"Settled at {point['name']} point, "
                        f"cycle {point['cycle']}/{self.pattern_cycles}."
                    )
            else:
                # Arrival is latched.  Keep zero velocity command while
                # holding, and only reacquire after leaving a wider radius.
                self.cmd_vx = 0.0
                self.cmd_vy = 0.0
                distance = math.hypot(
                    self.local_position.x - self.cmd_x,
                    self.local_position.y - self.cmd_y,
                )

                if distance > self.point_reacquire_tolerance_m:
                    self.get_logger().warning(
                        f"Drifted {distance:.2f} m from {point['name']}; "
                        "reacquiring endpoint."
                    )
                    self.cmd_vx = 0.0
                    self.cmd_vy = 0.0
                    self.xy_position_hold_active = False
                    self.point_reached_time_s = None
                elif now - self.point_reached_time_s >= point["hold_s"]:
                    self.pattern_index += 1
                    self.point_reached_time_s = None

                    if self.pattern_index >= len(self.pattern_points):
                        self.cmd_x = self.start_x
                        self.cmd_y = self.start_y
                        self.cmd_vx = 0.0
                        self.cmd_vy = 0.0
                        self.xy_position_hold_active = True
                        self.trigger_landing("movement pattern complete")
                        return

                    next_point = self.pattern_points[self.pattern_index]
                    self.set_xy_endpoint(next_point["x"], next_point["y"])
                    self.get_logger().info(
                        f"Next: {next_point['name']}, "
                        f"cycle {next_point['cycle']}/{self.pattern_cycles}."
                    )
            return

        if self.state == self.LAND_REQUESTED:
            # Keep the last Offboard setpoint alive only until PX4 accepts Land
            # and actually leaves Offboard. This avoids an Offboard-loss race.
            if self.is_offboard():
                if now - self.last_land_request_s >= 1.0:
                    self.request_land()
            else:
                self.get_logger().info(
                    "PX4 left Offboard for landing. External setpoints stopped."
                )
                self.enter_state(self.WAIT_FOR_DISARM)


def main():
    rclpy.init()
    node = AdvancedIndoorOffboardTest()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.record_sample()
        node.save_results(make_plot=not node.is_armed())
        print(
            "Stopped by user. If armed, take over immediately with SC/RC. "
            "Use SH Kill only for imminent collision."
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
