#!/usr/bin/python3
"""
pid_velocity_controller.py
==========================
ROS 2 Humble node: simple dual-axis PID velocity controller.

Subscribes : /cmd_vel_raw  (geometry_msgs/msg/Twist)  – desired velocity
Publishes  : /cmd_vel      (geometry_msgs/msg/Twist)  – PID-filtered output

Two independent PID loops:o
  • linear.x  – forward / backward velocity
  • angular.z – rotational velocity

ROS 2 parameters (tunable at runtime via `ros2 param set`):
  linear_kp  / linear_ki  / linear_kd
  angular_kp / angular_ki / angular_kd
  max_linear_vel   (m/s)  – output saturation
  max_angular_vel  (rad/s)
  publish_rate     (Hz)   – control loop rate

Anti-windup: integrator is clamped to ±windup_limit (also a parameter).
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time


class PIDController:
    """Single-axis PID with anti-windup integrator clamp."""

    def __init__(self, kp: float, ki: float, kd: float,
                 windup_limit: float = 1.0,
                 output_limit: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_limit = windup_limit
        self.output_limit = output_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, setpoint: float, measured: float, now: float) -> float:
        """
        Compute PID output.

        Parameters
        ----------
        setpoint : desired value
        measured : current measured value  (we treat setpoint as measured here
                   because we have no plant feedback — see note below)
        now      : current time in seconds (float)

        Note
        ----
        Without a separate wheel-speed sensor we use the *command* itself as
        the 'measured' value, which turns this into a *command smoother /
        rate-limiter* rather than a true closed-loop PID.  When odometry /
        wheel encoders are wired up, replace `measured` with actual velocity.
        """
        error = setpoint - measured

        if self._prev_time is None:
            dt = 0.0
        else:
            dt = now - self._prev_time

        self._prev_time = now

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        if dt > 0:
            self._integral += error * dt
            self._integral = max(-self.windup_limit,
                                 min(self.windup_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative (on error; avoid derivative kick on setpoint change)
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        output = p_term + i_term + d_term

        # Output saturation
        output = max(-self.output_limit, min(self.output_limit, output))
        return output


class PIDVelocityController(Node):

    def __init__(self):
        super().__init__('pid_velocity_controller')

        # ── Declare parameters ────────────────────────────────────────────
        def _desc(desc_str, ptype=ParameterType.PARAMETER_DOUBLE):
            return ParameterDescriptor(description=desc_str, type=ptype)

        self.declare_parameter('linear_kp',       0.8,  _desc('Linear P gain'))
        self.declare_parameter('linear_ki',       0.05, _desc('Linear I gain'))
        self.declare_parameter('linear_kd',       0.1,  _desc('Linear D gain'))

        self.declare_parameter('angular_kp',      1.0,  _desc('Angular P gain'))
        self.declare_parameter('angular_ki',      0.05, _desc('Angular I gain'))
        self.declare_parameter('angular_kd',      0.05, _desc('Angular D gain'))

        self.declare_parameter('max_linear_vel',  0.5,  _desc('Max linear output (m/s)'))
        self.declare_parameter('max_angular_vel', 1.0,  _desc('Max angular output (rad/s)'))
        self.declare_parameter('max_linear_accel', 0.5, _desc('Fallback linear slew rate (m/s^2)'))
        self.declare_parameter('max_angular_accel', 1.0, _desc('Fallback angular slew rate (rad/s^2)'))
        self.declare_parameter('windup_limit',    1.0,  _desc('Integrator anti-windup clamp'))
        self.declare_parameter('publish_rate',   20.0,  _desc('Control loop rate (Hz)'))

        # ── Build PID objects ─────────────────────────────────────────────
        self._linear_pid  = self._make_linear_pid()
        self._angular_pid = self._make_angular_pid()

        # ── State ─────────────────────────────────────────────────────────
        self._setpoint_linear  = 0.0   # from /cmd_vel_raw
        self._setpoint_angular = 0.0
        self._measured_linear  = 0.0   # will be updated from /odom if available
        self._measured_angular = 0.0
        self._last_linear_output = 0.0
        self._last_angular_output = 0.0
        self._last_loop_time = time.monotonic()

        # Timeout: if no /cmd_vel_raw within this many seconds, stop the robot
        self._cmd_timeout_sec = 0.5
        self._last_cmd_time = 0.0
        self._last_odom_time = 0.0

        # ── PubSub ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self._cmd_callback,
            10
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        # ── Control loop timer ────────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._control_loop)

        # ── Parameter callback (live re-tune) ─────────────────────────────
        self.add_on_set_parameters_callback(self._param_callback)

        self.get_logger().info(
            'PID Velocity Controller started.\n'
            '  Subscribing : /cmd_vel_raw\n'
            '  Publishing  : /cmd_vel\n'
            '  Linear  PID : Kp=%.2f Ki=%.2f Kd=%.2f\n'
            '  Angular PID : Kp=%.2f Ki=%.2f Kd=%.2f' % (
                self.get_parameter('linear_kp').value,
                self.get_parameter('linear_ki').value,
                self.get_parameter('linear_kd').value,
                self.get_parameter('angular_kp').value,
                self.get_parameter('angular_ki').value,
                self.get_parameter('angular_kd').value,
            )
        )

    # ── PID factory helpers ───────────────────────────────────────────────

    def _make_linear_pid(self):
        return PIDController(
            kp=self.get_parameter('linear_kp').value,
            ki=self.get_parameter('linear_ki').value,
            kd=self.get_parameter('linear_kd').value,
            windup_limit=self.get_parameter('windup_limit').value,
            output_limit=self.get_parameter('max_linear_vel').value,
        )

    def _make_angular_pid(self):
        return PIDController(
            kp=self.get_parameter('angular_kp').value,
            ki=self.get_parameter('angular_ki').value,
            kd=self.get_parameter('angular_kd').value,
            windup_limit=self.get_parameter('windup_limit').value,
            output_limit=self.get_parameter('max_angular_vel').value,
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cmd_callback(self, msg: Twist):
        """Receive raw velocity command."""
        self._setpoint_linear  = msg.linear.x
        self._setpoint_angular = msg.angular.z
        self._last_cmd_time    = time.monotonic()

    # Uncomment for real odom feedback:
    def _odom_callback(self, msg):
        self._measured_linear = msg.twist.twist.linear.x
        self._measured_angular = msg.twist.twist.angular.z
        self._last_odom_time = time.monotonic()

    def _param_callback(self, params) -> SetParametersResult:
        """Allow live re-tuning via `ros2 param set`."""
        rebuild = False
        for p in params:
            if p.name in ('linear_kp', 'linear_ki', 'linear_kd',
                          'angular_kp', 'angular_ki', 'angular_kd',
                          'max_linear_vel', 'max_angular_vel', 'windup_limit'):
                rebuild = True

        if rebuild:
            self._linear_pid  = self._make_linear_pid()
            self._angular_pid = self._make_angular_pid()
            self.get_logger().info('PID gains updated – PIDs reset.')

        return SetParametersResult(successful=True)

    def _control_loop(self):
        """Main PID control loop — runs at publish_rate Hz."""
        now = time.monotonic()
        dt = now - self._last_loop_time
        self._last_loop_time = now

        # Safety: if command is stale, command zero
        if now - self._last_cmd_time > self._cmd_timeout_sec and self._last_cmd_time > 0.0:
            self._setpoint_linear  = 0.0
            self._setpoint_angular = 0.0
            self._linear_pid.reset()
            self._angular_pid.reset()

        if now - self._last_odom_time < 0.5:
            # Closed-loop velocity PID when odometry is available.
            lin_out = self._linear_pid.compute(
                setpoint=self._setpoint_linear,
                measured=self._measured_linear,
                now=now
            )
            ang_out = self._angular_pid.compute(
                setpoint=self._setpoint_angular,
                measured=self._measured_angular,
                now=now
            )
        else:
            # No measured velocity yet: behave as a stable command smoother.
            lin_out = self._slew(
                self._last_linear_output,
                self._setpoint_linear,
                self.get_parameter('max_linear_accel').value,
                dt
            )
            ang_out = self._slew(
                self._last_angular_output,
                self._setpoint_angular,
                self.get_parameter('max_angular_accel').value,
                dt
            )

        self._last_linear_output = lin_out
        self._last_angular_output = ang_out

        # Publish
        out_msg = Twist()
        out_msg.linear.x  = lin_out
        out_msg.angular.z = ang_out
        self._pub.publish(out_msg)

    @staticmethod
    def _slew(current: float, target: float, max_rate: float, dt: float) -> float:
        if dt <= 0:
            return current

        max_step = abs(max_rate) * dt
        error = target - current
        if error > max_step:
            return current + max_step
        if error < -max_step:
            return current - max_step
        return target


def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
