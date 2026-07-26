"""
parity_check.py — verify Python node output matches C++ node within tolerance.

Subscribes to both /filtered/... topics from C++ and Python nodes and
compares sample by sample.

Tolerances (from spec):
  integer streams : bit-exact  (diff must be 0)
  float streams   : rtol=1e-6  (relative tolerance)

Run after launching both sig_proc_cpp and sig_proc_py:
  ros2 run sig_proc_py parity_check
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

from sig_msgs.msg import StampedInt64, StampedFloat64

PIPELINE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


class ParityCheck(Node):
    def __init__(self):
        super().__init__('parity_check')

        self._cpp_enc_lp = {}   # ts_ns -> value
        self._py_enc_lp  = {}

        self._cpp_acc_lp = {}
        self._py_acc_lp  = {}

        self._total = 0
        self._pass  = 0

        # C++ filtered topics
        self.create_subscription(StampedInt64,
            '/filtered/encoder/lowpass',
            lambda m: self._store(self._cpp_enc_lp, m, int),
            PIPELINE_QOS)

        # Python filtered topics (different node, same topic names with /py prefix)
        self.create_subscription(StampedInt64,
            '/py/filtered/encoder/lowpass',
            lambda m: self._store(self._py_enc_lp, m, int),
            PIPELINE_QOS)

        self.create_subscription(StampedFloat64,
            '/filtered/accel/lowpass',
            lambda m: self._store(self._cpp_acc_lp, m, float),
            PIPELINE_QOS)

        self.create_subscription(StampedFloat64,
            '/py/filtered/accel/lowpass',
            lambda m: self._store(self._py_acc_lp, m, float),
            PIPELINE_QOS)

        self.create_timer(2.0, self._report)
        self.get_logger().info('ParityCheck running — comparing C++ vs Python outputs')

    def _store(self, store, msg, cast):
        ts = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        store[ts] = cast(msg.value)
        self._compare()

    def _compare(self):
        # Check integer encoder parity
        for ts in list(self._cpp_enc_lp):
            if ts in self._py_enc_lp:
                cpp_v = self._cpp_enc_lp.pop(ts)
                py_v  = self._py_enc_lp.pop(ts)
                self._total += 1
                if cpp_v == py_v:
                    self._pass += 1
                else:
                    self.get_logger().error(
                        f'INT MISMATCH ts={ts}: cpp={cpp_v} py={py_v} diff={cpp_v-py_v}')

        # Check float accel parity
        for ts in list(self._cpp_acc_lp):
            if ts in self._py_acc_lp:
                cpp_v = self._cpp_acc_lp.pop(ts)
                py_v  = self._py_acc_lp.pop(ts)
                self._total += 1
                rtol = abs(cpp_v - py_v) / (abs(cpp_v) + 1e-12)
                if rtol <= 1e-6:
                    self._pass += 1
                else:
                    self.get_logger().error(
                        f'FLOAT MISMATCH ts={ts}: cpp={cpp_v:.8f} py={py_v:.8f} rtol={rtol:.2e}')

    def _report(self):
        if self._total > 0:
            self.get_logger().info(
                f'Parity: {self._pass}/{self._total} passed '
                f'({100*self._pass/self._total:.1f}%)')


def main(args=None):
    rclpy.init(args=args)
    node = ParityCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
