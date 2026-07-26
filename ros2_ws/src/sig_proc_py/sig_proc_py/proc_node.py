"""
sig_proc_py — Python signal processing node via pybind11 bindings.

Mirrors sig_proc_cpp exactly:
  Subscribes : /sensor/encoder   (StampedInt64)
               /sensor/accel     (StampedFloat64)
  Publishes  : /filtered/encoder/lowpass   (StampedInt64)
               /filtered/encoder/mavg      (StampedInt64)
               /filtered/accel/lowpass     (StampedFloat64)
               /filtered/accel/mavg        (StampedFloat64)

Filter parameters (from FINDINGS.md — same as C++ node):
  Low-pass cutoff : 20 Hz
  MA window       : 25 ms (time-window mode)

Why header.stamp not rclpy.clock.now()?
  header.stamp = sensor acquisition time.
  rclpy.clock.now() = arrival time = sensor time + transport jitter.
  The filter uses timestamps to compute dt for alpha. Transport jitter
  would corrupt every alpha computation.

Parity with C++ node:
  Integer streams must be bit-exact.
  Float streams must match within rtol=1e-6.
  This proves the pybind11 bindings correctly expose the same C++ logic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

# pybind11 module — same C++ logic as sig_proc_cpp but called from Python
import siglib_py

from sig_msgs.msg import StampedInt64, StampedFloat64


PIPELINE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


class SigProcPy(Node):
    def __init__(self):
        super().__init__('sig_proc_py')

        # ── Filters (same parameters as C++ node) ─────────────────────────
        # LowPass: fc=20Hz, derived from FINDINGS.md (97.4% signal power ≤20Hz)
        self.lp_int = siglib_py.LowPassI32(20.0)
        self.lp_flt = siglib_py.LowPassF32(20.0)

        # MovingAverage: time-window 25ms, from FINDINGS.md T=25ms
        # capacity=10 gives headroom for bursts within the 25ms window
        self.ma_int = siglib_py.MovingAverageI32(capacity=5,  window_ns=25_000_000)
        self.ma_flt = siglib_py.MovingAverageF32(capacity=10, window_ns=25_000_000)

        # ── Subscriptions ──────────────────────────────────────────────────
        self.sub_enc = self.create_subscription(
            StampedInt64, '/sensor/encoder',
            self.on_encoder, PIPELINE_QOS)

        self.sub_acc = self.create_subscription(
            StampedFloat64, '/sensor/accel',
            self.on_accel, PIPELINE_QOS)

        # ── Publishers ─────────────────────────────────────────────────────
        self.pub_enc_lp = self.create_publisher(
            StampedInt64, '/filtered/encoder/lowpass', PIPELINE_QOS)
        self.pub_enc_ma = self.create_publisher(
            StampedInt64, '/filtered/encoder/mavg', PIPELINE_QOS)
        self.pub_acc_lp = self.create_publisher(
            StampedFloat64, '/filtered/accel/lowpass', PIPELINE_QOS)
        self.pub_acc_ma = self.create_publisher(
            StampedFloat64, '/filtered/accel/mavg', PIPELINE_QOS)

        self.get_logger().info('sig_proc_py ready — fc=20Hz, MA window=25ms')

    def on_encoder(self, msg: StampedInt64):
        # Timestamp from header — not arrival time
        ts_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds

        # np.int32 required — noconvert() in bindings rejects plain Python int
        val = np.int32(msg.value)

        out_lp = StampedInt64()
        out_lp.header = msg.header
        out_lp.value  = int(self.lp_int.update(ts_ns, val))
        self.pub_enc_lp.publish(out_lp)

        out_ma = StampedInt64()
        out_ma.header = msg.header
        out_ma.value  = int(self.ma_int.update(ts_ns, val))
        self.pub_enc_ma.publish(out_ma)

    def on_accel(self, msg: StampedFloat64):
        ts_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        val   = float(msg.value)

        out_lp = StampedFloat64()
        out_lp.header = msg.header
        out_lp.value  = float(self.lp_flt.update(ts_ns, val))
        self.pub_acc_lp.publish(out_lp)

        out_ma = StampedFloat64()
        out_ma.header = msg.header
        out_ma.value  = float(self.ma_flt.update(ts_ns, val))
        self.pub_acc_ma.publish(out_ma)


def main(args=None):
    rclpy.init(args=args)
    node = SigProcPy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
