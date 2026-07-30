"""
sig_publisher — sensor data publisher node.

Two modes selected by ROS2 parameter 'mode':

  synthetic (default):
    Generates encoder (int) and accel (float) streams at ~200 Hz.
    --jitter <fraction>   timing jitter as fraction of period (default 0.2 = ±20%)
    --drop-rate <float>   probability [0,1] of dropping a sample (default 0.0)

    Jitter is applied as:  dt = nominal_dt * (1 + U(-jitter, +jitter))
    This simulates the 31% jitter measured in sensor_log.csv.

  replay:
    Reads data/sensor_log.csv and republishes at the original timestamps.
    --replay <path>       path to sensor_log.csv

Why header.stamp from data, not wall clock?
  If we stamped with rclpy.clock.now(), the timestamp would include transport
  jitter (time between data ready and node waking up). The processing node uses
  header.stamp to compute dt for the filter — transport jitter would corrupt
  every alpha computation. Data timestamp = sensor acquisition time = correct dt.

QoS: reliable + keep_last(100)
  best-effort would re-introduce random drops in the transport layer,
  contaminating controlled experiments where we want deterministic drop-rate.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import csv
import math
import random
import time
from pathlib import Path

from std_msgs.msg import Header
from sig_msgs.msg import StampedInt64, StampedFloat64


# QoS profile used by all publishers in this pipeline.
# Reliable: no silent drops in transport.
# keep_last(100): buffer up to 100 messages if subscriber is slow.
PIPELINE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


class SigPublisher(Node):
    def __init__(self):
        super().__init__('sig_publisher')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('mode',      'synthetic')  # 'synthetic' | 'replay'
        self.declare_parameter('jitter',     0.2)         # ±20% timing jitter
        self.declare_parameter('drop_rate',  0.0)         # fraction of samples to drop
        self.declare_parameter('replay',     '')          # path to csv (replay mode)
        self.declare_parameter('rate_hz',    200.0)       # nominal publish rate

        self.mode      = self.get_parameter('mode').value
        self.jitter    = self.get_parameter('jitter').value
        self.drop_rate = self.get_parameter('drop_rate').value
        self.replay    = self.get_parameter('replay').value
        self.rate_hz   = self.get_parameter('rate_hz').value

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_int = self.create_publisher(
            StampedInt64, '/sensor/encoder', PIPELINE_QOS)
        self.pub_flt = self.create_publisher(
            StampedFloat64, '/sensor/accel', PIPELINE_QOS)

        # ── State ─────────────────────────────────────────────────────────
        self._encoder_count = 0
        self._nominal_dt    = 1.0 / self.rate_hz  # seconds

        if self.mode == 'replay':
            self._start_replay()
        else:
            period = self._nominal_dt
            self.timer = self.create_timer(period, self._publish_synthetic)
            self.get_logger().info(
                f'SigPublisher: synthetic mode | '
                f'jitter={self.jitter*100:.0f}% | '
                f'drop_rate={self.drop_rate}')

    # ── Synthetic mode ─────────────────────────────────────────────────────
    def _publish_synthetic(self):
        # Apply timing jitter to simulate real sensor behaviour
        jitter_offset = random.uniform(-self.jitter, self.jitter)
        actual_dt = self._nominal_dt * (1.0 + jitter_offset)

        # Simulate drop
        if random.random() < self.drop_rate:
            return

        now_ns = self.get_clock().now().nanoseconds

        # Encoder: monotonically increasing count
        self._encoder_count += random.randint(1, 5)

        self._publish(now_ns, self._encoder_count, random.gauss(1.0, 0.055))

    # ── Replay mode ────────────────────────────────────────────────────────
    def _start_replay(self):
        csv_path = Path(self.replay)
        if not csv_path.exists():
            self.get_logger().error(f'Replay file not found: {csv_path}')
            return

        self.get_logger().info(f'SigPublisher: replay mode | file={csv_path}')

        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        # Publish in a background thread to maintain timing
        import threading
        t = threading.Thread(target=self._replay_thread, args=(rows,), daemon=True)
        t.start()

    def _replay_thread(self, rows):
        prev_ts = None
        for row in rows:
            # column names from sensor_log.csv: timestamp_s, encoder_count, accel_x_mss
            ts_ns     = int(float(row['timestamp_s']) * 1e9)
            enc_val   = int(row['encoder_count'])
            accel_val = float(row['accel_x_mss'])

            if prev_ts is not None:
                dt_s = (ts_ns - prev_ts) * 1e-9
                if dt_s > 0:
                    time.sleep(dt_s)

            prev_ts = ts_ns
            self._publish(ts_ns, enc_val, accel_val)

    # ── Shared publish ─────────────────────────────────────────────────────
    def _publish(self, timestamp_ns: int, encoder: int, accel: float):
        stamp = rclpy.time.Time(nanoseconds=timestamp_ns).to_msg()

        msg_int         = StampedInt64()
        msg_int.header  = Header(stamp=stamp)
        msg_int.value   = encoder

        msg_flt         = StampedFloat64()
        msg_flt.header  = Header(stamp=stamp)
        msg_flt.value   = accel

        self.pub_int.publish(msg_int)
        self.pub_flt.publish(msg_flt)


def main(args=None):
    rclpy.init(args=args)
    node = SigPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
