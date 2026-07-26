// proc_node.cpp — C++ signal processing node
//
// Subscribes to:
//   /sensor/encoder   (StampedInt64)  — encoder counts
//   /sensor/accel     (StampedFloat64) — accelerometer m/s²
//
// Publishes filtered outputs on:
//   /filtered/encoder  (StampedInt64)
//   /filtered/accel    (StampedFloat64)
//
// Filter configuration (from FINDINGS.md):
//   Low-pass cutoff : 20 Hz  (retains 97.4% of accel signal power)
//   MA window       : 25 ms  (time-window mode, T = 25 ms)
//   MA count        : 5      (count-mode fallback)
//
// dt computation: uses header.stamp of incoming message, never rclcpp::now().
//   Reason: arrival time includes transport jitter on top of sensor jitter.
//   header.stamp = sensor acquisition time = correct dt for filter alpha.
//
// Dropout policy: dt-extended.
//   No special case for the 157ms gap. alpha = dt/(tau+dt) naturally
//   approaches 1 for large dt — filter output jumps toward input and
//   re-converges over the next few samples.
//
// QoS: reliable + keep_last(100)
//   best-effort would re-introduce drops in transport, contaminating
//   controlled drop-rate experiments.

#include <rclcpp/rclcpp.hpp>
#include <sig_msgs/msg/stamped_int64.hpp>
#include <sig_msgs/msg/stamped_float64.hpp>

#include "siglib/low_pass.hpp"
#include "siglib/moving_average.hpp"
#include "siglib/sample.hpp"

using StampedInt64   = sig_msgs::msg::StampedInt64;
using StampedFloat64 = sig_msgs::msg::StampedFloat64;

// QoS profile: reliable delivery, buffer last 100 messages.
static rclcpp::QoS pipeline_qos() {
    return rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
}

class SigProcCpp : public rclcpp::Node {
public:
    SigProcCpp()
    : Node("sig_proc_cpp")
    // Filter parameters from FINDINGS.md
    , lp_int_(20.0)               // LowPass<int32_t>  fc=20Hz
    , lp_flt_(20.0)               // LowPass<float>    fc=20Hz
    , ma_int_(5,  25'000'000LL)   // MA<int32_t>  time-window 25ms
    , ma_flt_(10, 25'000'000LL)   // MA<float>    time-window 25ms, capacity=10
    {
        // Subscriptions
        sub_enc_ = create_subscription<StampedInt64>(
            "/sensor/encoder", pipeline_qos(),
            [this](StampedInt64::SharedPtr msg) { on_encoder(msg); });

        sub_acc_ = create_subscription<StampedFloat64>(
            "/sensor/accel", pipeline_qos(),
            [this](StampedFloat64::SharedPtr msg) { on_accel(msg); });

        // Publishers
        pub_enc_lp_ = create_publisher<StampedInt64>("/filtered/encoder/lowpass",  pipeline_qos());
        pub_enc_ma_ = create_publisher<StampedInt64>("/filtered/encoder/mavg",      pipeline_qos());
        pub_acc_lp_ = create_publisher<StampedFloat64>("/filtered/accel/lowpass",   pipeline_qos());
        pub_acc_ma_ = create_publisher<StampedFloat64>("/filtered/accel/mavg",      pipeline_qos());

        RCLCPP_INFO(get_logger(), "sig_proc_cpp ready — fc=20Hz, MA window=25ms");
    }

private:
    // ── Encoder callback ──────────────────────────────────────────────────
    void on_encoder(const StampedInt64::SharedPtr msg) {
        // Extract timestamp from header — NOT arrival time
        const int64_t ts_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        const siglib::Sample<int32_t> s{ts_ns, static_cast<int32_t>(msg->value)};

        // Low-pass filtered encoder
        StampedInt64 out_lp;
        out_lp.header = msg->header;
        out_lp.value  = lp_int_.update(s);
        pub_enc_lp_->publish(out_lp);

        // Moving average filtered encoder
        StampedInt64 out_ma;
        out_ma.header = msg->header;
        out_ma.value  = ma_int_.update(s);
        pub_enc_ma_->publish(out_ma);
    }

    // ── Accel callback ────────────────────────────────────────────────────
    void on_accel(const StampedFloat64::SharedPtr msg) {
        const int64_t ts_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        const siglib::Sample<float> s{ts_ns, static_cast<float>(msg->value)};

        StampedFloat64 out_lp;
        out_lp.header = msg->header;
        out_lp.value  = lp_flt_.update(s);
        pub_acc_lp_->publish(out_lp);

        StampedFloat64 out_ma;
        out_ma.header = msg->header;
        out_ma.value  = ma_flt_.update(s);
        pub_acc_ma_->publish(out_ma);
    }

    // ── Filters (constructed once, zero alloc in callbacks) ───────────────
    siglib::LowPass<int32_t>       lp_int_;
    siglib::LowPass<float>         lp_flt_;
    siglib::MovingAverage<int32_t> ma_int_;
    siglib::MovingAverage<float>   ma_flt_;

    // ── Subscriptions & publishers ────────────────────────────────────────
    rclcpp::Subscription<StampedInt64>::SharedPtr   sub_enc_;
    rclcpp::Subscription<StampedFloat64>::SharedPtr sub_acc_;

    rclcpp::Publisher<StampedInt64>::SharedPtr   pub_enc_lp_;
    rclcpp::Publisher<StampedInt64>::SharedPtr   pub_enc_ma_;
    rclcpp::Publisher<StampedFloat64>::SharedPtr pub_acc_lp_;
    rclcpp::Publisher<StampedFloat64>::SharedPtr pub_acc_ma_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SigProcCpp>());
    rclcpp::shutdown();
    return 0;
}
