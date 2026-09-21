#!/usr/bin/env python3
"""Compare two nav_msgs/Odometry topics from one or two ROS 2 bags."""

import argparse
import math
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    print(f"ROS 2 Python modules are required: {exc}", file=sys.stderr)
    raise SystemExit(2)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def read_odom(uri, topic):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3")
    reader.open(storage, rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in types:
        raise RuntimeError(f"topic {topic!r} not found in {uri}")
    message_type = get_message(types[topic])
    values = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        message = deserialize_message(data, message_type)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        p = message.pose.pose.position
        values.append((stamp, p.x, p.y, yaw(message.pose.pose.orientation)))
    return values


def read_tf(uri, topic, parent_frame, child_frame):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3")
    reader.open(storage, rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in types or types[topic] != "tf2_msgs/msg/TFMessage":
        raise RuntimeError(f"TF topic {topic!r} not found in {uri}")
    message_type = get_message(types[topic])
    values = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        message = deserialize_message(data, message_type)
        for transform in message.transforms:
            if (transform.header.frame_id != parent_frame or
                    transform.child_frame_id != child_frame):
                continue
            stamp = (transform.header.stamp.sec +
                     transform.header.stamp.nanosec * 1e-9)
            p = transform.transform.translation
            values.append((stamp, p.x, p.y, yaw(transform.transform.rotation)))
    if not values:
        raise RuntimeError(
            f"TF {parent_frame!r} -> {child_frame!r} not found on {topic!r}")
    return values


def associate(reference, estimate, tolerance):
    pairs = []
    j = 0
    for item in reference:
        while j + 1 < len(estimate) and abs(estimate[j + 1][0] - item[0]) <= abs(estimate[j][0] - item[0]):
            j += 1
        if estimate and abs(estimate[j][0] - item[0]) <= tolerance:
            pairs.append((item, estimate[j]))
            j += 1
            if j >= len(estimate):
                break
    return pairs


def align_first(pairs):
    reference0, estimate0 = pairs[0]
    angle = wrap(reference0[3] - estimate0[3])
    c, s = math.cos(angle), math.sin(angle)
    tx = reference0[1] - (c * estimate0[1] - s * estimate0[2])
    ty = reference0[2] - (s * estimate0[1] + c * estimate0[2])
    aligned = []
    for ref, est in pairs:
        x = c * est[1] - s * est[2] + tx
        y = s * est[1] + c * est[2] + ty
        aligned.append((ref, (est[0], x, y, wrap(est[3] + angle))))
    return aligned


def path_length(values):
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(values, values[1:]))


def evaluate(pairs):
    position_errors = [math.hypot(ref[1] - est[1], ref[2] - est[2]) for ref, est in pairs]
    yaw_errors = [abs(wrap(ref[3] - est[3])) for ref, est in pairs]
    translation_rpe = []
    rotation_rpe = []
    for (ra, ea), (rb, eb) in zip(pairs, pairs[1:]):
        translation_rpe.append(math.hypot((rb[1] - ra[1]) - (eb[1] - ea[1]),
                                          (rb[2] - ra[2]) - (eb[2] - ea[2])))
        rotation_rpe.append(abs(wrap((rb[3] - ra[3]) - (eb[3] - ea[3]))))
    rmse = lambda values: math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")
    references = [pair[0] for pair in pairs]
    estimates = [pair[1] for pair in pairs]
    reference_length = path_length(references)
    estimate_length = path_length(estimates)
    return {
        "ATE_RMSE_m": rmse(position_errors),
        "ATE_max_m": max(position_errors),
        "yaw_RMSE_deg": math.degrees(rmse(yaw_errors)),
        "translation_RPE_RMSE_m": rmse(translation_rpe),
        "rotation_RPE_RMSE_deg": math.degrees(rmse(rotation_rpe)),
        "path_length_ratio": estimate_length / reference_length if reference_length else float("nan"),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("reference_bag")
    parser.add_argument("estimate_bag")
    parser.add_argument("--reference-topic", default="/orbslam3/odom")
    parser.add_argument("--estimate-topic", default="/rgbd_odometry/odom")
    parser.add_argument("--reference-parent-frame")
    parser.add_argument("--reference-child-frame")
    parser.add_argument("--max-time-difference", type=float, default=0.03)
    args = parser.parse_args()
    tf_reference = bool(args.reference_parent_frame or args.reference_child_frame)
    if tf_reference and not (args.reference_parent_frame and args.reference_child_frame):
        parser.error("both --reference-parent-frame and --reference-child-frame are required")
    if tf_reference:
        reference = read_tf(args.reference_bag, args.reference_topic,
                            args.reference_parent_frame, args.reference_child_frame)
        reference_name = (f"{args.reference_topic}: {args.reference_parent_frame} -> "
                          f"{args.reference_child_frame}")
    else:
        reference = read_odom(args.reference_bag, args.reference_topic)
        reference_name = args.reference_topic
    estimate = read_odom(args.estimate_bag, args.estimate_topic)
    pairs = associate(reference, estimate, args.max_time_difference)
    if len(pairs) < 2:
        raise RuntimeError("fewer than two timestamp-associated pose pairs")
    pairs = align_first(pairs)
    metrics = evaluate(pairs)
    print(f"alignment: first pose rigid SE(2), no scale")
    print(f"association tolerance: {args.max_time_difference:.6f} s")
    print(f"reference: {reference_name}")
    print(f"matched poses: {len(pairs)} / estimate poses: {len(estimate)}")
    overlapping_reference = [p for p in reference if estimate[0][0] <= p[0] <= estimate[-1][0]]
    association_success = min(100.0, 100.0 * len(pairs) / len(overlapping_reference))
    print(f"tracking success: {association_success:.2f}% (unique reference associations)")
    for name, value in metrics.items():
        print(f"{name}: {value:.6f}")


if __name__ == "__main__":
    main()
