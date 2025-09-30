import rclpy.serialization
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# === CONFIG ===
bag_path = "rosbag2_2025_09_30-15_13_05"
topic_name = "/target_poses"
msg_type = "geometry_msgs/msg/PoseArray"

# === ROS2 bag reader ===
storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id="mcap"
)

converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr"
)

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
topic_type_dict = {t.name: t.type for t in topic_types}

msg_class = get_message(msg_type)

timestamps = []
xs, ys, norms = [], [], []

while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == topic_name:
        msg = deserialize_message(data, msg_class)
        if msg.poses:  # array non vuoto
            pose0 = msg.poses[0]
            x = pose0.position.x
            y = pose0.position.y
            norm = np.sqrt(x**2 + y**2)

            timestamps.append(t * 1e-9)  # ns -> s
            xs.append(x)
            ys.append(y)
            norms.append(norm)

# === DataFrame e plot ===
df = pd.DataFrame({
    "time_s": timestamps,
    "x": xs,
    "y": ys,
    "norm": norms
})

plt.figure()
plt.plot(df["time_s"], df["x"], label="x")
plt.plot(df["time_s"], df["y"], label="y")
plt.plot(df["time_s"], df["norm"], label="norm")
plt.xlabel("Time [s]")
plt.ylabel("Value")
plt.title("PoseArray[0] position")
plt.legend()
plt.show()
