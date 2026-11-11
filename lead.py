"""Load a lead vehicle speed profile from a ROS1 bag."""

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

# Tried in order when no topic is given. The first is the lead car's own speed
# in multi-car bags; the second is the recording car's speed, which then
# serves as the lead trajectory for the simulated platoon.
DEFAULT_TOPICS = ("/leadcar/car/state/vel_x", "/car/state/vel_x")


def load_lead_from_bag(bag_path, dt, topic=None, start=0.0, end=None):
    """
    Read a speed topic (std_msgs/Float64, m/s) from a bag and resample it onto
    a uniform grid with step dt.

    Parameters:
    bag_path: Path to the .bag file.
    dt: Time step to resample onto, matching the sim's dt.
    topic: Speed topic to read. Defaults to the first of DEFAULT_TOPICS present.
    start: Seconds after the bag's start to begin the profile.
    end: Seconds after the bag's start to end the profile (default: bag end).

    Returns:
    lead_vel (np.ndarray): Lead speed at each time step, starting at `start`.
    """
    typestore = get_typestore(Stores.ROS1_NOETIC)
    with Reader(bag_path) as reader:
        topics = {c.topic for c in reader.connections}
        if topic is None:
            topic = next((t for t in DEFAULT_TOPICS if t in topics), None)
            if topic is None:
                raise ValueError(
                    f"None of {DEFAULT_TOPICS} are in {bag_path}; pass a topic explicitly."
                )
        elif topic not in topics:
            raise ValueError(f"Topic {topic!r} is not in {bag_path}.")

        times, speeds = [], []
        connections = [c for c in reader.connections if c.topic == topic]
        for conn, stamp, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(raw, conn.msgtype)
            times.append((stamp - reader.start_time) / 1e9)
            speeds.append(float(msg.data))

    times = np.asarray(times)
    speeds = np.asarray(speeds)
    end = times[-1] if end is None else min(end, times[-1])
    start = max(start, times[0])
    if end <= start:
        raise ValueError(f"Empty time window: start={start:.2f}s, end={end:.2f}s.")

    grid = np.arange(start, end, dt)
    return np.interp(grid, times, speeds)
