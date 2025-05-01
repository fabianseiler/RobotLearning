#!/usr/bin/env python

import rosbag
import pandas as pd
from sensor_msgs.msg import JointState
from tqdm import tqdm

bag_path = '/root/catkin_ws/src/rl_PaP/grip1.bag'
output_csv = 'joint_states.csv'

# Leere Liste zum Speichern der Einträge
data = []

# Bag-Datei öffnen
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in tqdm(bag.read_messages(topics=['/joint_states'])):
        time_sec = msg.header.stamp.to_sec()
        for i, name in enumerate(msg.name):
            data.append({
                'time': time_sec,
                'joint': name,
                'position': msg.position[i] if i < len(msg.position) else None,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else None,
                'effort': msg.effort[i] if i < len(msg.effort) else None
            })

# In DataFrame umwandeln und als CSV speichern
df = pd.DataFrame(data)
df.to_csv(output_csv, index=False)
print(f"CSV gespeichert unter: {output_csv}")
