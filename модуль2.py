import numpy as np
import json
from Robots.class_Robot_Roki_2 import Robot
import os
from pathlib import Path
import datetime

today = datetime.date.today()

#print(int(today.year), int(today.month), int(today.day))
try:
    records = np.load("basketball_records.npy")
except Exception:
    records = np.zeros((1,5), dtype = np.int16)
record = np.array([[int(today.year), int(today.month), int(today.day), 1, 1]], dtype = np.int16)
new_records = np.append(records, record, axis=0)
np.save("basketball_records.npy", new_records)
print(new_records)