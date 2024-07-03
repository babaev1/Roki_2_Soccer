import numpy as np
import json
from Robots.class_Robot_Roki_2 import Robot
import os
from pathlib import Path

cwd = os.getcwd()
p = Path(cwd + "\\Soccer\\Motion\\motion_slots")
print(p)
for file in list(p.glob('**/*.json')):
    print(file)
print(file.name)
print(file.parent)


#print(cppText)