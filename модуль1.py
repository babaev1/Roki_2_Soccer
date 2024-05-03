from math import pi
import numpy as np
from mdh.kinematic_chain import KinematicChain
from mdh import UnReachable # exception
import time

# make it print better
np.set_printoptions(suppress=True)

# modified DH parameters: alpha a theta d
# types: revolute=1, prismatic=2 (not implemented yet)
dh = [
    {'alpha': 0,  'a': 0, 'theta': 0, 'd': 0, 'type': 1},
    {'alpha': pi/2, 'a': 52, 'theta': 0, 'd': 0, 'type': 1},
    {'alpha': 0, 'a': 89, 'theta': 0, 'd': 0, 'type': 1},
    {'alpha': 0, 'a': 90, 'theta': 0, 'd': 0, 'type': 1},
    {'alpha': 0, 'a': 95, 'theta': 0, 'd': 0, 'type': 1},
    {'alpha': 0, 'a': 95, 'theta': 0, 'd': 0, 'type': 1}
]

kc = KinematicChain.from_parameters(dh)

start0 = time.perf_counter()
# forward kinematics
for _ in range(1000):
    angles = np.deg2rad([-45.00, 77.41, -98.15, -69.27, 0])
    angles = np.deg2rad([0, 0, 0, 0, 0, 0])
    t = kc.forward(angles)
print('time: ', time.perf_counter() - start0)
print(f">> {t}")

# inverse kinematics
#pt = [110,0,-70]
#deg = kc.inverse(pt)
#rad = np.rad2deg(deg)
#print(f">> {rad}")

j = [
    [204.93, 0, -32.067],
    #[110,-110,-40],
    #[110,110,-40],
    [110,0,-70]
]
start0 = time.perf_counter()
for _ in range(500):
    for jj in j:
        p = np.array([
            [0,1,0, jj[0]],
            [0,0,-1,jj[1]],
            [-1,0,0,jj[2]],
            [0,0,0,1]
        ])
        rads = kc.inverse(p)
print('time: ', time.perf_counter() - start0)

print(">>", np.rad2deg(rads))
