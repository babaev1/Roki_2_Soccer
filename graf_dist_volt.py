import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splrep, BSpline, splev, CubicSpline
from scipy.interpolate import interp1d

#voltages1 = [11.6, 11.61, 11.65, 11.67, 11.7, 11.72, 11.76, 11.88, 11.91, 11.94, 11.99, 12.05, 12.1, 12.12, 12.19, 12.24, 12.51, 12.61]
#angles1 = [600,500,400,350,300,220,180,80,50,0,-50, -150, -220, -230, -250, -250, -350, -450]
#voltages2 = [12.14, 12.2, 12.23, 12.25, 12.4, 12.44, 12.5, 12.55]
#angles2 = [450, 350, 350, 300, 190, 150, 50, 0]
#voltages3 = [11.47, 11.52, 11.53, 11.57, 11.62, 11.65, 11.7, 11.7, 11.72]
#angles3 = [6400, 6260, 6230, 6200, 6150, 6150, 6150, 6100, 6000]

#f2 = splrep(voltages1, angles1, s=20)

#cs = CubicSpline(voltages1, angles1)
#print(cs(12.7))

#xnew = np.arange(11.6, 12.6, 0.1)

#plt.plot(voltages1, angles1, voltages2, angles2)
#plt.show()


data = np.load("basketball_records.npy")

battery_1_volt = []
battery_1_anlgle = []
battery_3_volt = []
battery_3_anlgle = []
for i in range( len(data)):
    if data[i][3] == 1:
        battery_1_volt.append(data[i][4]/270.2)
        battery_1_anlgle.append(data[i][5])
    if data[i][3] == 3:
        battery_3_volt.append(data[i][4]/270.2)
        battery_3_anlgle.append(data[i][5])

plt.plot(battery_1_volt, battery_1_anlgle, 'ro', battery_3_volt, battery_3_anlgle, 'bo')#, battery_2_volt, battery_2_anlgle)
plt.show()