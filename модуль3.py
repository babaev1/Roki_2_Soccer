from math import pi
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

#plt.style.use('_mpl-gallery')

X, Y, Z = axes3d.get_test_data(0.05)
r = np.arange(1,10, 0.1, dtype=np.float16)
Xlim = np.arccos(- 1/ r)
#plt.plot(r, Xlim)

##X = np.arange(0, Xlim, 0.1, dtype=np.float16)
#Z = np.arccos((r + np.cos(Xlim))/ np.sqrt(r**2 + 1+ 2*r*np.cos(Xlim)))

#fig, ax = plt.subplots(subplot_kw ={"projection": "3d"})
##ax.plot_wireframe(X, Y, Z, rstride = 10, cstride = 10)
#ax.plot_wireframe(r, Xlim, Z, rstride = 10, cstride = 10)

#ax.set(xticklabels = [],
#       yticklabels = [],
#       zticklabels = [])

r = 1
alpha = np.arange(0, 0.75*pi, 0.1, dtype=np.float16 )
betta = np.arccos((r + np.cos(alpha))/ np.sqrt(r**2 + 1+ 2*r*np.cos(alpha)))
derivative_betta = - ((r * np.sin(alpha) * (np.cos(alpha) + r))/(r**2 + 2 * r * np.cos(alpha) + 1)**1.5 - np.sin(alpha)/np.sqrt(r**2 + 2*r*np.cos(alpha) + 1))/ np.sqrt(1 - (np.cos(alpha) + r)**2/ (r**2 + 2*r*np.cos(alpha) + 1))

plt.plot(alpha, betta, alpha, derivative_betta)
plt.show()

