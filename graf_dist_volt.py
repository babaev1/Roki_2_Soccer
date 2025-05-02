import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splrep, BSpline, splev, CubicSpline
from scipy.interpolate import interp1d
from sklearn.preprocessing import PolynomialFeatures
from sklearn import linear_model

def dist_angle_from_voltage(battery_number, voltage):
    data = np.load("basketball_records.npy")
    volt = []
    angle = []
    battery = []
    for i in range( len(data)):
        if data[i][3] == battery_number:
            battery.append([data[i][4]/270.2, data[i][5]])
    battery_s = sorted(battery)
    for i in range( len(battery_s)):
        if battery_s[i][0] == battery_s[i-1][0]:
            angle[-1] = (angle[-1] + battery_s[i][1]) / 2
        else:
            volt.append(battery_s[i][0])
            angle.append(battery_s[i][1])
    cs = CubicSpline(volt, angle)
    tck = splrep(volt, angle, k = 5, s = 0)
    tck_s = splrep(volt, angle,  k = 3, s = 20000)
    xnew = np.arange(11.4, 12.8, 0.01)
    plt.plot(volt, angle, 'ro')
    #plt.plot( xnew, cs(xnew))
    plt.plot( xnew, BSpline(*tck_s)(xnew))
    return cs(voltage)

def linear_ransac_curve_fit(x, y):
    x1 = np.array(x).reshape((-1, 1))
    y1 = np.array(y).reshape((-1, 1))
    xi = np.linspace(min(x), max(x), 500).reshape((-1, 1))
    
    reg = linear_model.RANSACRegressor(linear_model.LinearRegression())
    reg.fit(x1, y1)
    yi = reg.predict(xi)
    coeff = reg.estimator_.coef_
    intercept = reg.estimator_.intercept_[0]
    coeff = np.array([intercept, coeff[0, 0]])

    inliers = reg.inlier_mask_
    outliers = np.logical_not(inliers)

    plt.plot(x[inliers], y[inliers], 'k.', label='inliers')
    plt.plot(x[outliers], y[outliers], 'r.', label='outliers')
    plt.plot(xi, yi, label='Linear Regression')
    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Linear')
    print('Equation: {0:.5f} + {1:.5f}x'.format(coeff[0], coeff[1]))
    print('Y-intercept: {}'.format(coeff[0]))
    plt.legend()


def quadratic_ransac_curve_fit(x, y):
    x1 = x.reshape((-1, 1))
    y1 = y.reshape((-1, 1))

    xi = np.linspace(min(x), max(x), 500).reshape((-1, 1))

    poly_2 = PolynomialFeatures(degree=2)
    x_2 = poly_2.fit_transform(x1)
    xi_2 = poly_2.fit_transform(xi)

    reg = linear_model.RANSACRegressor(linear_model.LinearRegression())
    reg.fit(x_2, y1)
    yi = reg.predict(xi_2)
    coeff = reg.estimator_.coef_
    intercept = reg.estimator_.intercept_[0]
    coeff = np.array([intercept, coeff[0, 1], coeff[0, 2]])

    inliers = reg.inlier_mask_
    outliers = np.logical_not(inliers)

    plt.plot(x[inliers], y[inliers], 'k.', label='inliers')
    plt.plot(x[outliers], y[outliers], 'r.', label='outliers')
    plt.plot(xi, yi, label='Quadratic Curve')

    plt.xlabel('Voltage')
    plt.ylabel('Angle, degrees')
    plt.title('Quadratic')
    print('Equation: {0:.5f} + {1:.5f}x + {2:.5f}x^2'.format(coeff[0], coeff[1], coeff[2]))
    print('Y-intercept: {}'.format(coeff[0]))
    plt.legend()
    
def cubic_ransac_curve_fit(x, y):
    x1 = x.reshape((-1, 1))
    y1 = y.reshape((-1, 1))

    xi = np.linspace(min(x), max(x), 500).reshape((-1, 1))

    poly_3 = PolynomialFeatures(degree=3)
    x_3 = poly_3.fit_transform(x1)
    xi_3 = poly_3.fit_transform(xi)

    reg = linear_model.RANSACRegressor(linear_model.LinearRegression())
    reg.fit(x_3, y1)
    yi = reg.predict(xi_3)
    coeff = reg.estimator_.coef_
    intercept = reg.estimator_.intercept_[0]
    coeff = np.array([intercept, coeff[0, 1], coeff[0, 2], coeff[0, 3]])

    inliers = reg.inlier_mask_
    outliers = np.logical_not(inliers)

    plt.plot(x[inliers], y[inliers], 'k.', label='inliers')
    plt.plot(x[outliers], y[outliers], 'r.', label='outliers')
    plt.plot(xi, yi, label='Cubic Curve')
    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Cubic')
    print('Equation: {0:.5f} + {1:.5f}x + {2:.5f}x^2 + {3:.5f}x^3'.format(coeff[0], coeff[1], coeff[2], coeff[3]))
    print('Y-intercept: {}'.format(coeff[0]))
    plt.legend()

    

if __name__=="__main__":
    battery_number = 1  # 1 or 3
    #voltage = 12
    #a = dist_angle_from_voltage(battery_number, voltage)
    #plt.show()

    # RANSAC interpolation
    data = np.load("basketball_records.npy")
    volt = []
    angle = []
    for i in range( len(data)):
        if data[i][3] == battery_number:
            voltage = data[i][4]/270.2
            #if voltage < 11.6: continue
            if voltage > 12.63: continue
            volt.append(voltage)
            angle.append(data[i][5] * 0.03375 + 20)
    quadratic_ransac_curve_fit(np.array(volt), np.array(angle))
    plt.show()

    # RANSAC result:
    # battery 1 : 10469.14829 + -1506.46893x + 55.11101x^2
    # battery 3 : 50435.37775 + -7951.56631x + 314.69345x^2