# Importing libraries
import matplotlib.pyplot as plt
import numpy as np
import math

# Using Numpy to create an array X
T = np.arange(0, 20, 0.5)

# Assign variables to the y axis part of the curve

# Angular Velocity With movement
xAm = np.array([-178.324997, -4.830000, -80.220001, -26.652500, -72.502502, -178.639999, -153.667496, 61.932499, 169.820007, -110.250000, -184.975006, 299.897491, -197.312500, -127.330002, 256.269989, -143.115005, -118.842499, 286.527496, -107.677498, -140.455002, 148.610001, -114.589996, -81.322502, 182.052505, -137.497498, 18.620001, 44.782501, -80.220001, 141.679993, -106.837502, -66.132500, 114.904999, 3.237500, -120.592499, 170.677505, -57.347500, -88.602501, 186.252502, -185.464996, 36.522499])
yAm = np.array([-222.074997, 24.500000, 377.772491, -158.147507, 94.849998, -442.557495, 0.000000, 225.014999, 19.022499, 202.772507, 33.092499, 164.342499, 66.080002, -26.337500, -23.362499, 154.244995, -116.427498, 89.599998, 46.445000, 34.002499, -18.952499, -45.009998, -51.782501, 27.597500, 99.697502, -140.524994, 139.002502, -82.652496, 34.632500, -74.217499, -4.637500, 26.075001, 54.687500, -47.845001, 13.475000, -38.132500, -24.307501, 54.827499, -76.492500, -4.445000])
zAm = np.array([-15.382500, 64.294998, -77.245003, -273.332489, -240.660004, -289.012512, 293.317505, 55.195000, -35.630001, -10.710000, -38.709999, -111.492500, 19.705000, 23.485001, -151.742493, -35.945000, 10.657500, 25.795000, -39.549999, 52.955002, -67.690002, -54.005001, 23.397499, -31.254999, -137.987503, 104.177498, 7.840000, -18.340000, -74.987503, 10.990000, 5.705000, -18.042500, 10.220000, 33.320000, 4.550000, -26.582500, 43.837502, 32.217499, -33.845001, 35.052502])

# Linear Velocity With movement
xLm = np.array([0.000000, -0.346990, 0.150780, -0.107135, 0.091700, 0.212275, -0.049945, -0.431200, -0.215775, 0.560140, 0.149450, -0.969745, 0.994420, -0.139965, -0.767200, 0.798770, -0.048545, -0.810740, 0.788410, 0.065555, -0.578130, 0.526400, -0.066535, -0.526750, 0.639100, -0.312235, -0.052325, 0.250005, -0.443800, 0.497035, -0.081410, -0.362075, 0.223335, 0.247660, -0.582540, 0.456050, 0.062510, -0.549710, 0.743435, -0.443975])
yLm = np.array([0.000000, -0.493150, -0.706545, 1.071840, -0.505995, 1.074815, -0.885115, -0.450030, 0.411985, -0.367500, 0.339360, -0.262500, 0.196525, 0.184835, -0.005950, -0.355215, 0.541345, -0.412055, 0.086310, 0.024885, 0.105910, 0.052115, 0.013545, -0.158760, -0.144200, 0.480445, -0.559055, 0.443310, -0.234570, 0.217700, -0.139160, -0.061425, -0.057225, 0.205065, -0.122640, 0.103215, -0.027650, -0.158270, 0.262640, -0.144095])
zLm = np.array([0.000000, -0.043823, 0.077847, 0.107848, -0.017970, 0.026594, -0.320282, 0.130967, 0.049954, -0.013706, 0.015400, 0.040030, -0.072159, -0.002079, 0.096375, -0.063689, -0.025631, -0.008326, 0.035940, -0.050878, 0.066355, -0.007527, -0.042571, 0.030059, 0.058703, -0.133191, 0.052986, 0.014399, 0.031156, -0.047288, 0.002907, 0.013061, -0.015544, -0.012705, 0.015824, 0.017123, -0.038731, 0.006391, 0.036334, -0.037894])

# Angular Velocity Static
xAs = np.array([-0.752500, -0.157500, 0.822500, 0.787500, 0.385000, -0.420000, -0.262500, -0.280000, -0.297500, -0.140000, -0.175000, -0.612500, -0.280000, -0.315000, -0.490000, -0.175000, -0.035000, -0.297500, -0.192500, -0.245000, -0.385000, -0.105000, -0.297500, -0.630000, -0.437500, -0.507500, -0.297500, -0.455000, -0.437500, -0.245000, -0.542500, -0.245000, -0.455000, -0.245000, -0.507500, -0.385000, -0.507500, -0.472500, -0.227500, -0.315000])
yAs = np.array([-0.367500, -0.052500, 0.280000, 0.420000, 0.210000, 0.315000, 0.210000, 0.367500, 0.087500, 0.140000, -0.070000, 0.157500, 0.175000, 0.332500, 0.140000, -0.070000, 0.315000, 0.402500, 0.210000, 0.350000, 0.105000, 0.315000, -0.105000, 0.280000, 0.192500, -0.105000, 0.122500, 0.402500, 0.472500, 0.157500, 0.140000, 0.385000, 0.070000, 0.157500, -0.262500, 0.087500, 0.052500, -0.157500, 0.087500, 0.000000])
zAs = np.array([0.000000, 0.105000, 0.227500, 0.735000, -0.350000, -0.017500, -0.262500, -0.262500, -0.035000, 0.070000, -0.105000, -0.402500, -0.052500, 0.192500, -0.105000, 0.210000, -0.262500, 0.210000, -0.087500, -0.437500, 0.210000, -0.052500, -0.052500, -0.140000, -0.210000, -0.070000, -0.210000, -0.105000, -0.210000, -0.087500, -0.175000, -0.105000, 0.192500, -0.087500, -0.070000, -0.245000, 0.287500, -0.245000, -0.035000, 0.000000])

# Linear Velocity Static
xLs = np.array([0.000000, -0.001190, -0.001960, 0.000070, 0.000805, 0.001610, -0.000315, 0.000035, 0.000035, -0.000315, 0.000070, 0.000875, -0.000665, 0.000070, 0.000350, -0.000630, -0.000280, 0.000525, -0.000210, 0.000105, 0.000280, -0.000560, 0.000385, 0.000665, -0.000385, 0.000140, -0.000420, 0.000315, -0.000035, -0.000385, 0.000595, -0.000595, 0.000420, -0.000420, 0.000525, -0.000245, 0.000245, -0.000070, -0.000490, 0.000175])
yLs = np.array([0.000000, -0.000630, -0.000665, -0.000280, 0.000420, -0.000210, 0.000210, -0.000315, 0.000560, -0.000105, 0.000420, -0.000455, -0.000035, -0.000315, 0.000385, 0.000420, -0.000770, -0.000175, 0.000385, -0.000280, 0.000490, -0.000420, 0.000840, -0.000770, 0.000175, 0.000595, -0.000455, -0.000560, -0.000140, 0.000630, 0.000035, -0.000490, 0.000630, -0.000175, 0.000840, -0.000700, 0.000070, 0.000420, -0.000490, 0.000175])
zLs = np.array([0.000000, -0.000058, -0.000067, -0.000279, 0.000597, -0.000183, 0.000135, 0.000000, -0.000125, -0.000058, 0.000096, 0.000164, -0.000192, -0.000135, 0.000164, -0.000173, 0.000260, -0.000260, 0.000164, 0.000192, -0.000356, 0.000144, 0.000000, 0.000048, 0.000038, -0.000077, 0.000077, -0.000058, 0.000058, -0.000067, 0.000048, -0.000038, -0.000164, 0.000154, -0.000010, 0.000096, -0.002493, 0.002493, -0.000116, -0.000019])

# Plotting both the curves simultaneously
plt.plot(T, xLm, color='r', label='X axis')
plt.plot(T, yLm, color='g', label='Y axis')
plt.plot(T, zLm, color='b', label='Z axis')

# Naming the x-axis, y-axis and the whole graph
plt.ylabel("Linear Velocity")
plt.xlabel("Time in s")
plt.title("Gyro Linear Velocity plot")

# Adding legend, which helps us recognize the curve according to it's color
plt.legend()

# To load the display window
plt.show()
