
import numpy as np
import matplotlib.pyplot as plt
import pickle

fig = plt.figure()
ax = plt.axes(projection="3d")

F = open('/Users/mac/PycharmProjects/RTPositionAnalizer/saved_paths/pickle/path-20200615-111712.pkl', 'rb')
L = pickle.load(F)
#fig = plt.figure()
#ax = plt.axes(projection="3d")

# plot a line
#z_line = np.linspace(0, 15, 1000)
#x_line = np.cos(z_line)
#y_line = np.sin(z_line)
#ax.plot3D(x_line, y_line, z_line, 'gray')

z_points = 15 * np.random.random(100)
x_points = np.cos(z_points) + 0.1 * np.random.randn(100)
y_points = np.sin(z_points) + 0.1 * np.random.randn(100)



ax.scatter3D(x_points, y_points, z_points, c=z_points, cmap='hsv');

plt.show()