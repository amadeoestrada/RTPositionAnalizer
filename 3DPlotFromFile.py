
import numpy as np
import matplotlib.pyplot as plt
import pickle
import glob
import os

# Resolution in cm
resolution = 15

# Open the latest file from the folder
list_of_files = glob.glob('/Users/mac/PycharmProjects/RTPositionAnalizer/saved_paths/pickle/*') # * means all if need specific format then *.csv
latest_file = max(list_of_files, key=os.path.getctime)

F = open(latest_file, 'rb')
L = pickle.load(F)
F.close()
size = len(L)



x_origin = L[0][2][0]
y_origin = L[0][2][1]
z_origin = L[0][2][2]
#z_origin = L[0][3]

# Set the plot window
fig = plt.figure()
ax = plt.axes(projection="3d")
ax.set_xlim3d(x_origin-resolution, x_origin + resolution)
ax.set_ylim3d(y_origin-resolution, y_origin + resolution)
ax.set_zlim3d(z_origin-resolution, z_origin + resolution)

ax.scatter3D(x_origin, y_origin, z_origin, c='m', marker='P');

# Initialize array for x, y and z points
x_points = np.zeros((size), np.float32)
y_points = np.zeros((size), np.float32)
z_points = np.zeros((size), np.float32)

# Fill in the data for the x, y  and z points into the arrays
i = 0
while i < size:
    x_points[i] = L[i][0][0]
    y_points[i] = L[i][0][1]
    z_points[i] = L[i][0][2]
    i += 1
i = 0

ax.scatter3D(x_points, y_points, z_points, cmap='Blues');

plt.show()