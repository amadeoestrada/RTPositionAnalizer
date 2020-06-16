import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import pickle
import glob
import os

# -- Set resolution of the axis relative to camera in cm
resolution = 15

# -- Load the camera matrix (intrinsic parameters)
workingFolder = "/Users/mac/PycharmProjects/RTPositionAnalizer/"
camera_matrix = np.loadtxt(workingFolder + 'cameraMatrix.txt', delimiter=',')

# -- Open the latest file from the folder
list_of_files = glob.glob('/Users/mac/PycharmProjects/RTPositionAnalizer/saved_paths/pickle/*') # * means all if need specific format then *.csv
latest_file = max(list_of_files, key=os.path.getctime)
F = open(latest_file, 'rb')
L = pickle.load(F)
F.close()
size = len(L)

# -- Initialize arrays
xyz_origins = np.zeros((3,1), np.float32)
x_points = np.zeros(size, np.float32)
y_points = np.zeros(size, np.float32)
z_points = np.zeros(size, np.float32)
x_points2 = np.zeros(size, np.float32)
y_points2 = np.zeros(size, np.float32)
z_points2 = np.zeros(size, np.float32)
r_vector = np.zeros((3, 1), np.float32)
r_matrix = np.zeros((3, 3), np.float32)
p_vectors = np.zeros((3,size), np.float32)
p_vectors2 = np.zeros((3,size), np.float32)
H = np.zeros((3, 3), np.float32)
frame_time = np.zeros(size, np.float32)

# -- Load origin
x_origin = L[0][2][0]
y_origin = L[0][2][1]
z_origin = L[0][2][2]
xyz_origins[0][0] = L[0][2][0]
xyz_origins[1][0] = L[0][2][1]
xyz_origins[2][0] = L[0][2][2]

# -- Load origin rotation vector
r_vector[0] = L[0][3][0]
r_vector[1] = L[0][3][1]
r_vector[2] = L[0][3][2]

# -- Homography calculation
inv_camera_matrix = np.linalg.inv(camera_matrix)
# -- Convert the rotation vector into a rotation matrix
cv.Rodrigues(r_vector, r_matrix)
# -- Use the Rotation along the camera center
H = camera_matrix.dot(r_matrix)
H = H.dot(inv_camera_matrix)

# -- Load the data for each point vector together with the time
i = 0
while i < size:
    p_vectors[0][i] = L[i][0][0]
    p_vectors[1][i] = L[i][0][1]
    p_vectors[2][i] = L[i][0][2]
    frame_time[i] = L[i][4]         # incremental time
    i += 1
i = 0

# -- Substract the origin to align the position to the camera position
p_vectors = np.subtract(p_vectors, xyz_origins)
# -- Calculate the homography using the homography matrix
p_vectors = p_vectors.T
p_vectors2 = p_vectors .dot(H)   # Homography vectors result

# -- Assign the data of the homography results to the xy coordinates arrays
# -- for the homography result plot. The z axis of this plot is time
i = 0
while i < size:
    x_points[i] = p_vectors2[i][0]
    y_points[i] = p_vectors2[i][1]
    i += 1
i = 0

# -- Assign the position relative to the camera to the xyz coordinates.
i = 0
while i < size:
    x_points2[i] = L[i][0][0]
    y_points2[i] = L[i][0][1]
    z_points2[i] = L[i][0][2]
    i += 1
i = 0

# -- Set the Position Relative to Camera 3D Plot
fig = plt.figure()
ax2 = plt.axes(projection="3d")
ax2.set_xlim3d(x_origin-resolution, x_origin + resolution)
ax2.set_ylim3d(y_origin-resolution, y_origin + resolution)
ax2.set_zlim3d(z_origin-resolution, z_origin + resolution)
ax2.set_xlabel('X axis (cm)')
ax2.set_ylabel('Y axis (cm)')
ax2.set_zlabel('Z axis (cm)')
plt.suptitle('Position Relative to Camera Plot')
# -- Display the origin as a pink cross
ax2.scatter3D(x_origin, y_origin, z_origin, c='m', marker='P')
# -- The yellow color shows the start of plot. The dark blue shows the last position
ax2.scatter3D(x_points2, y_points2, z_points2, c=frame_time, cmap='plasma')

# -- Set the Homography Result 3D Plot
fig = plt.figure()
ax = plt.axes(projection="3d")
ax.set_xlim3d(x_origin-resolution, x_origin + resolution)
ax.set_ylim3d(y_origin-resolution, y_origin + resolution)
ax.set_zlim3d(0, frame_time[size-1] +1 )
ax.set_xlabel('X axis (cm)')
ax.set_ylabel('Y axis (cm)')
ax.set_zlabel('T time (s)')
plt.suptitle('Homography Result Plot')
# -- The Z axis is the time
ax.scatter3D(x_points, y_points, frame_time, c=frame_time, cmap='winter');

# -- Set movement along X Axis 2D Plot
fig = plt.figure()
plt.plot(frame_time, x_points,   'ro')
plt.xlabel('Time (s)')
plt.ylabel('X Distance (cm)')
plt.suptitle('Movement Along X Axis Plot')

# -- Set movement along Y Axis 2D Plot
fig = plt.figure()
plt.plot(z_points, y_points,   'go')
plt.xlabel('Time (s)')
plt.ylabel('Y Distance (cm)')
plt.suptitle('Movement Along Y Axis Plot')

# -- Show all plots
plt.show()