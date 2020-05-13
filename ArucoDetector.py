import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import time

#--- Define Tag
id_to_find  = 30
marker_size  = 7 #- [cm]

#--- Define fps variables
number_frames = 30      # Check time every 30 frames
start = 0.0             # time for frame 1
stop = 0.0              # time for frame number_frames
seconds = 0.0           # store the number_frames / (stop - start)
frame_num  = -1         # number of frame
fps_calculation = 0.0   # calculated fps

#--- Get the camera calibration path
calib_path  = "//"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

#Load the dictionary that was used to generate the markers.
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv.aruco.DetectorParameters_create()

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv.VideoCapture(0)

#-- Set the camera size as the one it was calibrated with
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

#-- Font for the text in the image
font = cv.FONT_HERSHEY_PLAIN

# -- Start the time counter
start = time.time()

while True:
    # -- Read the camera frame
    ret, frame = cap.read()

    # -- FPS calculation and frame_num reset
    if frame_num >= number_frames:
        stop = time.time()
        seconds = stop - start
        frame_num = -1
        start = time.time()

    # -- Convert in gray scale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

    # -- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250), parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    if ids is not None and ids[0] == id_to_find:
        # -- ret = [rvec, tvec, ?]
        # -- array of rotation and position of each marker in camera frame
        # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        # -- Unpack the output, get only the first
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

        # -- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 5)

        # -- Print the tag position in camera frame
        str_position = "ARUCO %d Posicion x=%4.1f  y=%4.1f  z=%4.1f" % (id_to_find, tvec[0], tvec[1], tvec[2])
        cv.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv.LINE_AA)

    # -- Display the frames per second
    if seconds != 0.0:
        fps_calculation = number_frames / seconds
        str_fps = "%3.0f FPS" % (fps_calculation)
    else:
        str_fps = "--- FPS"
    cv.putText(frame, str_fps, (0, 470), font, 1, (0, 255, 0), 2, cv.LINE_AA)
    frame_num += 1      # Frame number incremented by one


    #--- Display the frame
    cv.imshow('frame', frame)

    # --- use 'q' to quit
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break
