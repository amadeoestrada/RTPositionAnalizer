from datetime import datetime
import time
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import pickle

# TODO Clean code 

# --- Define Tag
id_to_find = 30
marker_size = 7  # - [cm]

# --- Define fps variables
number_frames = 30  # Check time every 30 frames
start = 0.0  # time for frame 1
stop = 0.0  # time for frame number_frames
seconds = 0.0  # store the number_frames / (stop - start)
frame_num = -1  # number of frame
fps_calculation = 0.0  # calculated fps
frame_counter = 0  # number of frames to display message
dist = 0.0  # distance to origin (in cm)
vel = 0.0  # velocity acumulator
delta = 0.0  # distance from frame to frame (in cm)
min_delta = 0.1  # minimum distance in cm
got_origin = 0  # FLAG: an origin had been defined
vel_filter = 3  # Velocity averaging filter
vel_disp = 0.0  # Final velocity to display in cm/s)
i = 0  # Counter used for velocity filtering
recording_start = 0  # FLAG: start recording coordinates
recording_stop = 0  # FLAG: stop recording coordinates
L_txt = []  # Empty list of positions
L_pkl = []  # Empty list of positions
L1 = []  # Empty list of positions
# Start a time variable
# Format for time stamp "%Y-%m-%d,%H:%M:%S"+
# Format for filename "%Y%m%d-%H%M%S"
timestr = time.strftime("%Y-%m-%d,%H:%M:%S")

# --- Get the camera calibration path
workingFolder = "/Users/mac/PycharmProjects/RTPositionAnalizer/"
camera_matrix = np.loadtxt(workingFolder + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(workingFolder + 'cameraDistortion.txt', delimiter=',')

# Load the dictionary that was used to generate the markers.
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv.aruco.DetectorParameters_create()

# --- Capture the webcam video input
cap = cv.VideoCapture(0)

# -- Set the camera size as the one it was calibrated with
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# -- Font for the text in the image
font = cv.FONT_HERSHEY_PLAIN

while True:
    # -- Read the camera frame
    ret, frame = cap.read()

    # -- FPS calculation and frame_num reset
    if frame_num >= number_frames:
        stop = time.time()
        seconds = stop - start
        frame_num = -1
        start = time.time()

    # -- Draw BLACK background to display frame information
    frame = cv.rectangle(frame, (0, 430), (640, 480), (0, 0, 0), -1)

    # -- Convert in gray scale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

    # -- Find all the Aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
                                                 parameters=parameters,
                                                 cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    # save keystroke
    key = cv.waitKey(1) & 0xFF  # Assign command to variable called key

    if ids is not None and ids[0] == id_to_find:
        # -- ret = [rvec, tvec, ?]
        # -- array of rotation and position of each marker in camera frame
        # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
        # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        # -- Unpack the output, get only the first
        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

        # -- Draw the detected marker and put a reference frame over it
        # RED square indicates the top left corner
        aruco.drawDetectedMarkers(frame, corners)
        # The Aruco axes' color are: x=RED   y=GREEN   z=BLUE
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 5)

        # -- Print the tag position in camera frame
        str_position = "POS: x=%4.1f  y=%4.1f  z=%4.1f" % (tvec[0], tvec[1], tvec[2])
        cv.putText(frame, str_position, (90, 470), font, 1, (0, 255, 0), 2, cv.LINE_AA)

        # -- use 'o' to redefine origin
        if key == ord('o'):
            frame_counter = 8  # display a message during this number of frames
            ovec = tvec  # make the current position the new origin
            got_origin = 1  # FLAG: new origin defined

        # -- If origin is defined, use 'r' to start recording coordinates
        if key == ord('r'):
            frame_counter = 8  # display a message during this number of frames
            recording_start = 1  # FLAG: start recording coordinates
            recording_stop = 0  # FLAG: stop recording coordinates

        # -- If origin is defined, use 'r' to stop recording coordinates
        if key == ord('s'):
            frame_counter = 8  # display a message during this number of frames
            if recording_start != 0:
                recording_stop = 1  # FLAG: start recording coordinates
                recording_start = 0  # FLAG: start recording coordinates

    # - - - - - - - - - - - - - - - If an origin is established - - - - - - - - - - - - - - - -

    if got_origin != 0:
        # -- Calculate distance to origin
        # Apply simple averaging filter to position
        tvec = (pvec + tvec) / 2

        # Calculate euclidean distance from origin to current position
        dist = np.linalg.norm(tvec - ovec)

        # Display the distance to origin
        message_origin = "From Origin: %3.1f" % dist
        cv.putText(frame, message_origin, (0, 450), font, 1, (0, 255, 0), 2, cv.LINE_AA)

        # -- Calculate instantaneous velocity
        delta = np.linalg.norm(tvec - pvec)

        # do not compute small data
        if delta < min_delta:
            vel = 0.0
        else:
            # velocity calculation in
            vel = fps_calculation * delta

        # increment velocity filter counter
        i += 1

        # Apply filter
        if i > vel_filter:
            vel_disp = vel / vel_filter
            i = 0
            vel = 0

        # - - - - - - - - - - - - - - - Record to file (if enabled - - - - - - - - - - - - - - - -

        # If recording is enabled, save the current position to the list of positions
        if recording_start:
            # Pack the current position data into a human readable format
            L1.append(str(tvec))
            L1.append(str(rvec))
            L1.append(str(ovec))
            L1.append(fps_calculation)
            timestr = datetime.now()
            timestr = timestr.strftime("%Y-%m-%d,%H:%M:%S.%f")
            L1.append(timestr)
            # Append the current position to the list of positions and flush L1
            L_txt.append(L1)
            # Reset current position list
            L1 = [tvec, rvec, ovec, fps_calculation, timestr]
            # Pack the current position data into a Python readable format
            # Append the current position to the list of positions
            L_pkl.append(L1)
            # Reset current position list
            L1 = []
            # Draw a red circle to inform recording is enabled
            cv.circle(frame, (620, 445), 5, (0, 0, 255), thickness=-1)

        # If recording is halted, save the list of position to a file with a time stamped name
        if recording_stop:
            # Format for filename "%Y%m%d-%H%M%S"
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = workingFolder + "saved_paths/path-" + timestr + ".txt"
            # Save data to human readable txt file
            F = open(filename, 'w')
            F.write(str(L_txt) + '\n')
            # Remove first '[' character
            F.seek(0, 0)
            F.write('\b')
            # Swap last ']' character for a ','
            F.seek(F.seek(0, 2) - 2, 0)
            F.write(',')
            F.close()
            # Save data to pickle file
            filename = workingFolder + "saved_paths/pickle/path-" + timestr + ".pkl"
            F = open(filename, 'wb')
            pickle.dump(L_pkl, F)
            F.close()
            # Re-start lists
            L_txt = []
            L_pkl = []
            L1 = []
            # Record only one file per event
            recording_stop = 0

        # - - - - - - - - - - - - - Add messages to current frame - - - - - - - - - - - - - - - -

        # Display instantaneous velocity
        message_velocity = "Inst vel: %3.2f" % vel_disp
        cv.putText(frame, message_velocity, (180, 450), font, 1, (0, 255, 0), 2, cv.LINE_AA)

    # TODO test if frames calculation can be done before calculating the distance to origin

    # -- Display the frames per second
    if seconds != 0.0:
        fps_calculation = number_frames / seconds
        str_fps = "%3.0f FPS" % fps_calculation
    else:
        str_fps = "-- FPS"
    cv.putText(frame, str_fps, (0, 470), font, 1, (0, 255, 0), 2, cv.LINE_AA)
    frame_num += 1  # Frame number incremented by one

    if frame_counter > 0:
        message = "NEW ORIGIN: %4.1f, %4.1f, %4.1f" % (ovec[0], ovec[1], ovec[2])
        cv.putText(frame, message, (400, 470), font, 1, (0, 0, 255), 2, cv.LINE_AA)
        frame_counter -= 1

    # --- Display the frame
    cv.imshow('frame', frame)

    # Save current vector as previous vector
    if ids is not None and ids[0] == id_to_find:
        pvec = tvec

    # --- use 'q' to quit
    if key == ord('q'):
        cap.release()
        cv.destroyAllWindows()
        break
# - - - - - - - - - - - - End of Program - - - - - - - - - - - - - -
