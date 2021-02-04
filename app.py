# import used libraries 
import serial
import cv2 as cv
import numpy as np
import dlib
import time
import math

# trigger
t = ''

# write result to arduino by serial communication
arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
def write_to_arduino(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)

# function to caculate midpoint
def midpoint(p1, p2):
    return int((p1.x + p2.x) / 2), int((p1.y + p2.y) / 2)

# variables to count frames
r_frames = 0
l_frames = 0
b_frames = 0
o_frames = 0

# switch on and off
f_on = False
b_on = False
r_on = False
l_on = False
norm = False
switch = False

cam = cv.VideoCapture(0) # to use webcam 

detector = dlib.get_frontal_face_detector() # detect face
predictor = dlib.shape_predictor('model/shape_predictor_68_face_landmarks.dat') # predect landmarks

while True:
    isTrue, frame = cam.read() # open webcam
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # turn it to gray frame
    size = gray.shape

    faces = detector(gray) # detect faces in that frame

    if len(faces) > 0: # check faces

        for face in faces:
    
            landmarks = predictor(gray, face) # predict landmarks of the face in that frame 
    
            # identify right eye horizontal and vertical line points
            r_left_point = (landmarks.part(36).x, landmarks.part(36).y)
            r_right_point = (landmarks.part(39).x, landmarks.part(39).y)
            r_center_top = midpoint(landmarks.part(37), landmarks.part(38))
            r_center_bottom = midpoint(landmarks.part(41), landmarks.part(40))
    
            # draw right eye horizontal line
            r_hor_line = cv.line(frame, r_left_point, r_right_point, (0, 255, 0), 2)
    
            # draw right eye vertical line
            r_ver_line = cv.line(frame, r_center_top, r_center_bottom, (0, 255, 0), 2)
    
            # calculate length of right eye horizontal and vertical lines and ratio
            r_hor_line_length = math.hypot((r_left_point[0] - r_right_point[0]), (r_left_point[1] - r_right_point[1]))
            r_ver_line_length = math.hypot((r_center_top[0] - r_center_bottom[0]), (r_center_top[1] - r_center_bottom[1]))
            r_ratio = r_hor_line_length / r_ver_line_length
    
            # identify left eye horizontal and vertical line points
            l_left_point = (landmarks.part(42).x, landmarks.part(42).y)
            l_right_point = (landmarks.part(45).x, landmarks.part(45).y)
            l_center_top = midpoint(landmarks.part(43), landmarks.part(44))
            l_center_bottom = midpoint(landmarks.part(47), landmarks.part(46))
    
            # draw left eye horizontal line
            l_hor_line = cv.line(frame, l_left_point, l_right_point, (0, 255, 0), 2)
    
            # draw left eye vertical line
            l_ver_line = cv.line(frame, l_center_top, l_center_bottom, (0, 255, 0), 2)
    
            #calculate length of left eye horizontal and vertical lines and ratio
            l_hor_line_length = math.hypot((l_left_point[0] - l_right_point[0]), (l_left_point[1] - l_right_point[1]))
            l_ver_line_length = math.hypot((l_center_top[0] - l_center_bottom[0]), (l_center_top[1] - l_center_bottom[1]))
            l_ratio = l_hor_line_length / l_ver_line_length

            #print("Left :", l_ratio)
            #print("Right :", r_ratio)

            #2D frame points
            image_points = np.array([
                                (landmarks.part(30).x, landmarks.part(30).y),     # Nose tip
                                (landmarks.part(8).x, landmarks.part(8).y),       # Chin
                                (landmarks.part(45).x, landmarks.part(45).y),     # Left eye left corner
                                (landmarks.part(36).x, landmarks.part(36).y),     # Right eye right corne
                                (landmarks.part(54).x, landmarks.part(54).y),     # Left Mouth corner
                                (landmarks.part(48).x, landmarks.part(48).y)      # Right mouth corner
                            ], dtype="double")
    
            # 3D model points
            model_points = np.array([
                                (0.0, 0.0, 0.0),             # Nose tip
                                (0.0, -330.0, -65.0),        # Chin
                                (-225.0, 170.0, -135.0),     # Left eye left corner
                                (225.0, 170.0, -135.0),      # Right eye right corne
                                (-150.0, -150.0, -125.0),    # Left Mouth corner
                                (150.0, -150.0, -125.0)      # Right mouth corner
                            ])
    
                            
            # Camera internals
            focal_length = size[1]
            center = (size[1]/2, size[0]/2)
            camera_matrix = np.array(
                             [[focal_length, 1, center[0]],
                             [0, focal_length, center[1]],
                             [0, 0, 1]], dtype = "double"
                             )
    
    
            dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
            (success, rotation_vector, translation_vector) = cv.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv.cv2.SOLVEPNP_ITERATIVE)
    
            # draw a line sticking out of the nose
            (nose_end_point2D, jacobian) = cv.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
    
            # draw circles
            for p in image_points:
                cv.circle(frame, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
    
    
            p1 = ( int(image_points[0][0]), int(image_points[0][1]))
            p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
    
            # display the line from the nose
            cv.line(frame, p1, p2, (255,0,0), 2)
    
            # convert rotation vector to rotation matrix
            rvec_matrix = cv.Rodrigues(rotation_vector)[0]
    
            # convert rotation matrix to euler angles
            proj_matrix = np.hstack((rvec_matrix, translation_vector))
            eulerAngles = cv.decomposeProjectionMatrix(proj_matrix)[6] 
    
            pitch, yaw, roll = [math.radians(_) for _ in eulerAngles]
    
            pitch = math.degrees(math.asin(math.sin(pitch))) # face up and down
            roll = -math.degrees(math.asin(math.sin(roll))) # right and left inclined face 
            yaw = math.degrees(math.asin(math.sin(yaw))) # face right and left

            # detect right or left blink
            if r_ratio > l_ratio and r_ratio < 8 and l_ratio < 8 and r_ratio - l_ratio > 0.4:
                #print("Right")
                r_frames += 1
                l_frames = 0
                o_frames = 0
                if r_frames > 12:
                    if f_on == True: 
                        f_on = False
                        r_frames = 0
                    elif f_on == False:
                        f_on = True
                        r_frames = 0
                    b_on = False

            elif r_ratio < l_ratio and r_ratio < 8 and l_ratio < 8 and l_ratio - r_ratio > 0.4:
                #print("Left")
                r_frames = 0
                l_frames += 1
                o_frames = 0
                if l_frames > 12:
                    if b_on == True: 
                        b_on = False
                        l_frames = 0
                    elif b_on == False:
                        b_on = True
                        l_frames = 0
                    f_on = False

            elif r_ratio >= 5 and l_ratio >= 5:
                #print("Both")
                r_frames = 0
                l_frames = 0
                b_frames += 1
                o_frames = 0
                if b_frames > 10:
                    if switch == False:
                        switch = True
                        b_frames = 0
                    elif switch == True:
                        switch = False
                        b_frames = 0

            elif r_ratio < 4 and l_ratio < 4:
                #print("Opened")
                b_frames = 0
                o_frames += 1

            # detect right or left 
            if roll >= 18:
                #print("Right")
                r_on =True
                l_on = False
                norm = False
                f_on = False
                b_on = False
            elif roll <= -18:
                #print("Left")
                r_on =False
                l_on = True
                norm = False
                f_on = False
                b_on = False
            else:
                print("Normal")
                r_on =False
                l_on = False
                norm = True
            
            c = 1;

            # check device is on or off
            if switch == True:
                if t != 'q' and c == 1:
                        write_to_arduino('q')
                        t = 'q'
                        c = c + 1
                print("Device is on")
                # print out movements
                if f_on == True and b_on == False and norm == True:
                    if t != 'f':
                        write_to_arduino('f')
                        t = 'f'
                    cv.putText(frame, "Forward", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
                elif b_on == True and f_on == False and norm == True:
                    if t != 'b':
                        write_to_arduino('b')
                        t = 'b'
                    cv.putText(frame, "Backward", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
                elif r_on == True and l_on == False and norm == False:
                    if t != 'r':
                        write_to_arduino('r')
                        t = 'r'
                    cv.putText(frame, "Right", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
                elif r_on == False and l_on == True and norm == False:
                    if t != 'l':
                        write_to_arduino('l')
                        t = 'l'
                    cv.putText(frame, "Left", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
                else:
                    if t != 'n':
                        write_to_arduino('n')
                        t = 'n'
            else:
                if t != 'w':
                        write_to_arduino('w')
                        t = 'w'
                        c = 1
                print("Device is off")

    else:
        cv.putText(frame, "No faces detected", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), thickness=2)

    cv.imshow('Video', frame) # show each frame in the loop

    if cv.waitKey(1) == 27: # condition of pressing 'ESC' to exit the loop and close the program
        break

cv.destroyAllWindows() # close everything
cam.release() # close camera
