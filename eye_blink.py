# import used libraries 
import cv2 as cv
import numpy as np
import dlib
import math 

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
switch = False

cam = cv.VideoCapture(0) # to use webcam 

detector = dlib.get_frontal_face_detector() # detect face
predictor = dlib.shape_predictor('model/shape_predictor_68_face_landmarks.dat') # predect landmarks

while True:
    isTrue, frame = cam.read() # open webcam
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # turn it to gray frame

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

            # check device is on or off
            if switch == True:
                print("Device is on")

                # print out movements
                if f_on == True and b_on == False:
                    cv.putText(frame, "Forward", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
                elif b_on == True and f_on == False:
                    cv.putText(frame, "Backward", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)

            else:
                print("Device is off")

    else:
        cv.putText(frame, "No faces detected", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), thickness=2)

    cv.imshow('Video', frame) # show each frame in the loop

    if cv.waitKey(1) == 27: # condition of pressing 'ESC' to exit the loop and close the program
        break

cv.destroyAllWindows() # close everything
cam.release() # close camera
