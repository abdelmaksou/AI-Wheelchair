# import used libraries 
import cv2 as cv
import numpy as np
import dlib
import math


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
    
            # print out roll direction
            if roll >= 18:
                #print("Right")
                cv.putText(frame, "Right", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
            elif roll <= -18:
                #print("Left")
                cv.putText(frame, "Left", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), thickness=3)
            else:
                print("Normal")
                
    else:
        cv.putText(frame, "No faces detected", (50, 150), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), thickness=2)

    # display frames
    cv.imshow('Video', frame) 

    if cv.waitKey(1) == 27: # condition of pressing 'ESC' to exit the loop and close the program
        break

cv.destroyAllWindows() # close everything
cam.release() # close camera
