import cv2 as cv
import dlib

cam = cv.VideoCapture(0) # to use webcam 

detector = dlib.get_frontal_face_detector() # detect face
predictor = dlib.shape_predictor('model/shape_predictor_68_face_landmarks.dat') # predect landmarks

while True:
    isTrue, frame = cam.read() # open webcam
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # turn it to gray frame

    faces = detector(gray) # detect faces in that frame
    for face in faces:
        # the next five lines of code were for marking the detected face
        #x1 = face.left()
        #y1 = face.top()
        #x2 = face.right()
        #y2 = face.bottom()
        #cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3) # (img, point1, point2, color, thickness)

        landmarks = predictor(gray, face) # predict landmarks of the face in that frame 

        for n in range(0, 68): # loop for marking these landmarks
            x = landmarks.part(n).x
            y = landmarks.part(n).y
            cv.circle(frame, (x, y), 2, (255, 0, 0), -1) # (img, coordinates, radius, color, thickness)

    cv.imshow('Video', frame) # show each frame in the loop

    if cv.waitKey(1) == 27: # condition of pressing 'ESC' to exit the loop and close the program
        break

cv.destroyAllWindows() # close everything
cam.release() # close camera