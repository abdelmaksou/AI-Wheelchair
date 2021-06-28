# Facial gestures controlled wheelchair Using facial landmarks recognition and Arduino
Detectes facial gistures:
1) Open and close both eyes for on and off switch
2) Open and close right eye for going forward switch
3) Open and close left eye for going backward switch
4) Incline head right for going right 
5) Incline head left for going left

and then excutes the moves.

The real-time analysis of the video is done by diving it into frames then apply Dlib facial landmarks classifier to recognize the different landmarks. Then, the landmarks are further manipulated to target specific areas of the face with definite gestures. pyFirmata is used to send the results of the app python code to the Arduino so it can react quickly to the facial gestures. This combination allows the disabled to move fast and efficiently with no much effort.
