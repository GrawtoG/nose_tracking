#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
from geometry_msgs.msg import Point

def main():
    rospy.init_node('face_detection_node', anonymous=True)
    pub = rospy.Publisher('nose_coordinates', Point, queue_size=5)
    rate = rospy.Rate(30) 

    mpFaceMesh = mp.solutions.face_mesh # type: ignore
    face_mesh = mpFaceMesh.FaceMesh(static_image_mode=False,
                                  max_num_faces=1,
                                  refine_landmarks=True,  
                                  min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)


    # Initialize the camera
    device = rospy.get_param("video_device_name", "/dev/video0")
    cap = cv2.VideoCapture(device) # type: ignore
    
    if not cap.isOpened():
        rospy.logerr("Failed to open camera check if correct camera was chosen")
        return
    
    # Set the camera codec to MJPG
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # type: ignore
   
  
    while not rospy.is_shutdown():
        success, frame = cap.read()
        if not success:
            rospy.logwarn("Failed to capture a frame")
            continue
        
        results = face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                h, w, _ = frame.shape

                nose_tip = face_landmarks.landmark[1]
                cx = int(nose_tip.x * w)
                cy = int(nose_tip.y * h)
                pub.publish(Point(cx, cy, 0))

        #         # Draw the nose tip
        #         cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        #         cv2.putText(frame, "Nose Tip", (cx + 5, cy), cv2.FONT_ITALIC, 0.5, (0,255,0), 1)
                
        # cv2.imshow("Nose Tip", frame)
        # cv2.waitKey(1)
        rate.sleep()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
