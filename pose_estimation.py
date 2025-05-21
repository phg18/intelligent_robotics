#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declare the mediapipe pose detector to be used
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=2,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
# Control message publisher
ackermann_command_publisher = rospy.Publisher(
        "/blue/preorder_ackermann_cmd",
        ackermann_msgs.msg.AckermannDrive,
        queue_size=10,
    )

def fingers_up(hand_landmarks):
    finger_tips = [4, 8, 12, 16, 20]
    finger_pips = [3, 6, 10, 14, 18]
    fingers = []
    for tip, pip in zip(finger_tips, finger_pips):
        fingers.append(1 if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[pip].y else 0)
    return fingers

#Operator image processing
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Processing the image with MediaPipe
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_image)
    # TODO Recognise the gesture by means of some classification from the landmarks.
    
    # TODO Draw landsmarks on the image
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            fingers = fingers_up(hand_landmarks)
            print(f"Fingers: {fingers}")

            drive_msg = ackermann_msgs.msg.AckermannDrive()
            drive_msg.speed = 1.0
            # TODO Interpret the obtained gesture and send the ackermann control command.
            if fingers == [0, 0, 0, 0, 0]:  # puño cerrado
                drive_msg.steering_angle = 0.0
            elif fingers == [0, 1, 0, 0, 0]:  #only indice
                drive_msg.steering_angle = -0.5  #right
            elif fingers == [0, 0, 0, 0, 1]:  #only meñique
                drive_msg.steering_angle = 0.5  #left
            elif fingers == [1, 1, 0, 0, 1]:
                drive_msg.speed = -1.0
                drive_msg.steering_angle = 0.0  #back
            else:
                drive_msg.speed = 0.0  #unkown = stop
        ackermann_command_publisher.publish(drive_msg)

    # Display image with detected landmarks/gestures
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)


def main():
    global ackermann_command_publisher
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
