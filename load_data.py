#!/usr/bin/env python
# license removed for brevity

import math
import sys
import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import rospy

cameraMatrix = np.array([[1144.0874006434067, 0.0, 960.5], [0.0, 1144.0874006434067, 540.5], [0.0, 0.0, 1.0]])
tvecs = np.array([0.000, 0.000, 7.591])


def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
        
                    
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

rvecs, _ = cv2.Rodrigues(eulerAnglesToRotationMatrix(np.array([0.000, 20, 0.000])))
print(rvecs)


def project(bbx_3d, img):
    bbx_3d = np.array(bbx_3d)
    #print(bbx_3d)
    imagePoints, _ = cv2.projectPoints(bbx_3d, rvecs, tvecs, cameraMatrix, None)

    print(imagePoints)


    x_max = x_min = imagePoints[0][0][0]
    y_max = y_min = imagePoints[0][0][1]
    for num in imagePoints:
        n = num[0]        
        x_max = max(n[0], x_max)
        x_min = min(n[0], x_min)
        y_max = max(n[1], y_max)
        y_min = min(n[1], y_min)

    x_max = int(x_max)
    x_min = int(x_min)
    y_max = int(y_max)
    y_min = int(y_min)

    bbx_2d = [(x_min, y_min), (x_max, y_max)]
    print(bbx_2d)
    return bbx_2d



def acquire_pose(pos_sub):
    bbx_3d = []
    poses = pos_sub.poses
    for pose in poses:
        position = pose.position    
        x = position.x
        y = position.y
        z = position.z
        bbx_3d.append([x, y, z])
    #print(bbx_3d)
    return bbx_3d


def callback(img_sub, pos_sub):
    print("topics synchronized!")
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_sub, "bgr8")
    except CvBridgeError as e:
        print(e)

    bbx_3d = acquire_pose(pos_sub)
    bbx_2d = project(bbx_3d, cv_image)
    
    img = cv2.rectangle(cv_image, bbx_2d[0], bbx_2d[1], (0,255, 255))
    cv2.imshow("bbx", img)
    cv2.waitKey(1)


def main():
    rospy.init_node('data_loader', anonymous=True)
    print("loading topics ...")

    img_sub = message_filters.Subscriber("/senser1/camera/image", Image)
    pos_sub = message_filters.Subscriber("/hatchback_red/modelFramePointsTopics", PoseArray)

    ts = message_filters.ApproximateTimeSynchronizer([img_sub, pos_sub], 10, 0.1)
    ts.registerCallback(callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutdown")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
