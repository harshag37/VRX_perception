import cv2
import numpy as np
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs import msg
from cv_bridge import CvBridge, CvBridgeError

class image_feature:

    def __init__(self):
        # topic where we publish
        self.image_pub = rospy.Publisher("impub",
            Image,queue_size = 10)
        self.bridge = CvBridge()
        self.pyrDown=2
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/wamv/sensors/cameras/front_right_camera/image_raw",
            Image, self.callback,  queue_size = 10)
        # self.result = cv2.VideoWriter('filename.avi', cv2.VideoWriter_fourcc(*'MJPG'),10, (360,640))
        self.i=0
    def imgpro(self,data):
        scenePyr=data
        if self.pyrDown > 0:
            for i in range(self.pyrDown):
                scenePyr = cv2.pyrDown(scenePyr)
        imghsv=cv2.cvtColor(scenePyr,cv2.COLOR_BGR2HSV)
<<<<<<< HEAD
        # height, width = scenePyr.shape[:2]
        # self.result.write(scenePyr)
        # imggray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # kernel=np.array((5,5))
        cv2.imwrite('kang'+str(self.i)+'.jpg',scenePyr)
        self.i+=1
=======
>>>>>>> 389afc196e40c195fa0c85b2ed619e2976f9b66e
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        img_mask=cv2.inRange(imghsv, lower_red, upper_red)
        cv2.imshow("Mask",img_mask)
        cnts, hierarchy = cv2.findContours(img_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for cn in cnts:
            cv2.fillPoly(scenePyr,pts=[cn],color=(0,255,0))
        return scenePyr
    def callback(self, ros_data):
        msg=self.bridge.imgmsg_to_cv2(ros_data,"bgr8")
        msg=self.imgpro(msg)
        msg=self.bridge.cv2_to_imgmsg(msg,"bgr8")
        self.image_pub.publish(msg)
        

def main(args=None):
    rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
