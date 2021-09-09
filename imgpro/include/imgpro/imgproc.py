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
        self.color={"red":np.array([[0, 231, 94],[10, 255, 124],[0],[],[]]),
        "blue":np.array([[119,244,89],[123,255,138],[0],[],[]]),
        "black":np.array([[0,0,0],[0, 0, 9],[0],[],[]]),
        "yellow":np.array([[27,226,93],[32, 255, 130],[0],[],[]]),
        "green":np.array([[49, 244, 90],[64, 255, 119],[0],[],[]])}
    def FindColor(self,img):
        imghsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        kernel=np.array((7,7))
        for key ,value in self.color.items():
            value[2][0]=0
            img_mask=cv2.inRange(imghsv,np.array(value[0]),np.array(value[1]))
            imgdilate=cv2.dilate(img_mask,kernel,iterations=30)
            cnts_red, hierarchy = cv2.findContours(img_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in cnts_red:
                area=cv2.contourArea(cnt)
                if area>500:
                    value[2][0]+=1
                    perimeter=cv2.arcLength(cnt,True)
                    approx=cv2.approxPolyDP(cnt,0.02*perimeter,True)
                    x,y,w,h=cv2.boundingRect(approx)
                    value[2+value[2][0]]=x,y,w,h
                    cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                    cv2.putText(img, key,(int(x+w/2),int(y+h/2)),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),1)
        return img
    def callback(self, ros_data):
        msg=self.bridge.imgmsg_to_cv2(ros_data,"bgr8")
        msg=self.FindColor(msg)
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
