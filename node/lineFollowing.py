import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
 
class image_converter:

  def __init__(self):

    # set node as publisher and subscriber
    self.image_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

    # intial values for line following
    self.cX = 400
    self.cY = 600
    self.cX_prev = 400
    

  def callback(self,data):
    try:
      cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #(rows,cols,channels) = cv_image.shape
      
    #convert image to greyscale and make mask
    grayImg = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(grayImg, 150, 255, 0)

    #invert color of mask and delete anything above row 160
    mask = cv2.bitwise_not(mask)
    mask[0:600] = 0

    # find centroid of road
    M = cv2.moments(mask)
    if M["m00"] == 0:   #if there is no road, error is set to maximum
      if(self.cX - 400 > 0):
        self.cX = 800
      else:
        self.cX = 0
      self.cX_prev = self.cX
    else:
      self.cX_prev = self.cX
      self.cX = int(M["m10"] / M["m00"])
      self.cY = int(M["m01"] / M["m00"])

    # draw circle on image and display
    cv2.circle(cv_image,(self.cX, self.cY), 23, (0, 0, 255), -1)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # send message to robot
    vel_msg = Twist()
    ##PID algorithm on cX
    vel_msg.angular.z = -(self.cX - 400)/400 * 1.45 - (self.cX_prev - self.cX)/400 * 0.4
    vel_msg.linear.x = 0.5
    self.image_pub.publish(vel_msg)
    
    #sleep at rate of 15Hz
    rospy.Rate(15).sleep()
 
def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

