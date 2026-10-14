import rospy
from geometry_msgs.msg import Pose


class Vision(object):
    def __init__(self, topic_name):
        self.Node_name = 'VisionIdentity'
        rospy.init_node(self.Node_name)
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Pose, queue_size=1000)
        #rospy.set_param('vision_enable', 0)
        #rospy.set_param('object_classify', 0)
        rospy.set_param('IsEyeInHand', 0)

    def msg_publish(self, data):
        vision = Pose()
        vision.position.x = data[1]
        vision.position.y = data[2]
        vision.position.z = data[3]
        vision.orientation.x, vision.orientation.y, vision.orientation.z, vision.orientation.w = 0, 0, 0, 0
        self.publisher.publish(vision)


if __name__ == '__main__':
    VS = Vision()
    data = [0, 3, -0.15859199655524847, 0.44451860772426804, 0.1643870342194476, 0, 0, 0]
    while not rospy.is_shutdown():
        VS.msg_publish(data)
        #print('publish')
