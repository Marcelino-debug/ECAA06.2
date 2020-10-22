import rospy
from std_msgs.msg import String

rospy.init_node('node1')

def soma_callBack(soma):
    print(soma)

def timerCallBack(event):
    msg = String()
    msg.data = '2016006831'
    pub.publish(msg)

sub = rospy.Subscriber('/soma',String,soma_callBack)
pub = rospy.Publisher('/matricula',String,queue_size=1)
timer = rospy.Timer(rospy.Duration(0.1), timerCallBack)

rospy.spin()