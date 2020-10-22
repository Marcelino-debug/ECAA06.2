import rospy
from std_msgs.msg import String

rospy.init_node('node2')

matricula = String()

def matricula_callBack(msg):
    global matricula
    matricula = msg

def timerCallBack(event):
    global matricula
    somaTotal = 0
    for x in matricula.data:
        num = int(x)
        somaTotal =  somaTotal + num
    soma = String()
    soma.data =  str(somaTotal)
    pub.publish(soma)

sub = rospy.Subscriber('/matricula', String, matricula_callBack)
pub = rospy.Publisher('/soma',String,queue_size=1)
timer = rospy.Timer(rospy.Duration(0.1), timerCallBack)

rospy.spin()