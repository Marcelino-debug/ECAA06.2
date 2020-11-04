import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

odom = Odometry()
scan = LaserScan()


rospy.init_node('cmd_node')

def getDirection(objeto):
    position = odom.pose.pose.position
    direcao = math.atan((objeto[1]-position.y)/(objeto[0]-position.x))
    direcao = (180 * direcao / math.pi)
    
    if position.x > 0 and position.y < 0:
        direcao = direcao
    elif position.x < 0 and position.y > 0:
        direcao = direcao + 180
    elif position.x < 0 and position.y < 0:
        direcao = direcao - 180
    
    return direcao
    
# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    
    cilindro = (2.39, 0.47)
    direcao = getDirection(cilindro)
    print(direcao)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

msg = Twist()
msg.angular.z = 0
msg.linear.x = 0
pub.publish(msg)

rospy.spin()