import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math
import random

# Variaveis de controle do angulo -----------------------------------
akp = 0.02
akd = 0.02 
aki = 0.01

aerror = 0
aerrorant = 0
aierror = 0

# Variaveis de controle da distancia --------------------------------
dkp = 0.02
dkd = 0.02 
dki = 0.01

derror = 0
derrorant = 0
dierror = 0

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw
    
def getCoordenate(matricula):
    matricula = 201813232

    random.seed(matricula)
    x = 0
    y = 0

    while x**2 + y**2 > 4**2 or x**2 + y**2 < 2**2:
        x = random.random() * 8 - 4
        y = random.random() * 8 - 4
    
    coord = (x, y)
    
    print(x, y)
    
    return coord

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg

# CONTROL FUNCTIONS -------------------------------------------------
def controlAngle():
    global aerrorant
    global aierror
    
    yaw = getAngle(odom)
    setpoint = 180
    aerror = (setpoint - yaw)
    
    if abs(aerror) > 180:
        if setpoint < 0:
            aerror += 360 
        else:
            aerror -= 360
    
    aderror = (aerror - aerrorant)/0.05
    
    aierror +=  (aerror - aerrorant)*0.05
    
    P = akp*aerror
    I = aki*aierror
    D = akd*aderror
    control = P+I+D
    
    aerrorant = aerror
    
    return control

def controlVel():
    global derrorant
    global dierror
    
    setpoint = (10,-1)
    position = odom.pose.pose.position
    dist = setpoint[0] - position.x #math.sqrt((setpoint[0] - position.x)**2 + (setpoint[1] - position.y) **2)
    derror = dist
    
    derror = (error - derrorant)/0.05
    
    dierror +=  (error - derrorant)*0.05
    
     scan_len = len(scan.ranges)
    if scan_len > 0:
        read = min(scan.ranges[scan_len-10 : scan_len+10])

        error = -(setpoint - read)
        P = dkp*derror
        I = dki*dierror
        D = dkd*dderror
        control = P+I+D
    else:
        control = 0
    
    derrorant = derror
    
    return control

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    
    #azcontrol = controlAngle()
    lxcontrol = controlVel()
    
    msg = Twist()
    msg.angular.z = azcontrol
    msg.linear.x = lxcontrol
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

rospy.spin()
