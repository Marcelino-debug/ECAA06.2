import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

# Variaveis de controle do angulo -----------------------------------
akp = 0.02
akd = 0.02 
aki = 0.01

aerror = 0
aerrorant = 0
aderror = 0
aierror = 0

# Variaveis de controle da distancia --------------------------------
dkp = 0.02
dkd = 0.02 
dki = 0.01

derror = 0
derrorant = 0
dderror = 0
dierror = 0

state = 'initial'
direcao = 0
cont = 1000

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

# CONTROL FUNCTIONS -------------------------------------------------
def controlAngle(setpoint):
    global aerrorant
    global aderror
    global aierror
    
    yaw = getAngle(odom)
    aerror = (setpoint - yaw)
    
    if abs(aerror) > 180:
        if setpoint < 0:
            aerror += 360 
        else:
            aerror -= 360
    
    aderror = (aerror - aerrorant)/0.05
    
    aierror += (aerror - aerrorant)*0.05
    
    P = akp*aerror
    I = aki*aierror
    D = akd*aderror
    control = P+I+D
    
    aerrorant = aerror
    
    print(setpoint)
    print(control)
    
    return control

def controlVel(setpoint):
    global derrorant
    global dderror
    global dierror
    
    scan_len = len(scan.ranges)
    if scan_len > 0:
        read = min(scan.ranges[scan_len-10 : scan_len+10])
        
        derror = -(setpoint - read)
        
        dderror = (derror - derrorant)/0.05
        
        dierror += (derror - derrorant)*0.05
        
        P = dkp*derror
        I = dki*dierror
        D = dkd*dderror
        control = P+I+D
        
        if control > 1:
            control = 1
        elif control < -1:
            control = -1
    
    else:
        control = 0
    
    derrorant = derror
    
    return control

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global state
    global direcao
    global cont
    msg = Twist()
    
    print(direcao)
    
    if state == 'initial':
        cilindro = (2.39, 0.47)
        direcao = getDirection(cilindro)
        state = 'state1'

    elif state == 'state1':
        msg.angular.z = controlAngle(direcao)
        if aerror < 1:
            state = 'state2'
            msg.angular.z = 0

    elif state == 'state2':
        distanciaCilindro = 0.5
        msg.linear.x = controlVel(distanciaCilindro)
        if derror < 0.1:
            state = 'state3'
            msg.linear.x = 0

    print(state)
    
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(0.05), timerCallBack)

msg = Twist()
msg.angular.z = 0
msg.linear.x = 0
pub.publish(msg)

rospy.spin()
