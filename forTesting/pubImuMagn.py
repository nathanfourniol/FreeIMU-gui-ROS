import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import random as rn

rospy.init_node('PubIMU', anonymous=True)
imu_topic = rospy.Publisher('/imu/imu', Imu, queue_size=10)
magn_topic = rospy.Publisher('/imu/magn', MagneticField, queue_size=10)
rate = rospy.Rate(100)

def _create_msg():
    imu_msg = Imu()
    magn_msg = MagneticField()
    
    imu_msg.header.stamp = rospy.Time.now() 
    magn_msg.header.stamp = rospy.Time.now() 
    imu_msg.linear_acceleration.x = rn.uniform(-10,10) 
    imu_msg.linear_acceleration.y = rn.uniform(-10,10) 
    imu_msg.linear_acceleration.z = rn.uniform(-10,10)
    magn_msg.magnetic_field.x = rn.uniform(-10,10)
    magn_msg.magnetic_field.y = rn.uniform(-10,10)
    magn_msg.magnetic_field.z = rn.uniform(-10,10)
    return imu_msg, magn_msg


while not rospy.is_shutdown():
    msg_imu, msg_magn = _create_msg()
    imu_topic.publish(msg_imu)
    magn_topic.publish(msg_magn)
    rate.sleep()

