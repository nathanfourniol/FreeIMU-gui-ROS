import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


rospy.init_node('PubIMU', anonymous=True)
imu_topic = rospy.Publisher('/imu/imu', Imu, queue_size=10)
magn_topic = rospy.Publisher('/imu/magn', MagneticField, queue_size=10)
rate = rospy.Rate(4)

def _create_msg():
    imu_msg = Imu()
    magn_msg = MagneticField()
    
    imu_msg.header.stamp = rospy.Time.now() 
    magn_msg.header.stamp = rospy.Time.now() 
    imu_msg.linear_acceleration.x = 2102. 
    imu_msg.linear_acceleration.y = 2. 
    imu_msg.linear_acceleration.z = 56.
    magn_msg.magnetic_field.x = 78.
    magn_msg.magnetic_field.y = 88.
    magn_msg.magnetic_field.z = 98.
    return imu_msg, magn_msg


while not rospy.is_shutdown():
    msg_imu, msg_magn = _create_msg()
    imu_topic.publish(msg_imu)
    magn_topic.publish(msg_magn)
    rate.sleep()

